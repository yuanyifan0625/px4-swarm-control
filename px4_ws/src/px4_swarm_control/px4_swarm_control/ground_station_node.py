"""Ground-station action surface for the first-version PX4 swarm."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from math import pi
from math import sqrt
from time import sleep
from time import monotonic
from typing import Callable, Dict

from builtin_interfaces.msg import Time
from px4_swarm_control.bridge_config import FIRST_VERSION_VEHICLES
from px4_swarm_control.collision_safety_gate import (
    CollisionSafetyConfig,
    evaluate_leader_movement,
    VehicleObservation,
)
from px4_swarm_control.geometry import (
    body_offset_to_world,
    formation_body_offset,
    FormationGeometry,
    staging_setpoint,
)
from px4_swarm_control.models import FormationMode as InternalFormationMode
from px4_swarm_control.models import MissionState
from px4_swarm_control.models import PositionYawSetpoint
from px4_swarm_control.models import VehicleLevelState
from px4_swarm_control.operation_profile import FORMATION_POSITION_TOLERANCE_M
from px4_swarm_control.operation_profile import LINE_ABREAST_LATERAL_SPACING_M
from px4_swarm_control.operation_profile import VEE_LATERAL_SPACING_M
from px4_swarm_control.operation_profile import VEE_TRAIL_SPACING_M
from px4_swarm_interfaces.action import (
    ArmSwarm,
    ChangeFormation,
    LandSwarm,
    MoveLeader,
    PauseSwarm,
    TakeoffSwarm,
)
from px4_swarm_interfaces.msg import (
    FailsafeCommand,
    FormationMode,
    LeaderGoal,
    MissionCommand,
    VehicleSetpoint,
    VehicleStatus,
)
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


@dataclass(frozen=True)
class GroundStationConfig:
    """Static ground-station settings for first-version swarm supervision."""

    total_vehicles: int = 3
    active_formation: str = InternalFormationMode.VEE.value
    staging_lateral_spacing_m: float = VEE_LATERAL_SPACING_M
    staging_trail_spacing_m: float = VEE_TRAIL_SPACING_M
    staging_position_tolerance_m: float = 0.5
    formation_vee_lateral_spacing_m: float = VEE_LATERAL_SPACING_M
    formation_vee_trail_spacing_m: float = VEE_TRAIL_SPACING_M
    formation_line_abreast_lateral_spacing_m: float = LINE_ABREAST_LATERAL_SPACING_M
    formation_position_tolerance_m: float = FORMATION_POSITION_TOLERANCE_M
    formation_yaw_tolerance_rad: float = 0.2
    telemetry_fresh_timeout_s: float = 1.0
    safety_minimum_horizontal_distance_m: float = 0.7


@dataclass(frozen=True)
class GroundStationPublishers:
    """Topic publishers used by the ground station core."""

    mission_command: object
    leader_goal: object
    formation_mode: object
    failsafe_command: object
    vehicle_setpoints: Dict[int, object]


@dataclass(frozen=True)
class ActionOutcome:
    """Action feedback/result pair produced by core command handling."""

    result: object
    feedback: object


@dataclass(frozen=True)
class ArmEligibility:
    """Mission-level decision about whether arm-only may be sent now."""

    eligible: bool
    rejection: str = ''


class GroundStationCore:
    """Mission-level behavior that is testable without spinning ROS."""

    def __init__(
        self,
        config: GroundStationConfig,
        publishers: GroundStationPublishers,
        logger,
        now_stamp: Callable[[], Time],
        now_s: Callable[[], float] = monotonic,
    ) -> None:
        self.config = config
        self.publishers = publishers
        self.logger = logger
        self.now_stamp = now_stamp
        self.now_s = now_s
        self.mission_state = MissionState.IDLE
        self.active_formation = config.active_formation
        self.vehicle_statuses: Dict[int, VehicleStatus] = {}
        self.vehicle_status_received_s: Dict[int, float] = {}
        self.staging_targets: Dict[int, PositionYawSetpoint] = {}
        self._staging_complete_logged = False
        self._takeoff_started_s: float | None = None
        self._takeoff_timeout_s: float | None = None
        self._takeoff_reason: str | None = None
        self._takeoff_rejection: str | None = None
        self._arm_started_s: float | None = None
        self._arm_timeout_s: float | None = None
        self._arm_rejection: str | None = None
        self._leader_goal: PositionYawSetpoint | None = None
        self._leader_goal_message: LeaderGoal | None = None
        self._move_leader_started_s: float | None = None
        self._move_leader_timeout_s: float | None = None
        self._move_leader_position_tolerance_m: float | None = None
        self._move_leader_yaw_tolerance_rad: float | None = None
        self._move_leader_rejection: str | None = None
        self._change_formation_started_s: float | None = None
        self._change_formation_timeout_s: float | None = None
        self._change_formation_target: str | None = None
        self._change_formation_message: FormationMode | None = None
        self._change_formation_rejection: str | None = None
        self._formation_established_logged = False
        self._land_started_s: float | None = None
        self._land_timeout_s: float | None = None

    def start_arm(self, request: ArmSwarm.Goal):
        if self.mission_state is MissionState.PAUSED:
            self._arm_rejection = 'ArmSwarm rejected while swarm is paused'
            return self.arm_feedback()
        eligibility = self._arm_eligibility()
        if not eligibility.eligible:
            self._arm_started_s = None
            self._arm_timeout_s = None
            self._arm_rejection = eligibility.rejection
            return self.arm_feedback()
        self.vehicle_statuses = {}
        self._clear_move_leader_state()
        self._clear_change_formation_state()
        self._arm_started_s = self.now_s()
        self._arm_timeout_s = max(float(request.timeout_sec), 0.0)
        self._arm_rejection = None
        self._transition_to(MissionState.ARMING, 'arm-only action accepted')
        self.publishers.mission_command.publish(
            self._mission_command(
                MissionCommand.ARM,
                f'timeout_sec={request.timeout_sec:.2f}',
            ),
        )
        return self.arm_feedback()

    def arm_feedback(self) -> ArmSwarm.Feedback:
        feedback = ArmSwarm.Feedback()
        feedback.current_state = self.mission_state.value
        feedback.vehicles_armed = self._count_armed_vehicles()
        feedback.total_vehicles = self.config.total_vehicles
        return feedback

    def arm_result(self) -> ArmSwarm.Result | None:
        result = ArmSwarm.Result()
        if self._arm_rejection is not None:
            result.success = False
            result.message = self._arm_rejection
            return result
        if self._all_vehicles_armed():
            result.success = True
            result.message = 'all vehicles reported armed'
            return result
        if self._arm_timed_out():
            self._transition_to(MissionState.ERROR, 'arm-only timed out')
            result.success = False
            result.message = 'arm-only timed out'
            return result
        return None

    def start_takeoff(self, request: TakeoffSwarm.Goal):
        if self.mission_state is MissionState.PAUSED:
            # Pause 期間拒絕起飛新任務，保護暫停語意只允許 resume/status/land。
            self._takeoff_rejection = 'TakeoffSwarm rejected while swarm is paused'
            return self.takeoff_feedback()
        leader_status = self.vehicle_statuses.get(1)
        leader_status_received_s = self.vehicle_status_received_s.get(1)
        if not _staging_anchor_status_is_fresh(
            leader_status,
            self.config.telemetry_fresh_timeout_s,
            cache_age_s=(
                self.now_s() - leader_status_received_s
                if leader_status_received_s is not None
                else float('inf')
            ),
        ):
            self._takeoff_rejection = (
                'takeoff rejected: fresh MAV1 staging anchor unavailable'
            )
            return self.takeoff_feedback()
        # 新任務先清掉舊 status，保護 action completion 不被上一輪 landed/staging 污染。
        self.vehicle_statuses = {}
        self._clear_move_leader_state()
        self._clear_change_formation_state()
        self._takeoff_rejection = None
        self._takeoff_started_s = self.now_s()
        self._takeoff_timeout_s = max(float(request.timeout_sec), 0.0)
        self._takeoff_reason = (
            f'altitude_m={request.altitude_m:.2f} timeout_sec={request.timeout_sec:.2f}'
        )
        self._land_started_s = None
        self._land_timeout_s = None
        self._transition_to(MissionState.TAKING_OFF, 'takeoff action accepted')
        self._publish_staging_setpoints(request.altitude_m, leader_status)
        self.publishers.mission_command.publish(
            self._mission_command(MissionCommand.TAKEOFF, self._takeoff_reason),
        )
        return self.takeoff_feedback()

    def takeoff_feedback(self) -> TakeoffSwarm.Feedback:
        feedback = TakeoffSwarm.Feedback()
        feedback.current_state = self.mission_state.value
        feedback.vehicles_staged = self._count_staged_vehicles()
        feedback.total_vehicles = self.config.total_vehicles
        feedback.progress = (
            float(feedback.vehicles_staged) / float(feedback.total_vehicles)
            if feedback.total_vehicles
            else 0.0
        )
        return feedback

    def takeoff_result(self) -> TakeoffSwarm.Result | None:
        result = TakeoffSwarm.Result()
        if self._takeoff_rejection is not None:
            result.success = False
            result.message = self._takeoff_rejection
            return result
        if self.mission_state is MissionState.STAGING:
            result.success = True
            result.message = 'all vehicles reached staging positions'
            return result
        if self._takeoff_timed_out():
            self._transition_to(MissionState.ERROR, 'takeoff staging timed out')
            result.success = False
            result.message = 'takeoff staging timed out'
            return result
        return None

    def start_move_leader(self, request: MoveLeader.Goal):
        if self.mission_state is MissionState.PAUSED:
            # Pause 期間拒絕新移動，保護 operator 以為系統停住時仍偷偷更新目標。
            self._move_leader_rejection = 'MoveLeader rejected while swarm is paused'
            return self.move_leader_feedback()
        self._move_leader_started_s = self.now_s()
        self._move_leader_timeout_s = max(float(request.timeout_sec), 0.0)
        self._move_leader_position_tolerance_m = float(request.position_tolerance_m)
        self._move_leader_yaw_tolerance_rad = float(request.yaw_tolerance_rad)
        self._move_leader_rejection = None
        self._clear_change_formation_state()
        self._leader_goal = None
        self._leader_goal_message = None

        if not self._valid_move_leader_request(request):
            self._move_leader_rejection = 'invalid leader goal'
            self._transition_to(MissionState.ERROR, self._move_leader_rejection)
            return self.move_leader_feedback()

        movement_safety = self._leader_movement_safety(request)
        if not movement_safety.allowed:
            self._move_leader_rejection = (
                f'MoveLeader rejected: {movement_safety.reason}'
            )
            return self.move_leader_feedback()

        self._transition_to(MissionState.FOLLOWING, 'leader goal accepted')
        msg = LeaderGoal()
        msg.stamp = self.now_stamp()
        msg.frame_id = 'world'
        msg.x = request.x
        msg.y = request.y
        msg.z = request.z
        msg.yaw = request.yaw
        self._leader_goal = PositionYawSetpoint(request.x, request.y, request.z, request.yaw)
        self._leader_goal_message = msg
        self.publishers.leader_goal.publish(msg)
        return self.move_leader_feedback()

    def republish_leader_goal(self) -> None:
        if self._leader_goal_message is None:
            return
        # MoveLeader 等待期間重送 leader goal，保護 late subscriber 不錯過 single-shot 目標。
        self.publishers.leader_goal.publish(self._leader_goal_message)

    def move_leader_feedback(self) -> MoveLeader.Feedback:
        feedback = MoveLeader.Feedback()
        feedback.current_state = self.mission_state.value
        feedback.remaining_distance_m = self._leader_remaining_distance_m()
        feedback.yaw_error_rad = self._leader_yaw_error_rad()
        return feedback

    def move_leader_result(self) -> MoveLeader.Result | None:
        result = MoveLeader.Result()
        if self._move_leader_rejection is not None:
            result.success = False
            result.message = self._move_leader_rejection
            return result
        if self._leader_goal_reached():
            result.success = True
            result.message = 'leader reached target'
            return result
        if self._move_leader_timed_out():
            self._transition_to(MissionState.ERROR, 'leader movement timed out')
            result.success = False
            result.message = 'leader movement timed out'
            return result
        return None

    def start_change_formation(self, request: ChangeFormation.Goal) -> ChangeFormation.Feedback:
        if self.mission_state is MissionState.PAUSED:
            # Pause 期間拒絕隊形變換，保護 followers 不在 operator 暫停時改追新 slot。
            self._change_formation_rejection = (
                'ChangeFormation rejected while swarm is paused'
            )
            return self.change_formation_feedback()
        self.vehicle_statuses = {}
        self._change_formation_started_s = self.now_s()
        self._change_formation_timeout_s = max(float(request.timeout_sec), 0.0)
        self._change_formation_target = None
        self._change_formation_message = None
        self._change_formation_rejection = None
        self._formation_established_logged = False

        if request.formation_mode not in _supported_formation_modes():
            # 未知隊形會讓 follower slot 解讀不一致，因此在 ground station 邊界拒絕。
            self._change_formation_rejection = (
                f'unsupported formation mode: {request.formation_mode}'
            )
            self._transition_to(MissionState.ERROR, 'unsupported formation mode')
            return self.change_formation_feedback()
        if self.mission_state is not MissionState.FOLLOWING:
            self._change_formation_rejection = 'ChangeFormation requires following state'
            self._transition_to(MissionState.ERROR, 'change formation requires following')
            return self.change_formation_feedback()

        self._transition_to(MissionState.RECONFIGURING, 'formation change accepted')
        self.active_formation = request.formation_mode
        msg = FormationMode()
        msg.stamp = self.now_stamp()
        msg.mode = request.formation_mode
        self._change_formation_target = request.formation_mode
        self._change_formation_message = msg
        self.publishers.formation_mode.publish(msg)
        return self.change_formation_feedback()

    def republish_formation_mode(self) -> None:
        if self._change_formation_message is None:
            return
        self.publishers.formation_mode.publish(self._change_formation_message)

    def change_formation_feedback(self) -> ChangeFormation.Feedback:
        feedback = ChangeFormation.Feedback()
        feedback.current_state = self.mission_state.value
        feedback.active_formation = self.active_formation
        feedback.progress = self._formation_progress()
        return feedback

    def change_formation_result(self) -> ChangeFormation.Result | None:
        result = ChangeFormation.Result()
        if self._change_formation_rejection is not None:
            result.success = False
            result.message = self._change_formation_rejection
            return result
        if self._formation_established():
            self._transition_to(MissionState.FOLLOWING, 'formation established')
            if not self._formation_established_logged:
                self.logger.info('formation established')
                self._formation_established_logged = True
            result.success = True
            result.message = 'formation established'
            return result
        if self._change_formation_timed_out():
            self._transition_to(MissionState.ERROR, 'formation change timed out')
            result.success = False
            result.message = 'formation change timed out'
            return result
        return None

    def handle_pause(self, request: PauseSwarm.Goal):
        command = MissionCommand.PAUSE if request.pause else MissionCommand.RESUME
        next_state = MissionState.PAUSED if request.pause else MissionState.HOLDING
        reason = request.reason or (
            'pause requested' if request.pause else 'resume requested'
        )
        if request.pause:
            self._clear_move_leader_state()
            self._clear_change_formation_state()
        else:
            # Resume 只回到安全 holding，保護 pause 前的舊 action 不被自動續跑。
            self._clear_move_leader_state()
            self._clear_change_formation_state()
        self._transition_to(next_state, reason)
        self.publishers.mission_command.publish(self._mission_command(command, reason))
        failsafe = FailsafeCommand()
        failsafe.stamp = self.now_stamp()
        failsafe.active = request.pause
        failsafe.action = FailsafeCommand.HOVER
        failsafe.reason = reason
        self.publishers.failsafe_command.publish(failsafe)

        feedback = PauseSwarm.Feedback()
        feedback.current_state = self.mission_state.value
        feedback.paused = request.pause

        result = PauseSwarm.Result()
        result.success = True
        result.message = f'{command} command accepted and published'
        return ActionOutcome(result=result, feedback=feedback)

    def start_land(self, request: LandSwarm.Goal):
        # LandSwarm 只看本輪降落後的新 status，避免上一輪 landed cache 直接完成 action。
        self.vehicle_statuses = {}
        self._clear_move_leader_state()
        self._clear_change_formation_state()
        self._land_started_s = self.now_s()
        self._land_timeout_s = max(float(request.timeout_sec), 0.0)
        self._takeoff_started_s = None
        self._takeoff_timeout_s = None
        self._takeoff_reason = None
        self._arm_started_s = None
        self._arm_timeout_s = None
        self._arm_rejection = None
        self._transition_to(MissionState.LANDING, 'land-all action accepted')
        self.publishers.mission_command.publish(
            self._mission_command(
                MissionCommand.LAND,
                f'timeout_sec={request.timeout_sec:.2f}',
            ),
        )
        return self.land_feedback()

    def land_feedback(self) -> LandSwarm.Feedback:
        feedback = LandSwarm.Feedback()
        feedback.current_state = self.mission_state.value
        feedback.vehicles_landed = self._count_landed_vehicles()
        feedback.total_vehicles = self.config.total_vehicles
        return feedback

    def land_result(self) -> LandSwarm.Result | None:
        result = LandSwarm.Result()
        if self.mission_state is MissionState.DONE:
            result.success = True
            result.message = 'all vehicles reported landed'
            return result
        if self._land_timed_out():
            self._transition_to(MissionState.ERROR, 'land-all timed out')
            result.success = False
            result.message = 'land-all timed out'
            return result
        return None

    def handle_vehicle_status(self, msg: VehicleStatus) -> None:
        vehicle_id = int(msg.vehicle_id)
        self.vehicle_statuses[vehicle_id] = msg
        self.vehicle_status_received_s[vehicle_id] = self.now_s()
        if msg.vehicle_state == VehicleLevelState.FAILSAFE.value:
            # 任一 vehicle 進入 failsafe 代表任務層要停止正常流程，避免繼續發布移動命令。
            self._transition_to(MissionState.FAILSAFE, 'vehicle reported failsafe')
            return
        if (
            self.mission_state is MissionState.LANDING
            and self._all_vehicles_landed()
        ):
            # 全隊 landed 才把任務收斂到 done，避免單機降落時誤判整隊完成。
            self._transition_to(MissionState.DONE, 'all vehicles reported landed')
            return
        if self.mission_state is MissionState.TAKING_OFF and self._all_vehicles_staged():
            # 三台都在 staging tolerance 內才宣布完成，保護起飛集結不因單機先到而提前進入下一階段。
            self._transition_to(MissionState.STAGING, 'all vehicles staged')
            if not self._staging_complete_logged:
                self.logger.info('all vehicles reached staging positions')
                self._staging_complete_logged = True

    def _mission_command(self, command: str, reason: str) -> MissionCommand:
        msg = MissionCommand()
        msg.stamp = self.now_stamp()
        msg.command = command
        msg.reason = reason
        return msg

    def _transition_to(self, next_state: MissionState, reason: str) -> None:
        if self.mission_state is next_state:
            return
        previous = self.mission_state
        self.mission_state = next_state
        self.logger.info(
            f'swarm mission {previous.value} -> {next_state.value}: {reason}',
        )

    def _all_known_vehicles_in_state(self, vehicle_state: str) -> bool:
        if len(self.vehicle_statuses) < self.config.total_vehicles:
            return False
        return all(
            status.vehicle_state == vehicle_state
            for status in self.vehicle_statuses.values()
        )

    def _publish_staging_setpoints(
        self,
        altitude_m: float,
        leader_status: VehicleStatus,
    ) -> None:
        # 起飛 staging 固定用 world frame，保護三機在離地前後維持水平安全間距。
        geometry = FormationGeometry(
            vee_lateral_spacing_m=self.config.staging_lateral_spacing_m,
            vee_trail_spacing_m=self.config.staging_trail_spacing_m,
            line_abreast_lateral_spacing_m=(
                self.config.formation_line_abreast_lateral_spacing_m
            ),
        )
        leader = PositionYawSetpoint(
            x=leader_status.x,
            y=leader_status.y,
            z=leader_status.z - abs(float(altitude_m)),
            yaw=leader_status.yaw,
        )
        self.staging_targets = {}
        self._staging_complete_logged = False
        for vehicle in FIRST_VERSION_VEHICLES:
            # 由 ground station 只分派目標位置，保護每台 vehicle node 仍只控制自己的 PX4。
            target = staging_setpoint(leader, vehicle.slot, geometry)
            self.staging_targets[vehicle.px4_instance] = target
            self._publish_vehicle_setpoint(vehicle.px4_instance, target)

    def republish_staging_setpoints(self) -> None:
        if not self.staging_targets:
            return
        # 等待 action 完成期間重送 staging target，保護 late subscriber 不錯過 single-shot 目標。
        for vehicle_id, target in self.staging_targets.items():
            self._publish_vehicle_setpoint(vehicle_id, target)

    def republish_takeoff_request(self) -> None:
        self.republish_staging_setpoints()
        if self._takeoff_reason is None:
            return
        # 等待起飛完成期間重送任務命令，保護 vehicle node 不因 topic timing 錯過 TAKEOFF。
        self.publishers.mission_command.publish(
            self._mission_command(MissionCommand.TAKEOFF, self._takeoff_reason),
        )

    def _publish_vehicle_setpoint(
        self,
        vehicle_id: int,
        target: PositionYawSetpoint,
    ) -> None:
        msg = VehicleSetpoint()
        msg.stamp = self.now_stamp()
        msg.vehicle_id = vehicle_id
        msg.frame_id = 'world'
        msg.x = target.x
        msg.y = target.y
        msg.z = target.z
        msg.yaw = target.yaw
        self.publishers.vehicle_setpoints[vehicle_id].publish(msg)

    def _all_vehicles_staged(self) -> bool:
        if len(self.staging_targets) < self.config.total_vehicles:
            return False
        if len(self.vehicle_statuses) < self.config.total_vehicles:
            return False
        return all(
            _staging_ready(
                self.vehicle_statuses[vehicle_id],
                target,
                self.config.staging_position_tolerance_m,
                self.config.telemetry_fresh_timeout_s,
            )
            for vehicle_id, target in self.staging_targets.items()
        )

    def _all_vehicles_landed(self) -> bool:
        if len(self.vehicle_statuses) < self.config.total_vehicles:
            return False
        return all(_landed_ready(status, self.config.telemetry_fresh_timeout_s)
                   for status in self.vehicle_statuses.values())

    def _all_vehicles_armed(self) -> bool:
        if len(self.vehicle_statuses) < self.config.total_vehicles:
            return False
        return all(
            _armed_ready(status, self.config.telemetry_fresh_timeout_s)
            for status in self.vehicle_statuses.values()
        )

    def _arm_eligibility(self) -> ArmEligibility:
        for vehicle_id in sorted(self.vehicle_statuses):
            status = self.vehicle_statuses[vehicle_id]
            rejection = _arm_ineligible_reason(
                status,
                self.config.telemetry_fresh_timeout_s,
            )
            if rejection:
                return ArmEligibility(False, rejection)
        return ArmEligibility(True)

    def _count_staged_vehicles(self) -> int:
        return sum(
            1
            for vehicle_id, target in self.staging_targets.items()
            if vehicle_id in self.vehicle_statuses
            and _staging_ready(
                self.vehicle_statuses[vehicle_id],
                target,
                self.config.staging_position_tolerance_m,
                self.config.telemetry_fresh_timeout_s,
            )
        )

    def _count_landed_vehicles(self) -> int:
        return sum(
            1
            for status in self.vehicle_statuses.values()
            if status.vehicle_state == VehicleLevelState.LANDED.value
        )

    def _count_armed_vehicles(self) -> int:
        return sum(
            1
            for status in self.vehicle_statuses.values()
            if _armed_ready(status, self.config.telemetry_fresh_timeout_s)
        )

    def _arm_timed_out(self) -> bool:
        return (
            self._arm_started_s is not None
            and self._arm_timeout_s is not None
            and self.now_s() - self._arm_started_s > self._arm_timeout_s
        )

    def _takeoff_timed_out(self) -> bool:
        return (
            self._takeoff_started_s is not None
            and self._takeoff_timeout_s is not None
            and self.now_s() - self._takeoff_started_s > self._takeoff_timeout_s
        )

    def _land_timed_out(self) -> bool:
        return (
            self._land_started_s is not None
            and self._land_timeout_s is not None
            and self.now_s() - self._land_started_s > self._land_timeout_s
        )

    def _change_formation_timed_out(self) -> bool:
        return (
            self._change_formation_started_s is not None
            and self._change_formation_timeout_s is not None
            and self.now_s() - self._change_formation_started_s
            > self._change_formation_timeout_s
        )

    def _valid_move_leader_request(self, request: MoveLeader.Goal) -> bool:
        values = (
            request.x,
            request.y,
            request.z,
            request.yaw,
            float(request.position_tolerance_m),
            float(request.yaw_tolerance_rad),
        )
        # Operator goal 必須是有限 world-frame 數值，保護 action 不把 NaN/Inf 送進 Offboard setpoint。
        return (
            all(isfinite(value) for value in values)
            and float(request.position_tolerance_m) > 0.0
            and float(request.yaw_tolerance_rad) > 0.0
        )

    def _leader_movement_safety(self, request: MoveLeader.Goal):
        leader_goal = PositionYawSetpoint(request.x, request.y, request.z, request.yaw)
        geometry = FormationGeometry(
            vee_lateral_spacing_m=self.config.formation_vee_lateral_spacing_m,
            vee_trail_spacing_m=self.config.formation_vee_trail_spacing_m,
            line_abreast_lateral_spacing_m=(
                self.config.formation_line_abreast_lateral_spacing_m
            ),
        )
        theoretical_positions = {1: leader_goal}
        for vehicle in _follower_bridge_expectations():
            theoretical_positions[vehicle.px4_instance] = body_offset_to_world(
                leader_goal,
                formation_body_offset(
                    InternalFormationMode(self.active_formation),
                    vehicle.slot,
                    geometry,
                ),
            )
        actual_observations = {
            vehicle.px4_instance: _guard_status_observation(
                self.vehicle_statuses.get(vehicle.px4_instance)
            )
            for vehicle in FIRST_VERSION_VEHICLES
        }
        return evaluate_leader_movement(
            actual_observations=actual_observations,
            theoretical_positions=theoretical_positions,
            config=CollisionSafetyConfig(
                minimum_horizontal_distance_m=(
                    self.config.safety_minimum_horizontal_distance_m
                ),
                telemetry_timeout_s=self.config.telemetry_fresh_timeout_s,
            ),
        )

    def _leader_status(self) -> VehicleStatus | None:
        return self.vehicle_statuses.get(1)

    def _fresh_leader_status(self) -> VehicleStatus | None:
        status = self._leader_status()
        if status is None:
            return None
        if (
            not isfinite(status.last_telemetry_age_sec)
            or status.last_telemetry_age_sec > self.config.telemetry_fresh_timeout_s
        ):
            return None
        # MoveLeader 完成要確認 PX4 仍在 Offboard 控制中，保護 landed/auto 狀態不誤判成功。
        if not status.armed or status.nav_state != 'offboard':
            return None
        return status

    def _fresh_formation_leader_status(self) -> VehicleStatus | None:
        status = self._fresh_leader_status()
        if status is None or status.vehicle_state != VehicleLevelState.FOLLOWING.value:
            return None
        if not _status_pose_is_finite(status):
            return None
        return status

    def _leader_remaining_distance_m(self) -> float:
        status = self._fresh_leader_status()
        if status is None or self._leader_goal is None:
            return float('inf')
        return sqrt(
            (status.x - self._leader_goal.x) ** 2
            + (status.y - self._leader_goal.y) ** 2
            + (status.z - self._leader_goal.z) ** 2,
        )

    def _leader_yaw_error_rad(self) -> float:
        status = self._fresh_leader_status()
        if status is None or self._leader_goal is None:
            return float('inf')
        return _yaw_error_rad(status.yaw, self._leader_goal.yaw)

    def _leader_goal_reached(self) -> bool:
        if (
            self._leader_goal is None
            or self._move_leader_position_tolerance_m is None
            or self._move_leader_yaw_tolerance_rad is None
        ):
            return False
        # 完成條件只看 fresh leader status，保護 MoveLeader 不被 follower 或舊 telemetry 提前完成。
        return (
            self._leader_remaining_distance_m()
            <= self._move_leader_position_tolerance_m
            and self._leader_yaw_error_rad() <= self._move_leader_yaw_tolerance_rad
        )

    def _formation_progress(self) -> float:
        followers = _follower_bridge_expectations()
        if not followers:
            return 0.0
        reached = sum(1 for follower in followers if self._follower_formation_ready(follower))
        return float(reached) / float(len(followers))

    def _formation_established(self) -> bool:
        if self._change_formation_target is None:
            return False
        # completion 必須等真實 status 進入 tolerance，保護 operator 不把 mode topic 發出誤認為隊形已完成。
        return self._formation_progress() >= 1.0

    def _follower_formation_ready(self, vehicle) -> bool:
        leader_status = self._fresh_formation_leader_status()
        if leader_status is None or self._change_formation_target is None:
            return False
        status = self.vehicle_statuses.get(vehicle.px4_instance)
        if not _fresh_follower_status(
            status,
            expected_slot=vehicle.slot.value,
            telemetry_fresh_timeout_s=self.config.telemetry_fresh_timeout_s,
        ):
            return False
        target = self._formation_target_for_follower(leader_status, vehicle)
        return _formation_status_close(
            status,
            target,
            self.config.formation_position_tolerance_m,
            self.config.formation_yaw_tolerance_rad,
        )

    def _formation_target_for_follower(
        self,
        leader_status: VehicleStatus,
        vehicle,
    ) -> PositionYawSetpoint:
        geometry = FormationGeometry(
            vee_lateral_spacing_m=self.config.formation_vee_lateral_spacing_m,
            vee_trail_spacing_m=self.config.formation_vee_trail_spacing_m,
            line_abreast_lateral_spacing_m=(
                self.config.formation_line_abreast_lateral_spacing_m
            ),
        )
        leader = PositionYawSetpoint(
            leader_status.x,
            leader_status.y,
            leader_status.z,
            leader_status.yaw,
        )
        return body_offset_to_world(
            leader,
            formation_body_offset(
                InternalFormationMode(self._change_formation_target),
                vehicle.slot,
                geometry,
            ),
        )

    def _move_leader_timed_out(self) -> bool:
        return (
            self._move_leader_started_s is not None
            and self._move_leader_timeout_s is not None
            and self.now_s() - self._move_leader_started_s
            > self._move_leader_timeout_s
        )

    def _clear_move_leader_state(self) -> None:
        # 任務階段切換時清掉 leader goal，保護 takeoff/land 不被上一個 MoveLeader 重送干擾。
        self._leader_goal = None
        self._leader_goal_message = None
        self._move_leader_started_s = None
        self._move_leader_timeout_s = None
        self._move_leader_position_tolerance_m = None
        self._move_leader_yaw_tolerance_rad = None
        self._move_leader_rejection = None

    def _clear_change_formation_state(self) -> None:
        self._change_formation_started_s = None
        self._change_formation_timeout_s = None
        self._change_formation_target = None
        self._change_formation_message = None
        self._change_formation_rejection = None
        self._formation_established_logged = False


class GroundStationNode(Node):
    """ROS 2 node exposing operator actions under `/swarm`."""

    def __init__(self) -> None:
        super().__init__('ground_station_node', namespace='/swarm')
        self.callback_group = ReentrantCallbackGroup()
        self._declare_parameters()
        self.config = self._load_config()
        publishers = GroundStationPublishers(
            mission_command=self.create_publisher(MissionCommand, 'mission_command', 10),
            leader_goal=self.create_publisher(LeaderGoal, 'leader_goal', 10),
            formation_mode=self.create_publisher(FormationMode, 'formation_mode', 10),
            failsafe_command=self.create_publisher(
                FailsafeCommand,
                'failsafe_command',
                10,
            ),
            vehicle_setpoints={
                vehicle.px4_instance: self.create_publisher(
                    VehicleSetpoint,
                    f'{vehicle.namespace}/staging_setpoint',
                    10,
                )
                for vehicle in FIRST_VERSION_VEHICLES
            },
        )
        self.core = GroundStationCore(
            self.config,
            publishers,
            self.get_logger(),
            self._now_stamp,
        )
        self.vehicle_status_subscriptions = [
            self.create_subscription(
                VehicleStatus,
                f'{vehicle.namespace}/status',
                self.core.handle_vehicle_status,
                10,
                callback_group=self.callback_group,
            )
            for vehicle in FIRST_VERSION_VEHICLES
        ]
        self.action_servers = (
            ActionServer(
                self,
                ArmSwarm,
                'arm',
                self._execute_arm,
                callback_group=self.callback_group,
            ),
            ActionServer(
                self,
                TakeoffSwarm,
                'takeoff',
                self._execute_takeoff,
                callback_group=self.callback_group,
            ),
            ActionServer(
                self,
                MoveLeader,
                'move_leader',
                self._execute_move_leader,
                callback_group=self.callback_group,
            ),
            ActionServer(
                self,
                ChangeFormation,
                'change_formation',
                self._execute_change_formation,
                callback_group=self.callback_group,
            ),
            ActionServer(
                self,
                PauseSwarm,
                'pause',
                self._execute_pause,
                callback_group=self.callback_group,
            ),
            ActionServer(
                self,
                LandSwarm,
                'land',
                self._execute_land,
                callback_group=self.callback_group,
            ),
        )

    def _execute_arm(self, goal_handle):
        self.core.start_arm(goal_handle.request)
        while rclpy.ok():
            goal_handle.publish_feedback(self.core.arm_feedback())
            result = self.core.arm_result()
            if result is not None:
                if result.success:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return result
            sleep(0.1)

        result = ArmSwarm.Result()
        result.success = False
        result.message = 'ROS shutdown before arm-only completed'
        goal_handle.abort()
        return result

    def _execute_takeoff(self, goal_handle):
        self.core.start_takeoff(goal_handle.request)
        while rclpy.ok():
            self.core.republish_takeoff_request()
            goal_handle.publish_feedback(self.core.takeoff_feedback())
            result = self.core.takeoff_result()
            if result is not None:
                if result.success:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return result
            sleep(0.1)

        result = TakeoffSwarm.Result()
        result.success = False
        result.message = 'ROS shutdown before takeoff staging completed'
        goal_handle.abort()
        return result

    def _execute_move_leader(self, goal_handle):
        self.core.start_move_leader(goal_handle.request)
        while rclpy.ok():
            self.core.republish_leader_goal()
            goal_handle.publish_feedback(self.core.move_leader_feedback())
            result = self.core.move_leader_result()
            if result is not None:
                if result.success:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return result
            sleep(0.1)

        result = MoveLeader.Result()
        result.success = False
        result.message = 'ROS shutdown before leader reached target'
        goal_handle.abort()
        return result

    def _execute_change_formation(self, goal_handle):
        self.core.start_change_formation(goal_handle.request)
        while rclpy.ok():
            self.core.republish_formation_mode()
            goal_handle.publish_feedback(self.core.change_formation_feedback())
            result = self.core.change_formation_result()
            if result is not None:
                if result.success:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return result
            sleep(0.1)

        result = ChangeFormation.Result()
        result.success = False
        result.message = 'ROS shutdown before formation established'
        goal_handle.abort()
        return result

    def _execute_pause(self, goal_handle):
        return self._finish_action(goal_handle, self.core.handle_pause(goal_handle.request))

    def _execute_land(self, goal_handle):
        self.core.start_land(goal_handle.request)
        while rclpy.ok():
            goal_handle.publish_feedback(self.core.land_feedback())
            result = self.core.land_result()
            if result is not None:
                if result.success:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return result
            sleep(0.1)

        result = LandSwarm.Result()
        result.success = False
        result.message = 'ROS shutdown before land-all completed'
        goal_handle.abort()
        return result

    def _finish_action(self, goal_handle, outcome):
        goal_handle.publish_feedback(outcome.feedback)
        if outcome.result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return outcome.result

    def _now_stamp(self) -> Time:
        return self.get_clock().now().to_msg()

    def _declare_parameters(self) -> None:
        self.declare_parameter('staging_lateral_spacing_m', VEE_LATERAL_SPACING_M)
        self.declare_parameter('staging_trail_spacing_m', VEE_TRAIL_SPACING_M)
        self.declare_parameter('staging_position_tolerance_m', 0.5)
        self.declare_parameter('formation_vee_lateral_spacing_m', VEE_LATERAL_SPACING_M)
        self.declare_parameter('formation_vee_trail_spacing_m', VEE_TRAIL_SPACING_M)
        self.declare_parameter(
            'formation_line_abreast_lateral_spacing_m',
            LINE_ABREAST_LATERAL_SPACING_M,
        )
        self.declare_parameter(
            'formation_position_tolerance_m',
            FORMATION_POSITION_TOLERANCE_M,
        )
        self.declare_parameter('formation_yaw_tolerance_rad', 0.2)
        self.declare_parameter('telemetry_fresh_timeout_s', 1.0)
        self.declare_parameter('safety_minimum_horizontal_distance_m', 0.7)

    def _load_config(self) -> GroundStationConfig:
        return GroundStationConfig(
            total_vehicles=len(FIRST_VERSION_VEHICLES),
            staging_lateral_spacing_m=float(
                self.get_parameter('staging_lateral_spacing_m').value,
            ),
            staging_trail_spacing_m=float(
                self.get_parameter('staging_trail_spacing_m').value,
            ),
            staging_position_tolerance_m=float(
                self.get_parameter('staging_position_tolerance_m').value,
            ),
            formation_vee_lateral_spacing_m=float(
                self.get_parameter('formation_vee_lateral_spacing_m').value,
            ),
            formation_vee_trail_spacing_m=float(
                self.get_parameter('formation_vee_trail_spacing_m').value,
            ),
            formation_line_abreast_lateral_spacing_m=float(
                self.get_parameter('formation_line_abreast_lateral_spacing_m').value,
            ),
            formation_position_tolerance_m=float(
                self.get_parameter('formation_position_tolerance_m').value,
            ),
            formation_yaw_tolerance_rad=float(
                self.get_parameter('formation_yaw_tolerance_rad').value,
            ),
            telemetry_fresh_timeout_s=float(
                self.get_parameter('telemetry_fresh_timeout_s').value,
            ),
            safety_minimum_horizontal_distance_m=float(
                self.get_parameter('safety_minimum_horizontal_distance_m').value,
            ),
        )


def default_ground_station_config() -> GroundStationConfig:
    return GroundStationConfig(total_vehicles=len(FIRST_VERSION_VEHICLES))


def _supported_formation_modes() -> set[str]:
    return {mode.value for mode in InternalFormationMode}


def _position_close(
    status: VehicleStatus,
    target: PositionYawSetpoint,
    tolerance_m: float,
) -> bool:
    return (
        abs(status.x - target.x) <= tolerance_m
        and abs(status.y - target.y) <= tolerance_m
        and abs(status.z - target.z) <= tolerance_m
    )


def _yaw_error_rad(current_yaw: float, target_yaw: float) -> float:
    wrapped = (current_yaw - target_yaw + pi) % (2.0 * pi) - pi
    return abs(wrapped)


def _status_pose_is_finite(status: VehicleStatus) -> bool:
    return all(isfinite(value) for value in (status.x, status.y, status.z, status.yaw))


def _staging_anchor_status_is_fresh(
    status: VehicleStatus | None,
    telemetry_fresh_timeout_s: float,
    *,
    cache_age_s: float,
) -> bool:
    return (
        status is not None
        and int(status.vehicle_id) == 1
        and isfinite(cache_age_s)
        and 0.0 <= cache_age_s <= telemetry_fresh_timeout_s
        and isfinite(status.last_telemetry_age_sec)
        and status.last_telemetry_age_sec <= telemetry_fresh_timeout_s
        and _status_pose_is_finite(status)
    )


def _guard_status_observation(status: VehicleStatus | None):
    if status is None or not status.armed or status.nav_state != 'offboard':
        return None
    return VehicleObservation(
        vehicle_id=int(status.vehicle_id),
        x=status.x,
        y=status.y,
        z=status.z,
        yaw=status.yaw,
        telemetry_age_s=status.last_telemetry_age_sec,
    )


def _formation_status_close(
    status: VehicleStatus,
    target: PositionYawSetpoint,
    position_tolerance_m: float,
    yaw_tolerance_rad: float,
) -> bool:
    return (
        _position_close(status, target, position_tolerance_m)
        and _yaw_error_rad(status.yaw, target.yaw) <= yaw_tolerance_rad
    )


def _fresh_follower_status(
    status: VehicleStatus | None,
    *,
    expected_slot: str,
    telemetry_fresh_timeout_s: float,
) -> bool:
    if status is None:
        return False
    return (
        status.slot == expected_slot
        and status.armed
        and status.nav_state == 'offboard'
        and isfinite(status.last_telemetry_age_sec)
        and status.last_telemetry_age_sec <= telemetry_fresh_timeout_s
        and _status_pose_is_finite(status)
    )


def _follower_bridge_expectations():
    return [vehicle for vehicle in FIRST_VERSION_VEHICLES if vehicle.px4_instance != 1]


def _staging_ready(
    status: VehicleStatus,
    target: PositionYawSetpoint,
    tolerance_m: float,
    telemetry_fresh_timeout_s: float,
) -> bool:
    # staging 完成必須等 PX4 真正進入 Offboard，保護任務層不把可見 setpoint 誤當已受控。
    return (
        status.armed
        and isfinite(status.last_telemetry_age_sec)
        and status.last_telemetry_age_sec <= telemetry_fresh_timeout_s
        and status.nav_state == 'offboard'
        and _position_close(status, target, tolerance_m)
    )


def _landed_ready(status: VehicleStatus, telemetry_fresh_timeout_s: float) -> bool:
    # landed completion 也要求新 telemetry，保護上一輪 landed cache 不完成本輪降落 action。
    return (
        status.vehicle_state == VehicleLevelState.LANDED.value
        and isfinite(status.last_telemetry_age_sec)
        and status.last_telemetry_age_sec <= telemetry_fresh_timeout_s
    )


def _armed_ready(status: VehicleStatus, telemetry_fresh_timeout_s: float) -> bool:
    return (
        status.armed
        and isfinite(status.last_telemetry_age_sec)
        and status.last_telemetry_age_sec <= telemetry_fresh_timeout_s
    )


def _arm_ineligible_reason(
    status: VehicleStatus,
    telemetry_fresh_timeout_s: float,
) -> str:
    if status.armed or status.vehicle_state != VehicleLevelState.LANDED.value:
        return ''
    vehicle_name = f'MAV{int(status.vehicle_id)}'
    if status.nav_state == 'offboard' and status.offboard_control_signal_lost:
        return (
            f'arm rejected: {vehicle_name} still in Offboard with lost offboard signal; '
            'wait for land-complete recovery'
        )
    if (
        not isfinite(status.last_telemetry_age_sec)
        or status.last_telemetry_age_sec > telemetry_fresh_timeout_s
    ):
        return ''
    if not status.pre_flight_checks_pass:
        return (
            f'arm rejected: {vehicle_name} pre-flight checks are not passing; '
            'inspect vehicle status health'
        )
    return ''


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundStationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
