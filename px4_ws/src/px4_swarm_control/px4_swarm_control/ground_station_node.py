"""Ground-station action surface for the first-version PX4 swarm."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from math import isfinite
from time import monotonic
from typing import Callable, Dict

from builtin_interfaces.msg import Time
from px4_swarm_control.bridge_config import FIRST_VERSION_VEHICLES
from px4_swarm_control.geometry import FormationGeometry, staging_setpoint
from px4_swarm_control.models import FormationMode as InternalFormationMode
from px4_swarm_control.models import MissionState
from px4_swarm_control.models import PositionYawSetpoint
from px4_swarm_control.models import VehicleLevelState
from px4_swarm_interfaces.action import (
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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


@dataclass(frozen=True)
class GroundStationConfig:
    """Static ground-station settings for first-version swarm supervision."""

    total_vehicles: int = 3
    active_formation: str = InternalFormationMode.VEE.value
    staging_lateral_spacing_m: float = 4.0
    staging_trail_spacing_m: float = 3.0
    staging_position_tolerance_m: float = 0.5
    telemetry_fresh_timeout_s: float = 1.0
    staging_yaw_rad: float = 0.0


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
        self.staging_targets: Dict[int, PositionYawSetpoint] = {}
        self._staging_complete_logged = False
        self._takeoff_started_s: float | None = None
        self._takeoff_timeout_s: float | None = None
        self._land_started_s: float | None = None
        self._land_timeout_s: float | None = None

    def start_takeoff(self, request: TakeoffSwarm.Goal):
        # 新任務先清掉舊 status，保護 action completion 不被上一輪 landed/staging 污染。
        self.vehicle_statuses = {}
        self._takeoff_started_s = self.now_s()
        self._takeoff_timeout_s = max(float(request.timeout_sec), 0.0)
        self._land_started_s = None
        self._land_timeout_s = None
        self._transition_to(MissionState.TAKING_OFF, 'takeoff action accepted')
        self._publish_staging_setpoints(request.altitude_m)
        mission = self._mission_command(
            MissionCommand.TAKEOFF,
            f'altitude_m={request.altitude_m:.2f} timeout_sec={request.timeout_sec:.2f}',
        )
        self.publishers.mission_command.publish(mission)
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

    def handle_move_leader(self, request: MoveLeader.Goal):
        self._transition_to(MissionState.FOLLOWING, 'leader goal accepted')
        msg = LeaderGoal()
        msg.stamp = self.now_stamp()
        msg.frame_id = 'world'
        msg.x = request.x
        msg.y = request.y
        msg.z = request.z
        msg.yaw = request.yaw
        self.publishers.leader_goal.publish(msg)

        feedback = MoveLeader.Feedback()
        feedback.current_state = self.mission_state.value
        feedback.remaining_distance_m = 0.0
        feedback.yaw_error_rad = 0.0

        result = MoveLeader.Result()
        result.success = True
        result.message = 'leader goal accepted and published'
        return ActionOutcome(result=result, feedback=feedback)

    def handle_change_formation(self, request: ChangeFormation.Goal):
        if request.formation_mode not in _supported_formation_modes():
            # 未知隊形會讓 follower slot 解讀不一致，因此在 ground station 邊界拒絕。
            self._transition_to(MissionState.ERROR, 'unsupported formation mode')
            result = ChangeFormation.Result()
            result.success = False
            result.message = f'unsupported formation mode: {request.formation_mode}'
            feedback = ChangeFormation.Feedback()
            feedback.current_state = self.mission_state.value
            feedback.active_formation = self.active_formation
            feedback.progress = 0.0
            return ActionOutcome(result=result, feedback=feedback)

        self._transition_to(MissionState.RECONFIGURING, 'formation change accepted')
        self.active_formation = request.formation_mode
        msg = FormationMode()
        msg.stamp = self.now_stamp()
        msg.mode = request.formation_mode
        self.publishers.formation_mode.publish(msg)

        feedback = ChangeFormation.Feedback()
        feedback.current_state = self.mission_state.value
        feedback.active_formation = self.active_formation
        feedback.progress = 0.0

        result = ChangeFormation.Result()
        result.success = True
        result.message = 'formation mode accepted and published'
        return ActionOutcome(result=result, feedback=feedback)

    def handle_pause(self, request: PauseSwarm.Goal):
        command = MissionCommand.PAUSE if request.pause else MissionCommand.RESUME
        next_state = MissionState.PAUSED if request.pause else MissionState.IDLE
        reason = request.reason or (
            'pause requested' if request.pause else 'resume requested'
        )
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
        self._land_started_s = self.now_s()
        self._land_timeout_s = max(float(request.timeout_sec), 0.0)
        self._takeoff_started_s = None
        self._takeoff_timeout_s = None
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
        self.vehicle_statuses[int(msg.vehicle_id)] = msg
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

    def _publish_staging_setpoints(self, altitude_m: float) -> None:
        # 起飛 staging 固定用 world frame，保護三機在離地前後維持水平安全間距。
        geometry = FormationGeometry(
            lateral_spacing_m=self.config.staging_lateral_spacing_m,
            trail_spacing_m=self.config.staging_trail_spacing_m,
        )
        leader = PositionYawSetpoint(
            x=0.0,
            y=0.0,
            z=-abs(float(altitude_m)),
            yaw=self.config.staging_yaw_rad,
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


class GroundStationNode(Node):
    """ROS 2 node exposing operator actions under `/swarm`."""

    def __init__(self) -> None:
        super().__init__('ground_station_node', namespace='/swarm')
        self.config = default_ground_station_config()
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
            )
            for vehicle in FIRST_VERSION_VEHICLES
        ]
        self.action_servers = (
            ActionServer(self, TakeoffSwarm, 'takeoff', self._execute_takeoff),
            ActionServer(self, MoveLeader, 'move_leader', self._execute_move_leader),
            ActionServer(
                self,
                ChangeFormation,
                'change_formation',
                self._execute_change_formation,
            ),
            ActionServer(self, PauseSwarm, 'pause', self._execute_pause),
            ActionServer(self, LandSwarm, 'land', self._execute_land),
        )

    async def _execute_takeoff(self, goal_handle):
        self.core.start_takeoff(goal_handle.request)
        while rclpy.ok():
            self.core.republish_staging_setpoints()
            goal_handle.publish_feedback(self.core.takeoff_feedback())
            result = self.core.takeoff_result()
            if result is not None:
                if result.success:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return result
            await asyncio.sleep(0.1)

        result = TakeoffSwarm.Result()
        result.success = False
        result.message = 'ROS shutdown before takeoff staging completed'
        goal_handle.abort()
        return result

    def _execute_move_leader(self, goal_handle):
        return self._finish_action(
            goal_handle,
            self.core.handle_move_leader(goal_handle.request),
        )

    def _execute_change_formation(self, goal_handle):
        return self._finish_action(
            goal_handle,
            self.core.handle_change_formation(goal_handle.request),
        )

    def _execute_pause(self, goal_handle):
        return self._finish_action(goal_handle, self.core.handle_pause(goal_handle.request))

    async def _execute_land(self, goal_handle):
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
            await asyncio.sleep(0.1)

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
        and (status.offboard_available or status.nav_state == 'offboard')
        and _position_close(status, target, tolerance_m)
    )


def _landed_ready(status: VehicleStatus, telemetry_fresh_timeout_s: float) -> bool:
    # landed completion 也要求新 telemetry，保護上一輪 landed cache 不完成本輪降落 action。
    return (
        status.vehicle_state == VehicleLevelState.LANDED.value
        and isfinite(status.last_telemetry_age_sec)
        and status.last_telemetry_age_sec <= telemetry_fresh_timeout_s
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundStationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
