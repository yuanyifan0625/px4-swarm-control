"""Ground-station action surface for the first-version PX4 swarm."""

from __future__ import annotations

from dataclasses import dataclass
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
    ) -> None:
        self.config = config
        self.publishers = publishers
        self.logger = logger
        self.now_stamp = now_stamp
        self.mission_state = MissionState.IDLE
        self.active_formation = config.active_formation
        self.vehicle_statuses: Dict[int, VehicleStatus] = {}
        self.staging_targets: Dict[int, PositionYawSetpoint] = {}
        self._staging_complete_logged = False

    def handle_takeoff(self, request: TakeoffSwarm.Goal):
        self._transition_to(MissionState.TAKING_OFF, 'takeoff action accepted')
        self._publish_staging_setpoints(request.altitude_m)
        mission = self._mission_command(
            MissionCommand.TAKEOFF,
            f'altitude_m={request.altitude_m:.2f} timeout_sec={request.timeout_sec:.2f}',
        )
        self.publishers.mission_command.publish(mission)

        feedback = TakeoffSwarm.Feedback()
        feedback.current_state = self.mission_state.value
        feedback.progress = 0.0
        feedback.vehicles_staged = 0
        feedback.total_vehicles = self.config.total_vehicles

        result = TakeoffSwarm.Result()
        result.success = True
        result.message = 'takeoff command accepted and published'
        return ActionOutcome(result=result, feedback=feedback)

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

    def handle_land(self, request: LandSwarm.Goal):
        self._transition_to(MissionState.LANDING, 'land-all action accepted')
        self.publishers.mission_command.publish(
            self._mission_command(
                MissionCommand.LAND,
                f'timeout_sec={request.timeout_sec:.2f}',
            ),
        )
        feedback = LandSwarm.Feedback()
        feedback.current_state = self.mission_state.value
        feedback.vehicles_landed = 0
        feedback.total_vehicles = self.config.total_vehicles

        result = LandSwarm.Result()
        result.success = True
        result.message = 'land-all command accepted and published'
        return ActionOutcome(result=result, feedback=feedback)

    def handle_vehicle_status(self, msg: VehicleStatus) -> None:
        self.vehicle_statuses[int(msg.vehicle_id)] = msg
        if msg.vehicle_state == VehicleLevelState.FAILSAFE.value:
            # 任一 vehicle 進入 failsafe 代表任務層要停止正常流程，避免繼續發布移動命令。
            self._transition_to(MissionState.FAILSAFE, 'vehicle reported failsafe')
            return
        if self._all_known_vehicles_in_state(VehicleLevelState.LANDED.value):
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
            msg = VehicleSetpoint()
            msg.stamp = self.now_stamp()
            msg.vehicle_id = vehicle.px4_instance
            msg.frame_id = 'world'
            msg.x = target.x
            msg.y = target.y
            msg.z = target.z
            msg.yaw = target.yaw
            self.publishers.vehicle_setpoints[vehicle.px4_instance].publish(msg)

    def _all_vehicles_staged(self) -> bool:
        if len(self.staging_targets) < self.config.total_vehicles:
            return False
        if len(self.vehicle_statuses) < self.config.total_vehicles:
            return False
        return all(
            _position_close(
                self.vehicle_statuses[vehicle_id],
                target,
                self.config.staging_position_tolerance_m,
            )
            for vehicle_id, target in self.staging_targets.items()
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

    def _execute_takeoff(self, goal_handle):
        return self._finish_action(
            goal_handle,
            self.core.handle_takeoff(goal_handle.request),
        )

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

    def _execute_land(self, goal_handle):
        return self._finish_action(goal_handle, self.core.handle_land(goal_handle.request))

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
