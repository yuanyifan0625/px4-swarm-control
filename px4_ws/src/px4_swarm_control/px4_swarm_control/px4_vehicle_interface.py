"""PX4 topic boundary for one swarm vehicle."""

from __future__ import annotations

from math import nan
from typing import Any, Callable, Optional

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus as Px4VehicleStatus,
)
from px4_swarm_control.bridge_config import PX4_V118_OUT_TOPIC_SUFFIXES
from px4_swarm_control.bridge_config import PX4_V118_LAND_DETECTED_TOPIC_SUFFIX
from px4_swarm_control.models import (
    CommandStatus,
    PositionYawSetpoint,
    VehicleCommandResult,
    VehicleLevelState,
    VehicleState,
)
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class Px4VehicleInterface:
    """Encapsulate PX4 `px4_msgs` publishers/subscribers for one vehicle."""

    def __init__(
        self,
        node,
        vehicle_id: str,
        px4_namespace: str,
        px4_target_system: int = 1,
        telemetry_timeout_s: float = 0.5,
        now_us: Optional[Callable[[Any], int]] = None,
    ) -> None:
        self.node = node
        self.vehicle_id = vehicle_id
        self.px4_namespace = _normalize_namespace(px4_namespace)
        self.px4_target_system = _validate_target_system(px4_target_system)
        self.telemetry_timeout_s = telemetry_timeout_s
        self._now_us = now_us or _node_now_us
        self.vehicle_level_state = VehicleLevelState.IDLE
        self.last_command_result: Optional[VehicleCommandResult] = None
        self._last_setpoint: Optional[PositionYawSetpoint] = None
        self._latest_local_position: Optional[VehicleLocalPosition] = None
        self._latest_vehicle_status: Optional[Px4VehicleStatus] = None
        self._latest_land_detected: Optional[VehicleLandDetected] = None

        qos_profile = _px4_qos_profile()
        self.offboard_control_mode_publisher = node.create_publisher(
            OffboardControlMode,
            self._topic('/fmu/in/offboard_control_mode'),
            qos_profile,
        )
        self.trajectory_setpoint_publisher = node.create_publisher(
            TrajectorySetpoint,
            self._topic('/fmu/in/trajectory_setpoint'),
            qos_profile,
        )
        self.vehicle_command_publisher = node.create_publisher(
            VehicleCommand,
            self._topic('/fmu/in/vehicle_command'),
            qos_profile,
        )
        self.vehicle_local_position_subscription = node.create_subscription(
            VehicleLocalPosition,
            # PX4 v1.18 會在部分 out topic 加版本尾綴，集中在邊界避免 controller 訂錯 topic。
            self._topic(PX4_V118_OUT_TOPIC_SUFFIXES[0]),
            self.handle_vehicle_local_position,
            qos_profile,
        )
        self.vehicle_status_subscription = node.create_subscription(
            Px4VehicleStatus,
            # PX4 v1.18 會在部分 out topic 加版本尾綴，集中在邊界避免 controller 訂錯 topic。
            self._topic(PX4_V118_OUT_TOPIC_SUFFIXES[1]),
            self.handle_vehicle_status,
            qos_profile,
        )
        self.vehicle_command_ack_subscription = node.create_subscription(
            VehicleCommandAck,
            # PX4 v1.18 會在部分 out topic 加版本尾綴，集中在邊界避免 controller 訂錯 topic。
            self._topic(PX4_V118_OUT_TOPIC_SUFFIXES[2]),
            self.handle_vehicle_command_ack,
            qos_profile,
        )
        self.vehicle_land_detected_subscription = node.create_subscription(
            VehicleLandDetected,
            # landed telemetry 由 PX4 commander 判定，保護 ROS 2 不用猜測接地狀態。
            self._topic(PX4_V118_LAND_DETECTED_TOPIC_SUFFIX),
            self.handle_vehicle_land_detected,
            qos_profile,
        )

    def publish_offboard_heartbeat(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp = self._timestamp_us()
        # Offboard heartbeat 只開 position，保護 PX4 仍負責速度/姿態/馬達等底層控制。
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_yaw_setpoint(self, setpoint: PositionYawSetpoint) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = self._timestamp_us()
        # 只填 position/yaw，其餘 setpoint 設 NaN，避免 PX4 同時追蹤多組控制目標。
        msg.position = [setpoint.x, setpoint.y, setpoint.z]
        msg.velocity = [nan, nan, nan]
        msg.acceleration = [nan, nan, nan]
        msg.jerk = [nan, nan, nan]
        msg.yaw = setpoint.yaw
        msg.yawspeed = nan
        self._last_setpoint = setpoint
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_safe_hover_setpoint(self) -> bool:
        if self._last_setpoint is None:
            return False
        self.publish_position_yaw_setpoint(self._last_setpoint)
        return True

    def arm(self) -> None:
        # Arm/disarm 只經 PX4 command topic，保護 cooperative logic 不碰 PX4 內部控制。
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )

    def disarm(self) -> None:
        # Arm/disarm 只經 PX4 command topic，保護 cooperative logic 不碰 PX4 內部控制。
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
        )

    def takeoff(self, altitude_m: float, yaw: float = nan) -> None:
        # Takeoff command 保留為 PX4 高階任務命令，避免 ROS 2 直接實作起飛控制器。
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param4=yaw,
            param7=altitude_m,
        )

    def land(self) -> None:
        # Land command 交給 PX4 landing mode，保護降落流程使用 PX4 既有安全邏輯。
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def set_offboard_mode(self) -> None:
        # Offboard mode 透過標準 PX4 mode command 切換，避免 vehicle node 繞過 commander。
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )

    def handle_vehicle_local_position(self, msg: VehicleLocalPosition) -> None:
        self._latest_local_position = msg

    def handle_vehicle_status(self, msg: Px4VehicleStatus) -> None:
        self._latest_vehicle_status = msg

    def handle_vehicle_command_ack(self, msg: VehicleCommandAck) -> None:
        self.last_command_result = VehicleCommandResult(
            status=_command_ack_status(msg.result),
            message=f'px4 command ack result={msg.result}',
        )

    def handle_vehicle_land_detected(self, msg: VehicleLandDetected) -> None:
        self._latest_land_detected = msg

    def vehicle_state(self) -> Optional[VehicleState]:
        if self._latest_local_position is None or self._latest_vehicle_status is None:
            return None

        local_position = self._latest_local_position
        vehicle_status = self._latest_vehicle_status
        landed = (
            self._latest_land_detected is not None
            and bool(self._latest_land_detected.landed)
        )
        vehicle_level_state = (
            VehicleLevelState.LANDED if landed else self.vehicle_level_state
        )
        # PX4 telemetry 在邊界轉成 internal model，保護 controller 不依賴 raw px4_msgs 欄位。
        return VehicleState(
            vehicle_id=self.vehicle_id,
            position=(local_position.x, local_position.y, local_position.z),
            yaw=local_position.heading,
            velocity=(local_position.vx, local_position.vy, local_position.vz),
            armed=vehicle_status.arming_state == Px4VehicleStatus.ARMING_STATE_ARMED,
            navigation_state=_navigation_state_name(vehicle_status.nav_state),
            offboard_available=vehicle_status.accepts_offboard_setpoints,
            telemetry_age_s=self._telemetry_age_s(),
            vehicle_level_state=vehicle_level_state,
            landed=landed,
        )

    def is_telemetry_stale(self) -> bool:
        if self._latest_local_position is None:
            return True
        return self._telemetry_age_s() > self.telemetry_timeout_s

    def _publish_vehicle_command(self, command: int, **params: float) -> None:
        msg = VehicleCommand()
        msg.timestamp = self._timestamp_us()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        msg.param7 = params.get('param7', 0.0)
        # 每台 PX4 instance 有自己的 MAV_SYS_ID，避免 takeoff/land command 打到錯的飛機。
        msg.target_system = self.px4_target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)

    def _telemetry_age_s(self) -> float:
        if self._latest_local_position is None:
            return float('inf')
        return (self._timestamp_us() - self._latest_local_position.timestamp) / 1_000_000.0

    def _timestamp_us(self) -> int:
        return int(self._now_us(self.node))

    def _topic(self, suffix: str) -> str:
        return f'{self.px4_namespace}{suffix}'


def _normalize_namespace(namespace: str) -> str:
    stripped = namespace.strip('/')
    if not stripped:
        return ''
    return f'/{stripped}'


def _validate_target_system(target_system: int) -> int:
    value = int(target_system)
    if value < 0 or value > 255:
        raise ValueError('px4_target_system must fit uint8, or 0 for broadcast')
    return value


def _node_now_us(node: Any) -> int:
    if hasattr(node, 'px4_now_us'):
        return int(node.px4_now_us())
    return int(node.get_clock().now().nanoseconds / 1000)


def _px4_qos_profile() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _navigation_state_name(nav_state: int) -> str:
    if nav_state == Px4VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        return 'offboard'
    if nav_state == Px4VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
        return 'auto_takeoff'
    if nav_state == Px4VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
        return 'auto_land'
    return str(nav_state)


def _command_ack_status(result: int) -> CommandStatus:
    if result in (
        VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED,
        VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS,
    ):
        return CommandStatus.ACCEPTED
    if result == VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED:
        return CommandStatus.FAILED
    return CommandStatus.REJECTED
