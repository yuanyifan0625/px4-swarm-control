"""Parameterized ROS 2 vehicle node for one PX4 swarm vehicle."""

from __future__ import annotations

from dataclasses import dataclass
from math import nan
from re import search
from typing import Any, Dict, Tuple

from px4_swarm_control.models import (
    PositionYawSetpoint,
    Slot,
    VehicleLevelState,
    VehicleRole,
)
from px4_swarm_control.px4_vehicle_interface import Px4VehicleInterface
from px4_swarm_interfaces.msg import LeaderGoal
from px4_swarm_interfaces.msg import VehicleStatus as SwarmVehicleStatus
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


@dataclass(frozen=True)
class VehicleNodeConfig:
    """Configuration that makes one executable represent one vehicle role."""

    role: VehicleRole
    vehicle_id: str
    px4_namespace: str
    slot: Slot
    hold_setpoint: PositionYawSetpoint
    control_loop_hz: float = 20.0
    status_loop_hz: float = 5.0
    telemetry_timeout_s: float = 0.5


class VehicleNodeCore:
    """Testable vehicle behavior that does not depend on the ROS node runtime."""

    def __init__(self, config, px4_interface, status_publisher, logger) -> None:
        self.config = config
        self.px4_interface = px4_interface
        self.status_publisher = status_publisher
        self.logger = logger
        self.vehicle_level_state = VehicleLevelState.IDLE
        self.active_setpoint = config.hold_setpoint
        self._warned_ignored_leader_goal = False
        self._warned_mismatched_vehicle_state = False
        self._sync_interface_state()

    def handle_leader_goal(self, msg: LeaderGoal) -> None:
        if self.config.role is not VehicleRole.LEADER:
            if not self._warned_ignored_leader_goal:
                self.logger.warning(
                    f'{self.config.role.value} {self.config.vehicle_id} ignored leader goal',
                )
                self._warned_ignored_leader_goal = True
            return

        self.active_setpoint = PositionYawSetpoint(msg.x, msg.y, msg.z, msg.yaw)
        self.transition_to(VehicleLevelState.HOLDING, 'leader goal accepted')

    def control_tick(self) -> None:
        self.px4_interface.publish_offboard_heartbeat()
        if self.px4_interface.is_telemetry_stale():
            # telemetry 過期時只重送最後安全 setpoint，保護 vehicle 不追 stale command。
            self.transition_to(VehicleLevelState.HOLDING, 'telemetry stale')
            self.px4_interface.publish_safe_hover_setpoint()
            return

        # 每個 tick 都補 heartbeat/setpoint，保護 PX4 Offboard mode 不因間隔過久退出。
        self.px4_interface.publish_position_yaw_setpoint(self.active_setpoint)
        self.transition_to(VehicleLevelState.HOLDING, 'holding active setpoint')

    def publish_status(self) -> None:
        state = self.px4_interface.vehicle_state()
        msg = SwarmVehicleStatus()
        msg.vehicle_id = vehicle_id_to_uint8(self.config.vehicle_id)
        msg.role = self.config.role.value
        msg.px4_namespace = self.config.px4_namespace
        msg.slot = self.config.slot.value

        if state is None:
            msg.x = nan
            msg.y = nan
            msg.z = nan
            msg.yaw = nan
            msg.vx = nan
            msg.vy = nan
            msg.vz = nan
            msg.armed = False
            msg.nav_state = 'unknown'
            msg.offboard_available = False
            msg.last_telemetry_age_sec = float('inf')
            msg.vehicle_state = self.vehicle_level_state.value
        else:
            # 丟棄 vehicle_id 不一致的 telemetry，保護 status topic 不混入其他飛機資料。
            if state.vehicle_id != self.config.vehicle_id:
                if not self._warned_mismatched_vehicle_state:
                    self.logger.warning(
                        f'{self.config.vehicle_id} ignored telemetry for '
                        f'{state.vehicle_id} from {self.config.px4_namespace}',
                    )
                    self._warned_mismatched_vehicle_state = True
                return

            msg.x, msg.y, msg.z = state.position
            msg.yaw = state.yaw
            msg.vx, msg.vy, msg.vz = state.velocity
            msg.armed = state.armed
            msg.nav_state = state.navigation_state
            msg.offboard_available = state.offboard_available
            msg.last_telemetry_age_sec = state.telemetry_age_s
            msg.vehicle_state = state.vehicle_level_state.value

        self.status_publisher.publish(msg)

    def transition_to(self, next_state: VehicleLevelState, reason: str) -> None:
        if self.vehicle_level_state is next_state:
            return
        previous = self.vehicle_level_state
        self.vehicle_level_state = next_state
        self._sync_interface_state()
        self.logger.info(
            f'{self.config.vehicle_id} state {previous.value} -> {next_state.value}: {reason}',
        )

    def _sync_interface_state(self) -> None:
        if hasattr(self.px4_interface, 'vehicle_level_state'):
            self.px4_interface.vehicle_level_state = self.vehicle_level_state


class VehicleNode(Node):
    """ROS 2 executable wrapper around `VehicleNodeCore`."""

    def __init__(self, px4_interface_factory=Px4VehicleInterface) -> None:
        super().__init__('vehicle_node')
        self._declare_parameters()
        self.config = parse_vehicle_node_config(self._parameter_values())
        self.px4_interface = px4_interface_factory(
            node=self,
            vehicle_id=self.config.vehicle_id,
            px4_namespace=self.config.px4_namespace,
            telemetry_timeout_s=self.config.telemetry_timeout_s,
        )
        self.status_publisher = self.create_publisher(
            SwarmVehicleStatus,
            f'{self.config.px4_namespace}/status',
            10,
        )
        self.core = VehicleNodeCore(
            self.config,
            self.px4_interface,
            self.status_publisher,
            self.get_logger(),
        )
        self.leader_goal_subscription = self.create_subscription(
            LeaderGoal,
            '/swarm/leader_goal',
            self.core.handle_leader_goal,
            10,
        )
        self.control_timer = self.create_timer(
            1.0 / self.config.control_loop_hz,
            self.core.control_tick,
        )
        self.status_timer = self.create_timer(
            1.0 / self.config.status_loop_hz,
            self.core.publish_status,
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('role', 'leader')
        self.declare_parameter('vehicle_id', 'vehicle_1')
        self.declare_parameter('px4_namespace', '/vehicle_1')
        self.declare_parameter('slot', 'leader')
        self.declare_parameter('control_loop_hz', 20.0)
        self.declare_parameter('status_loop_hz', 5.0)
        self.declare_parameter('telemetry_timeout_s', 0.5)
        self.declare_parameter('hold_x', 0.0)
        self.declare_parameter('hold_y', 0.0)
        self.declare_parameter('hold_z', -2.0)
        self.declare_parameter('hold_yaw', 0.0)

    def _parameter_values(self) -> Dict[str, Any]:
        names = (
            'role',
            'vehicle_id',
            'px4_namespace',
            'slot',
            'control_loop_hz',
            'status_loop_hz',
            'telemetry_timeout_s',
            'hold_x',
            'hold_y',
            'hold_z',
            'hold_yaw',
        )
        return {name: self.get_parameter(name).value for name in names}


def parse_vehicle_node_config(values: Dict[str, Any]) -> VehicleNodeConfig:
    role = _enum_value(VehicleRole, values.get('role', 'leader'), 'role')
    slot = _enum_value(Slot, values.get('slot', 'leader'), 'slot')
    control_loop_hz = float(values.get('control_loop_hz', 20.0))
    status_loop_hz = float(values.get('status_loop_hz', 5.0))
    telemetry_timeout_s = float(values.get('telemetry_timeout_s', 0.5))

    if control_loop_hz <= 0.0:
        raise ValueError('control_loop_hz must be positive')
    if status_loop_hz <= 0.0:
        raise ValueError('status_loop_hz must be positive')
    if telemetry_timeout_s <= 0.0:
        raise ValueError('telemetry_timeout_s must be positive')

    config = VehicleNodeConfig(
        role=role,
        vehicle_id=str(values.get('vehicle_id', 'vehicle_1')),
        px4_namespace=_normalize_namespace(str(values.get('px4_namespace', '/vehicle_1'))),
        slot=slot,
        hold_setpoint=PositionYawSetpoint(
            float(values.get('hold_x', 0.0)),
            float(values.get('hold_y', 0.0)),
            float(values.get('hold_z', -2.0)),
            float(values.get('hold_yaw', 0.0)),
        ),
        control_loop_hz=control_loop_hz,
        status_loop_hz=status_loop_hz,
        telemetry_timeout_s=telemetry_timeout_s,
    )
    _validate_first_version_mapping(config)
    return config


def default_vehicle_node_configs() -> Tuple[
    VehicleNodeConfig,
    VehicleNodeConfig,
    VehicleNodeConfig,
]:
    """Return the first-version three-node layout used for manual namespace validation."""
    return (
        parse_vehicle_node_config(
            {
                'role': 'leader',
                'vehicle_id': 'vehicle_1',
                'px4_namespace': '/vehicle_1',
                'slot': 'leader',
            },
        ),
        parse_vehicle_node_config(
            {
                'role': 'follower',
                'vehicle_id': 'vehicle_2',
                'px4_namespace': '/vehicle_2',
                'slot': 'follower_left',
            },
        ),
        parse_vehicle_node_config(
            {
                'role': 'follower',
                'vehicle_id': 'vehicle_3',
                'px4_namespace': '/vehicle_3',
                'slot': 'follower_right',
            },
        ),
    )


def vehicle_id_to_uint8(vehicle_id: str) -> int:
    match = search(r'(\d+)$', vehicle_id)
    if match is None:
        raise ValueError(f'vehicle_id must end with a number: {vehicle_id}')
    value = int(match.group(1))
    if value < 0 or value > 255:
        raise ValueError(f'vehicle_id must fit uint8: {vehicle_id}')
    return value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _enum_value(enum_type, value: Any, parameter_name: str):
    try:
        return enum_type(str(value))
    except ValueError as exc:
        allowed = ', '.join(item.value for item in enum_type)
        raise ValueError(f'{parameter_name} must be one of: {allowed}') from exc


def _normalize_namespace(namespace: str) -> str:
    stripped = namespace.strip('/')
    if not stripped:
        return ''
    return f'/{stripped}'


def _validate_first_version_mapping(config: VehicleNodeConfig) -> None:
    expected = {
        'vehicle_1': ('/vehicle_1', VehicleRole.LEADER, Slot.LEADER),
        'vehicle_2': ('/vehicle_2', VehicleRole.FOLLOWER, Slot.FOLLOWER_LEFT),
        'vehicle_3': ('/vehicle_3', VehicleRole.FOLLOWER, Slot.FOLLOWER_RIGHT),
    }
    if config.vehicle_id not in expected:
        raise ValueError(f'vehicle_id must be one of: {", ".join(expected)}')

    expected_namespace, expected_role, expected_slot = expected[config.vehicle_id]
    if (
        config.px4_namespace != expected_namespace
        or config.role is not expected_role
        or config.slot is not expected_slot
    ):
        # 固定 vehicle_id/namespace/slot 對應，保護三機 telemetry 不被錯接到別台狀態 topic。
        raise ValueError(
            f'{config.vehicle_id} must map to '
            f'{expected_namespace}, {expected_role.value}, {expected_slot.value}',
        )


if __name__ == '__main__':
    main()
