"""Parameterized ROS 2 vehicle node for one PX4 swarm vehicle."""

from __future__ import annotations

from dataclasses import dataclass
from math import nan
from re import search
from time import monotonic
from typing import Any, Callable, Dict, Optional, Tuple

from px4_swarm_control.bridge_config import (
    FIRST_VERSION_BY_VEHICLE_ID,
    FIRST_VERSION_LEADER,
    FIRST_VERSION_VEHICLES,
)
from px4_swarm_control.follower_controller import (
    derive_follower_setpoint,
    leader_status_is_fresh,
    leader_status_setpoint,
)
from px4_swarm_control.geometry import FormationGeometry
from px4_swarm_control.models import (
    FormationMode as InternalFormationMode,
    PositionYawSetpoint,
    Slot,
    VehicleLevelState,
    VehicleRole,
)
from px4_swarm_control.px4_vehicle_interface import Px4VehicleInterface
from px4_swarm_interfaces.msg import FormationMode as SwarmFormationMode
from px4_swarm_interfaces.msg import LeaderGoal
from px4_swarm_interfaces.msg import MissionCommand
from px4_swarm_interfaces.msg import VehicleSetpoint
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
    px4_target_system: int
    slot: Slot
    hold_setpoint: PositionYawSetpoint
    control_loop_hz: float = 20.0
    status_loop_hz: float = 5.0
    telemetry_timeout_s: float = 0.5
    takeoff_altitude_tolerance_m: float = 1.0
    offboard_warmup_s: float = 1.0
    offboard_mode_retry_s: float = 1.0
    following_lateral_spacing_m: float = 4.0
    following_trail_spacing_m: float = 3.0


class VehicleNodeCore:
    """Testable vehicle behavior that does not depend on the ROS node runtime."""

    def __init__(
        self,
        config,
        px4_interface,
        status_publisher,
        logger,
        now_s: Optional[Callable[[], float]] = None,
    ) -> None:
        self.config = config
        self.px4_interface = px4_interface
        self.status_publisher = status_publisher
        self.logger = logger
        self._now_s = now_s or monotonic
        self.vehicle_level_state = VehicleLevelState.IDLE
        self.active_setpoint = config.hold_setpoint
        self.active_formation = InternalFormationMode.VEE
        self.leader_status: Optional[SwarmVehicleStatus] = None
        self._leader_goal_active = False
        self._warned_ignored_leader_goal = False
        self._warned_mismatched_vehicle_state = False
        self._takeoff_to_staging_active = False
        self._takeoff_command_sent_for_staging = False
        self._pending_takeoff_until_staging = False
        self._staging_setpoint_received = False
        self._last_takeoff_command_request_s: Optional[float] = None
        self._offboard_warmup_started_s: Optional[float] = None
        self._last_offboard_mode_request_s: Optional[float] = None
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
        # 只有 leader 會啟用此旗標，保護 follower 不把 operator goal 當成自己的目標。
        self._leader_goal_active = True
        self.transition_to(VehicleLevelState.FOLLOWING, 'leader goal accepted')

    def handle_leader_status(self, msg: SwarmVehicleStatus) -> None:
        if self.config.role is not VehicleRole.FOLLOWER:
            return
        if int(msg.vehicle_id) != 1:
            return
        self.leader_status = msg

    def handle_formation_mode(self, msg: SwarmFormationMode) -> None:
        if self.config.role is not VehicleRole.FOLLOWER:
            return
        try:
            self.active_formation = InternalFormationMode(msg.mode)
        except ValueError:
            self.logger.warning(f'ignored unsupported formation mode: {msg.mode}')

    def handle_staging_setpoint(self, msg: VehicleSetpoint) -> None:
        if int(msg.vehicle_id) != vehicle_id_to_uint8(self.config.vehicle_id):
            return
        # 只接受自己的 staging 目標，保護多機共用 topic 流程下不追錯飛機位置。
        self.active_setpoint = PositionYawSetpoint(msg.x, msg.y, msg.z, msg.yaw)
        self._leader_goal_active = False
        self._staging_setpoint_received = True
        if not self._takeoff_to_staging_active:
            self.transition_to(VehicleLevelState.STAGING, 'staging setpoint accepted')
        if self._pending_takeoff_until_staging:
            self._pending_takeoff_until_staging = False
            self._start_takeoff_without_qgc()

    def handle_mission_command(self, msg: MissionCommand) -> None:
        if msg.command == MissionCommand.TAKEOFF:
            self._start_takeoff_without_qgc()
            return
        if msg.command == MissionCommand.LAND:
            self._leader_goal_active = False
            self._takeoff_to_staging_active = False
            self._takeoff_command_sent_for_staging = False
            self._pending_takeoff_until_staging = False
            # 降落代表任務輪次結束，清掉 staging latch 以保護下一輪不吃上一輪目標。
            self._staging_setpoint_received = False
            self._last_takeoff_command_request_s = None
            self.px4_interface.land()
            self.transition_to(VehicleLevelState.LANDING, 'land command accepted')
            return
        if msg.command == MissionCommand.PAUSE:
            self._leader_goal_active = False
            self.transition_to(VehicleLevelState.PAUSED, 'pause command accepted')
            self.px4_interface.publish_safe_hover_setpoint()
            return
        if msg.command == MissionCommand.RESUME:
            self.transition_to(VehicleLevelState.HOLDING, 'resume command accepted')

    def _start_takeoff_without_qgc(self) -> None:
        if self._takeoff_to_staging_active:
            # TAKEOFF 會在 action 等待期間重送，這裡保護 PX4 不被重複 arm/takeoff 指令洗版。
            return
        if self._takeoff_command_sent_for_staging:
            # 同一輪任務已送過 PX4 takeoff 後，忽略 retry 以保護起飛初期 landed telemetry 抖動不重啟流程。
            return
        if not self._staging_setpoint_received:
            # TAKEOFF 可能比 staging topic 先到，等待目標可避免用 hold altitude 起飛到錯高度。
            self._pending_takeoff_until_staging = True
            self.transition_to(VehicleLevelState.ARMING, 'waiting for staging setpoint')
            self.logger.info(
                f'{self.config.vehicle_id} received takeoff before staging setpoint; '
                'waiting for staging target',
            )
            return
        self._takeoff_to_staging_active = True
        self._offboard_warmup_started_s = None
        self._last_offboard_mode_request_s = None
        self._send_takeoff_command_to_px4()
        self.transition_to(VehicleLevelState.TAKING_OFF, 'takeoff command accepted')
        self.logger.info(
            '不依賴 QGC：先用 PX4 NAV_TAKEOFF 到安全高度，'
            '再 warm up Offboard 後切換 staging control。',
        )

    def control_tick(self) -> None:
        state = self.px4_interface.vehicle_state()
        if self.vehicle_level_state is VehicleLevelState.LANDING:
            # 降落期間不再送空中 staging setpoint，保護 PX4 landing mode 不被 ROS 2 搶控制權。
            if self._state_is_landed(state):
                self.transition_to(VehicleLevelState.LANDED, 'PX4 landed telemetry')
            return
        if self.vehicle_level_state is VehicleLevelState.PAUSED:
            # Pause 每個 tick 都只維持 hover，保護舊 leader/formation 目標不被自動續跑。
            self.px4_interface.publish_offboard_heartbeat()
            self.px4_interface.publish_safe_hover_setpoint()
            return
        if self._takeoff_to_staging_active:
            self._control_takeoff_to_staging(state)
            return
        if self._state_is_landed(state):
            self.transition_to(VehicleLevelState.LANDED, 'PX4 landed telemetry')
            return

        self.px4_interface.publish_offboard_heartbeat()
        if self.px4_interface.is_telemetry_stale():
            # telemetry 過期時只重送最後安全 setpoint，保護 vehicle 不追 stale command。
            self.transition_to(VehicleLevelState.FAILSAFE, 'vehicle telemetry stale')
            self.px4_interface.publish_safe_hover_setpoint()
            return
        if self.config.role is VehicleRole.FOLLOWER:
            if not self._update_follower_setpoint():
                # leader 資訊過期時不追舊 setpoint，保護 follower 不被 stale leader 狀態拖走。
                next_state = (
                    VehicleLevelState.FAILSAFE
                    if self._leader_status_is_stale()
                    else VehicleLevelState.HOLDING
                )
                self.transition_to(next_state, 'leader status stale')
                self.px4_interface.publish_safe_hover_setpoint()
                return
            follower_setpoint_active = True
        else:
            follower_setpoint_active = False

        # 每個 tick 都補 heartbeat/setpoint，保護 PX4 Offboard mode 不因間隔過久退出。
        self.px4_interface.publish_position_yaw_setpoint(self.active_setpoint)
        next_state = (
            VehicleLevelState.STAGING
            if self.vehicle_level_state is VehicleLevelState.STAGING
            else VehicleLevelState.FOLLOWING
            if self._leader_goal_active or follower_setpoint_active
            else VehicleLevelState.HOLDING
        )
        self.transition_to(next_state, 'active setpoint published')

    def _update_follower_setpoint(self) -> bool:
        if not leader_status_is_fresh(
            self.leader_status,
            self.config.telemetry_timeout_s,
        ):
            return False
        geometry = FormationGeometry(
            lateral_spacing_m=self.config.following_lateral_spacing_m,
            trail_spacing_m=self.config.following_trail_spacing_m,
        )
        self.active_setpoint = derive_follower_setpoint(
            leader_status_setpoint(self.leader_status),
            self.active_formation,
            self.config.slot,
            geometry,
        )
        return True

    def _leader_status_is_stale(self) -> bool:
        if self.leader_status is None:
            return False
        age = self.leader_status.last_telemetry_age_sec
        return age != age or age > self.config.telemetry_timeout_s

    def _control_takeoff_to_staging(self, state) -> None:
        if state is None or self.px4_interface.is_telemetry_stale():
            return
        if self._offboard_accepted(state):
            self._takeoff_to_staging_active = False
            self.px4_interface.publish_offboard_heartbeat()
            self.px4_interface.publish_position_yaw_setpoint(self.active_setpoint)
            self.transition_to(VehicleLevelState.STAGING, 'PX4 accepted Offboard')
            return
        if not self._takeoff_altitude_reached(state):
            self._retry_takeoff_if_still_grounded(state)
            return

        # 高度達標後才連續送 heartbeat/setpoint，保護 PX4 Offboard 切換有足夠 warmup。
        self.px4_interface.publish_offboard_heartbeat()
        self.px4_interface.publish_position_yaw_setpoint(self.active_setpoint)
        now_s = self._now_s()
        if self._offboard_warmup_started_s is None:
            self._offboard_warmup_started_s = now_s
            return
        if now_s - self._offboard_warmup_started_s < self.config.offboard_warmup_s:
            return
        if (
            self._last_offboard_mode_request_s is None
            or now_s - self._last_offboard_mode_request_s
            >= self.config.offboard_mode_retry_s
        ):
            self.px4_interface.set_offboard_mode()
            self._last_offboard_mode_request_s = now_s

    def _send_takeoff_command_to_px4(self) -> None:
        # 先讓 PX4 自己管理起飛，保護 Offboard 不會在尚未離地時被太早拒絕。
        self.px4_interface.arm()
        self.px4_interface.takeoff(
            altitude_m=abs(self.active_setpoint.z),
            yaw=self.active_setpoint.yaw,
        )
        self._takeoff_command_sent_for_staging = True
        self._last_takeoff_command_request_s = self._now_s()

    def _retry_takeoff_if_still_grounded(self, state) -> None:
        if state.armed and not getattr(state, 'landed', False):
            return
        now_s = self._now_s()
        if (
            self._last_takeoff_command_request_s is not None
            and now_s - self._last_takeoff_command_request_s
            < self.config.offboard_mode_retry_s
        ):
            return
        # PX4 在落地後可能短暫忽略第一個 takeoff，低頻重送保護一次 action 能完成。
        self._send_takeoff_command_to_px4()

    def _takeoff_altitude_reached(self, state) -> bool:
        return state.position[2] <= (
            self.active_setpoint.z + self.config.takeoff_altitude_tolerance_m
        )

    def _offboard_accepted(self, state) -> bool:
        if state.navigation_state != 'offboard':
            return False
        if getattr(state, 'landed', False) and state.position[2] > -0.5:
            # PX4 可能短暫回報 Offboard 但仍未離地，這裡保護 staging 不被 landed telemetry 打回。
            return False
        # Offboard 要等到起飛高度達標才承認，保護 staging 水平移動不在地面附近開始。
        return self._takeoff_altitude_reached(state)

    def _state_is_landed(self, state) -> bool:
        if state is None or not getattr(state, 'landed', False):
            return False
        if self._takeoff_to_staging_active:
            return False
        if state.armed and state.position[2] < -0.5:
            return False
        return True

    def _status_vehicle_level_state(self, state) -> VehicleLevelState:
        if self.vehicle_level_state in (
            VehicleLevelState.PAUSED,
            VehicleLevelState.FAILSAFE,
        ):
            return self.vehicle_level_state
        if self._state_is_landed(state):
            return VehicleLevelState.LANDED
        if state.vehicle_level_state is VehicleLevelState.LANDED:
            # PX4 已回報非 landed 時清掉舊狀態，保護下一次 takeoff 不被上次 landing 殘留卡住。
            self.transition_to(VehicleLevelState.HOLDING, 'PX4 airborne telemetry')
            return VehicleLevelState.HOLDING
        return state.vehicle_level_state

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
            status_vehicle_state = self._status_vehicle_level_state(state)
            if status_vehicle_state is VehicleLevelState.LANDED:
                self.transition_to(VehicleLevelState.LANDED, 'PX4 landed telemetry')

            msg.x, msg.y, msg.z = state.position
            msg.yaw = state.yaw
            msg.vx, msg.vy, msg.vz = state.velocity
            msg.armed = state.armed
            msg.nav_state = state.navigation_state
            msg.offboard_available = state.offboard_available
            msg.last_telemetry_age_sec = state.telemetry_age_s
            msg.vehicle_state = status_vehicle_state.value

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
            px4_target_system=self.config.px4_target_system,
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
        self.mission_command_subscription = self.create_subscription(
            MissionCommand,
            '/swarm/mission_command',
            self.core.handle_mission_command,
            10,
        )
        self.staging_setpoint_subscription = self.create_subscription(
            VehicleSetpoint,
            f'{self.config.px4_namespace}/staging_setpoint',
            self.core.handle_staging_setpoint,
            10,
        )
        self.leader_status_subscription = None
        self.formation_mode_subscription = None
        if self.config.role is VehicleRole.FOLLOWER:
            self.leader_status_subscription = self.create_subscription(
                SwarmVehicleStatus,
                f'{FIRST_VERSION_LEADER.namespace}/status',
                self.core.handle_leader_status,
                10,
            )
            self.formation_mode_subscription = self.create_subscription(
                SwarmFormationMode,
                '/swarm/formation_mode',
                self.core.handle_formation_mode,
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
        self.declare_parameter('vehicle_id', FIRST_VERSION_LEADER.vehicle_id)
        self.declare_parameter('px4_namespace', FIRST_VERSION_LEADER.namespace)
        self.declare_parameter('px4_target_system', 2)
        self.declare_parameter('slot', 'leader')
        self.declare_parameter('control_loop_hz', 20.0)
        self.declare_parameter('status_loop_hz', 5.0)
        self.declare_parameter('telemetry_timeout_s', 0.5)
        self.declare_parameter('takeoff_altitude_tolerance_m', 1.0)
        self.declare_parameter('offboard_warmup_s', 1.0)
        self.declare_parameter('offboard_mode_retry_s', 1.0)
        self.declare_parameter('following_lateral_spacing_m', 4.0)
        self.declare_parameter('following_trail_spacing_m', 3.0)
        self.declare_parameter('hold_x', 0.0)
        self.declare_parameter('hold_y', 0.0)
        self.declare_parameter('hold_z', -2.0)
        self.declare_parameter('hold_yaw', 0.0)

    def _parameter_values(self) -> Dict[str, Any]:
        names = (
            'role',
            'vehicle_id',
            'px4_namespace',
            'px4_target_system',
            'slot',
            'control_loop_hz',
            'status_loop_hz',
            'telemetry_timeout_s',
            'takeoff_altitude_tolerance_m',
            'offboard_warmup_s',
            'offboard_mode_retry_s',
            'following_lateral_spacing_m',
            'following_trail_spacing_m',
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
    takeoff_altitude_tolerance_m = float(
        values.get('takeoff_altitude_tolerance_m', 1.0),
    )
    offboard_warmup_s = float(values.get('offboard_warmup_s', 1.0))
    offboard_mode_retry_s = float(values.get('offboard_mode_retry_s', 1.0))
    following_lateral_spacing_m = float(values.get('following_lateral_spacing_m', 4.0))
    following_trail_spacing_m = float(values.get('following_trail_spacing_m', 3.0))

    if control_loop_hz <= 0.0:
        raise ValueError('control_loop_hz must be positive')
    if status_loop_hz <= 0.0:
        raise ValueError('status_loop_hz must be positive')
    if telemetry_timeout_s <= 0.0:
        raise ValueError('telemetry_timeout_s must be positive')
    if takeoff_altitude_tolerance_m <= 0.0:
        raise ValueError('takeoff_altitude_tolerance_m must be positive')
    if offboard_warmup_s <= 0.0:
        raise ValueError('offboard_warmup_s must be positive')
    if offboard_mode_retry_s <= 0.0:
        raise ValueError('offboard_mode_retry_s must be positive')
    if following_lateral_spacing_m <= 0.0:
        raise ValueError('following_lateral_spacing_m must be positive')
    if following_trail_spacing_m <= 0.0:
        raise ValueError('following_trail_spacing_m must be positive')

    config = VehicleNodeConfig(
        role=role,
        vehicle_id=str(values.get('vehicle_id', FIRST_VERSION_LEADER.vehicle_id)),
        px4_namespace=_normalize_namespace(
            str(values.get('px4_namespace', FIRST_VERSION_LEADER.namespace)),
        ),
        px4_target_system=_target_system_for_values(values),
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
        takeoff_altitude_tolerance_m=takeoff_altitude_tolerance_m,
        offboard_warmup_s=offboard_warmup_s,
        offboard_mode_retry_s=offboard_mode_retry_s,
        following_lateral_spacing_m=following_lateral_spacing_m,
        following_trail_spacing_m=following_trail_spacing_m,
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
                'vehicle_id': 'MAV1',
                'px4_namespace': '/MAV1',
                'px4_target_system': 2,
                'slot': 'leader',
            },
        ),
        parse_vehicle_node_config(
            {
                'role': 'follower',
                'vehicle_id': 'MAV2',
                'px4_namespace': '/MAV2',
                'px4_target_system': 3,
                'slot': 'follower_left',
            },
        ),
        parse_vehicle_node_config(
            {
                'role': 'follower',
                'vehicle_id': 'MAV3',
                'px4_namespace': '/MAV3',
                'px4_target_system': 4,
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


def _target_system_for_values(values: Dict[str, Any]) -> int:
    if 'px4_target_system' in values:
        target_system = int(values['px4_target_system'])
    else:
        expectation = FIRST_VERSION_BY_VEHICLE_ID.get(
            str(values.get('vehicle_id', FIRST_VERSION_LEADER.vehicle_id)),
        )
        target_system = expectation.px4_target_system if expectation else -1
    if target_system < 0 or target_system > 255:
        raise ValueError('px4_target_system must fit uint8, or 0 for broadcast')
    return target_system


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
        vehicle.vehicle_id: (
            vehicle.namespace,
            vehicle.px4_target_system,
            vehicle.role,
            vehicle.slot,
        )
        for vehicle in FIRST_VERSION_VEHICLES
    }
    if config.vehicle_id not in expected:
        raise ValueError(f'vehicle_id must be one of: {", ".join(expected)}')

    expected_namespace, expected_target_system, expected_role, expected_slot = expected[
        config.vehicle_id
    ]
    if (
        config.px4_namespace != expected_namespace
        or config.px4_target_system != expected_target_system
        or config.role is not expected_role
        or config.slot is not expected_slot
    ):
        # 固定 vehicle_id/namespace/target 對應，保護三機 telemetry/command 不被錯接。
        raise ValueError(
            f'{config.vehicle_id} must map to '
            f'{expected_namespace}, target_system {expected_target_system}, '
            f'{expected_role.value}, {expected_slot.value}',
        )


if __name__ == '__main__':
    main()
