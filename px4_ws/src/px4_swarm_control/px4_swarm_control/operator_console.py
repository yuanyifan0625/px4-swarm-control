"""Terminal operator console for the first-version PX4 swarm action surface."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from math import isfinite, pi, sqrt
from time import monotonic
from typing import Callable, Iterable, Protocol

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from px4_swarm_control.bridge_config import FIRST_VERSION_VEHICLES
from px4_swarm_control.geometry import (
    body_offset_to_world,
    formation_body_offset,
    FormationGeometry,
)
from px4_swarm_control.models import FormationMode, PositionYawSetpoint, Slot
from px4_swarm_interfaces.action import (
    ArmSwarm,
    ChangeFormation,
    LandSwarm,
    MoveLeader,
    PauseSwarm,
    TakeoffSwarm,
)
from px4_swarm_interfaces.msg import VehicleStatus


@dataclass(frozen=True)
class ConsoleActionResult:
    """Small command outcome used by tests and the terminal UI."""

    success: bool
    message: str


@dataclass(frozen=True)
class OperatorConsoleConfig:
    """Configurable defaults for short operator commands."""

    takeoff_altitude_m: float = 5.0
    default_timeout_sec: float = 60.0
    move_step_x_m: float = 3.0
    move_step_y_m: float = 3.0
    altitude_step_m: float = 1.0
    yaw_step_rad: float = pi / 3.0
    move_position_tolerance_m: float = 0.3
    move_yaw_tolerance_rad: float = 0.2
    status_wait_timeout_s: float = 2.0
    settle_stable_duration_s: float = 1.0
    settle_timeout_sec: float = 30.0
    settle_position_tolerance_m: float = 0.5
    settle_yaw_tolerance_rad: float = 0.25
    settle_telemetry_timeout_s: float = 1.0
    settle_lateral_spacing_m: float = 4.0
    settle_trail_spacing_m: float = 3.0
    demo_commands: tuple[str, ...] = (
        '1',
        '2',
        'settle',
        '5',
        'settle',
        '7',
        'settle',
        '6',
        'settle',
        'home_yaw',
        'settle',
        'home',
        'settle',
        '8',
    )


class SwarmActionGateway(Protocol):
    """Boundary used by the console core to call existing swarm actions only."""

    def get_leader_status(self) -> VehicleStatus | None:
        """Return the latest leader status if one has been observed."""

    def is_paused(self) -> bool:
        """Return whether the swarm currently appears paused."""

    def describe_status(self) -> str:
        """Return a terminal-friendly status summary."""

    def wait_for_formation_settle(
        self,
        formation_mode: str,
        config: OperatorConsoleConfig,
    ) -> ConsoleActionResult:
        """Wait until followers have visibly settled into the current formation."""

    def takeoff(self, altitude_m: float, timeout_sec: float) -> ConsoleActionResult:
        """Call TakeoffSwarm."""

    def arm(self, timeout_sec: float) -> ConsoleActionResult:
        """Call ArmSwarm without requesting takeoff."""

    def move_leader(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        position_tolerance_m: float,
        yaw_tolerance_rad: float,
        timeout_sec: float,
    ) -> ConsoleActionResult:
        """Call MoveLeader."""

    def change_formation(
        self,
        formation_mode: str,
        timeout_sec: float,
    ) -> ConsoleActionResult:
        """Call ChangeFormation."""

    def pause(self, pause: bool, reason: str) -> ConsoleActionResult:
        """Call PauseSwarm."""

    def land(self, timeout_sec: float) -> ConsoleActionResult:
        """Call LandSwarm."""


class ConsoleCommandDispatcher:
    """Map short terminal commands onto the existing operator action API."""

    def __init__(self, config: OperatorConsoleConfig, gateway: SwarmActionGateway):
        self._config = config
        self._gateway = gateway
        self._active_formation = FormationMode.VEE.value

    def dispatch(self, command: str) -> ConsoleActionResult:
        command = command.strip()
        if command in {'h', 'help'}:
            return ConsoleActionResult(True, _help_text())
        if command in {'s', 'status'}:
            return ConsoleActionResult(True, self._gateway.describe_status())
        if command in {'p', 'pause'}:
            return self._gateway.pause(True, 'operator console pause')
        if command in {'r', 'resume'}:
            return self._gateway.pause(False, 'operator console resume')
        if command == '0':
            return self._run_arm_command()
        if command == '1':
            return self._gateway.takeoff(
                self._config.takeoff_altitude_m,
                self._config.default_timeout_sec,
            )
        if command in {'2', '3', '4', '5'}:
            return self._run_motion_command(command)
        if command == '6':
            return self._run_formation_command('vee')
        if command == '7':
            return self._run_formation_command('line_abreast')
        if command == 'settle':
            return self._run_settle_command()
        if command == '8':
            return self._gateway.land(self._config.default_timeout_sec)
        if command == '9':
            return self._run_demo_macro()
        return ConsoleActionResult(False, f'unknown command: {command}. Use h for help.')

    def _run_arm_command(self) -> ConsoleActionResult:
        if self._gateway.is_paused():
            return ConsoleActionResult(False, 'arm command blocked while swarm is paused')
        return self._gateway.arm(self._config.default_timeout_sec)

    def _run_motion_command(self, command: str) -> ConsoleActionResult:
        if self._gateway.is_paused():
            return ConsoleActionResult(False, 'movement command blocked while swarm is paused')
        leader = self._gateway.get_leader_status()
        if leader is None:
            return ConsoleActionResult(False, 'leader status unavailable for relative command')

        x, y, z, yaw = leader.x, leader.y, leader.z, leader.yaw
        if command == '2':
            x += self._config.move_step_x_m
        elif command == '3':
            y += self._config.move_step_y_m
        elif command == '4':
            z -= self._config.altitude_step_m
        elif command == '5':
            yaw = _normalize_yaw_rad(yaw + self._config.yaw_step_rad)

        # Console 只把 operator 的相對 jog 轉成既有 leader absolute goal，保護 follower 不被直接指定目標。
        return self._gateway.move_leader(
            x,
            y,
            z,
            yaw,
            self._config.move_position_tolerance_m,
            self._config.move_yaw_tolerance_rad,
            self._config.default_timeout_sec,
        )

    def _run_formation_command(self, formation_mode: str) -> ConsoleActionResult:
        if self._gateway.is_paused():
            return ConsoleActionResult(
                False,
                'formation command blocked while swarm is paused',
            )
        result = self._gateway.change_formation(
            formation_mode,
            self._config.default_timeout_sec,
        )
        if result.success:
            self._active_formation = formation_mode
        return result

    def _run_settle_command(self) -> ConsoleActionResult:
        if self._gateway.is_paused():
            return ConsoleActionResult(False, 'settle command blocked while swarm is paused')
        # Demo settle 只觀察三機 status，保護 macro 不在 followers 尚未收斂時送下一步。
        return self._gateway.wait_for_formation_settle(
            self._active_formation,
            self._config,
        )

    def _run_demo_macro(self) -> ConsoleActionResult:
        if self._gateway.is_paused():
            return ConsoleActionResult(False, 'demo macro blocked while swarm is paused')

        home: VehicleStatus | None = None
        for command in self._config.demo_commands:
            # Macro 每一步都等前一步成功，保護 demo 不在中途失敗後繼續送危險命令。
            if command == 'home':
                if home is None:
                    return ConsoleActionResult(False, 'demo macro has no home status')
                result = self._move_to_status(home)
            elif command == 'home_yaw':
                if home is None:
                    return ConsoleActionResult(False, 'demo macro has no home status')
                result = self._yaw_to_status(home)
            else:
                result = self.dispatch(command)
                if command == '1' and result.success:
                    home = self._gateway.get_leader_status()

            if not result.success:
                return ConsoleActionResult(False, f'demo macro stopped: {result.message}')
        return ConsoleActionResult(True, 'demo macro completed')

    def _move_to_status(self, status: VehicleStatus) -> ConsoleActionResult:
        if self._gateway.is_paused():
            return ConsoleActionResult(False, 'home move blocked while swarm is paused')
        return self._gateway.move_leader(
            status.x,
            status.y,
            status.z,
            status.yaw,
            self._config.move_position_tolerance_m,
            self._config.move_yaw_tolerance_rad,
            self._config.default_timeout_sec,
        )

    def _yaw_to_status(self, status: VehicleStatus) -> ConsoleActionResult:
        if self._gateway.is_paused():
            return ConsoleActionResult(False, 'home yaw blocked while swarm is paused')
        leader = self._gateway.get_leader_status()
        if leader is None:
            return ConsoleActionResult(False, 'leader status unavailable for home yaw')
        return self._gateway.move_leader(
            leader.x,
            leader.y,
            leader.z,
            status.yaw,
            self._config.move_position_tolerance_m,
            self._config.move_yaw_tolerance_rad,
            self._config.default_timeout_sec,
        )


class RosSwarmActionGateway:
    """ROS 2 adapter that talks to the existing `/swarm` action surface."""

    def __init__(self, node: Node, config: OperatorConsoleConfig):
        self._node = node
        self._config = config
        self._statuses: dict[int, VehicleStatus] = {}
        self._arm_client = ActionClient(node, ArmSwarm, '/swarm/arm')
        self._takeoff_client = ActionClient(node, TakeoffSwarm, '/swarm/takeoff')
        self._move_client = ActionClient(node, MoveLeader, '/swarm/move_leader')
        self._formation_client = ActionClient(
            node,
            ChangeFormation,
            '/swarm/change_formation',
        )
        self._pause_client = ActionClient(node, PauseSwarm, '/swarm/pause')
        self._land_client = ActionClient(node, LandSwarm, '/swarm/land')
        self._subscriptions = [
            node.create_subscription(
                VehicleStatus,
                f'{vehicle.namespace}/status',
                self._handle_status,
                10,
            )
            for vehicle in FIRST_VERSION_VEHICLES
        ]

    def get_leader_status(self) -> VehicleStatus | None:
        self._spin_for_status(1)
        return self._statuses.get(1)

    def is_paused(self) -> bool:
        self._spin_once()
        return any(status.vehicle_state == 'paused' for status in self._statuses.values())

    def describe_status(self) -> str:
        deadline = monotonic() + self._config.status_wait_timeout_s
        while len(self._statuses) < 3 and monotonic() < deadline:
            self._spin_once()
        if not self._statuses:
            return 'no vehicle status observed yet'
        lines = []
        for vehicle_id in sorted(self._statuses):
            status = self._statuses[vehicle_id]
            lines.append(
                f'MAV{vehicle_id}: state={status.vehicle_state} '
                f'armed={status.armed} '
                f'pos=({status.x:.2f}, {status.y:.2f}, {status.z:.2f}) '
                f'yaw={status.yaw:.2f}'
            )
        return '\n'.join(lines)

    def wait_for_formation_settle(
        self,
        formation_mode: str,
        config: OperatorConsoleConfig,
    ) -> ConsoleActionResult:
        deadline = monotonic() + config.settle_timeout_sec
        gate = FormationSettleGate(config)
        while monotonic() < deadline:
            self._spin_once()
            if any(status.vehicle_state == 'paused' for status in self._statuses.values()):
                return ConsoleActionResult(False, 'formation settle stopped while paused')
            if gate.update(self._statuses, formation_mode):
                return ConsoleActionResult(True, 'formation settled')
        return ConsoleActionResult(False, 'formation settle timed out')

    def takeoff(self, altitude_m: float, timeout_sec: float) -> ConsoleActionResult:
        goal = TakeoffSwarm.Goal()
        goal.altitude_m = float(altitude_m)
        goal.timeout_sec = float(timeout_sec)
        return self._call_action(self._takeoff_client, goal, '/swarm/takeoff')

    def arm(self, timeout_sec: float) -> ConsoleActionResult:
        goal = ArmSwarm.Goal()
        goal.timeout_sec = float(timeout_sec)
        return self._call_action(self._arm_client, goal, '/swarm/arm')

    def move_leader(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        position_tolerance_m: float,
        yaw_tolerance_rad: float,
        timeout_sec: float,
    ) -> ConsoleActionResult:
        goal = MoveLeader.Goal()
        goal.x = float(x)
        goal.y = float(y)
        goal.z = float(z)
        goal.yaw = float(yaw)
        goal.position_tolerance_m = float(position_tolerance_m)
        goal.yaw_tolerance_rad = float(yaw_tolerance_rad)
        goal.timeout_sec = float(timeout_sec)
        return self._call_action(self._move_client, goal, '/swarm/move_leader')

    def change_formation(
        self,
        formation_mode: str,
        timeout_sec: float,
    ) -> ConsoleActionResult:
        goal = ChangeFormation.Goal()
        goal.formation_mode = formation_mode
        goal.timeout_sec = float(timeout_sec)
        return self._call_action(self._formation_client, goal, '/swarm/change_formation')

    def pause(self, pause: bool, reason: str) -> ConsoleActionResult:
        goal = PauseSwarm.Goal()
        goal.pause = pause
        goal.reason = reason
        return self._call_action(self._pause_client, goal, '/swarm/pause')

    def land(self, timeout_sec: float) -> ConsoleActionResult:
        goal = LandSwarm.Goal()
        goal.timeout_sec = float(timeout_sec)
        return self._call_action(self._land_client, goal, '/swarm/land')

    def _handle_status(self, msg: VehicleStatus) -> None:
        self._statuses[msg.vehicle_id] = msg

    def _spin_for_status(self, vehicle_id: int) -> None:
        deadline = monotonic() + self._config.status_wait_timeout_s
        while vehicle_id not in self._statuses and monotonic() < deadline:
            self._spin_once()

    def _spin_once(self) -> None:
        rclpy.spin_once(self._node, timeout_sec=0.1)

    def _call_action(self, client: ActionClient, goal, action_name: str) -> ConsoleActionResult:
        if not client.wait_for_server(timeout_sec=5.0):
            return ConsoleActionResult(False, f'action server unavailable: {action_name}')

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self._node, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return ConsoleActionResult(False, f'action goal rejected: {action_name}')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        result = result_future.result().result
        return ConsoleActionResult(bool(result.success), result.message)


class OperatorConsoleNode(Node):
    """ROS 2 node wrapper for the terminal operator console."""

    def __init__(self) -> None:
        super().__init__('operator_console')
        self._declare_parameters()
        self.config = self._load_config()
        self.gateway = RosSwarmActionGateway(self, self.config)
        self.dispatcher = ConsoleCommandDispatcher(self.config, self.gateway)

    def _declare_parameters(self) -> None:
        self.declare_parameter('takeoff_altitude_m', 5.0)
        self.declare_parameter('default_timeout_sec', 60.0)
        self.declare_parameter('move_step_x_m', 3.0)
        self.declare_parameter('move_step_y_m', 3.0)
        self.declare_parameter('altitude_step_m', 1.0)
        self.declare_parameter('yaw_step_deg', 60.0)
        self.declare_parameter('move_position_tolerance_m', 0.3)
        self.declare_parameter('move_yaw_tolerance_rad', 0.2)
        self.declare_parameter('status_wait_timeout_s', 2.0)
        self.declare_parameter('settle_stable_duration_s', 1.0)
        self.declare_parameter('settle_timeout_sec', 30.0)
        self.declare_parameter('settle_position_tolerance_m', 0.5)
        self.declare_parameter('settle_yaw_tolerance_rad', 0.25)
        self.declare_parameter('settle_telemetry_timeout_s', 1.0)
        self.declare_parameter('settle_lateral_spacing_m', 4.0)
        self.declare_parameter('settle_trail_spacing_m', 3.0)
        self.declare_parameter(
            'demo_commands',
            [
                '1',
                '2',
                'settle',
                '5',
                'settle',
                '7',
                'settle',
                '6',
                'settle',
                'home_yaw',
                'settle',
                'home',
                'settle',
                '8',
            ],
        )

    def _load_config(self) -> OperatorConsoleConfig:
        yaw_step_deg = float(self.get_parameter('yaw_step_deg').value)
        return OperatorConsoleConfig(
            takeoff_altitude_m=float(self.get_parameter('takeoff_altitude_m').value),
            default_timeout_sec=float(self.get_parameter('default_timeout_sec').value),
            move_step_x_m=float(self.get_parameter('move_step_x_m').value),
            move_step_y_m=float(self.get_parameter('move_step_y_m').value),
            altitude_step_m=float(self.get_parameter('altitude_step_m').value),
            yaw_step_rad=yaw_step_deg * pi / 180.0,
            move_position_tolerance_m=float(
                self.get_parameter('move_position_tolerance_m').value
            ),
            move_yaw_tolerance_rad=float(self.get_parameter('move_yaw_tolerance_rad').value),
            status_wait_timeout_s=float(self.get_parameter('status_wait_timeout_s').value),
            settle_stable_duration_s=float(
                self.get_parameter('settle_stable_duration_s').value
            ),
            settle_timeout_sec=float(self.get_parameter('settle_timeout_sec').value),
            settle_position_tolerance_m=float(
                self.get_parameter('settle_position_tolerance_m').value
            ),
            settle_yaw_tolerance_rad=float(
                self.get_parameter('settle_yaw_tolerance_rad').value
            ),
            settle_telemetry_timeout_s=float(
                self.get_parameter('settle_telemetry_timeout_s').value
            ),
            settle_lateral_spacing_m=float(
                self.get_parameter('settle_lateral_spacing_m').value
            ),
            settle_trail_spacing_m=float(self.get_parameter('settle_trail_spacing_m').value),
            demo_commands=tuple(self.get_parameter('demo_commands').value),
        )


def run_interactive_console(node: OperatorConsoleNode) -> None:
    print(_help_text())
    while rclpy.ok():
        try:
            command = input('swarm> ')
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if command.strip() in {'q', 'quit'}:
            break
        result = node.dispatcher.dispatch(command)
        print(('OK: ' if result.success else 'FAIL: ') + result.message)


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OperatorConsoleNode()
    try:
        parser = argparse.ArgumentParser(description='PX4 swarm operator short-command console')
        parser.add_argument('--command', help='run one console command and exit')
        parsed = parser.parse_args(remove_ros_args(args=args)[1:])
        if parsed.command:
            result = node.dispatcher.dispatch(parsed.command)
            print(('OK: ' if result.success else 'FAIL: ') + result.message)
            return
        run_interactive_console(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _normalize_yaw_rad(yaw: float) -> float:
    while yaw > pi:
        yaw -= 2.0 * pi
    while yaw <= -pi:
        yaw += 2.0 * pi
    return yaw


def _help_text() -> str:
    return (
        'PX4 swarm operator console commands:\n'
        '  s: status\n'
        '  p: pause\n'
        '  r: resume\n'
        '  q: quit\n'
        '  h: help\n'
        '  0: ArmSwarm without takeoff\n'
        '  1: TakeoffSwarm\n'
        '  2: move leader world x + step\n'
        '  3: move leader world y + step\n'
        '  4: move leader up by altitude step (NED z -= step)\n'
        '  5: rotate leader yaw + step\n'
        '  6: ChangeFormation vee\n'
        '  7: ChangeFormation line_abreast\n'
        '  8: LandSwarm\n'
        '  9: demo macro\n'
        '  settle: wait until followers settle into the current formation\n'
    )


class FormationSettleGate:
    """Track a continuous stable window for follower formation readiness."""

    def __init__(
        self,
        config: OperatorConsoleConfig,
        now_s: Callable[[], float] = monotonic,
    ) -> None:
        self._config = config
        self._now_s = now_s
        self._stable_since_s: float | None = None

    def update(self, statuses: dict[int, VehicleStatus], formation_mode: str) -> bool:
        now_s = self._now_s()
        if not formation_settle_ready(statuses, formation_mode, self._config):
            self._stable_since_s = None
            return False
        if self._stable_since_s is None:
            self._stable_since_s = now_s
            return False
        return now_s - self._stable_since_s >= self._config.settle_stable_duration_s


def formation_settle_ready(
    statuses: dict[int, VehicleStatus],
    formation_mode: str,
    config: OperatorConsoleConfig,
) -> bool:
    try:
        mode = FormationMode(formation_mode)
    except ValueError:
        return False

    leader_status = statuses.get(1)
    if not _fresh_status(leader_status, config.settle_telemetry_timeout_s):
        return False

    for vehicle_id, slot in (
        (2, Slot.FOLLOWER_LEFT),
        (3, Slot.FOLLOWER_RIGHT),
    ):
        follower_status = statuses.get(vehicle_id)
        if not _fresh_status(follower_status, config.settle_telemetry_timeout_s):
            return False
        if follower_status.slot != slot.value:
            return False
        target = _formation_target_for_follower(leader_status, mode, slot, config)
        if not _status_close_to_setpoint(
            follower_status,
            target,
            config.settle_position_tolerance_m,
            config.settle_yaw_tolerance_rad,
        ):
            return False
    return True


def _formation_target_for_follower(
    leader_status: VehicleStatus,
    formation_mode: FormationMode,
    slot: Slot,
    config: OperatorConsoleConfig,
) -> PositionYawSetpoint:
    geometry = FormationGeometry(
        lateral_spacing_m=config.settle_lateral_spacing_m,
        trail_spacing_m=config.settle_trail_spacing_m,
    )
    leader = PositionYawSetpoint(
        leader_status.x,
        leader_status.y,
        leader_status.z,
        leader_status.yaw,
    )
    return body_offset_to_world(
        leader,
        formation_body_offset(formation_mode, slot, geometry),
    )


def _fresh_status(status: VehicleStatus | None, telemetry_timeout_s: float) -> bool:
    if status is None:
        return False
    return (
        _status_pose_is_finite(status)
        and status.armed
        and status.nav_state == 'offboard'
        and isfinite(status.last_telemetry_age_sec)
        and status.last_telemetry_age_sec <= telemetry_timeout_s
    )


def _status_pose_is_finite(status: VehicleStatus) -> bool:
    return all(isfinite(value) for value in (status.x, status.y, status.z, status.yaw))


def _status_close_to_setpoint(
    status: VehicleStatus,
    target: PositionYawSetpoint,
    position_tolerance_m: float,
    yaw_tolerance_rad: float,
) -> bool:
    distance_m = sqrt(
        (status.x - target.x) ** 2
        + (status.y - target.y) ** 2
        + (status.z - target.z) ** 2
    )
    return (
        distance_m <= position_tolerance_m
        and _yaw_error_rad(status.yaw, target.yaw) <= yaw_tolerance_rad
    )


def _yaw_error_rad(current_yaw: float, target_yaw: float) -> float:
    return abs((current_yaw - target_yaw + pi) % (2.0 * pi) - pi)


if __name__ == '__main__':
    main()
