"""Terminal operator console for the first-version PX4 swarm action surface."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from math import pi
from time import monotonic
from typing import Iterable, Protocol

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from px4_swarm_interfaces.action import (
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
    move_step_x_m: float = 1.0
    move_step_y_m: float = 1.0
    altitude_step_m: float = 1.0
    yaw_step_rad: float = pi / 4.0
    move_position_tolerance_m: float = 0.3
    move_yaw_tolerance_rad: float = 0.2
    status_wait_timeout_s: float = 2.0
    demo_commands: tuple[str, ...] = ('1', '2', '5', '7', '6', 'home', '8')


class SwarmActionGateway(Protocol):
    """Boundary used by the console core to call existing swarm actions only."""

    def get_leader_status(self) -> VehicleStatus | None:
        """Return the latest leader status if one has been observed."""

    def is_paused(self) -> bool:
        """Return whether the swarm currently appears paused."""

    def describe_status(self) -> str:
        """Return a terminal-friendly status summary."""

    def takeoff(self, altitude_m: float, timeout_sec: float) -> ConsoleActionResult:
        """Call TakeoffSwarm."""

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
        if command == '8':
            return self._gateway.land(self._config.default_timeout_sec)
        if command == '9':
            return self._run_demo_macro()
        return ConsoleActionResult(False, f'unknown command: {command}. Use h for help.')

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
        return self._gateway.change_formation(
            formation_mode,
            self._config.default_timeout_sec,
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


class RosSwarmActionGateway:
    """ROS 2 adapter that talks to the existing `/swarm` action surface."""

    def __init__(self, node: Node, config: OperatorConsoleConfig):
        self._node = node
        self._config = config
        self._statuses: dict[int, VehicleStatus] = {}
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
                f'/vehicle_{vehicle_id}/status',
                self._handle_status,
                10,
            )
            for vehicle_id in (1, 2, 3)
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
                f'vehicle_{vehicle_id}: state={status.vehicle_state} '
                f'armed={status.armed} '
                f'pos=({status.x:.2f}, {status.y:.2f}, {status.z:.2f}) '
                f'yaw={status.yaw:.2f}'
            )
        return '\n'.join(lines)

    def takeoff(self, altitude_m: float, timeout_sec: float) -> ConsoleActionResult:
        goal = TakeoffSwarm.Goal()
        goal.altitude_m = float(altitude_m)
        goal.timeout_sec = float(timeout_sec)
        return self._call_action(self._takeoff_client, goal, '/swarm/takeoff')

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
        self.declare_parameter('move_step_x_m', 1.0)
        self.declare_parameter('move_step_y_m', 1.0)
        self.declare_parameter('altitude_step_m', 1.0)
        self.declare_parameter('yaw_step_deg', 45.0)
        self.declare_parameter('move_position_tolerance_m', 0.3)
        self.declare_parameter('move_yaw_tolerance_rad', 0.2)
        self.declare_parameter('status_wait_timeout_s', 2.0)
        self.declare_parameter('demo_commands', ['1', '2', '5', '7', '6', 'home', '8'])

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
        '  1: TakeoffSwarm\n'
        '  2: move leader world x + step\n'
        '  3: move leader world y + step\n'
        '  4: move leader up by altitude step (NED z -= step)\n'
        '  5: rotate leader yaw + step\n'
        '  6: ChangeFormation vee\n'
        '  7: ChangeFormation line_abreast\n'
        '  8: LandSwarm\n'
        '  9: demo macro\n'
    )


if __name__ == '__main__':
    main()
