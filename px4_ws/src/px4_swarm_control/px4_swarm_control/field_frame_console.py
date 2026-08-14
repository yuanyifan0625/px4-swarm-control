"""Field-frame operator console that adapts commands to PX4 local NED goals."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from math import pi
from typing import Iterable

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from px4_swarm_control.operation_profile import ALTITUDE_STEP_M
from px4_swarm_control.operation_profile import FORMATION_POSITION_TOLERANCE_M
from px4_swarm_control.operation_profile import LINE_ABREAST_LATERAL_SPACING_M
from px4_swarm_control.operation_profile import MOVE_STEP_X_M
from px4_swarm_control.operation_profile import MOVE_STEP_Y_M
from px4_swarm_control.operation_profile import SETTLE_STABLE_DURATION_S
from px4_swarm_control.operation_profile import TAKEOFF_ALTITUDE_M
from px4_swarm_control.operation_profile import VEE_LATERAL_SPACING_M
from px4_swarm_control.operation_profile import VEE_TRAIL_SPACING_M
from px4_swarm_control.operation_profile import YAW_STEP_DEG
from px4_swarm_control.operator_console import ConsoleActionResult
from px4_swarm_control.operator_console import ConsoleCommandDispatcher
from px4_swarm_control.operator_console import OperatorConsoleConfig
from px4_swarm_control.operator_console import RosSwarmActionGateway
from px4_swarm_control.operator_console import SwarmActionGateway


PX4_AXES = ('px4_x', 'px4_y', 'px4_z')
FIELD_SIGNS = ('positive', 'negative')


@dataclass(frozen=True)
class FieldAxisMapping:
    """Map one positive field-frame axis onto a signed PX4 local NED axis."""

    axis: str
    sign: str

    def __post_init__(self) -> None:
        if self.axis not in PX4_AXES:
            raise ValueError(f'unsupported field axis: {self.axis}')
        if self.sign not in FIELD_SIGNS:
            raise ValueError(f'unsupported field sign: {self.sign}')

    @property
    def index(self) -> int:
        return PX4_AXES.index(self.axis)

    @property
    def multiplier(self) -> float:
        return 1.0 if self.sign == 'positive' else -1.0


@dataclass(frozen=True)
class FieldFrameMapping:
    """Full field-frame to PX4 local NED mapping."""

    field_x: FieldAxisMapping
    field_y: FieldAxisMapping
    field_up: FieldAxisMapping

    def __post_init__(self) -> None:
        axes = {self.field_x.axis, self.field_y.axis, self.field_up.axis}
        if len(axes) != 3:
            raise ValueError('field frame mapping must use three distinct PX4 axes')

    @classmethod
    def gazebo_visual_default(cls) -> 'FieldFrameMapping':
        """Return the SITL mapping that makes Gazebo visual motion intuitive."""
        return cls(
            field_x=FieldAxisMapping(axis='px4_y', sign='positive'),
            field_y=FieldAxisMapping(axis='px4_x', sign='positive'),
            field_up=FieldAxisMapping(axis='px4_z', sign='negative'),
        )


@dataclass(frozen=True)
class FieldFrameConsoleConfig:
    """Config for the field-frame console adapter."""

    operator: OperatorConsoleConfig = field(default_factory=OperatorConsoleConfig)
    mapping: FieldFrameMapping = field(default_factory=FieldFrameMapping.gazebo_visual_default)


def field_delta_to_px4_delta(
    mapping: FieldFrameMapping,
    *,
    field_x: float,
    field_y: float,
    field_up: float,
) -> tuple[float, float, float]:
    """Translate a field-frame delta into PX4 local NED x/y/z deltas."""
    px4 = [0.0, 0.0, 0.0]
    for value, axis_mapping in (
        (field_x, mapping.field_x),
        (field_y, mapping.field_y),
        (field_up, mapping.field_up),
    ):
        px4[axis_mapping.index] += float(value) * axis_mapping.multiplier
    return (px4[0], px4[1], px4[2])


class FieldFrameCommandDispatcher(ConsoleCommandDispatcher):
    """Map field-frame short commands onto the existing `/swarm` action API."""

    def __init__(self, config: FieldFrameConsoleConfig, gateway: SwarmActionGateway):
        super().__init__(config.operator, gateway)
        self._field_config = config
        self._home_status = None

    def dispatch(self, command: str) -> ConsoleActionResult:
        command = command.strip()
        if command in {'h', 'help'}:
            return ConsoleActionResult(True, _field_help_text(self._field_config.mapping))
        if command in {'2', '3', '4', '5', 'x', 'y', 'z', 'c'}:
            return self._run_field_motion_command(command)
        if command == '1':
            result = super().dispatch(command)
            if result.success:
                self._home_status = self._gateway.get_leader_status()
            return result
        if command == 'home':
            if self._home_status is None:
                return ConsoleActionResult(False, 'home status has not been captured yet')
            return self._move_to_status(self._home_status)
        if command == 'home_yaw':
            if self._home_status is None:
                return ConsoleActionResult(False, 'home status has not been captured yet')
            return self._yaw_to_status(self._home_status)
        return super().dispatch(command)

    def _run_field_motion_command(self, command: str) -> ConsoleActionResult:
        if self._gateway.is_paused():
            return ConsoleActionResult(False, 'movement command blocked while swarm is paused')
        leader = self._gateway.get_leader_status()
        if leader is None:
            return ConsoleActionResult(False, 'leader status unavailable for field command')

        x, y, z, yaw = leader.x, leader.y, leader.z, leader.yaw
        if command in {'2', 'x', '3', 'y', '4', 'z'}:
            dx, dy, dz = self._field_delta_for_command(command)
            x += dx
            y += dy
            z += dz
        elif command == '5':
            yaw = _normalize_yaw_rad(yaw + self._config.yaw_step_rad)
        elif command == 'c':
            yaw = _normalize_yaw_rad(yaw - self._config.yaw_step_rad)

        return self._gateway.move_leader(
            x,
            y,
            z,
            yaw,
            self._config.move_position_tolerance_m,
            self._config.move_yaw_tolerance_rad,
            self._config.default_timeout_sec,
        )

    def _field_delta_for_command(self, command: str) -> tuple[float, float, float]:
        if command == '2':
            return field_delta_to_px4_delta(
                self._field_config.mapping,
                field_x=self._config.move_step_x_m,
                field_y=0.0,
                field_up=0.0,
            )
        if command == 'x':
            return field_delta_to_px4_delta(
                self._field_config.mapping,
                field_x=-self._config.move_step_x_m,
                field_y=0.0,
                field_up=0.0,
            )
        if command == '3':
            return field_delta_to_px4_delta(
                self._field_config.mapping,
                field_x=0.0,
                field_y=self._config.move_step_y_m,
                field_up=0.0,
            )
        if command == 'y':
            return field_delta_to_px4_delta(
                self._field_config.mapping,
                field_x=0.0,
                field_y=-self._config.move_step_y_m,
                field_up=0.0,
            )
        if command == '4':
            return field_delta_to_px4_delta(
                self._field_config.mapping,
                field_x=0.0,
                field_y=0.0,
                field_up=self._config.altitude_step_m,
            )
        if command == 'z':
            return field_delta_to_px4_delta(
                self._field_config.mapping,
                field_x=0.0,
                field_y=0.0,
                field_up=-self._config.altitude_step_m,
            )
        raise ValueError(f'unsupported field command: {command}')


class FieldFrameConsoleNode(Node):
    """ROS 2 node wrapper for the field-frame terminal console."""

    def __init__(self) -> None:
        super().__init__('field_frame_console')
        self._declare_parameters()
        self.config = self._load_config()
        self.gateway = RosSwarmActionGateway(self, self.config.operator)
        self.dispatcher = FieldFrameCommandDispatcher(self.config, self.gateway)

    def _declare_parameters(self) -> None:
        self.declare_parameter('takeoff_altitude_m', TAKEOFF_ALTITUDE_M)
        self.declare_parameter('default_timeout_sec', 60.0)
        self.declare_parameter('move_step_x_m', MOVE_STEP_X_M)
        self.declare_parameter('move_step_y_m', MOVE_STEP_Y_M)
        self.declare_parameter('altitude_step_m', ALTITUDE_STEP_M)
        self.declare_parameter('yaw_step_deg', YAW_STEP_DEG)
        self.declare_parameter('move_position_tolerance_m', 0.3)
        self.declare_parameter('move_yaw_tolerance_rad', 0.2)
        self.declare_parameter('status_wait_timeout_s', 2.0)
        self.declare_parameter('settle_stable_duration_s', SETTLE_STABLE_DURATION_S)
        self.declare_parameter('settle_timeout_sec', 30.0)
        self.declare_parameter('settle_position_tolerance_m', FORMATION_POSITION_TOLERANCE_M)
        self.declare_parameter('settle_yaw_tolerance_rad', 0.25)
        self.declare_parameter('settle_telemetry_timeout_s', 1.0)
        self.declare_parameter('settle_vee_lateral_spacing_m', VEE_LATERAL_SPACING_M)
        self.declare_parameter('settle_vee_trail_spacing_m', VEE_TRAIL_SPACING_M)
        self.declare_parameter(
            'settle_line_abreast_lateral_spacing_m',
            LINE_ABREAST_LATERAL_SPACING_M,
        )
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
        self.declare_parameter('field_x_axis', 'px4_y')
        self.declare_parameter('field_x_sign', 'positive')
        self.declare_parameter('field_y_axis', 'px4_x')
        self.declare_parameter('field_y_sign', 'positive')
        self.declare_parameter('field_up_axis', 'px4_z')
        self.declare_parameter('field_up_sign', 'negative')

    def _load_config(self) -> FieldFrameConsoleConfig:
        yaw_step_deg = float(self.get_parameter('yaw_step_deg').value)
        operator = OperatorConsoleConfig(
            takeoff_altitude_m=float(self.get_parameter('takeoff_altitude_m').value),
            default_timeout_sec=float(self.get_parameter('default_timeout_sec').value),
            move_step_x_m=float(self.get_parameter('move_step_x_m').value),
            move_step_y_m=float(self.get_parameter('move_step_y_m').value),
            altitude_step_m=float(self.get_parameter('altitude_step_m').value),
            yaw_step_rad=yaw_step_deg * pi / 180.0,
            move_position_tolerance_m=float(
                self.get_parameter('move_position_tolerance_m').value,
            ),
            move_yaw_tolerance_rad=float(self.get_parameter('move_yaw_tolerance_rad').value),
            status_wait_timeout_s=float(self.get_parameter('status_wait_timeout_s').value),
            settle_stable_duration_s=float(
                self.get_parameter('settle_stable_duration_s').value,
            ),
            settle_timeout_sec=float(self.get_parameter('settle_timeout_sec').value),
            settle_position_tolerance_m=float(
                self.get_parameter('settle_position_tolerance_m').value,
            ),
            settle_yaw_tolerance_rad=float(
                self.get_parameter('settle_yaw_tolerance_rad').value,
            ),
            settle_telemetry_timeout_s=float(
                self.get_parameter('settle_telemetry_timeout_s').value,
            ),
            settle_vee_lateral_spacing_m=float(
                self.get_parameter('settle_vee_lateral_spacing_m').value,
            ),
            settle_vee_trail_spacing_m=float(
                self.get_parameter('settle_vee_trail_spacing_m').value,
            ),
            settle_line_abreast_lateral_spacing_m=float(
                self.get_parameter('settle_line_abreast_lateral_spacing_m').value,
            ),
            demo_commands=tuple(self.get_parameter('demo_commands').value),
        )
        mapping = FieldFrameMapping(
            field_x=FieldAxisMapping(
                axis=str(self.get_parameter('field_x_axis').value),
                sign=str(self.get_parameter('field_x_sign').value),
            ),
            field_y=FieldAxisMapping(
                axis=str(self.get_parameter('field_y_axis').value),
                sign=str(self.get_parameter('field_y_sign').value),
            ),
            field_up=FieldAxisMapping(
                axis=str(self.get_parameter('field_up_axis').value),
                sign=str(self.get_parameter('field_up_sign').value),
            ),
        )
        return FieldFrameConsoleConfig(operator=operator, mapping=mapping)


def run_interactive_console(node: FieldFrameConsoleNode) -> None:
    print(_field_help_text(node.config.mapping))
    while rclpy.ok():
        try:
            command = input('field-swarm> ')
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if command.strip() in {'q', 'quit'}:
            break
        result = node.dispatcher.dispatch(command)
        print(('OK: ' if result.success else 'FAIL: ') + result.message)


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = FieldFrameConsoleNode()
        parser = argparse.ArgumentParser(
            description='PX4 swarm field-frame operator console',
        )
        parser.add_argument('--command', help='run one field-frame console command and exit')
        parsed = parser.parse_args(remove_ros_args(args=args)[1:])
        if parsed.command:
            result = node.dispatcher.dispatch(parsed.command)
            print(('OK: ' if result.success else 'FAIL: ') + result.message)
            return
        run_interactive_console(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def _normalize_yaw_rad(yaw: float) -> float:
    while yaw > pi:
        yaw -= 2.0 * pi
    while yaw <= -pi:
        yaw += 2.0 * pi
    return yaw


def _field_help_text(mapping: FieldFrameMapping) -> str:
    return (
        'PX4 swarm field-frame operator console commands:\n'
        '  field-frame operator console: converts field-frame movement to /swarm actions\n'
        '  default mapping is Gazebo visual profile; override for real field calibration\n'
        '  use operator_console for raw PX4 local NED commands\n'
        '  real vehicles must run coordinate_frame_probe before choosing field mapping\n'
        '  do not assume Gazebo visual profile matches the real field frame\n'
        f'  mapping: field +X -> {_mapping_label(mapping.field_x)}, '
        f'field +Y -> {_mapping_label(mapping.field_y)}, '
        f'field up -> {_mapping_label(mapping.field_up)}\n'
        '  s: status\n'
        '  p: pause\n'
        '  r: resume\n'
        '  q: quit\n'
        '  h: help\n'
        '  0: ArmSwarm without takeoff\n'
        '  1: TakeoffSwarm\n'
        '  2: move leader field +X step\n'
        '  x: move leader field -X step\n'
        '  3: move leader field +Y step\n'
        '  y: move leader field -Y step\n'
        '  4: move leader field up step\n'
        '  z: move leader field down step\n'
        '  5: rotate leader yaw + step\n'
        '  c: rotate leader yaw - step\n'
        '  6: ChangeFormation vee\n'
        '  7: ChangeFormation line_abreast\n'
        '  8: LandSwarm\n'
        '  9: demo macro using field-frame movement\n'
        '  settle: wait until followers settle into the current formation\n'
        '  home: return leader to captured home pose\n'
        '  home_yaw: restore captured home yaw\n'
    )


def _mapping_label(mapping: FieldAxisMapping) -> str:
    sign = '+' if mapping.sign == 'positive' else '-'
    axis = mapping.axis.removeprefix('px4_').upper()
    return f'PX4 {sign}{axis}'


if __name__ == '__main__':
    main()
