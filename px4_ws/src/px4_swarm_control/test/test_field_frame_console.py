from math import isclose, pi

import pytest

from px4_swarm_control.field_frame_console import (
    FieldAxisMapping,
    FieldFrameCommandDispatcher,
    FieldFrameConsoleConfig,
    FieldFrameMapping,
    field_delta_to_px4_delta,
)
from px4_swarm_control.operator_console import (
    ConsoleActionResult,
    OperatorConsoleConfig,
    SwarmActionGateway,
)
from px4_swarm_interfaces.msg import VehicleStatus


class FakeGateway(SwarmActionGateway):
    def __init__(self, leader_status=None, paused=False, action_results=None):
        self.leader_status = leader_status
        self.paused = paused
        self.action_results = action_results or {}
        self.calls = []

    def get_leader_status(self):
        return self.leader_status

    def is_paused(self):
        return self.paused

    def describe_status(self):
        return 'paused' if self.paused else 'running'

    def takeoff(self, altitude_m, timeout_sec):
        self.calls.append(('takeoff', altitude_m, timeout_sec))
        return self.action_results.get('takeoff', ConsoleActionResult(True, 'takeoff ok'))

    def arm(self, timeout_sec):
        self.calls.append(('arm', timeout_sec))
        return self.action_results.get('arm', ConsoleActionResult(True, 'arm ok'))

    def move_leader(
        self,
        x,
        y,
        z,
        yaw,
        position_tolerance_m,
        yaw_tolerance_rad,
        timeout_sec,
    ):
        self.calls.append(
            (
                'move_leader',
                x,
                y,
                z,
                yaw,
                position_tolerance_m,
                yaw_tolerance_rad,
                timeout_sec,
            )
        )
        return self.action_results.get('move_leader', ConsoleActionResult(True, 'move ok'))

    def change_formation(self, formation_mode, timeout_sec):
        self.calls.append(('change_formation', formation_mode, timeout_sec))
        return self.action_results.get(
            'change_formation',
            ConsoleActionResult(True, 'formation ok'),
        )

    def pause(self, pause, reason):
        self.calls.append(('pause', pause, reason))
        self.paused = pause
        return self.action_results.get('pause', ConsoleActionResult(True, 'pause ok'))

    def land(self, timeout_sec):
        self.calls.append(('land', timeout_sec))
        return self.action_results.get('land', ConsoleActionResult(True, 'land ok'))

    def wait_for_formation_settle(self, formation_mode, config):
        self.calls.append(
            (
                'settle',
                formation_mode,
                config.settle_stable_duration_s,
                config.settle_timeout_sec,
                config.settle_position_tolerance_m,
                config.settle_yaw_tolerance_rad,
            )
        )
        return self.action_results.get('settle', ConsoleActionResult(True, 'settle ok'))


def leader_status(x=1.0, y=2.0, z=-5.0, yaw=0.0):
    status = VehicleStatus()
    status.vehicle_id = 1
    status.x = x
    status.y = y
    status.z = z
    status.yaw = yaw
    status.vehicle_state = 'following'
    status.armed = True
    status.nav_state = 'offboard'
    status.last_telemetry_age_sec = 0.1
    return status


def move_calls(gateway):
    return [call for call in gateway.calls if call[0] == 'move_leader']


def test_default_gazebo_visual_profile_maps_field_axes_to_px4_ned_deltas():
    mapping = FieldFrameMapping.gazebo_visual_default()

    assert field_delta_to_px4_delta(mapping, field_x=1.0, field_y=0.0, field_up=0.0) == (
        0.0,
        1.0,
        0.0,
    )
    assert field_delta_to_px4_delta(mapping, field_x=0.0, field_y=1.0, field_up=0.0) == (
        1.0,
        0.0,
        0.0,
    )
    assert field_delta_to_px4_delta(mapping, field_x=0.0, field_y=0.0, field_up=1.0) == (
        0.0,
        0.0,
        -1.0,
    )


def test_invalid_field_frame_mapping_rejects_unknown_axis_sign_and_duplicate_axes():
    with pytest.raises(ValueError, match='unsupported field axis'):
        FieldAxisMapping(axis='px4_w', sign='positive')
    with pytest.raises(ValueError, match='unsupported field sign'):
        FieldAxisMapping(axis='px4_x', sign='forward')
    with pytest.raises(ValueError, match='must use three distinct PX4 axes'):
        FieldFrameMapping(
            field_x=FieldAxisMapping(axis='px4_x', sign='positive'),
            field_y=FieldAxisMapping(axis='px4_x', sign='negative'),
            field_up=FieldAxisMapping(axis='px4_z', sign='negative'),
        )


def test_field_frame_movement_commands_convert_to_absolute_px4_goals():
    config = FieldFrameConsoleConfig(
        operator=OperatorConsoleConfig(
            move_step_x_m=1.0,
            move_step_y_m=1.0,
            altitude_step_m=1.0,
            yaw_step_rad=pi / 4.0,
            move_position_tolerance_m=0.3,
            move_yaw_tolerance_rad=0.2,
            default_timeout_sec=60.0,
        ),
        mapping=FieldFrameMapping.gazebo_visual_default(),
    )
    gateway = FakeGateway(leader_status=leader_status(x=1.0, y=2.0, z=-5.0, yaw=0.0))
    dispatcher = FieldFrameCommandDispatcher(config, gateway)

    for command in ('2', 'x', '3', 'y', '4', 'z', '5', 'c'):
        dispatcher.dispatch(command)

    calls = move_calls(gateway)
    assert calls[0][1:5] == (1.0, 3.0, -5.0, 0.0)
    assert calls[1][1:5] == (1.0, 1.0, -5.0, 0.0)
    assert calls[2][1:5] == (2.0, 2.0, -5.0, 0.0)
    assert calls[3][1:5] == (0.0, 2.0, -5.0, 0.0)
    assert calls[4][1:5] == (1.0, 2.0, -6.0, 0.0)
    assert calls[5][1:5] == (1.0, 2.0, -4.0, 0.0)
    assert calls[6][1:4] == (1.0, 2.0, -5.0)
    assert isclose(calls[6][4], pi / 4.0)
    assert calls[7][1:4] == (1.0, 2.0, -5.0)
    assert isclose(calls[7][4], -pi / 4.0)


def test_field_frame_mapping_parameters_can_flip_real_field_direction():
    mapping = FieldFrameMapping(
        field_x=FieldAxisMapping(axis='px4_y', sign='negative'),
        field_y=FieldAxisMapping(axis='px4_x', sign='positive'),
        field_up=FieldAxisMapping(axis='px4_z', sign='negative'),
    )
    config = FieldFrameConsoleConfig(
        operator=OperatorConsoleConfig(move_step_x_m=1.0),
        mapping=mapping,
    )
    gateway = FakeGateway(leader_status=leader_status(x=1.0, y=2.0, z=-5.0))
    dispatcher = FieldFrameCommandDispatcher(config, gateway)

    dispatcher.dispatch('2')
    dispatcher.dispatch('x')

    assert move_calls(gateway)[0][1:5] == (1.0, 1.0, -5.0, 0.0)
    assert move_calls(gateway)[1][1:5] == (1.0, 3.0, -5.0, 0.0)


def test_non_movement_commands_reuse_existing_swarm_action_flow():
    gateway = FakeGateway(leader_status=leader_status(x=1.0, y=2.0, z=-1.5, yaw=0.5))
    dispatcher = FieldFrameCommandDispatcher(FieldFrameConsoleConfig(), gateway)

    status = dispatcher.dispatch('s')
    dispatcher.dispatch('0')
    dispatcher.dispatch('p')
    dispatcher.dispatch('r')
    dispatcher.dispatch('1')
    dispatcher.dispatch('6')
    dispatcher.dispatch('7')
    dispatcher.dispatch('settle')
    dispatcher.dispatch('8')

    assert status.message == 'running'
    assert gateway.calls == [
        ('arm', 60.0),
        ('pause', True, 'operator console pause'),
        ('pause', False, 'operator console resume'),
        ('takeoff', 1.5, 60.0),
        ('move_leader', 1.0, 2.0, -1.5, 0.5, 0.3, 0.2, 60.0),
        ('change_formation', 'vee', 60.0),
        ('move_leader', 1.0, 2.0, -1.5, 0.5, 0.3, 0.2, 60.0),
        ('change_formation', 'line_abreast', 60.0),
        ('settle', 'line_abreast', 1.5, 30.0, 0.10, 0.25),
        ('land', 60.0),
    ]


def test_home_commands_use_captured_home_pose_without_changing_raw_console():
    gateway = FakeGateway(leader_status=leader_status(x=1.0, y=2.0, z=-1.5, yaw=0.5))
    dispatcher = FieldFrameCommandDispatcher(FieldFrameConsoleConfig(), gateway)

    dispatcher.dispatch('1')
    gateway.leader_status = leader_status(x=1.5, y=2.5, z=-1.2, yaw=1.0)
    home = dispatcher.dispatch('home')
    home_yaw = dispatcher.dispatch('home_yaw')

    assert home.success is True
    assert home_yaw.success is True
    assert gateway.calls == [
        ('takeoff', 1.5, 60.0),
        ('move_leader', 1.0, 2.0, -1.5, 0.5, 0.3, 0.2, 60.0),
        ('move_leader', 1.5, 2.5, -1.2, 0.5, 0.3, 0.2, 60.0),
    ]


def test_demo_macro_movement_uses_field_frame_mapping():
    config = FieldFrameConsoleConfig(
        operator=OperatorConsoleConfig(demo_commands=('1', '2', 'settle', '8')),
        mapping=FieldFrameMapping.gazebo_visual_default(),
    )
    gateway = FakeGateway(leader_status=leader_status(x=0.0, y=0.0, z=-1.5, yaw=0.0))
    dispatcher = FieldFrameCommandDispatcher(config, gateway)

    def takeoff(altitude_m, timeout_sec):
        gateway.calls.append(('takeoff', altitude_m, timeout_sec))
        gateway.leader_status = leader_status(x=0.0, y=0.0, z=-1.5, yaw=0.0)
        return ConsoleActionResult(True, 'takeoff ok')

    def move_leader(x, y, z, yaw, position_tolerance_m, yaw_tolerance_rad, timeout_sec):
        FakeGateway.move_leader(
            gateway,
            x,
            y,
            z,
            yaw,
            position_tolerance_m,
            yaw_tolerance_rad,
            timeout_sec,
        )
        gateway.leader_status = leader_status(x=x, y=y, z=z, yaw=yaw)
        return ConsoleActionResult(True, 'move ok')

    gateway.takeoff = takeoff
    gateway.move_leader = move_leader

    result = dispatcher.dispatch('9')

    assert result.success is True
    assert gateway.calls == [
        ('takeoff', 1.5, 60.0),
        ('move_leader', 0.0, 1.0, -1.5, 0.0, 0.3, 0.2, 60.0),
        ('settle', 'vee', 1.5, 30.0, 0.10, 0.25),
        ('land', 60.0),
    ]


def test_help_text_names_field_frame_adapter_and_raw_console_escape_hatch():
    gateway = FakeGateway(leader_status=leader_status())
    dispatcher = FieldFrameCommandDispatcher(FieldFrameConsoleConfig(), gateway)

    result = dispatcher.dispatch('h')

    assert result.success is True
    assert 'field-frame operator console' in result.message
    assert 'default mapping is Gazebo visual profile' in result.message
    assert 'operator_console for raw PX4 local NED' in result.message
    assert 'real vehicles must run coordinate_frame_probe' in result.message
    assert 'do not assume Gazebo visual profile matches the real field frame' in result.message
    assert '2: move leader field +X step' in result.message
    assert 'home: return leader to captured home pose' in result.message
    assert 'home_yaw: restore captured home yaw' in result.message
    assert '9: demo macro using field-frame movement' in result.message
