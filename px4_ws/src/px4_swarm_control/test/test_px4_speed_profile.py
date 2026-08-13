from pathlib import Path

import pytest

from px4_swarm_control.px4_speed_profile import (
    ExplicitApplyRequiredError,
    Px4SpeedProfile,
    SUPPORTED_PX4_SPEED_PARAMS,
    build_pxh_apply_commands,
    build_pxh_check_commands,
    check_profile,
    diff_profile,
    load_current_values,
    load_speed_profile,
    render_diff_report,
    validate_speed_profile,
)


def test_load_speed_profile_from_yaml(tmp_path):
    profile_path = tmp_path / 'slow_demo.yaml'
    profile_path.write_text(
        '''
version: 1
name: slow_demo
description: Slow demo profile.
intended_use: sitl_demo
parameters:
  MPC_XY_VEL_MAX: 2.0
  MPC_Z_VEL_MAX_UP: 1.0
  MPC_Z_VEL_MAX_DN: 0.8
  MPC_ACC_HOR: 2.0
  MPC_JERK_AUTO: 1.0
  MPC_YAWRAUTO_MAX: 25
  MPC_YAWRAUTO_ACC: 10
''',
        encoding='utf-8',
    )

    profile = load_speed_profile(profile_path)

    assert profile == Px4SpeedProfile(
        version=1,
        name='slow_demo',
        description='Slow demo profile.',
        intended_use='sitl_demo',
        parameters={
            'MPC_XY_VEL_MAX': 2.0,
            'MPC_Z_VEL_MAX_UP': 1.0,
            'MPC_Z_VEL_MAX_DN': 0.8,
            'MPC_ACC_HOR': 2.0,
            'MPC_JERK_AUTO': 1.0,
            'MPC_YAWRAUTO_MAX': 25.0,
            'MPC_YAWRAUTO_ACC': 10.0,
        },
    )


def test_validate_rejects_unsupported_px4_parameter_name():
    profile = Px4SpeedProfile(
        version=1,
        name='bad',
        description='Bad profile.',
        intended_use='test',
        parameters={
            'MPC_XY_VEL_MAX': 2.0,
            'SYS_AUTOSTART': 4001.0,
        },
    )

    with pytest.raises(ValueError, match='unsupported PX4 speed parameter'):
        validate_speed_profile(profile)


def test_load_speed_profile_rejects_missing_or_invalid_profile(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_speed_profile(tmp_path / 'missing.yaml')

    profile_path = tmp_path / 'invalid.yaml'
    profile_path.write_text(
        '''
version: 1
name: invalid
description: Missing parameters.
intended_use: test
parameters: {}
''',
        encoding='utf-8',
    )

    with pytest.raises(ValueError, match='at least one parameter'):
        load_speed_profile(profile_path)


def test_diff_report_shows_current_desired_and_match_state():
    profile = Px4SpeedProfile(
        version=1,
        name='slow_demo',
        description='Slow demo profile.',
        intended_use='sitl_demo',
        parameters={'MPC_XY_VEL_MAX': 2.0, 'MPC_ACC_HOR': 2.0},
    )
    current_values = {
        'MAV1': {'MPC_XY_VEL_MAX': 2.0, 'MPC_ACC_HOR': 4.0},
        'MAV2': {'MPC_XY_VEL_MAX': 12.0, 'MPC_ACC_HOR': 2.0},
    }

    rows = diff_profile(profile, current_values)
    report = render_diff_report(rows)

    assert rows[0].vehicle_id == 'MAV1'
    assert rows[0].parameter == 'MPC_XY_VEL_MAX'
    assert rows[0].matches is True
    assert rows[1].parameter == 'MPC_ACC_HOR'
    assert rows[1].matches is False
    assert 'MAV1 MPC_XY_VEL_MAX current=2.0 desired=2.0 match=yes' in report
    assert 'MAV2 MPC_XY_VEL_MAX current=12.0 desired=2.0 match=no' in report


def test_load_current_values_from_yaml(tmp_path):
    current_path = tmp_path / 'current.yaml'
    current_path.write_text(
        '''
MAV1:
  MPC_XY_VEL_MAX: 2.0
MAV2:
  MPC_XY_VEL_MAX: "12.0"
''',
        encoding='utf-8',
    )

    values = load_current_values(current_path)

    assert values == {
        'MAV1': {'MPC_XY_VEL_MAX': 2.0},
        'MAV2': {'MPC_XY_VEL_MAX': 12.0},
    }


def test_check_mode_reads_current_values_without_applying():
    class FakeClient:
        def __init__(self):
            self.values = {'MPC_XY_VEL_MAX': 12.0}
            self.set_calls = []

        def get_param(self, name):
            return self.values[name]

        def set_param(self, name, value):
            self.set_calls.append((name, value))

    profile = Px4SpeedProfile(
        version=1,
        name='slow_demo',
        description='Slow demo profile.',
        intended_use='sitl_demo',
        parameters={'MPC_XY_VEL_MAX': 2.0},
    )
    client = FakeClient()

    rows = check_profile(profile, {'MAV1': client})

    assert rows[0].current == 12.0
    assert rows[0].desired == 2.0
    assert client.set_calls == []


def test_apply_mode_requires_explicit_confirmation():
    profile = Px4SpeedProfile(
        version=1,
        name='slow_demo',
        description='Slow demo profile.',
        intended_use='sitl_demo',
        parameters={'MPC_XY_VEL_MAX': 2.0},
    )

    with pytest.raises(ExplicitApplyRequiredError):
        build_pxh_apply_commands(profile, explicit_apply=False)

    commands = build_pxh_apply_commands(profile, explicit_apply=True)

    assert 'param set MPC_XY_VEL_MAX 2.0' in commands
    assert 'param save' in commands


def test_command_generation_keeps_check_and_apply_separate():
    profile = Px4SpeedProfile(
        version=1,
        name='slow_demo',
        description='Slow demo profile.',
        intended_use='sitl_demo',
        parameters={'MPC_XY_VEL_MAX': 2.0},
    )

    check_commands = build_pxh_check_commands(profile)

    assert 'param show MPC_XY_VEL_MAX' in check_commands
    assert 'param set' not in check_commands
    assert set(profile.parameters).issubset(SUPPORTED_PX4_SPEED_PARAMS)


def test_three_vehicle_node_config_does_not_contain_px4_mpc_parameters():
    config = Path(__file__).parents[1] / 'config' / 'three_vehicle_nodes.yaml'
    text = config.read_text(encoding='utf-8')

    assert 'MPC_' not in text
