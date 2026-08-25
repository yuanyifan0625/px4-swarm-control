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
    main,
    render_diff_report,
    validate_speed_profile,
)


REQUIRED_CAUTION_SPEED_PARAMETERS = {
    'MPC_XY_VEL_MAX': 0.3,
    'MPC_Z_VEL_MAX_UP': 0.3,
    'MPC_YAWRAUTO_MAX': 30.0,
}


@pytest.mark.parametrize('profile_name', ('slow_demo', 'real_cautious'))
def test_packaged_profiles_set_only_the_agreed_px4_speed_limits(profile_name):
    profile_path = (
        Path(__file__).parents[1]
        / 'config'
        / 'px4_speed_profiles'
        / f'{profile_name}.yaml'
    )

    profile = load_speed_profile(profile_path)

    assert profile.parameters == REQUIRED_CAUTION_SPEED_PARAMETERS


def test_load_speed_profile_from_yaml(tmp_path):
    profile_path = tmp_path / 'slow_demo.yaml'
    profile_path.write_text(
        '''
version: 1
name: slow_demo
description: Slow demo profile.
intended_use: sitl_demo
parameters:
  MPC_XY_VEL_MAX: 0.3
  MPC_Z_VEL_MAX_UP: 0.3
  MPC_YAWRAUTO_MAX: 30
''',
        encoding='utf-8',
    )

    profile = load_speed_profile(profile_path)

    assert profile == Px4SpeedProfile(
        version=1,
        name='slow_demo',
        description='Slow demo profile.',
        intended_use='sitl_demo',
        parameters=REQUIRED_CAUTION_SPEED_PARAMETERS,
    )


def test_validate_rejects_unsupported_px4_parameter_name():
    profile = Px4SpeedProfile(
        version=1,
        name='bad',
        description='Bad profile.',
        intended_use='test',
        parameters={
            **REQUIRED_CAUTION_SPEED_PARAMETERS,
            'SYS_AUTOSTART': 4001.0,
        },
    )

    with pytest.raises(ValueError, match='unsupported PX4 speed parameter'):
        validate_speed_profile(profile)


def test_validate_rejects_descent_limit_so_px4_descent_setting_is_preserved():
    profile = Px4SpeedProfile(
        version=1,
        name='changes_descent',
        description='Must not change descent limit.',
        intended_use='test',
        parameters={
            **REQUIRED_CAUTION_SPEED_PARAMETERS,
            'MPC_Z_VEL_MAX_DN': 0.3,
        },
    )

    with pytest.raises(ValueError, match='unsupported PX4 speed parameter'):
        validate_speed_profile(profile)


def test_validate_requires_all_agreed_px4_speed_limits():
    profile = Px4SpeedProfile(
        version=1,
        name='incomplete',
        description='Missing yaw limit.',
        intended_use='test',
        parameters={
            'MPC_XY_VEL_MAX': 0.3,
            'MPC_Z_VEL_MAX_UP': 0.3,
        },
    )

    with pytest.raises(ValueError, match='missing required PX4 speed parameters'):
        validate_speed_profile(profile)


@pytest.mark.parametrize('invalid_value', (0.0, -0.1, float('nan'), float('inf')))
def test_validate_rejects_non_positive_or_non_finite_speed_limits(invalid_value):
    parameters = dict(REQUIRED_CAUTION_SPEED_PARAMETERS)
    parameters['MPC_XY_VEL_MAX'] = invalid_value
    profile = Px4SpeedProfile(
        version=1,
        name='invalid_limit',
        description='Invalid speed limit.',
        intended_use='test',
        parameters=parameters,
    )

    with pytest.raises(ValueError, match='finite and greater than zero'):
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
        parameters=dict(REQUIRED_CAUTION_SPEED_PARAMETERS),
    )
    current_values = {
        'MAV1': dict(REQUIRED_CAUTION_SPEED_PARAMETERS),
        'MAV2': {
            **REQUIRED_CAUTION_SPEED_PARAMETERS,
            'MPC_XY_VEL_MAX': 0.5,
        },
    }

    rows = diff_profile(profile, current_values)
    report = render_diff_report(rows)

    assert rows[0].vehicle_id == 'MAV1'
    assert rows[0].parameter == 'MPC_XY_VEL_MAX'
    assert rows[0].matches is True
    assert rows[1].parameter == 'MPC_Z_VEL_MAX_UP'
    assert rows[1].matches is True
    assert 'MAV1 MPC_XY_VEL_MAX current=0.3 desired=0.3 match=yes' in report
    assert 'MAV2 MPC_XY_VEL_MAX current=0.5 desired=0.3 match=no' in report


def test_diff_report_includes_missing_rows_for_each_requested_vehicle():
    profile = Px4SpeedProfile(
        version=1,
        name='slow_demo',
        description='Slow demo profile.',
        intended_use='sitl_demo',
        parameters=dict(REQUIRED_CAUTION_SPEED_PARAMETERS),
    )
    current_values = {
        'MAV1': dict(REQUIRED_CAUTION_SPEED_PARAMETERS),
    }

    rows = diff_profile(profile, current_values, vehicle_ids=('MAV1', 'MAV2', 'MAV3'))
    report = render_diff_report(rows)

    assert len(rows) == 9
    assert 'MAV1 MPC_XY_VEL_MAX current=0.3 desired=0.3 match=yes' in report
    assert 'MAV2 MPC_XY_VEL_MAX current=missing desired=0.3 match=no' in report
    assert 'MAV3 MPC_YAWRAUTO_MAX current=missing desired=30.0 match=no' in report


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
            self.values = {
                **REQUIRED_CAUTION_SPEED_PARAMETERS,
                'MPC_XY_VEL_MAX': 0.5,
            }
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
        parameters=dict(REQUIRED_CAUTION_SPEED_PARAMETERS),
    )
    client = FakeClient()

    rows = check_profile(profile, {'MAV1': client})

    assert rows[0].current == 0.5
    assert rows[0].desired == 0.3
    assert client.set_calls == []


def test_apply_mode_requires_explicit_confirmation():
    profile = Px4SpeedProfile(
        version=1,
        name='slow_demo',
        description='Slow demo profile.',
        intended_use='sitl_demo',
        parameters=dict(REQUIRED_CAUTION_SPEED_PARAMETERS),
    )

    with pytest.raises(ExplicitApplyRequiredError):
        build_pxh_apply_commands(profile, explicit_apply=False)

    commands = build_pxh_apply_commands(profile, explicit_apply=True)

    assert 'param set MPC_XY_VEL_MAX 0.3' in commands
    assert 'param save' in commands


def test_command_generation_keeps_check_and_apply_separate():
    profile = Px4SpeedProfile(
        version=1,
        name='slow_demo',
        description='Slow demo profile.',
        intended_use='sitl_demo',
        parameters=dict(REQUIRED_CAUTION_SPEED_PARAMETERS),
    )

    check_commands = build_pxh_check_commands(profile)

    assert 'param show MPC_XY_VEL_MAX' in check_commands
    assert 'param set' not in check_commands
    assert set(profile.parameters).issubset(SUPPORTED_PX4_SPEED_PARAMS)


def test_check_cli_reports_match_mismatch_and_missing_for_all_mavs(tmp_path, capsys):
    current_path = tmp_path / 'current.yaml'
    current_path.write_text(
        '''
MAV1:
  MPC_XY_VEL_MAX: 0.3
  MPC_Z_VEL_MAX_UP: 0.3
  MPC_YAWRAUTO_MAX: 30
MAV2:
  MPC_XY_VEL_MAX: 0.5
  MPC_Z_VEL_MAX_UP: 0.3
  MPC_YAWRAUTO_MAX: 30
''',
        encoding='utf-8',
    )
    profile_dir = Path(__file__).parents[1] / 'config' / 'px4_speed_profiles'

    result = main(
        [
            'check',
            '--profile',
            'slow_demo',
            '--profile-dir',
            str(profile_dir),
            '--current-values',
            str(current_path),
        ]
    )
    output = capsys.readouterr().out

    assert result == 0
    assert 'MAV1 MPC_XY_VEL_MAX current=0.3 desired=0.3 match=yes' in output
    assert 'MAV2 MPC_XY_VEL_MAX current=0.5 desired=0.3 match=no' in output
    assert 'MAV3 MPC_XY_VEL_MAX current=missing desired=0.3 match=no' in output


def test_check_cli_without_live_values_prints_commands_for_each_px4_shell(capsys):
    profile_dir = Path(__file__).parents[1] / 'config' / 'px4_speed_profiles'

    result = main(
        [
            'check',
            '--profile',
            'real_cautious',
            '--profile-dir',
            str(profile_dir),
        ]
    )
    output = capsys.readouterr().out

    assert result == 0
    assert '目前沒有 live PX4 parameter client' in output
    assert output.count('param show MPC_XY_VEL_MAX') == 3
    assert '# MAV1 PX4 shell' in output
    assert '# MAV2 PX4 shell' in output
    assert '# MAV3 PX4 shell' in output


def test_three_vehicle_node_config_does_not_contain_px4_mpc_parameters():
    config = Path(__file__).parents[1] / 'config' / 'three_vehicle_nodes.yaml'
    text = config.read_text(encoding='utf-8')

    assert 'MPC_' not in text
