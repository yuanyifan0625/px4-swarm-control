from px4_msgs import msg as px4_msg

from px4_swarm_control import px4_compatibility
from px4_swarm_control.bridge_config import PX4_V117
from px4_swarm_control.px4_compatibility import compare_message_definitions
from px4_swarm_control.px4_compatibility import CompatibilityResult
from px4_swarm_control.px4_compatibility import inspect_runtime_contract


def test_runtime_contract_reports_v117_versions_fields_and_topics():
    result = inspect_runtime_contract()

    assert result.compatible is True
    assert result.errors == ()
    assert result.topic_suffixes == (
        '/fmu/out/failsafe_flags',
        '/fmu/in/offboard_control_mode',
        '/fmu/in/trajectory_setpoint',
        '/fmu/in/vehicle_command',
        '/fmu/out/vehicle_command_ack',
        '/fmu/out/vehicle_land_detected',
        '/fmu/out/vehicle_local_position_v1',
        '/fmu/out/vehicle_status_v1',
    )
    assert 'VehicleStatus: version=1' in result.summary
    assert 'pre_flight_checks_pass=present' in result.summary
    assert 'accepts_offboard_setpoints=absent' in result.summary


def test_runtime_contract_rejects_incompatible_vehicle_status_shape():
    class IncompatibleVehicleStatus:
        MESSAGE_VERSION = 2
        pre_flight_checks_pass = False
        accepts_offboard_setpoints = True

    message_types = {
        contract.message_type: getattr(px4_msg, contract.message_type)
        for contract in PX4_V117.message_contracts
    }
    message_types['VehicleStatus'] = IncompatibleVehicleStatus

    result = inspect_runtime_contract(message_types)

    assert result.compatible is False
    assert any('VehicleStatus version expected 1, actual 2' in error for error in result.errors)
    assert any(
        'VehicleStatus field must be absent: accepts_offboard_setpoints' in error
        for error in result.errors
    )


def test_message_definition_comparison_covers_all_profile_types():
    px4_definitions = {
        contract.message_type: f'{contract.message_type} definition'
        for contract in PX4_V117.message_contracts
    }
    px4_msgs_definitions = dict(px4_definitions)

    assert compare_message_definitions(px4_definitions, px4_msgs_definitions) == ()

    px4_msgs_definitions['VehicleStatus'] = 'different definition'
    assert compare_message_definitions(px4_definitions, px4_msgs_definitions) == (
        'VehicleStatus definition differs between PX4 and px4_msgs',
    )


def test_compatibility_cli_returns_nonzero_for_incompatible_runtime(monkeypatch):
    monkeypatch.setattr(
        px4_compatibility,
        'inspect_runtime_contract',
        lambda: CompatibilityResult(
            compatible=False,
            errors=('VehicleStatus version mismatch',),
            topic_suffixes=(),
            summary='profile=PX4_V117',
        ),
    )
    definitions = {
        contract.message_type: contract.message_type
        for contract in PX4_V117.message_contracts
    }
    monkeypatch.setattr(
        px4_compatibility,
        'read_fixed_git_definitions',
        lambda *args, **kwargs: definitions,
    )

    assert px4_compatibility.main([]) == 1


def test_compatibility_cli_rejects_unsupported_px4_source_head(monkeypatch):
    definitions = {
        contract.message_type: contract.message_type
        for contract in PX4_V117.message_contracts
    }
    monkeypatch.setattr(
        px4_compatibility,
        'read_fixed_git_definitions',
        lambda *args, **kwargs: definitions,
    )
    monkeypatch.setattr(
        px4_compatibility,
        'read_git_head',
        lambda repository: (
            'c890d9db0a300795594fd5ba6c045be9ebd71c09'
            if repository.name == 'PX4-Autopilot'
            else PX4_V117.px4_msgs_commit
        ),
    )

    assert px4_compatibility.main([]) == 1
