from px4_swarm_control.bridge_config import FIRST_VERSION_VEHICLES, PX4_V117
from px4_swarm_control.bridge_config import versioned_topic_suffix


class VersionZeroMessage:
    pass


class VersionOneMessage:
    MESSAGE_VERSION = 1


def test_versioned_topic_suffix_is_derived_from_message_metadata():
    base = '/fmu/out/vehicle_status'

    assert versioned_topic_suffix(base, VersionZeroMessage) == base
    assert versioned_topic_suffix(base, VersionOneMessage) == f'{base}_v1'


def test_px4_v117_profile_centralizes_revisions_and_used_message_contracts():
    assert PX4_V117.name == 'PX4_V117'
    assert PX4_V117.px4_firmware_commit == (
        'd6f12ad1c4f70ad3230afd7d86e971421e02fef4'
    )
    assert PX4_V117.px4_msgs_commit == (
        '86d8239e962f6939e05c3737784f60c02fa884db'
    )
    assert {
        contract.message_type: contract.message_version
        for contract in PX4_V117.message_contracts
    } == {
        'FailsafeFlags': 0,
        'OffboardControlMode': 0,
        'TrajectorySetpoint': 0,
        'VehicleCommand': 0,
        'VehicleCommandAck': 0,
        'VehicleLandDetected': 0,
        'VehicleLocalPosition': 1,
        'VehicleStatus': 1,
    }


def test_first_version_bridge_keeps_sitl_instance_distinct_from_vehicle_identity():
    assert [vehicle.px4_instance for vehicle in FIRST_VERSION_VEHICLES] == [1, 2, 3]
    assert [vehicle.px4_target_system for vehicle in FIRST_VERSION_VEHICLES] == [1, 2, 3]
    assert [vehicle.sitl_instance for vehicle in FIRST_VERSION_VEHICLES] == [0, 1, 2]
