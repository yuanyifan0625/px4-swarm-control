"""Shared first-version PX4 bridge identity and topic conventions."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

from px4_swarm_control.models import Slot, VehicleRole


@dataclass(frozen=True)
class Px4MessageContract:
    """One px4_msgs type and its unversioned ROS topic in a compatibility profile."""

    message_type: str
    topic_suffix: str
    message_version: int
    px4_definition_path: str
    required_fields: tuple[str, ...] = ()
    absent_fields: tuple[str, ...] = ()


@dataclass(frozen=True)
class Px4CompatibilityProfile:
    """Pinned PX4/px4_msgs contract consumed only at the bridge boundary."""

    name: str
    px4_firmware_commit: str
    px4_msgs_commit: str
    message_contracts: tuple[Px4MessageContract, ...]

    def message(self, message_type: str) -> Px4MessageContract:
        for contract in self.message_contracts:
            if contract.message_type == message_type:
                return contract
        raise KeyError(f'{message_type} is not part of {self.name}')


PX4_V117 = Px4CompatibilityProfile(
    name='PX4_V117',
    px4_firmware_commit='d6f12ad1c4f70ad3230afd7d86e971421e02fef4',
    px4_msgs_commit='86d8239e962f6939e05c3737784f60c02fa884db',
    message_contracts=(
        Px4MessageContract(
            'FailsafeFlags', '/fmu/out/failsafe_flags', 0, 'msg/FailsafeFlags.msg',
        ),
        Px4MessageContract(
            'OffboardControlMode',
            '/fmu/in/offboard_control_mode',
            0,
            'msg/OffboardControlMode.msg',
        ),
        Px4MessageContract(
            'TrajectorySetpoint',
            '/fmu/in/trajectory_setpoint',
            0,
            'msg/versioned/TrajectorySetpoint.msg',
        ),
        Px4MessageContract(
            'VehicleCommand',
            '/fmu/in/vehicle_command',
            0,
            'msg/versioned/VehicleCommand.msg',
        ),
        Px4MessageContract(
            'VehicleCommandAck',
            '/fmu/out/vehicle_command_ack',
            0,
            'msg/versioned/VehicleCommandAck.msg',
        ),
        Px4MessageContract(
            'VehicleLandDetected',
            '/fmu/out/vehicle_land_detected',
            0,
            'msg/versioned/VehicleLandDetected.msg',
        ),
        Px4MessageContract(
            'VehicleLocalPosition',
            '/fmu/out/vehicle_local_position',
            1,
            'msg/versioned/VehicleLocalPosition.msg',
        ),
        Px4MessageContract(
            'VehicleStatus',
            '/fmu/out/vehicle_status',
            1,
            'msg/versioned/VehicleStatus.msg',
            required_fields=('pre_flight_checks_pass',),
            absent_fields=('accepts_offboard_setpoints',),
        ),
    ),
)


def versioned_topic_suffix(base_topic: str, message_type: type) -> str:
    """Derive the PX4 ROS topic suffix from generated message metadata."""
    version = int(getattr(message_type, 'MESSAGE_VERSION', 0))
    return base_topic if version == 0 else f'{base_topic}_v{version}'


@dataclass(frozen=True)
class VehicleBridgeExpectation:
    """Expected live bridge identity for one first-version vehicle."""

    vehicle_id: str
    namespace: str
    px4_instance: int
    px4_target_system: int
    model_name: str
    role: VehicleRole
    slot: Slot
    spawn_pose: str | None


FIRST_VERSION_VEHICLES: Tuple[
    VehicleBridgeExpectation,
    VehicleBridgeExpectation,
    VehicleBridgeExpectation,
] = (
    VehicleBridgeExpectation(
        'MAV1',
        '/MAV1',
        1,
        2,
        'x500_1',
        VehicleRole.LEADER,
        Slot.LEADER,
        None,
    ),
    VehicleBridgeExpectation(
        'MAV2',
        '/MAV2',
        2,
        3,
        'x500_2',
        VehicleRole.FOLLOWER,
        Slot.FOLLOWER_LEFT,
        '0,2,0',
    ),
    VehicleBridgeExpectation(
        'MAV3',
        '/MAV3',
        3,
        4,
        'x500_3',
        VehicleRole.FOLLOWER,
        Slot.FOLLOWER_RIGHT,
        '0,-2,0',
    ),
)


FIRST_VERSION_LEADER = FIRST_VERSION_VEHICLES[0]


FIRST_VERSION_BY_VEHICLE_ID = {
    vehicle.vehicle_id: vehicle for vehicle in FIRST_VERSION_VEHICLES
}
