"""Shared first-version PX4 bridge identity and topic conventions."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

from px4_swarm_control.models import Slot, VehicleRole


PX4_V118_OUT_TOPIC_SUFFIXES = (
    '/fmu/out/vehicle_local_position_v1',
    '/fmu/out/vehicle_status_v4',
    '/fmu/out/vehicle_command_ack_v1',
)


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
        'vehicle_1',
        '/vehicle_1',
        1,
        2,
        'x500_1',
        VehicleRole.LEADER,
        Slot.LEADER,
        None,
    ),
    VehicleBridgeExpectation(
        'vehicle_2',
        '/vehicle_2',
        2,
        3,
        'x500_2',
        VehicleRole.FOLLOWER,
        Slot.FOLLOWER_LEFT,
        '0,2,0',
    ),
    VehicleBridgeExpectation(
        'vehicle_3',
        '/vehicle_3',
        3,
        4,
        'x500_3',
        VehicleRole.FOLLOWER,
        Slot.FOLLOWER_RIGHT,
        '0,-2,0',
    ),
)


FIRST_VERSION_BY_VEHICLE_ID = {
    vehicle.vehicle_id: vehicle for vehicle in FIRST_VERSION_VEHICLES
}
