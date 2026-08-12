"""Internal domain models for PX4 swarm-control logic."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Tuple


Vector3 = Tuple[float, float, float]


class VehicleRole(str, Enum):
    """Stable vehicle behavior roles."""

    LEADER = 'leader'
    FOLLOWER = 'follower'


class Slot(str, Enum):
    """Fixed first-version formation slots."""

    LEADER = 'leader'
    FOLLOWER_LEFT = 'follower_left'
    FOLLOWER_RIGHT = 'follower_right'


class FormationMode(str, Enum):
    """Supported first-version formation modes."""

    VEE = 'vee'
    LINE_ABREAST = 'line_abreast'


class MissionState(str, Enum):
    """Ground-station mission-level states."""

    IDLE = 'idle'
    ARMING = 'arming'
    TAKING_OFF = 'taking_off'
    STAGING = 'staging'
    HOLDING = 'holding'
    FORMING = 'forming'
    FOLLOWING = 'following'
    RECONFIGURING = 'reconfiguring'
    PAUSED = 'paused'
    LANDING = 'landing'
    DONE = 'done'
    FAILSAFE = 'failsafe'
    ERROR = 'error'


class VehicleLevelState(str, Enum):
    """Per-vehicle local control states."""

    IDLE = 'idle'
    ARMING = 'arming'
    TAKING_OFF = 'taking_off'
    STAGING = 'staging'
    HOLDING = 'holding'
    FOLLOWING = 'following'
    RECONFIGURING = 'reconfiguring'
    PAUSED = 'paused'
    LANDING = 'landing'
    LANDED = 'landed'
    FAILSAFE = 'failsafe'
    ERROR = 'error'


class CommandStatus(str, Enum):
    """High-level command result status."""

    ACCEPTED = 'accepted'
    REJECTED = 'rejected'
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'
    TIMED_OUT = 'timed_out'


@dataclass(frozen=True)
class VehicleConfig:
    """Static identity and role parameters for one vehicle node."""

    vehicle_id: str
    px4_namespace: str
    role: VehicleRole
    slot: Slot


@dataclass(frozen=True)
class PositionYawSetpoint:
    """Controller-facing high-level position plus yaw command."""

    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class VehicleState:
    """Controller-facing vehicle telemetry summary without raw PX4 message types."""

    vehicle_id: str
    position: Vector3
    yaw: float
    velocity: Vector3
    armed: bool
    navigation_state: str
    offboard_available: bool
    telemetry_age_s: float
    vehicle_level_state: VehicleLevelState
    landed: bool = False


@dataclass(frozen=True)
class VehicleCommandResult:
    """Normalized command outcome used outside the PX4 topic boundary."""

    status: CommandStatus
    message: str = ''


def default_vehicle_configs() -> Tuple[VehicleConfig, VehicleConfig, VehicleConfig]:
    """Return the fixed first-version vehicle role and slot assignments."""
    return (
        VehicleConfig('vehicle_1', '/vehicle_1', VehicleRole.LEADER, Slot.LEADER),
        VehicleConfig('vehicle_2', '/vehicle_2', VehicleRole.FOLLOWER, Slot.FOLLOWER_LEFT),
        VehicleConfig('vehicle_3', '/vehicle_3', VehicleRole.FOLLOWER, Slot.FOLLOWER_RIGHT),
    )
