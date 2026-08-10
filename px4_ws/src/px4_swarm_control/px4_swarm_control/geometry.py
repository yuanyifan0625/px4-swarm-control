"""Formation geometry helpers for the PX4 swarm-control package."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin

from px4_swarm_control.models import FormationMode, PositionYawSetpoint, Slot, Vector3


@dataclass(frozen=True)
class FormationGeometry:
    """Tunable spacing used by staging and formation helpers."""

    lateral_spacing_m: float
    trail_spacing_m: float


def formation_body_offset(
    formation_mode: FormationMode,
    slot: Slot,
    geometry: FormationGeometry,
) -> Vector3:
    """Return a slot offset as (forward, left, up) in the leader body frame."""
    if slot is Slot.LEADER:
        return (0.0, 0.0, 0.0)

    left_m = _slot_left_distance(slot, geometry.lateral_spacing_m)

    if formation_mode is FormationMode.VEE:
        return (-geometry.trail_spacing_m, left_m, 0.0)
    if formation_mode is FormationMode.LINE_ABREAST:
        return (0.0, left_m, 0.0)

    raise ValueError(f'unsupported formation mode: {formation_mode}')


def staging_setpoint(
    leader_initial_setpoint: PositionYawSetpoint,
    slot: Slot,
    geometry: FormationGeometry,
) -> PositionYawSetpoint:
    """Return the world-frame staging setpoint for one slot."""
    # 起飛集結使用 leader 初始朝向，保護三機在進入編隊前保持固定安全間距。
    offset = formation_body_offset(FormationMode.VEE, slot, geometry)
    return body_offset_to_world(leader_initial_setpoint, offset)


def body_offset_to_world(
    leader_setpoint: PositionYawSetpoint,
    body_offset: Vector3,
) -> PositionYawSetpoint:
    """Rotate a leader body-frame offset into the world frame."""
    forward_m, left_m, up_m = body_offset
    yaw = leader_setpoint.yaw
    # 以 leader yaw 旋轉 body offset，保護左右隊形在轉向後仍維持相對方向。
    world_dx = forward_m * cos(yaw) - left_m * sin(yaw)
    world_dy = forward_m * sin(yaw) + left_m * cos(yaw)

    return PositionYawSetpoint(
        x=leader_setpoint.x + world_dx,
        y=leader_setpoint.y + world_dy,
        z=leader_setpoint.z + up_m,
        yaw=leader_setpoint.yaw,
    )


def _slot_left_distance(slot: Slot, lateral_spacing_m: float) -> float:
    if slot is Slot.FOLLOWER_LEFT:
        return lateral_spacing_m
    if slot is Slot.FOLLOWER_RIGHT:
        return -lateral_spacing_m
    raise ValueError(f'unsupported follower slot: {slot}')
