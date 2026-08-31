"""Formation geometry in the measured real-vehicle PX4 local frame.

Body offsets use positive forward, left, and up.  PX4 local x/y/z are
East/South/Down, with yaw 0=North and yaw +pi/2=West.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin

from px4_swarm_control.models import FormationMode, PositionYawSetpoint, Slot, Vector3
from px4_swarm_control.operation_profile import LINE_ABREAST_LATERAL_SPACING_M
from px4_swarm_control.operation_profile import VEE_LATERAL_SPACING_M
from px4_swarm_control.operation_profile import VEE_TRAIL_SPACING_M


@dataclass(frozen=True, init=False)
class FormationGeometry:
    """Tunable spacing used by staging and formation helpers."""

    vee_lateral_spacing_m: float
    vee_trail_spacing_m: float
    line_abreast_lateral_spacing_m: float

    def __init__(
        self,
        vee_lateral_spacing_m: float | None = None,
        vee_trail_spacing_m: float | None = None,
        line_abreast_lateral_spacing_m: float | None = None,
        *,
        lateral_spacing_m: float | None = None,
        trail_spacing_m: float | None = None,
    ) -> None:
        """Create formation geometry.

        ``lateral_spacing_m`` and ``trail_spacing_m`` are legacy aliases kept
        for older launch/test overrides. New callers should pass the explicit
        VEE and line-abreast fields.
        """
        if vee_lateral_spacing_m is None:
            vee_lateral_spacing_m = (
                VEE_LATERAL_SPACING_M
                if lateral_spacing_m is None
                else lateral_spacing_m
            )
        if vee_trail_spacing_m is None:
            vee_trail_spacing_m = (
                VEE_TRAIL_SPACING_M
                if trail_spacing_m is None
                else trail_spacing_m
            )
        if line_abreast_lateral_spacing_m is None:
            line_abreast_lateral_spacing_m = (
                LINE_ABREAST_LATERAL_SPACING_M
                if lateral_spacing_m is None
                else lateral_spacing_m
            )

        object.__setattr__(self, 'vee_lateral_spacing_m', vee_lateral_spacing_m)
        object.__setattr__(self, 'vee_trail_spacing_m', vee_trail_spacing_m)
        object.__setattr__(
            self,
            'line_abreast_lateral_spacing_m',
            line_abreast_lateral_spacing_m,
        )


def formation_body_offset(
    formation_mode: FormationMode,
    slot: Slot,
    geometry: FormationGeometry,
) -> Vector3:
    """Return a slot offset as (forward, left, up) in the leader body frame."""
    if slot is Slot.LEADER:
        return (0.0, 0.0, 0.0)

    if formation_mode is FormationMode.VEE:
        left_m = _slot_left_distance(slot, geometry.vee_lateral_spacing_m)
        return (-geometry.vee_trail_spacing_m, left_m, 0.0)
    if formation_mode is FormationMode.LINE_ABREAST:
        left_m = _slot_left_distance(
            slot,
            geometry.line_abreast_lateral_spacing_m,
        )
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
    """Rotate a positive-forward/left/up body offset into PX4 local coordinates."""
    forward_m, left_m, up_m = body_offset
    yaw = leader_setpoint.yaw
    # Measured contract: yaw=0 points North (-y), and body-left points West
    # (-x).  This preserves body-relative formation slots at every yaw.
    world_dx = -forward_m * sin(yaw) - left_m * cos(yaw)
    world_dy = -forward_m * cos(yaw) + left_m * sin(yaw)

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
