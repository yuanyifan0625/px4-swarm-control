"""Fixed operator field-frame to PX4 local NED transformations."""

from __future__ import annotations

def field_delta_to_ned_delta(
    *,
    field_x: float,
    field_y: float,
    field_up: float,
) -> tuple[float, float, float]:
    """Convert fixed field +X/+Y/up into local NED +Y/+X/-Z."""
    return (float(field_y), float(field_x), -float(field_up))
