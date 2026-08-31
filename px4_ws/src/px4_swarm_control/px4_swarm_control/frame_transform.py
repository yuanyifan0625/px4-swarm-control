"""Fixed field-frame to measured PX4 local-coordinate transformations."""

from __future__ import annotations

def field_delta_to_px4_delta(
    *,
    field_x: float,
    field_y: float,
    field_up: float,
) -> tuple[float, float, float]:
    """Convert field North/West/up into PX4 East/South/Down.

    Field +X is North and therefore PX4 -y; field +Y is West and therefore
    PX4 -x; field up is PX4 -z.  All three vehicles share this origin/frame.
    """
    return (-float(field_y), -float(field_x), -float(field_up))
