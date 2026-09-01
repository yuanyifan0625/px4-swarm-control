"""Pure coordinate profiles at the PX4/raw to canonical-control seam."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, pi


GAZEBO_ENU_COMMON_WORLD = 'gazebo_enu_common_world'
RAW_PX4_LOCAL = 'raw_px4_local'
GAZEBO_ENU_YAW_OFFSET_RAD = 0.102


@dataclass(frozen=True)
class CoordinatePose:
    """A position/yaw expressed in one coordinate frame."""

    position: tuple[float, float, float]
    yaw: float


@dataclass(frozen=True)
class CoordinateProfile:
    """Bidirectional conversion between PX4 raw local and canonical control."""

    name: str
    origin_enu: tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw_offset_rad: float = 0.0

    @classmethod
    def gazebo_enu_common_world(
        cls,
        *,
        origin_enu: tuple[float, float, float],
    ) -> 'CoordinateProfile':
        return cls(
            name=GAZEBO_ENU_COMMON_WORLD,
            origin_enu=_finite_vector(origin_enu, 'origin_enu'),
            yaw_offset_rad=GAZEBO_ENU_YAW_OFFSET_RAD,
        )

    @classmethod
    def raw_px4_local(cls) -> 'CoordinateProfile':
        return cls(name=RAW_PX4_LOCAL)

    def raw_to_common_pose(
        self,
        position: tuple[float, float, float],
        yaw: float,
    ) -> CoordinatePose:
        x, y, z = _finite_vector(position, 'raw position')
        _finite_yaw(yaw)
        if self.name == RAW_PX4_LOCAL:
            return CoordinatePose((x, y, z), float(yaw))
        if self.name == GAZEBO_ENU_COMMON_WORLD:
            east, north, up = self.origin_enu
            return CoordinatePose(
                (east + y, north + x, up - z),
                _wrap_yaw(pi / 2.0 - yaw + self.yaw_offset_rad),
            )
        raise ValueError(f'unsupported coordinate profile: {self.name}')

    def common_to_raw_pose(
        self,
        position: tuple[float, float, float],
        yaw: float,
    ) -> CoordinatePose:
        east, north, up = _finite_vector(position, 'common position')
        _finite_yaw(yaw)
        if self.name == RAW_PX4_LOCAL:
            return CoordinatePose((east, north, up), float(yaw))
        if self.name == GAZEBO_ENU_COMMON_WORLD:
            origin_east, origin_north, origin_up = self.origin_enu
            return CoordinatePose(
                (north - origin_north, east - origin_east, origin_up - up),
                _wrap_yaw(pi / 2.0 + self.yaw_offset_rad - yaw),
            )
        raise ValueError(f'unsupported coordinate profile: {self.name}')

    def raw_to_common_vector(
        self,
        vector: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        x, y, z = _finite_vector(vector, 'raw vector')
        if self.name == RAW_PX4_LOCAL:
            return (x, y, z)
        if self.name == GAZEBO_ENU_COMMON_WORLD:
            return (y, x, -z)
        raise ValueError(f'unsupported coordinate profile: {self.name}')


def _finite_vector(
    values: tuple[float, float, float],
    label: str,
) -> tuple[float, float, float]:
    result = tuple(float(value) for value in values)
    if len(result) != 3 or not all(isfinite(value) for value in result):
        raise ValueError(f'{label} must contain three finite values')
    return result  # type: ignore[return-value]


def _finite_yaw(yaw: float) -> None:
    if not isfinite(yaw):
        raise ValueError('yaw must be finite')


def _wrap_yaw(yaw: float) -> float:
    return (float(yaw) + pi) % (2.0 * pi) - pi
