"""Follower fixed-slot control helpers."""

from math import isfinite

from px4_swarm_control.geometry import (
    body_offset_to_world,
    formation_body_offset,
    FormationGeometry,
)
from px4_swarm_control.models import FormationMode, PositionYawSetpoint, Slot
from px4_swarm_interfaces.msg import VehicleStatus


def derive_follower_setpoint(
    leader_setpoint: PositionYawSetpoint,
    formation_mode: FormationMode,
    slot: Slot,
    geometry: FormationGeometry,
) -> PositionYawSetpoint:
    """Return this follower's world-frame setpoint for the active formation slot."""
    # 用 leader yaw 旋轉 body-frame slot，保護編隊在 leader 轉向時不固定在 world frame。
    return body_offset_to_world(
        leader_setpoint,
        formation_body_offset(formation_mode, slot, geometry),
    )


def leader_status_is_fresh(status: VehicleStatus | None, timeout_s: float) -> bool:
    if status is None:
        return False
    values = (status.x, status.y, status.z, status.yaw, status.last_telemetry_age_sec)
    # follower 只信任已進 Offboard 的 leader，保護編隊不跟隨起飛或降落中的暫態資料。
    return (
        all(isfinite(value) for value in values)
        and status.vehicle_id == 1
        and status.armed
        and status.nav_state == 'offboard'
        and status.vehicle_state == 'following'
        and status.last_telemetry_age_sec <= timeout_s
    )


def leader_status_setpoint(status: VehicleStatus) -> PositionYawSetpoint:
    return PositionYawSetpoint(status.x, status.y, status.z, status.yaw)
