from math import pi

from px4_swarm_control.follower_controller import derive_follower_setpoint
from px4_swarm_control.geometry import FormationGeometry
from px4_swarm_control.models import FormationMode, PositionYawSetpoint, Slot


def assert_setpoint_close(actual, expected):
    assert abs(actual.x - expected.x) <= 1e-6
    assert abs(actual.y - expected.y) <= 1e-6
    assert abs(actual.z - expected.z) <= 1e-6
    assert abs(actual.yaw - expected.yaw) <= 1e-6


def test_vee_follower_slots_from_leader_yaw_zero_keep_left_and_right_signs():
    leader = PositionYawSetpoint(10.0, 20.0, -5.0, 0.0)
    geometry = FormationGeometry(lateral_spacing_m=4.0, trail_spacing_m=3.0)

    left = derive_follower_setpoint(leader, FormationMode.VEE, Slot.FOLLOWER_LEFT, geometry)
    right = derive_follower_setpoint(
        leader,
        FormationMode.VEE,
        Slot.FOLLOWER_RIGHT,
        geometry,
    )

    assert_setpoint_close(left, PositionYawSetpoint(7.0, 16.0, -5.0, 0.0))
    assert_setpoint_close(right, PositionYawSetpoint(7.0, 24.0, -5.0, 0.0))


def test_vee_follower_slots_rotate_with_leader_yaw_ninety_degrees():
    leader = PositionYawSetpoint(10.0, 20.0, -5.0, pi / 2.0)
    geometry = FormationGeometry(lateral_spacing_m=4.0, trail_spacing_m=3.0)

    left = derive_follower_setpoint(leader, FormationMode.VEE, Slot.FOLLOWER_LEFT, geometry)
    right = derive_follower_setpoint(
        leader,
        FormationMode.VEE,
        Slot.FOLLOWER_RIGHT,
        geometry,
    )

    assert_setpoint_close(left, PositionYawSetpoint(14.0, 17.0, -5.0, pi / 2.0))
    assert_setpoint_close(right, PositionYawSetpoint(6.0, 17.0, -5.0, pi / 2.0))


def test_vee_follower_slots_rotate_with_leader_yaw_one_eighty_degrees():
    leader = PositionYawSetpoint(10.0, 20.0, -5.0, pi)
    geometry = FormationGeometry(lateral_spacing_m=4.0, trail_spacing_m=3.0)

    left = derive_follower_setpoint(leader, FormationMode.VEE, Slot.FOLLOWER_LEFT, geometry)
    right = derive_follower_setpoint(
        leader,
        FormationMode.VEE,
        Slot.FOLLOWER_RIGHT,
        geometry,
    )

    assert_setpoint_close(left, PositionYawSetpoint(13.0, 24.0, -5.0, pi))
    assert_setpoint_close(right, PositionYawSetpoint(13.0, 16.0, -5.0, pi))
