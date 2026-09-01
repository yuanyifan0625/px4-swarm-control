from math import pi

import pytest

from px4_swarm_control.frame_transform import CoordinateProfile
def test_gazebo_enu_profile_converts_mav2_raw_pose_to_common_world_and_back():
    profile = CoordinateProfile.gazebo_enu_common_world(
        origin_enu=(-1.0, 1.0, 0.0),
    )

    common = profile.raw_to_common_pose(
        position=(2.0, 3.0, -0.5),
        yaw=pi / 2.0,
    )

    assert common.position == (2.0, 3.0, 0.5)
    assert common.yaw == pytest.approx(0.102)
    assert profile.common_to_raw_pose(
        position=common.position,
        yaw=common.yaw,
    ).position == pytest.approx((2.0, 3.0, -0.5))


def test_gazebo_enu_profile_wraps_yaw_and_raw_profile_is_identity():
    gazebo = CoordinateProfile.gazebo_enu_common_world(origin_enu=(0.0, 0.0, 0.0))
    raw = CoordinateProfile.raw_px4_local()

    assert gazebo.raw_to_common_pose((0.0, 0.0, 0.0), pi).yaw == pytest.approx(
        -pi / 2.0 + 0.102,
    )
    assert raw.raw_to_common_pose((1.0, 2.0, 3.0), -0.7).position == (1.0, 2.0, 3.0)
    assert raw.common_to_raw_pose((1.0, 2.0, 3.0), -0.7).yaw == -0.7


def test_gazebo_enu_profile_maps_each_validated_spawn_origin_to_its_common_origin():
    for origin in ((0.0, 0.0, 0.0), (-1.0, 1.0, 0.0), (-1.0, -1.0, 0.0)):
        profile = CoordinateProfile.gazebo_enu_common_world(origin_enu=origin)
        assert profile.raw_to_common_pose((0.0, 0.0, 0.0), 0.0).position == origin


def test_coordinate_profile_rejects_non_finite_values_and_unknown_names():
    with pytest.raises(ValueError, match='origin_enu'):
        CoordinateProfile.gazebo_enu_common_world(origin_enu=(0.0, float('nan'), 0.0))
    with pytest.raises(ValueError, match='unsupported coordinate profile'):
        CoordinateProfile(name='unknown').raw_to_common_pose((0.0, 0.0, 0.0), 0.0)
