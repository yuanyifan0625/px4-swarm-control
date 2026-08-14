from math import isclose, pi

from px4_swarm_control.geometry import (
    body_offset_to_world,
    formation_body_offset,
    FormationGeometry,
    staging_setpoint,
)
from px4_swarm_control.models import (
    CommandStatus,
    default_vehicle_configs,
    FormationMode,
    MissionState,
    PositionYawSetpoint,
    Slot,
    VehicleCommandResult,
    VehicleConfig,
    VehicleLevelState,
    VehicleRole,
    VehicleState,
)


def assert_setpoint_close(actual, expected):
    assert isclose(actual.x, expected.x, abs_tol=1e-6)
    assert isclose(actual.y, expected.y, abs_tol=1e-6)
    assert isclose(actual.z, expected.z, abs_tol=1e-6)
    assert isclose(actual.yaw, expected.yaw, abs_tol=1e-6)


def test_internal_models_cover_vehicle_and_command_concepts():
    config = VehicleConfig(
        vehicle_id='MAV2',
        px4_namespace='/MAV2',
        role=VehicleRole.FOLLOWER,
        slot=Slot.FOLLOWER_LEFT,
    )
    state = VehicleState(
        vehicle_id=config.vehicle_id,
        position=(1.0, 2.0, -3.0),
        yaw=0.25,
        velocity=(0.1, 0.2, 0.3),
        armed=True,
        navigation_state='OFFBOARD',
        offboard_available=True,
        telemetry_age_s=0.05,
        vehicle_level_state=VehicleLevelState.FOLLOWING,
    )
    result = VehicleCommandResult(
        status=CommandStatus.ACCEPTED,
        message='queued',
    )

    assert config.role is VehicleRole.FOLLOWER
    assert config.slot is Slot.FOLLOWER_LEFT
    assert state.vehicle_level_state is VehicleLevelState.FOLLOWING
    assert result.status is CommandStatus.ACCEPTED
    assert MissionState.STAGING.value == 'staging'


def test_default_vehicle_configs_preserve_leader_left_right_assignments():
    configs = default_vehicle_configs()

    assert configs == (
        VehicleConfig('MAV1', '/MAV1', VehicleRole.LEADER, Slot.LEADER),
        VehicleConfig('MAV2', '/MAV2', VehicleRole.FOLLOWER, Slot.FOLLOWER_LEFT),
        VehicleConfig('MAV3', '/MAV3', VehicleRole.FOLLOWER, Slot.FOLLOWER_RIGHT),
    )


def test_vee_and_line_abreast_offsets_keep_left_right_signs():
    geometry = FormationGeometry(
        vee_lateral_spacing_m=0.4,
        vee_trail_spacing_m=0.6928,
        line_abreast_lateral_spacing_m=0.8,
    )

    assert formation_body_offset(FormationMode.VEE, Slot.FOLLOWER_LEFT, geometry) == (
        -0.6928,
        0.4,
        0.0,
    )
    assert formation_body_offset(FormationMode.VEE, Slot.FOLLOWER_RIGHT, geometry) == (
        -0.6928,
        -0.4,
        0.0,
    )
    assert formation_body_offset(
        FormationMode.LINE_ABREAST,
        Slot.FOLLOWER_LEFT,
        geometry,
    ) == (0.0, 0.8, 0.0)
    assert formation_body_offset(
        FormationMode.LINE_ABREAST,
        Slot.FOLLOWER_RIGHT,
        geometry,
    ) == (0.0, -0.8, 0.0)


def test_staging_positions_use_leader_initial_yaw_and_keep_left_slot_left():
    leader = PositionYawSetpoint(x=10.0, y=20.0, z=-5.0, yaw=pi / 2.0)
    geometry = FormationGeometry(
        vee_lateral_spacing_m=0.4,
        vee_trail_spacing_m=0.6928,
        line_abreast_lateral_spacing_m=0.8,
    )

    left = staging_setpoint(leader, Slot.FOLLOWER_LEFT, geometry)
    right = staging_setpoint(leader, Slot.FOLLOWER_RIGHT, geometry)

    assert_setpoint_close(
        left,
        PositionYawSetpoint(x=9.6, y=19.3072, z=-5.0, yaw=pi / 2.0),
    )
    assert_setpoint_close(
        right,
        PositionYawSetpoint(x=10.4, y=19.3072, z=-5.0, yaw=pi / 2.0),
    )


def test_body_frame_following_rotates_with_current_leader_yaw():
    leader = PositionYawSetpoint(x=10.0, y=20.0, z=-5.0, yaw=-pi / 2.0)
    geometry = FormationGeometry(
        vee_lateral_spacing_m=0.4,
        vee_trail_spacing_m=0.6928,
        line_abreast_lateral_spacing_m=0.8,
    )

    left_offset = formation_body_offset(FormationMode.VEE, Slot.FOLLOWER_LEFT, geometry)
    left_world = body_offset_to_world(leader, left_offset)

    assert_setpoint_close(
        left_world,
        PositionYawSetpoint(x=10.4, y=20.6928, z=-5.0, yaw=-pi / 2.0),
    )


def test_leader_slot_is_centered_for_staging_and_formations():
    leader = PositionYawSetpoint(x=1.0, y=2.0, z=-3.0, yaw=0.4)
    geometry = FormationGeometry(
        vee_lateral_spacing_m=0.4,
        vee_trail_spacing_m=0.6928,
        line_abreast_lateral_spacing_m=0.8,
    )

    assert formation_body_offset(FormationMode.VEE, Slot.LEADER, geometry) == (0.0, 0.0, 0.0)
    assert_setpoint_close(staging_setpoint(leader, Slot.LEADER, geometry), leader)
