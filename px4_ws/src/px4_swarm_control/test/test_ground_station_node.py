from math import isclose

from px4_swarm_control import ground_station_node
from px4_swarm_control.ground_station_node import (
    default_ground_station_config,
    GroundStationCore,
    GroundStationPublishers,
    GroundStationNode,
)
from px4_swarm_control.models import MissionState
from px4_swarm_interfaces.action import (
    ArmSwarm,
    ChangeFormation,
    LandSwarm,
    MoveLeader,
    PauseSwarm,
    TakeoffSwarm,
)
from px4_swarm_interfaces.msg import (
    FailsafeCommand,
    FormationMode,
    LeaderGoal,
    MissionCommand,
    VehicleSetpoint,
    VehicleStatus,
)
import rclpy


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, msg):
        self.infos.append(msg)

    def warning(self, msg):
        self.warnings.append(msg)


def make_core(now_stamp=None, now_s=None):
    vehicle_setpoints = {
        1: FakePublisher(),
        2: FakePublisher(),
        3: FakePublisher(),
    }
    publishers = GroundStationPublishers(
        mission_command=FakePublisher(),
        leader_goal=FakePublisher(),
        formation_mode=FakePublisher(),
        failsafe_command=FakePublisher(),
        vehicle_setpoints=vehicle_setpoints,
    )
    logger = FakeLogger()
    core = GroundStationCore(
        config=default_ground_station_config(),
        publishers=publishers,
        logger=logger,
        now_stamp=now_stamp or (lambda: object()),
        now_s=now_s or (lambda: 0.0),
    )
    return core, publishers, logger


def test_takeoff_action_starts_mission_without_reporting_success_before_staging():
    core, publishers, logger = make_core()
    publish_fresh_staging_anchor(core)
    request = TakeoffSwarm.Goal()
    request.altitude_m = 5.0
    request.timeout_sec = 30.0

    feedback = core.start_takeoff(request)

    assert core.mission_state is MissionState.TAKING_OFF
    assert core.takeoff_result() is None
    assert feedback.current_state == 'taking_off'
    assert feedback.vehicles_staged == 0
    assert feedback.total_vehicles == 3
    msg = publishers.mission_command.messages[-1]
    assert isinstance(msg, MissionCommand)
    assert msg.command == MissionCommand.TAKEOFF
    assert msg.reason == 'altitude_m=5.00 timeout_sec=30.00'
    leader = publishers.vehicle_setpoints[1].messages[-1]
    left = publishers.vehicle_setpoints[2].messages[-1]
    right = publishers.vehicle_setpoints[3].messages[-1]
    assert isinstance(leader, VehicleSetpoint)
    assert (leader.x, leader.y, leader.z, leader.yaw) == (0.0, 0.0, -5.0, 0.0)
    assert isclose(left.x, -1.0, abs_tol=1e-6)
    assert isclose(left.y, -1.0, abs_tol=1e-6)
    assert (left.z, left.yaw) == (-5.0, 0.0)
    assert isclose(right.x, -1.0, abs_tol=1e-6)
    assert isclose(right.y, 1.0, abs_tol=1e-6)
    assert (right.z, right.yaw) == (-5.0, 0.0)
    assert logger.infos[-1].startswith('swarm mission idle -> taking_off')


def test_takeoff_staging_uses_fresh_leader_position_and_yaw_as_anchor():
    core, publishers, _ = make_core()
    core.handle_vehicle_status(
        vehicle_status(
            1,
            x=10.0,
            y=20.0,
            z=0.2,
            yaw=1.5707963267948966,
            vehicle_state='landed',
        )
    )
    request = TakeoffSwarm.Goal()
    request.altitude_m = 1.5
    request.timeout_sec = 30.0

    core.start_takeoff(request)

    leader = publishers.vehicle_setpoints[1].messages[-1]
    left = publishers.vehicle_setpoints[2].messages[-1]
    right = publishers.vehicle_setpoints[3].messages[-1]
    assert isclose(leader.x, 10.0, abs_tol=1e-6)
    assert isclose(leader.y, 20.0, abs_tol=1e-6)
    assert isclose(leader.z, -1.3, abs_tol=1e-6)
    assert isclose(leader.yaw, 1.5707963267948966, abs_tol=1e-6)
    assert isclose(left.x, 11.0, abs_tol=1e-6)
    assert isclose(left.y, 19.0, abs_tol=1e-6)
    assert isclose(right.x, 9.0, abs_tol=1e-6)
    assert isclose(right.y, 19.0, abs_tol=1e-6)


def test_takeoff_rejects_when_leader_staging_anchor_is_unavailable():
    core, publishers, _ = make_core()
    request = TakeoffSwarm.Goal()
    request.altitude_m = 1.5
    request.timeout_sec = 30.0

    feedback = core.start_takeoff(request)
    result = core.takeoff_result()

    assert feedback.current_state == 'idle'
    assert result.success is False
    assert result.message == 'takeoff rejected: fresh MAV1 staging anchor unavailable'
    assert publishers.mission_command.messages == []
    assert all(not publisher.messages for publisher in publishers.vehicle_setpoints.values())


def test_takeoff_rejects_stale_leader_staging_anchor_without_publishing_commands():
    core, publishers, _ = make_core()
    core.handle_vehicle_status(
        vehicle_status(
            1,
            x=10.0,
            y=20.0,
            z=0.0,
            yaw=0.5,
            last_telemetry_age_sec=5.0,
            vehicle_state='landed',
        )
    )
    request = TakeoffSwarm.Goal()
    request.altitude_m = 1.5
    request.timeout_sec = 30.0

    core.start_takeoff(request)
    result = core.takeoff_result()

    assert result.success is False
    assert result.message == 'takeoff rejected: fresh MAV1 staging anchor unavailable'
    assert publishers.mission_command.messages == []
    assert all(not publisher.messages for publisher in publishers.vehicle_setpoints.values())


def test_takeoff_rejects_cached_anchor_after_status_delivery_stops():
    clock = [10.0]
    core, publishers, _ = make_core(now_s=lambda: clock[0])
    publish_fresh_staging_anchor(core)
    clock[0] = 12.0
    request = TakeoffSwarm.Goal()
    request.altitude_m = 1.5
    request.timeout_sec = 30.0

    core.start_takeoff(request)
    result = core.takeoff_result()

    assert result.success is False
    assert result.message == 'takeoff rejected: fresh MAV1 staging anchor unavailable'
    assert publishers.mission_command.messages == []
    assert all(not publisher.messages for publisher in publishers.vehicle_setpoints.values())


def test_default_ground_station_config_uses_small_field_operation_profile():
    config = default_ground_station_config()

    assert config.staging_lateral_spacing_m == 1.0
    assert config.staging_trail_spacing_m == 1.0
    assert config.formation_vee_lateral_spacing_m == 1.0
    assert config.formation_vee_trail_spacing_m == 1.0
    assert config.formation_line_abreast_lateral_spacing_m == 1.0
    assert config.formation_position_tolerance_m == 0.02


def test_arm_action_publishes_arm_command_without_staging_or_takeoff():
    core, publishers, _ = make_core()
    request = ArmSwarm.Goal()
    request.timeout_sec = 12.0

    feedback = core.start_arm(request)

    assert feedback.current_state == 'arming'
    assert feedback.vehicles_armed == 0
    assert feedback.total_vehicles == 3
    assert core.arm_result() is None
    assert publishers.mission_command.messages[-1].command == MissionCommand.ARM
    assert publishers.vehicle_setpoints[1].messages == []
    assert publishers.vehicle_setpoints[2].messages == []
    assert publishers.vehicle_setpoints[3].messages == []


def test_arm_action_rejects_while_paused_without_leaving_pause():
    core, publishers, _ = make_core()
    pause = PauseSwarm.Goal()
    pause.pause = True
    core.handle_pause(pause)
    request = ArmSwarm.Goal()
    request.timeout_sec = 12.0

    feedback = core.start_arm(request)
    result = core.arm_result()

    assert feedback.current_state == 'paused'
    assert result.success is False
    assert 'paused' in result.message
    assert core.mission_state is MissionState.PAUSED
    assert len(publishers.mission_command.messages) == 1
    assert publishers.mission_command.messages[-1].command == MissionCommand.PAUSE


def test_arm_action_rejects_offboard_signal_lost_without_publishing_arm_command():
    core, publishers, _ = make_core()
    core.handle_vehicle_status(
        vehicle_status(
            1,
            armed=False,
            nav_state='offboard',
            vehicle_state='landed',
            pre_flight_checks_pass=False,
            offboard_control_signal_lost=True,
        )
    )
    request = ArmSwarm.Goal()
    request.timeout_sec = 12.0

    feedback = core.start_arm(request)
    result = core.arm_result()

    assert feedback.current_state == 'idle'
    assert result.success is False
    assert result.message == (
        'arm rejected: MAV1 still in Offboard with lost offboard signal; '
        'wait for land-complete recovery'
    )
    assert publishers.mission_command.messages == []


def test_arm_action_rejects_stale_offboard_signal_lost_without_publishing_arm_command():
    core, publishers, _ = make_core()
    core.handle_vehicle_status(
        vehicle_status(
            1,
            armed=False,
            nav_state='offboard',
            vehicle_state='landed',
            pre_flight_checks_pass=False,
            offboard_control_signal_lost=True,
            last_telemetry_age_sec=5.0,
        )
    )
    request = ArmSwarm.Goal()
    request.timeout_sec = 12.0

    core.start_arm(request)
    result = core.arm_result()

    assert result.success is False
    assert result.message == (
        'arm rejected: MAV1 still in Offboard with lost offboard signal; '
        'wait for land-complete recovery'
    )
    assert publishers.mission_command.messages == []


def test_arm_action_succeeds_after_three_fresh_armed_statuses_while_landed():
    core, _, _ = make_core()
    request = ArmSwarm.Goal()
    request.timeout_sec = 12.0
    core.start_arm(request)

    core.handle_vehicle_status(
        vehicle_status(1, armed=True, nav_state='auto_loiter', vehicle_state='landed')
    )
    core.handle_vehicle_status(
        vehicle_status(2, armed=True, nav_state='auto_loiter', vehicle_state='landed')
    )

    assert core.arm_result() is None
    assert core.arm_feedback().vehicles_armed == 2

    core.handle_vehicle_status(
        vehicle_status(3, armed=True, nav_state='auto_loiter', vehicle_state='landed')
    )

    result = core.arm_result()
    assert result.success is True
    assert result.message == 'all vehicles reported armed'


def test_takeoff_action_result_succeeds_only_after_current_staging_completion():
    core, _, _ = make_core()
    publish_fresh_staging_anchor(core)
    request = TakeoffSwarm.Goal()
    request.altitude_m = 5.0
    request.timeout_sec = 30.0
    core.start_takeoff(request)

    assert core.takeoff_result() is None

    publish_staged_statuses(core)

    result = core.takeoff_result()
    assert result.success is True
    assert result.message == 'all vehicles reached staging positions'


def test_takeoff_timeout_pauses_every_vehicle_once_when_staging_never_completes():
    clock = [10.0]
    core, publishers, _ = make_core(now_s=lambda: clock[0])
    publish_fresh_staging_anchor(core)
    request = TakeoffSwarm.Goal()
    request.altitude_m = 5.0
    request.timeout_sec = 2.0
    core.start_takeoff(request)

    clock[0] = 12.1

    result = core.takeoff_result()
    assert result.success is False
    assert 'timed out' in result.message
    assert core.mission_state is MissionState.PAUSED
    assert publishers.mission_command.messages[-1].command == MissionCommand.PAUSE
    assert publishers.mission_command.messages[-1].reason == 'takeoff staging timed out'

    core.takeoff_result()
    assert [message.command for message in publishers.mission_command.messages].count(
        MissionCommand.PAUSE,
    ) == 1


def test_move_leader_action_publishes_world_frame_leader_goal_without_follower_targets():
    core, publishers, _ = make_core()
    publish_safe_following_statuses(core)
    request = MoveLeader.Goal()
    request.x = 1.0
    request.y = 2.0
    request.z = -3.0
    request.yaw = 0.75
    request.position_tolerance_m = 0.5
    request.yaw_tolerance_rad = 0.2
    request.timeout_sec = 12.0

    feedback = core.start_move_leader(request)

    assert core.mission_state is MissionState.FOLLOWING
    assert feedback.current_state == 'following'
    assert core.move_leader_result() is None
    msg = publishers.leader_goal.messages[-1]
    assert isinstance(msg, LeaderGoal)
    assert msg.frame_id == 'world'
    assert (msg.x, msg.y, msg.z, msg.yaw) == (1.0, 2.0, -3.0, 0.75)
    assert publishers.vehicle_setpoints[1].messages == []
    assert publishers.vehicle_setpoints[2].messages == []
    assert publishers.vehicle_setpoints[3].messages == []


def test_move_leader_rejects_goal_too_close_to_actual_follower_with_reason():
    core, publishers, _ = make_core()
    core.handle_vehicle_status(
        vehicle_status(1, x=0.0, y=0.0, slot='leader', vehicle_state='following')
    )
    core.handle_vehicle_status(
        vehicle_status(
            2,
            x=-1.0,
            y=1.0,
            slot='follower_left',
            vehicle_state='following',
        )
    )
    core.handle_vehicle_status(
        vehicle_status(
            3,
            x=-1.0,
            y=-1.0,
            slot='follower_right',
            vehicle_state='following',
        )
    )
    request = MoveLeader.Goal()
    request.x = -1.1
    request.y = 1.0
    request.z = -5.0
    request.yaw = 0.0
    request.position_tolerance_m = 0.5
    request.yaw_tolerance_rad = 0.2
    request.timeout_sec = 12.0

    feedback = core.start_move_leader(request)
    result = core.move_leader_result()

    assert feedback.current_state == 'idle'
    assert result.success is False
    assert 'MAV2 actual position' in result.message
    assert publishers.leader_goal.messages == []


def test_move_leader_result_requires_fresh_leader_status_within_tolerance():
    core, _, _ = make_core()
    publish_safe_following_statuses(core)
    request = MoveLeader.Goal()
    request.x = 1.0
    request.y = 2.0
    request.z = -3.0
    request.yaw = 0.75
    request.position_tolerance_m = 0.5
    request.yaw_tolerance_rad = 0.2
    request.timeout_sec = 12.0
    core.start_move_leader(request)

    stale_leader = vehicle_status(
        1,
        x=1.0,
        y=2.0,
        z=-3.0,
        yaw=0.75,
        last_telemetry_age_sec=5.0,
    )
    core.handle_vehicle_status(stale_leader)
    assert core.move_leader_result() is None

    follower_at_goal = vehicle_status(
        2,
        x=1.0,
        y=2.0,
        z=-3.0,
        yaw=0.75,
        last_telemetry_age_sec=0.1,
    )
    core.handle_vehicle_status(follower_at_goal)
    assert core.move_leader_result() is None

    leader_far = vehicle_status(
        1,
        x=1.4,
        y=2.4,
        z=-3.4,
        yaw=1.2,
        last_telemetry_age_sec=0.1,
    )
    core.handle_vehicle_status(leader_far)
    feedback = core.move_leader_feedback()
    assert feedback.remaining_distance_m > 0.0
    assert feedback.yaw_error_rad > 0.2
    assert core.move_leader_result() is None

    leader_reached = vehicle_status(
        1,
        x=1.1,
        y=1.9,
        z=-3.1,
        yaw=0.8,
        last_telemetry_age_sec=0.1,
    )
    core.handle_vehicle_status(leader_reached)

    result = core.move_leader_result()
    assert result.success is True
    assert result.message == 'leader reached target'


def test_move_leader_result_requires_leader_armed_and_offboard():
    core, _, _ = make_core()
    publish_safe_following_statuses(core)
    request = MoveLeader.Goal()
    request.x = 1.0
    request.y = 2.0
    request.z = -3.0
    request.yaw = 0.75
    request.position_tolerance_m = 0.5
    request.yaw_tolerance_rad = 0.2
    request.timeout_sec = 12.0
    core.start_move_leader(request)

    disarmed_at_goal = vehicle_status(
        1,
        x=1.0,
        y=2.0,
        z=-3.0,
        yaw=0.75,
        armed=False,
        nav_state='offboard',
    )
    core.handle_vehicle_status(disarmed_at_goal)
    assert core.move_leader_result() is None

    auto_at_goal = vehicle_status(
        1,
        x=1.0,
        y=2.0,
        z=-3.0,
        yaw=0.75,
        armed=True,
        nav_state='auto_loiter',
    )
    core.handle_vehicle_status(auto_at_goal)
    assert core.move_leader_result() is None

    offboard_at_goal = vehicle_status(
        1,
        x=1.0,
        y=2.0,
        z=-3.0,
        yaw=0.75,
        armed=True,
        nav_state='offboard',
    )
    core.handle_vehicle_status(offboard_at_goal)

    result = core.move_leader_result()
    assert result.success is True


def test_move_leader_rejects_non_finite_goal_and_non_positive_tolerance():
    core, publishers, _ = make_core()
    request = MoveLeader.Goal()
    request.x = float('nan')
    request.y = 2.0
    request.z = -3.0
    request.yaw = 0.75
    request.position_tolerance_m = 0.5
    request.yaw_tolerance_rad = 0.2
    request.timeout_sec = 12.0

    rejected = core.start_move_leader(request)

    assert rejected.current_state == 'error'
    result = core.move_leader_result()
    assert result.success is False
    assert 'invalid leader goal' in result.message
    assert publishers.leader_goal.messages == []

    core, publishers, _ = make_core()
    request.x = 1.0
    request.position_tolerance_m = 0.0

    rejected = core.start_move_leader(request)

    assert rejected.current_state == 'error'
    result = core.move_leader_result()
    assert result.success is False
    assert 'invalid leader goal' in result.message
    assert publishers.leader_goal.messages == []


def test_move_leader_result_times_out_before_leader_reaches_goal():
    clock = [10.0]
    core, _, _ = make_core(now_s=lambda: clock[0])
    publish_safe_following_statuses(core)
    request = MoveLeader.Goal()
    request.x = 1.0
    request.y = 2.0
    request.z = -3.0
    request.yaw = 0.75
    request.position_tolerance_m = 0.5
    request.yaw_tolerance_rad = 0.2
    request.timeout_sec = 2.0
    core.start_move_leader(request)

    clock[0] = 12.1

    result = core.move_leader_result()
    assert result.success is False
    assert 'timed out' in result.message


def test_takeoff_and_land_clear_pending_move_leader_goal_republish():
    core, publishers, _ = make_core()
    publish_safe_following_statuses(core)
    move = MoveLeader.Goal()
    move.x = 1.0
    move.y = 2.0
    move.z = -3.0
    move.yaw = 0.75
    move.position_tolerance_m = 0.5
    move.yaw_tolerance_rad = 0.2
    move.timeout_sec = 12.0
    core.start_move_leader(move)
    assert len(publishers.leader_goal.messages) == 1

    takeoff = TakeoffSwarm.Goal()
    takeoff.altitude_m = 5.0
    takeoff.timeout_sec = 30.0
    core.start_takeoff(takeoff)
    core.republish_leader_goal()

    assert len(publishers.leader_goal.messages) == 1

    publish_safe_following_statuses(core)
    core.start_move_leader(move)
    assert len(publishers.leader_goal.messages) == 2

    land = LandSwarm.Goal()
    land.timeout_sec = 30.0
    core.start_land(land)
    core.republish_leader_goal()

    assert len(publishers.leader_goal.messages) == 2


def test_change_formation_rejects_unknown_mode_and_non_following_state():
    core, publishers, _ = make_core()
    request = ChangeFormation.Goal()
    request.formation_mode = FormationMode.LINE_ABREAST
    request.timeout_sec = 8.0

    rejected_state = core.start_change_formation(request)

    assert rejected_state.current_state == 'error'
    result = core.change_formation_result()
    assert result.success is False
    assert 'requires following' in result.message
    assert publishers.formation_mode.messages == []

    bad = ChangeFormation.Goal()
    bad.formation_mode = 'circle'
    core, publishers, _ = make_core()
    core.mission_state = MissionState.FOLLOWING

    rejected = core.start_change_formation(bad)

    assert rejected.current_state == 'error'
    result = core.change_formation_result()
    assert result.success is False
    assert 'unsupported formation mode' in result.message
    assert core.mission_state is MissionState.ERROR
    assert publishers.formation_mode.messages == []


def test_change_formation_broadcasts_mode_without_follower_absolute_targets():
    core, publishers, _ = make_core()
    core.mission_state = MissionState.FOLLOWING
    request = ChangeFormation.Goal()
    request.formation_mode = FormationMode.LINE_ABREAST
    request.timeout_sec = 8.0

    feedback = core.start_change_formation(request)

    assert feedback.current_state == 'reconfiguring'
    assert feedback.active_formation == FormationMode.LINE_ABREAST
    assert feedback.progress == 0.0
    assert core.change_formation_result() is None
    assert core.mission_state is MissionState.RECONFIGURING
    msg = publishers.formation_mode.messages[-1]
    assert isinstance(msg, FormationMode)
    assert msg.mode == FormationMode.LINE_ABREAST
    assert publishers.vehicle_setpoints[1].messages == []
    assert publishers.vehicle_setpoints[2].messages == []
    assert publishers.vehicle_setpoints[3].messages == []

    core.republish_formation_mode()

    assert len(publishers.formation_mode.messages) == 2
    assert publishers.vehicle_setpoints[1].messages == []
    assert publishers.vehicle_setpoints[2].messages == []
    assert publishers.vehicle_setpoints[3].messages == []


def test_change_formation_progress_and_success_wait_for_followers_inside_tolerance():
    core, _, logger = make_core()
    core.mission_state = MissionState.FOLLOWING
    request = ChangeFormation.Goal()
    request.formation_mode = FormationMode.LINE_ABREAST
    request.timeout_sec = 8.0
    core.start_change_formation(request)

    core.handle_vehicle_status(
        vehicle_status(1, x=10.0, y=20.0, z=-5.0, yaw=0.0, vehicle_state='following'),
    )
    core.handle_vehicle_status(
        vehicle_status(
            2,
            x=10.0,
            y=19.0,
                z=-5.0,
            yaw=0.1,
            slot='follower_left',
            vehicle_state='following',
        ),
    )
    core.handle_vehicle_status(
        vehicle_status(
            3,
            x=8.0,
            y=19.0,
            z=-5.0,
            yaw=0.0,
            slot='follower_right',
            vehicle_state='following',
        ),
    )

    feedback = core.change_formation_feedback()

    assert feedback.current_state == 'reconfiguring'
    assert feedback.progress == 0.5
    assert core.change_formation_result() is None

    core.handle_vehicle_status(
        vehicle_status(
            3,
            x=10.0,
            y=21.0,
            z=-5.0,
            yaw=0.19,
            slot='follower_right',
            vehicle_state='following',
        ),
    )

    result = core.change_formation_result()
    assert result.success is True
    assert result.message == 'formation established'
    assert core.mission_state is MissionState.FOLLOWING
    assert 'formation established' in logger.infos


def test_change_formation_rejects_observed_loose_vee_triangle():
    core, _, _ = make_core()
    core.mission_state = MissionState.FOLLOWING
    request = ChangeFormation.Goal()
    request.formation_mode = FormationMode.VEE
    request.timeout_sec = 8.0
    core.start_change_formation(request)

    core.handle_vehicle_status(
        vehicle_status(1, x=0.97, y=-0.02, z=-1.5, yaw=0.0, vehicle_state='following'),
    )
    core.handle_vehicle_status(
        vehicle_status(
            2,
            x=0.43,
            y=0.50,
            z=-1.5,
            slot='follower_left',
            vehicle_state='following',
        ),
    )
    core.handle_vehicle_status(
        vehicle_status(
            3,
            x=0.50,
            y=-0.47,
            z=-1.5,
            slot='follower_right',
            vehicle_state='following',
        ),
    )

    assert core.change_formation_feedback().progress == 0.0
    assert core.change_formation_result() is None


def test_change_formation_rejects_stale_status_and_wrong_follower_slots():
    core, _, _ = make_core()
    core.mission_state = MissionState.FOLLOWING
    request = ChangeFormation.Goal()
    request.formation_mode = FormationMode.LINE_ABREAST
    request.timeout_sec = 8.0
    core.start_change_formation(request)

    core.handle_vehicle_status(
        vehicle_status(
            1,
            x=10.0,
            y=20.0,
            z=-5.0,
            yaw=0.0,
            last_telemetry_age_sec=5.0,
            vehicle_state='following',
        ),
    )
    core.handle_vehicle_status(
        vehicle_status(
            2,
            x=10.0,
            y=21.0,
            z=-5.0,
            yaw=0.0,
            slot='follower_left',
            vehicle_state='following',
        ),
    )
    core.handle_vehicle_status(
        vehicle_status(
            3,
            x=10.0,
            y=19.0,
            z=-5.0,
            yaw=0.0,
            slot='follower_right',
            vehicle_state='following',
        ),
    )

    assert core.change_formation_feedback().progress == 0.0
    assert core.change_formation_result() is None

    core.handle_vehicle_status(
        vehicle_status(1, x=10.0, y=20.0, z=-5.0, yaw=0.0, vehicle_state='following'),
    )
    core.handle_vehicle_status(
        vehicle_status(
            2,
            x=10.0,
            y=21.0,
            z=-5.0,
            yaw=0.0,
            slot='follower_right',
            vehicle_state='following',
        ),
    )

    assert core.change_formation_feedback().progress == 0.5
    assert core.change_formation_result() is None


def test_change_formation_result_times_out_when_formation_never_establishes():
    clock = [10.0]
    core, _, _ = make_core(now_s=lambda: clock[0])
    core.mission_state = MissionState.FOLLOWING
    request = ChangeFormation.Goal()
    request.formation_mode = FormationMode.LINE_ABREAST
    request.timeout_sec = 2.0
    core.start_change_formation(request)

    clock[0] = 12.1

    result = core.change_formation_result()
    assert result.success is False
    assert 'timed out' in result.message


def test_pause_action_publishes_pause_or_resume_mission_command():
    core, publishers, _ = make_core()
    pause = PauseSwarm.Goal()
    pause.pause = True
    pause.reason = 'operator check'

    paused = core.handle_pause(pause)

    assert paused.result.success is True
    assert paused.feedback.paused is True
    assert core.mission_state is MissionState.PAUSED
    assert publishers.mission_command.messages[-1].command == MissionCommand.PAUSE
    assert publishers.mission_command.messages[-1].reason == 'operator check'
    assert publishers.failsafe_command.messages[-1].active is True
    assert publishers.failsafe_command.messages[-1].action == FailsafeCommand.HOVER

    resume = PauseSwarm.Goal()
    resume.pause = False
    resumed = core.handle_pause(resume)

    assert resumed.result.success is True
    assert resumed.feedback.paused is False
    assert core.mission_state is MissionState.HOLDING
    assert publishers.mission_command.messages[-1].command == MissionCommand.RESUME
    assert publishers.failsafe_command.messages[-1].active is False


def test_resume_clears_old_move_and_formation_without_republishing_stale_commands():
    core, publishers, _ = make_core()
    publish_safe_following_statuses(core)
    move = MoveLeader.Goal()
    move.x = 4.0
    move.y = 0.0
    move.z = -5.0
    move.yaw = 0.0
    move.position_tolerance_m = 0.5
    move.yaw_tolerance_rad = 0.2
    move.timeout_sec = 30.0
    core.start_move_leader(move)
    assert len(publishers.leader_goal.messages) == 1

    pause = PauseSwarm.Goal()
    pause.pause = True
    core.handle_pause(pause)
    resume = PauseSwarm.Goal()
    resume.pause = False
    core.handle_pause(resume)
    core.republish_leader_goal()
    core.republish_formation_mode()

    assert core.mission_state is MissionState.HOLDING
    assert len(publishers.leader_goal.messages) == 1
    assert publishers.formation_mode.messages == []


def test_paused_state_rejects_move_leader_and_change_formation_without_leaving_pause():
    core, publishers, _ = make_core()
    pause = PauseSwarm.Goal()
    pause.pause = True
    core.handle_pause(pause)
    paused_status = vehicle_status(1, x=3.0, y=2.0, z=-5.0)
    core.handle_vehicle_status(paused_status)

    move = MoveLeader.Goal()
    move.x = 4.0
    move.y = 0.0
    move.z = -5.0
    move.yaw = 0.0
    move.position_tolerance_m = 0.5
    move.yaw_tolerance_rad = 0.2
    move.timeout_sec = 30.0

    move_feedback = core.start_move_leader(move)
    move_result = core.move_leader_result()

    assert move_feedback.current_state == 'paused'
    assert move_result.success is False
    assert 'paused' in move_result.message
    assert core.mission_state is MissionState.PAUSED
    assert core.vehicle_statuses[1] is paused_status
    assert publishers.leader_goal.messages == []

    change = ChangeFormation.Goal()
    change.formation_mode = FormationMode.LINE_ABREAST
    change.timeout_sec = 30.0

    change_feedback = core.start_change_formation(change)
    change_result = core.change_formation_result()

    assert change_feedback.current_state == 'paused'
    assert change_result.success is False
    assert 'paused' in change_result.message
    assert core.mission_state is MissionState.PAUSED
    assert core.vehicle_statuses[1] is paused_status
    assert publishers.formation_mode.messages == []


def test_paused_state_rejects_takeoff_without_leaving_pause():
    core, publishers, _ = make_core()
    pause = PauseSwarm.Goal()
    pause.pause = True
    core.handle_pause(pause)
    takeoff = TakeoffSwarm.Goal()
    takeoff.altitude_m = 5.0
    takeoff.timeout_sec = 20.0

    feedback = core.start_takeoff(takeoff)
    result = core.takeoff_result()

    assert feedback.current_state == 'paused'
    assert result.success is False
    assert 'paused' in result.message
    assert core.mission_state is MissionState.PAUSED
    assert len(publishers.mission_command.messages) == 1
    assert publishers.mission_command.messages[-1].command == MissionCommand.PAUSE
    assert publishers.vehicle_setpoints[1].messages == []
    assert publishers.vehicle_setpoints[2].messages == []
    assert publishers.vehicle_setpoints[3].messages == []


def test_land_action_starts_mission_without_reporting_success_before_landed():
    core, publishers, _ = make_core()
    request = LandSwarm.Goal()
    request.timeout_sec = 20.0

    feedback = core.start_land(request)

    assert feedback.current_state == 'landing'
    assert core.land_result() is None
    assert core.mission_state is MissionState.LANDING
    mission = publishers.mission_command.messages[-1]
    assert mission.command == MissionCommand.LAND
    assert publishers.failsafe_command.messages == []


def test_land_action_remains_available_from_paused_and_failsafe_states():
    for initial_state in (MissionState.PAUSED, MissionState.FAILSAFE):
        core, publishers, _ = make_core()
        core.mission_state = initial_state
        request = LandSwarm.Goal()
        request.timeout_sec = 20.0

        feedback = core.start_land(request)

        assert feedback.current_state == 'landing'
        assert core.mission_state is MissionState.LANDING
        assert publishers.mission_command.messages[-1].command == MissionCommand.LAND


def test_land_action_result_succeeds_only_after_current_landed_statuses():
    core, _, _ = make_core()
    request = LandSwarm.Goal()
    request.timeout_sec = 20.0
    core.start_land(request)

    assert core.land_result() is None

    for vehicle_id in (1, 2, 3):
        landed = VehicleStatus()
        landed.vehicle_id = vehicle_id
        landed.vehicle_state = 'landed'
        landed.last_telemetry_age_sec = 0.1
        core.handle_vehicle_status(landed)

    result = core.land_result()
    assert result.success is True
    assert result.message == 'all vehicles reported landed'


def test_land_action_result_times_out_when_landing_never_completes():
    clock = [10.0]
    core, _, _ = make_core(now_s=lambda: clock[0])
    request = LandSwarm.Goal()
    request.timeout_sec = 2.0
    core.start_land(request)

    clock[0] = 12.1

    result = core.land_result()
    assert result.success is False
    assert 'timed out' in result.message


def test_vehicle_status_updates_swarm_status_summary():
    core, _, _ = make_core()
    status = VehicleStatus()
    status.vehicle_id = 2
    status.role = 'follower'
    status.px4_namespace = '/MAV2'
    status.vehicle_state = 'holding'
    status.last_telemetry_age_sec = 0.25

    core.handle_vehicle_status(status)

    assert core.vehicle_statuses[2].px4_namespace == '/MAV2'
    assert core.vehicle_statuses[2].vehicle_state == 'holding'
    assert isclose(core.vehicle_statuses[2].last_telemetry_age_sec, 0.25)


def test_vehicle_status_updates_mission_state_for_failsafe_and_all_landed():
    core, _, _ = make_core()
    failsafe = VehicleStatus()
    failsafe.vehicle_id = 2
    failsafe.vehicle_state = 'failsafe'

    core.handle_vehicle_status(failsafe)

    assert core.mission_state is MissionState.FAILSAFE

    core, _, _ = make_core()
    for vehicle_id in (1, 2, 3):
        landed = VehicleStatus()
        landed.vehicle_id = vehicle_id
        landed.vehicle_state = 'landed'
        core.handle_vehicle_status(landed)

    assert core.mission_state is MissionState.IDLE


def test_takeoff_ignores_cached_landed_statuses_from_previous_mission():
    core, _, _ = make_core()
    for vehicle_id in (1, 2, 3):
        landed = VehicleStatus()
        landed.vehicle_id = vehicle_id
        landed.vehicle_state = 'landed'
        core.handle_vehicle_status(landed)

    request = TakeoffSwarm.Goal()
    request.altitude_m = 5.0
    request.timeout_sec = 30.0
    core.start_takeoff(request)

    assert core.mission_state is MissionState.TAKING_OFF
    assert core.vehicle_statuses == {}
    assert core.takeoff_result() is None


def test_second_takeoff_after_land_uses_new_fresh_leader_anchor_without_restart():
    core, publishers, _ = make_core()
    publish_fresh_staging_anchor(core)
    takeoff = TakeoffSwarm.Goal()
    takeoff.altitude_m = 1.5
    takeoff.timeout_sec = 30.0
    core.start_takeoff(takeoff)
    publish_staged_statuses(core, altitude_m=1.5)
    assert core.takeoff_result().success is True

    land = LandSwarm.Goal()
    land.timeout_sec = 30.0
    core.start_land(land)
    for vehicle_id in (1, 2, 3):
        core.handle_vehicle_status(
            vehicle_status(
                vehicle_id,
                x=2.0 if vehicle_id == 1 else 0.0,
                y=3.0 if vehicle_id == 1 else 0.0,
                z=0.1,
                yaw=0.5 if vehicle_id == 1 else 0.0,
                vehicle_state='landed',
            )
        )
    assert core.land_result().success is True

    core.start_takeoff(takeoff)

    second_leader_target = publishers.vehicle_setpoints[1].messages[-1]
    assert core.mission_state is MissionState.TAKING_OFF
    assert (second_leader_target.x, second_leader_target.y) == (2.0, 3.0)
    assert isclose(second_leader_target.z, -1.4, abs_tol=1e-6)
    assert isclose(second_leader_target.yaw, 0.5, abs_tol=1e-6)


def test_land_ignores_cached_landed_statuses_until_current_mission_reports_land():
    core, _, _ = make_core()
    for vehicle_id in (1, 2, 3):
        landed = VehicleStatus()
        landed.vehicle_id = vehicle_id
        landed.vehicle_state = 'landed'
        core.handle_vehicle_status(landed)

    request = LandSwarm.Goal()
    request.timeout_sec = 30.0
    core.start_land(request)

    assert core.mission_state is MissionState.LANDING
    assert core.vehicle_statuses == {}
    assert core.land_result() is None


def test_land_does_not_complete_from_stale_landed_statuses():
    core, _, _ = make_core()
    request = LandSwarm.Goal()
    request.timeout_sec = 30.0
    core.start_land(request)

    for vehicle_id in (1, 2, 3):
        landed = VehicleStatus()
        landed.vehicle_id = vehicle_id
        landed.vehicle_state = 'landed'
        landed.last_telemetry_age_sec = 5.0
        core.handle_vehicle_status(landed)

    assert core.mission_state is MissionState.LANDING
    assert core.land_result() is None


def test_republish_staging_setpoints_resends_current_targets_for_all_vehicles():
    core, publishers, _ = make_core()
    publish_fresh_staging_anchor(core)
    request = TakeoffSwarm.Goal()
    request.altitude_m = 5.0
    request.timeout_sec = 30.0
    core.start_takeoff(request)

    core.republish_staging_setpoints()

    assert len(publishers.vehicle_setpoints[1].messages) == 2
    assert len(publishers.vehicle_setpoints[2].messages) == 2
    assert len(publishers.vehicle_setpoints[3].messages) == 2
    left = (
        publishers.vehicle_setpoints[2].messages[-1].x,
        publishers.vehicle_setpoints[2].messages[-1].y,
        publishers.vehicle_setpoints[2].messages[-1].z,
    )
    assert isclose(left[0], -1.0, abs_tol=1e-6)
    assert isclose(left[1], -1.0, abs_tol=1e-6)
    assert left[2] == -5.0


def test_republish_takeoff_request_resends_staging_targets_and_takeoff_command():
    core, publishers, _ = make_core()
    publish_fresh_staging_anchor(core)
    request = TakeoffSwarm.Goal()
    request.altitude_m = 5.0
    request.timeout_sec = 30.0
    core.start_takeoff(request)

    core.republish_takeoff_request()

    assert len(publishers.vehicle_setpoints[1].messages) == 2
    assert len(publishers.vehicle_setpoints[2].messages) == 2
    assert len(publishers.vehicle_setpoints[3].messages) == 2
    assert len(publishers.mission_command.messages) == 2
    assert publishers.mission_command.messages[-1].command == MissionCommand.TAKEOFF


def test_vehicle_status_detects_all_staged_and_logs_progress_once():
    core, _, logger = make_core()
    publish_fresh_staging_anchor(core)
    request = TakeoffSwarm.Goal()
    request.altitude_m = 5.0
    core.start_takeoff(request)

    for vehicle_id, point in (
        (1, (0.1, 0.1, -5.0)),
        (2, (-1.0, -1.0, -5.0)),
        (3, (-1.0, 1.0, -5.0)),
    ):
        status = VehicleStatus()
        status.vehicle_id = vehicle_id
        status.x, status.y, status.z = point
        status.armed = True
        status.nav_state = 'offboard'
        status.offboard_available = True
        status.last_telemetry_age_sec = 0.1
        status.vehicle_state = 'holding'
        core.handle_vehicle_status(status)

    assert core.mission_state is MissionState.STAGING
    assert logger.infos.count('all vehicles reached staging positions') == 1

    status = VehicleStatus()
    status.vehicle_id = 3
    status.x, status.y, status.z = (-1.0, -1.0, -5.0)
    status.armed = True
    status.nav_state = 'offboard'
    status.offboard_available = True
    status.last_telemetry_age_sec = 0.1
    status.vehicle_state = 'holding'
    core.handle_vehicle_status(status)

    assert logger.infos.count('all vehicles reached staging positions') == 1


def test_vehicle_status_does_not_mark_staged_without_armed_telemetry_and_offboard_ready():
    core, _, logger = make_core()
    publish_fresh_staging_anchor(core)
    request = TakeoffSwarm.Goal()
    request.altitude_m = 5.0
    core.start_takeoff(request)

    for vehicle_id, point in (
        (1, (0.0, 0.0, -5.0)),
        (2, (-1.0, -1.0, -5.0)),
        (3, (-1.0, 1.0, -5.0)),
    ):
        status = VehicleStatus()
        status.vehicle_id = vehicle_id
        status.x, status.y, status.z = point
        status.vehicle_state = 'holding'
        status.last_telemetry_age_sec = float('inf')
        core.handle_vehicle_status(status)

    assert core.mission_state is MissionState.TAKING_OFF
    assert 'all vehicles reached staging positions' not in logger.infos


def test_vehicle_status_does_not_mark_staged_with_stale_finite_telemetry_age():
    core, _, logger = make_core()
    publish_fresh_staging_anchor(core)
    request = TakeoffSwarm.Goal()
    request.altitude_m = 5.0
    core.start_takeoff(request)

    for vehicle_id, point in (
        (1, (0.0, 0.0, -5.0)),
        (2, (-1.0, -1.0, -5.0)),
        (3, (-1.0, 1.0, -5.0)),
    ):
        status = VehicleStatus()
        status.vehicle_id = vehicle_id
        status.x, status.y, status.z = point
        status.armed = True
        status.nav_state = 'offboard'
        status.offboard_available = True
        status.last_telemetry_age_sec = 5.0
        status.vehicle_state = 'staging'
        core.handle_vehicle_status(status)

    assert core.mission_state is MissionState.TAKING_OFF
    assert 'all vehicles reached staging positions' not in logger.infos


def test_ground_station_node_starts_under_swarm_namespace_with_actions(monkeypatch):
    action_servers = []

    class FakeActionServer:
        def __init__(self, node, action_type, name, execute_callback, **kwargs):
            self.node = node
            self.action_type = action_type
            self.name = name
            self.execute_callback = execute_callback
            self.callback_group = kwargs.get('callback_group')
            action_servers.append(self)

    monkeypatch.setattr(ground_station_node, 'ActionServer', FakeActionServer)
    rclpy.init()
    node = GroundStationNode()
    try:
        assert node.get_namespace() == '/swarm'
        assert [server.name for server in action_servers] == [
            'arm',
            'takeoff',
            'move_leader',
            'change_formation',
            'pause',
            'land',
        ]
        assert all(server.callback_group is node.callback_group for server in action_servers)
        assert node.resolve_topic_name('mission_command') == '/swarm/mission_command'
        assert node.resolve_topic_name('leader_goal') == '/swarm/leader_goal'
        assert node.resolve_topic_name('formation_mode') == '/swarm/formation_mode'
        assert node.resolve_topic_name('failsafe_command') == '/swarm/failsafe_command'
        assert node.resolve_topic_name('/MAV1/staging_setpoint') == (
            '/MAV1/staging_setpoint'
        )
        assert sorted(node.core.publishers.vehicle_setpoints) == [1, 2, 3]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def publish_staged_statuses(core, *, altitude_m=5.0):
    for vehicle_id, point in (
        (1, (0.0, 0.0, -altitude_m)),
        (2, (-1.0, -1.0, -altitude_m)),
        (3, (-1.0, 1.0, -altitude_m)),
    ):
        status = VehicleStatus()
        status.vehicle_id = vehicle_id
        status.x, status.y, status.z = point
        status.armed = True
        status.nav_state = 'offboard'
        status.offboard_available = True
        status.last_telemetry_age_sec = 0.1
        status.vehicle_state = 'staging'
        core.handle_vehicle_status(status)


def publish_fresh_staging_anchor(core, *, x=0.0, y=0.0, z=0.0, yaw=0.0):
    core.handle_vehicle_status(
        vehicle_status(
            1,
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            slot='leader',
            vehicle_state='landed',
        )
    )


def publish_safe_following_statuses(core):
    for vehicle_id, point, slot in (
        (1, (0.0, 0.0, -5.0), 'leader'),
        (2, (-1.0, -1.0, -5.0), 'follower_left'),
        (3, (-1.0, 1.0, -5.0), 'follower_right'),
    ):
        core.handle_vehicle_status(
            vehicle_status(
                vehicle_id,
                x=point[0],
                y=point[1],
                z=point[2],
                slot=slot,
                vehicle_state='following',
            )
        )


def vehicle_status(
    vehicle_id,
    *,
    x=0.0,
    y=0.0,
    z=-5.0,
    yaw=0.0,
    armed=True,
    nav_state='offboard',
    last_telemetry_age_sec=0.1,
    slot='',
    vehicle_state='staging',
    pre_flight_checks_pass=True,
    offboard_control_signal_lost=False,
):
    status = VehicleStatus()
    status.vehicle_id = vehicle_id
    status.x = x
    status.y = y
    status.z = z
    status.yaw = yaw
    status.armed = armed
    status.nav_state = nav_state
    status.offboard_available = True
    status.pre_flight_checks_pass = pre_flight_checks_pass
    status.offboard_control_signal_lost = offboard_control_signal_lost
    status.last_telemetry_age_sec = last_telemetry_age_sec
    status.slot = slot
    status.vehicle_state = vehicle_state
    return status
