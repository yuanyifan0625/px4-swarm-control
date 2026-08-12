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
    assert (left.x, left.y, left.z, left.yaw) == (-3.0, 4.0, -5.0, 0.0)
    assert (right.x, right.y, right.z, right.yaw) == (-3.0, -4.0, -5.0, 0.0)
    assert logger.infos[-1].startswith('swarm mission idle -> taking_off')


def test_takeoff_action_result_succeeds_only_after_current_staging_completion():
    core, _, _ = make_core()
    request = TakeoffSwarm.Goal()
    request.altitude_m = 5.0
    request.timeout_sec = 30.0
    core.start_takeoff(request)

    assert core.takeoff_result() is None

    publish_staged_statuses(core)

    result = core.takeoff_result()
    assert result.success is True
    assert result.message == 'all vehicles reached staging positions'


def test_takeoff_action_result_times_out_when_staging_never_completes():
    clock = [10.0]
    core, _, _ = make_core(now_s=lambda: clock[0])
    request = TakeoffSwarm.Goal()
    request.altitude_m = 5.0
    request.timeout_sec = 2.0
    core.start_takeoff(request)

    clock[0] = 12.1

    result = core.takeoff_result()
    assert result.success is False
    assert 'timed out' in result.message


def test_move_leader_action_publishes_world_frame_leader_goal():
    core, publishers, _ = make_core()
    request = MoveLeader.Goal()
    request.x = 1.0
    request.y = 2.0
    request.z = -3.0
    request.yaw = 0.75
    request.timeout_sec = 12.0

    outcome = core.handle_move_leader(request)

    assert core.mission_state is MissionState.FOLLOWING
    assert outcome.result.success is True
    assert outcome.feedback.current_state == 'following'
    msg = publishers.leader_goal.messages[-1]
    assert isinstance(msg, LeaderGoal)
    assert msg.frame_id == 'world'
    assert (msg.x, msg.y, msg.z, msg.yaw) == (1.0, 2.0, -3.0, 0.75)


def test_change_formation_accepts_supported_mode_and_rejects_unknown_mode():
    core, publishers, _ = make_core()
    request = ChangeFormation.Goal()
    request.formation_mode = FormationMode.LINE_ABREAST
    request.timeout_sec = 8.0

    accepted = core.handle_change_formation(request)

    assert accepted.result.success is True
    assert accepted.feedback.active_formation == FormationMode.LINE_ABREAST
    assert core.mission_state is MissionState.RECONFIGURING
    msg = publishers.formation_mode.messages[-1]
    assert isinstance(msg, FormationMode)
    assert msg.mode == FormationMode.LINE_ABREAST

    bad = ChangeFormation.Goal()
    bad.formation_mode = 'circle'
    rejected = core.handle_change_formation(bad)

    assert rejected.result.success is False
    assert rejected.feedback.current_state == 'error'
    assert core.mission_state is MissionState.ERROR
    assert len(publishers.formation_mode.messages) == 1


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
    assert core.mission_state is MissionState.IDLE
    assert publishers.mission_command.messages[-1].command == MissionCommand.RESUME
    assert publishers.failsafe_command.messages[-1].active is False


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
    status.px4_namespace = '/vehicle_2'
    status.vehicle_state = 'holding'
    status.last_telemetry_age_sec = 0.25

    core.handle_vehicle_status(status)

    assert core.vehicle_statuses[2].px4_namespace == '/vehicle_2'
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
    request = TakeoffSwarm.Goal()
    request.altitude_m = 5.0
    request.timeout_sec = 30.0
    core.start_takeoff(request)

    core.republish_staging_setpoints()

    assert len(publishers.vehicle_setpoints[1].messages) == 2
    assert len(publishers.vehicle_setpoints[2].messages) == 2
    assert len(publishers.vehicle_setpoints[3].messages) == 2
    assert (
        publishers.vehicle_setpoints[2].messages[-1].x,
        publishers.vehicle_setpoints[2].messages[-1].y,
        publishers.vehicle_setpoints[2].messages[-1].z,
    ) == (-3.0, 4.0, -5.0)


def test_vehicle_status_detects_all_staged_and_logs_progress_once():
    core, _, logger = make_core()
    request = TakeoffSwarm.Goal()
    request.altitude_m = 5.0
    core.start_takeoff(request)

    for vehicle_id, point in (
        (1, (0.1, 0.1, -5.0)),
        (2, (-3.1, 4.1, -5.0)),
        (3, (-3.1, -4.1, -5.0)),
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
    status.x, status.y, status.z = (-3.0, -4.0, -5.0)
    status.armed = True
    status.nav_state = 'offboard'
    status.offboard_available = True
    status.last_telemetry_age_sec = 0.1
    status.vehicle_state = 'holding'
    core.handle_vehicle_status(status)

    assert logger.infos.count('all vehicles reached staging positions') == 1


def test_vehicle_status_does_not_mark_staged_without_armed_telemetry_and_offboard_ready():
    core, _, logger = make_core()
    request = TakeoffSwarm.Goal()
    request.altitude_m = 5.0
    core.start_takeoff(request)

    for vehicle_id, point in (
        (1, (0.0, 0.0, -5.0)),
        (2, (-3.0, 4.0, -5.0)),
        (3, (-3.0, -4.0, -5.0)),
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
    request = TakeoffSwarm.Goal()
    request.altitude_m = 5.0
    core.start_takeoff(request)

    for vehicle_id, point in (
        (1, (0.0, 0.0, -5.0)),
        (2, (-3.0, 4.0, -5.0)),
        (3, (-3.0, -4.0, -5.0)),
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
        def __init__(self, node, action_type, name, execute_callback):
            self.node = node
            self.action_type = action_type
            self.name = name
            self.execute_callback = execute_callback
            action_servers.append(self)

    monkeypatch.setattr(ground_station_node, 'ActionServer', FakeActionServer)
    rclpy.init()
    node = GroundStationNode()
    try:
        assert node.get_namespace() == '/swarm'
        assert [server.name for server in action_servers] == [
            'takeoff',
            'move_leader',
            'change_formation',
            'pause',
            'land',
        ]
        assert node.resolve_topic_name('mission_command') == '/swarm/mission_command'
        assert node.resolve_topic_name('leader_goal') == '/swarm/leader_goal'
        assert node.resolve_topic_name('formation_mode') == '/swarm/formation_mode'
        assert node.resolve_topic_name('failsafe_command') == '/swarm/failsafe_command'
        assert node.resolve_topic_name('/vehicle_1/staging_setpoint') == (
            '/vehicle_1/staging_setpoint'
        )
        assert sorted(node.core.publishers.vehicle_setpoints) == [1, 2, 3]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def publish_staged_statuses(core):
    for vehicle_id, point in (
        (1, (0.0, 0.0, -5.0)),
        (2, (-3.0, 4.0, -5.0)),
        (3, (-3.0, -4.0, -5.0)),
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
