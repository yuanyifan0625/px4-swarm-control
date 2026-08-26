from px4_swarm_control.models import (
    PositionYawSetpoint,
    Slot,
    VehicleLevelState,
    VehicleRole,
    VehicleState,
)
from px4_swarm_control.vehicle_node import (
    default_vehicle_node_configs,
    parse_vehicle_node_config,
    vehicle_id_to_uint8,
    VehicleNodeConfig,
    VehicleNodeCore,
)
from px4_swarm_interfaces.msg import (
    FormationMode,
    LeaderGoal,
    MissionCommand,
    VehicleSetpoint,
    VehicleStatus as SwarmVehicleStatus,
)


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


class FakePx4Interface:
    def __init__(self, state=None, stale=False, local_ready=True):
        self.state = state
        self.stale = stale
        self.local_ready = local_ready
        self.heartbeats = 0
        self.setpoints = []
        self.safe_hover_calls = 0
        self.offboard_mode_calls = 0
        self.ground_safe_mode_calls = 0
        self.arm_calls = 0
        self.land_calls = 0

    def publish_offboard_heartbeat(self):
        self.heartbeats += 1

    def publish_position_yaw_setpoint(self, setpoint):
        self.setpoints.append(setpoint)

    def publish_safe_hover_setpoint(self):
        self.safe_hover_calls += 1
        return True

    def set_offboard_mode(self):
        self.offboard_mode_calls += 1

    def set_ground_safe_mode(self):
        self.ground_safe_mode_calls += 1

    def arm(self):
        self.arm_calls += 1

    def land(self):
        self.land_calls += 1

    def vehicle_state(self):
        return self.state

    def is_telemetry_stale(self):
        return self.stale

    def local_position_ready(self):
        return self.local_ready


def test_parse_vehicle_node_config_accepts_leader_and_follower_values():
    leader = parse_vehicle_node_config(
        {
            'role': 'leader',
            'vehicle_id': 'MAV1',
            'px4_namespace': '/MAV1',
            'slot': 'leader',
            'control_loop_hz': 20.0,
            'hold_x': 1.0,
            'hold_y': 2.0,
            'hold_z': -3.0,
            'hold_yaw': 0.5,
        },
    )
    follower = parse_vehicle_node_config(
        {
            'role': 'follower',
            'vehicle_id': 'MAV2',
            'px4_namespace': 'MAV2',
            'slot': 'follower_left',
        },
    )

    assert leader.role is VehicleRole.LEADER
    assert leader.slot is Slot.LEADER
    assert leader.px4_target_system == 1
    assert leader.hold_setpoint == PositionYawSetpoint(1.0, 2.0, -3.0, 0.5)
    assert follower.role is VehicleRole.FOLLOWER
    assert follower.px4_namespace == '/MAV2'
    assert follower.px4_target_system == 2
    assert follower.slot is Slot.FOLLOWER_LEFT


def test_parse_vehicle_node_config_rejects_invalid_role_slot_and_rate():
    for values in (
        {'role': 'pilot'},
        {'slot': 'column'},
        {'control_loop_hz': 0.0},
    ):
        try:
            parse_vehicle_node_config(values)
        except ValueError:
            continue
        raise AssertionError(f'expected ValueError for {values}')


def test_parse_vehicle_node_config_rejects_swapped_vehicle_namespace_mapping():
    for values in (
        {'vehicle_id': 'MAV2', 'px4_namespace': '/MAV3', 'slot': 'follower_left'},
        {'vehicle_id': 'MAV2', 'px4_namespace': '/MAV2', 'slot': 'follower_right'},
        {'vehicle_id': 'MAV2', 'px4_namespace': '/MAV2', 'px4_target_system': 4, 'slot': 'follower_left'},
        {'vehicle_id': 'MAV3', 'px4_namespace': '/MAV3', 'role': 'leader'},
    ):
        try:
            parse_vehicle_node_config(values)
        except ValueError:
            continue
        raise AssertionError(f'expected ValueError for {values}')


def test_default_vehicle_node_configs_match_first_version_three_vehicle_layout():
    configs = default_vehicle_node_configs()

    assert [
        (
            config.vehicle_id,
            config.px4_namespace,
            config.px4_target_system,
            config.role,
            config.slot,
        )
        for config in configs
    ] == [
        ('MAV1', '/MAV1', 1, VehicleRole.LEADER, Slot.LEADER),
        ('MAV2', '/MAV2', 2, VehicleRole.FOLLOWER, Slot.FOLLOWER_LEFT),
        ('MAV3', '/MAV3', 3, VehicleRole.FOLLOWER, Slot.FOLLOWER_RIGHT),
    ]


def test_leader_goal_updates_only_leader_setpoint():
    leader_core = make_core(
        VehicleNodeConfig(
            role=VehicleRole.LEADER,
            vehicle_id='MAV1',
            px4_namespace='/MAV1',
            px4_target_system=2,
            slot=Slot.LEADER,
            hold_setpoint=PositionYawSetpoint(0.0, 0.0, -2.0, 0.0),
        ),
    )
    follower_core = make_core(
        VehicleNodeConfig(
            role=VehicleRole.FOLLOWER,
            vehicle_id='MAV2',
            px4_namespace='/MAV2',
            px4_target_system=3,
            slot=Slot.FOLLOWER_LEFT,
            hold_setpoint=PositionYawSetpoint(0.0, 0.0, -2.0, 0.0),
        ),
    )
    goal = LeaderGoal()
    goal.x = 4.0
    goal.y = 5.0
    goal.z = -6.0
    goal.yaw = 0.7

    leader_core.handle_leader_goal(goal)
    follower_core.handle_leader_goal(goal)

    assert leader_core.active_setpoint == PositionYawSetpoint(4.0, 5.0, -6.0, 0.7)
    assert follower_core.active_setpoint == follower_core.config.hold_setpoint
    assert follower_core.logger.warnings == ['follower MAV2 ignored leader goal']


def test_leader_goal_moves_leader_in_following_state():
    px4_interface = FakePx4Interface()
    core = make_core(px4_interface=px4_interface)
    goal = LeaderGoal()
    goal.x = 4.0
    goal.y = 5.0
    goal.z = -6.0
    goal.yaw = 0.7

    core.handle_leader_goal(goal)
    core.control_tick()

    assert core.vehicle_level_state is VehicleLevelState.FOLLOWING
    assert px4_interface.setpoints == [PositionYawSetpoint(4.0, 5.0, -6.0, 0.7)]


def test_follower_ignores_leader_goal_and_waits_for_leader_status():
    px4_interface = FakePx4Interface()
    core = make_core(
        config=VehicleNodeConfig(
            role=VehicleRole.FOLLOWER,
            vehicle_id='MAV2',
            px4_namespace='/MAV2',
            px4_target_system=3,
            slot=Slot.FOLLOWER_LEFT,
            hold_setpoint=PositionYawSetpoint(-3.0, 4.0, -5.0, 0.0),
        ),
        px4_interface=px4_interface,
    )
    goal = LeaderGoal()
    goal.x = 4.0
    goal.y = 5.0
    goal.z = -6.0
    goal.yaw = 0.7

    core.handle_leader_goal(goal)
    core.control_tick()

    assert core.vehicle_level_state is VehicleLevelState.HOLDING
    assert px4_interface.setpoints == []
    assert px4_interface.safe_hover_calls == 1


def test_follower_left_derives_vee_setpoint_from_fresh_leader_status():
    px4_interface = FakePx4Interface()
    core = make_core(
        config=follower_config('MAV2', Slot.FOLLOWER_LEFT),
        px4_interface=px4_interface,
    )
    prepare_safe_follower_telemetry(core, px4_interface, 'MAV2')

    core.handle_leader_status(leader_status(x=10.0, y=20.0, z=-5.0, yaw=0.0))
    core.control_tick()

    assert core.vehicle_level_state is VehicleLevelState.FOLLOWING
    assert px4_interface.setpoints == [
        PositionYawSetpoint(9.0, 19.0, -5.0, 0.0),
    ]


def test_follower_right_derives_vee_setpoint_from_fresh_leader_status():
    px4_interface = FakePx4Interface()
    core = make_core(
        config=follower_config('MAV3', Slot.FOLLOWER_RIGHT),
        px4_interface=px4_interface,
    )
    prepare_safe_follower_telemetry(core, px4_interface, 'MAV3')

    core.handle_leader_status(leader_status(x=10.0, y=20.0, z=-5.0, yaw=0.0))
    core.control_tick()

    assert core.vehicle_level_state is VehicleLevelState.FOLLOWING
    assert px4_interface.setpoints == [
        PositionYawSetpoint(9.0, 21.0, -5.0, 0.0),
    ]


def test_follower_uses_current_formation_mode_topic_for_local_offset():
    px4_interface = FakePx4Interface()
    core = make_core(
        config=follower_config('MAV2', Slot.FOLLOWER_LEFT),
        px4_interface=px4_interface,
    )
    prepare_safe_follower_telemetry(core, px4_interface, 'MAV2')
    mode = FormationMode()
    mode.mode = FormationMode.VEE

    core.handle_formation_mode(mode)
    core.handle_leader_status(leader_status(x=10.0, y=20.0, z=-5.0, yaw=0.0))
    core.control_tick()

    assert px4_interface.setpoints == [
        PositionYawSetpoint(9.0, 19.0, -5.0, 0.0),
    ]


def test_follower_line_abreast_mode_uses_same_row_body_frame_offset():
    px4_interface = FakePx4Interface()
    core = make_core(
        config=follower_config('MAV2', Slot.FOLLOWER_LEFT),
        px4_interface=px4_interface,
    )
    prepare_safe_follower_telemetry(core, px4_interface, 'MAV2')
    mode = FormationMode()
    mode.mode = FormationMode.LINE_ABREAST

    core.handle_formation_mode(mode)
    core.handle_leader_status(leader_status(x=10.0, y=20.0, z=-5.0, yaw=0.0))
    core.control_tick()

    assert px4_interface.setpoints == [
        PositionYawSetpoint(10.0, 19.0, -5.0, 0.0),
    ]


def test_follower_control_tick_enters_collision_hold_and_publishes_slot_fallback():
    clock = [0.0]
    px4_interface = FakePx4Interface(
        state=vehicle_state(
            vehicle_id='MAV2',
            x=0.0,
            y=0.0,
            z=-1.0,
            vehicle_level_state=VehicleLevelState.FOLLOWING,
        ),
    )
    core = make_core(
        config=follower_config('MAV2', Slot.FOLLOWER_LEFT),
        px4_interface=px4_interface,
        now_s=lambda: clock[0],
    )
    core.handle_leader_status(leader_status(x=0.0, y=2.0, z=-1.0, yaw=0.0))
    core.handle_peer_status(
        peer_status(3, x=0.6, y=0.0, slot='follower_right'),
    )

    core.control_tick()
    clock[0] = 1.0
    core.control_tick()

    assert px4_interface.setpoints[0] == PositionYawSetpoint(-1.0, 1.0, -1.0, 0.0)
    assert px4_interface.setpoints[1] == PositionYawSetpoint(0.0, -0.3, -1.0, 0.0)
    assert core.vehicle_level_state is VehicleLevelState.HOLDING


def test_follower_holds_when_leader_status_is_stale():
    px4_interface = FakePx4Interface()
    core = make_core(
        config=follower_config('MAV2', Slot.FOLLOWER_LEFT),
        px4_interface=px4_interface,
    )

    core.handle_leader_status(
        leader_status(x=10.0, y=20.0, z=-5.0, yaw=0.0, last_telemetry_age_sec=5.0),
    )
    core.control_tick()

    assert core.vehicle_level_state is VehicleLevelState.FAILSAFE
    assert px4_interface.setpoints == []
    assert px4_interface.safe_hover_calls == 1


def test_follower_freezes_last_safe_target_when_leader_telemetry_becomes_stale():
    px4_interface = FakePx4Interface()
    core = make_core(
        config=follower_config('MAV2', Slot.FOLLOWER_LEFT),
        px4_interface=px4_interface,
    )
    prepare_safe_follower_telemetry(core, px4_interface, 'MAV2')
    core.handle_leader_status(leader_status(x=10.0, y=20.0, z=-5.0, yaw=0.0))
    core.control_tick()

    core.handle_leader_status(
        leader_status(
            x=10.0,
            y=20.0,
            z=-5.0,
            yaw=0.0,
            last_telemetry_age_sec=5.0,
        )
    )
    core.control_tick()

    expected = PositionYawSetpoint(9.0, 19.0, -5.0, 0.0)
    assert px4_interface.setpoints == [expected, expected]
    assert px4_interface.safe_hover_calls == 0
    assert core.vehicle_level_state is VehicleLevelState.FAILSAFE


def test_follower_freezes_last_safe_target_when_own_telemetry_becomes_stale():
    px4_interface = FakePx4Interface()
    core = make_core(
        config=follower_config('MAV2', Slot.FOLLOWER_LEFT),
        px4_interface=px4_interface,
    )
    prepare_safe_follower_telemetry(core, px4_interface, 'MAV2')
    core.handle_leader_status(leader_status(x=10.0, y=20.0, z=-5.0, yaw=0.0))
    core.control_tick()

    px4_interface.stale = True
    core.control_tick()

    expected = PositionYawSetpoint(9.0, 19.0, -5.0, 0.0)
    assert px4_interface.setpoints == [expected, expected]
    assert px4_interface.safe_hover_calls == 0
    assert core.vehicle_level_state is VehicleLevelState.FAILSAFE


def test_follower_holds_while_leader_is_only_staged_not_following():
    px4_interface = FakePx4Interface()
    core = make_core(
        config=follower_config('MAV2', Slot.FOLLOWER_LEFT),
        px4_interface=px4_interface,
    )

    core.handle_leader_status(
        leader_status(
            x=10.0,
            y=20.0,
            z=-5.0,
            yaw=0.0,
            vehicle_state='staging',
        ),
    )
    core.control_tick()

    assert core.vehicle_level_state is VehicleLevelState.HOLDING
    assert px4_interface.setpoints == []
    assert px4_interface.safe_hover_calls == 1


def test_control_tick_publishes_heartbeat_and_active_setpoint():
    px4_interface = FakePx4Interface()
    core = make_core(px4_interface=px4_interface)

    core.control_tick()

    assert px4_interface.heartbeats == 1
    assert px4_interface.setpoints == [core.config.hold_setpoint]
    assert px4_interface.safe_hover_calls == 0


def test_control_tick_uses_safe_hover_when_telemetry_is_stale():
    px4_interface = FakePx4Interface(stale=True)
    core = make_core(px4_interface=px4_interface)

    core.control_tick()

    assert px4_interface.heartbeats == 1
    assert px4_interface.setpoints == []
    assert px4_interface.safe_hover_calls == 1
    assert core.vehicle_level_state is VehicleLevelState.FAILSAFE


def test_pause_holds_safe_setpoint_until_resume_without_continuing_old_leader_goal():
    px4_interface = FakePx4Interface()
    core = make_core(px4_interface=px4_interface)
    goal = LeaderGoal()
    goal.x = 4.0
    goal.y = 5.0
    goal.z = -6.0
    goal.yaw = 0.7
    core.handle_leader_goal(goal)
    core.control_tick()
    assert core.vehicle_level_state is VehicleLevelState.FOLLOWING

    pause = MissionCommand()
    pause.command = MissionCommand.PAUSE
    core.handle_mission_command(pause)
    core.control_tick()
    core.control_tick()

    assert core.vehicle_level_state is VehicleLevelState.PAUSED
    assert px4_interface.safe_hover_calls == 3
    assert px4_interface.setpoints == [PositionYawSetpoint(4.0, 5.0, -6.0, 0.7)]

    resume = MissionCommand()
    resume.command = MissionCommand.RESUME
    core.handle_mission_command(resume)
    core.control_tick()

    assert core.vehicle_level_state is VehicleLevelState.HOLDING
    assert px4_interface.setpoints[-1] == PositionYawSetpoint(4.0, 5.0, -6.0, 0.7)


def test_follower_pause_resume_waits_for_fresh_following_leader_before_following_again():
    px4_interface = FakePx4Interface()
    core = make_core(
        config=follower_config('MAV2', Slot.FOLLOWER_LEFT),
        px4_interface=px4_interface,
    )
    prepare_safe_follower_telemetry(core, px4_interface, 'MAV2')
    core.handle_leader_status(leader_status(x=10.0, y=20.0, z=-5.0, yaw=0.0))
    core.control_tick()
    assert core.vehicle_level_state is VehicleLevelState.FOLLOWING

    pause = MissionCommand()
    pause.command = MissionCommand.PAUSE
    core.handle_mission_command(pause)
    resume = MissionCommand()
    resume.command = MissionCommand.RESUME
    core.handle_mission_command(resume)
    core.handle_leader_status(
        leader_status(x=10.0, y=20.0, z=-5.0, yaw=0.0, vehicle_state='holding'),
    )
    core.control_tick()

    assert core.vehicle_level_state is VehicleLevelState.HOLDING
    assert px4_interface.setpoints == [
        PositionYawSetpoint(9.0, 19.0, -5.0, 0.0),
    ]
    assert px4_interface.safe_hover_calls >= 1


def test_publish_status_exposes_paused_and_failsafe_vehicle_states():
    status_publisher = FakePublisher()
    paused_core = make_core(status_publisher=status_publisher)
    pause = MissionCommand()
    pause.command = MissionCommand.PAUSE

    paused_core.handle_mission_command(pause)
    paused_core.publish_status()

    assert status_publisher.messages[-1].vehicle_state == 'paused'

    status_publisher = FakePublisher()
    paused_with_telemetry = make_core(
        px4_interface=FakePx4Interface(
            state=vehicle_state(vehicle_level_state=VehicleLevelState.FOLLOWING),
        ),
        status_publisher=status_publisher,
    )
    paused_with_telemetry.handle_mission_command(pause)
    paused_with_telemetry.publish_status()

    assert status_publisher.messages[-1].vehicle_state == 'paused'

    status_publisher = FakePublisher()
    failsafe_core = make_core(
        px4_interface=FakePx4Interface(stale=True),
        status_publisher=status_publisher,
    )

    failsafe_core.control_tick()
    failsafe_core.publish_status()

    assert status_publisher.messages[-1].vehicle_state == 'failsafe'


def legacy_test_vehicle_staging_setpoint_and_takeoff_command_warm_up_offboard_without_qgc():
    px4_interface = FakePx4Interface()
    logger = FakeLogger()
    core = make_core(px4_interface=px4_interface, logger=logger)
    setpoint = VehicleSetpoint()
    setpoint.vehicle_id = 1
    setpoint.x = 0.0
    setpoint.y = 0.0
    setpoint.z = -5.0
    setpoint.yaw = 0.25
    mission = MissionCommand()
    mission.command = MissionCommand.TAKEOFF

    core.handle_staging_setpoint(setpoint)
    core.handle_mission_command(mission)

    assert core.active_setpoint == PositionYawSetpoint(0.0, 0.0, -5.0, 0.25)
    assert px4_interface.heartbeats == 0
    assert px4_interface.setpoints == []
    assert px4_interface.offboard_mode_calls == 0
    assert px4_interface.arm_calls == 1
    assert px4_interface.takeoff_altitudes == [(5.0, 0.25)]
    assert core.vehicle_level_state is VehicleLevelState.TAKING_OFF
    assert logger.infos[-1] == (
        '不依賴 QGC：先用 PX4 NAV_TAKEOFF 到安全高度，'
        '再 warm up Offboard 後切換 staging control。'
    )


def legacy_test_arm_mission_command_only_arms_without_takeoff_or_staging_flow():
    px4_interface = FakePx4Interface()
    core = make_core(px4_interface=px4_interface)
    mission = MissionCommand()
    mission.command = MissionCommand.ARM

    core.handle_mission_command(mission)

    assert px4_interface.arm_calls == 1
    assert px4_interface.takeoff_altitudes == []
    assert px4_interface.offboard_mode_calls == 0
    assert px4_interface.ground_safe_mode_calls == 0
    assert px4_interface.setpoints == []
    assert core.vehicle_level_state is VehicleLevelState.ARMING
    assert core._takeoff_to_staging_active is False
    assert core._pending_takeoff_until_staging is False


def legacy_test_takeoff_command_waits_until_staging_setpoint_has_arrived():
    px4_interface = FakePx4Interface()
    logger = FakeLogger()
    core = make_core(px4_interface=px4_interface, logger=logger)
    mission = MissionCommand()
    mission.command = MissionCommand.TAKEOFF

    core.handle_mission_command(mission)

    assert px4_interface.arm_calls == 0
    assert px4_interface.takeoff_altitudes == []
    assert core.vehicle_level_state is VehicleLevelState.ARMING
    assert logger.infos[-1] == (
        'MAV1 received takeoff before staging setpoint; waiting for staging target'
    )

    core.handle_staging_setpoint(staging_setpoint(z=-5.0, yaw=0.25))

    assert px4_interface.arm_calls == 1
    assert px4_interface.takeoff_altitudes == [(5.0, 0.25)]
    assert core.vehicle_level_state is VehicleLevelState.TAKING_OFF


def legacy_test_repeated_takeoff_command_is_idempotent_once_sequence_started():
    px4_interface = FakePx4Interface()
    core = make_core(px4_interface=px4_interface)
    core.handle_staging_setpoint(staging_setpoint(z=-5.0, yaw=0.25))
    mission = MissionCommand()
    mission.command = MissionCommand.TAKEOFF

    core.handle_mission_command(mission)
    core.handle_mission_command(mission)

    assert px4_interface.arm_calls == 1
    assert px4_interface.takeoff_altitudes == [(5.0, 0.25)]
    assert core.vehicle_level_state is VehicleLevelState.TAKING_OFF


def legacy_test_republished_staging_setpoint_does_not_leave_active_takeoff_sequence():
    px4_interface = FakePx4Interface()
    core = make_core(px4_interface=px4_interface)
    mission = MissionCommand()
    mission.command = MissionCommand.TAKEOFF

    core.handle_staging_setpoint(staging_setpoint(z=-5.0))
    core.handle_mission_command(mission)
    core.handle_staging_setpoint(staging_setpoint(z=-5.0))

    assert core.vehicle_level_state is VehicleLevelState.TAKING_OFF
    assert px4_interface.arm_calls == 1
    assert px4_interface.takeoff_altitudes == [(5.0, 0.0)]


def legacy_test_repeated_takeoff_command_is_ignored_after_airborne_staging_started():
    px4_interface = FakePx4Interface()
    core = make_core(px4_interface=px4_interface)
    core.handle_staging_setpoint(staging_setpoint(z=-5.0, yaw=0.25))
    mission = MissionCommand()
    mission.command = MissionCommand.TAKEOFF

    core.handle_mission_command(mission)
    core._takeoff_to_staging_active = False
    core.transition_to(VehicleLevelState.STAGING, 'PX4 accepted Offboard')
    px4_interface.state = vehicle_state(
        z=-5.0,
        armed=True,
        navigation_state='offboard',
        offboard_available=True,
        vehicle_level_state=VehicleLevelState.STAGING,
    )
    core.handle_mission_command(mission)

    assert px4_interface.arm_calls == 1
    assert px4_interface.takeoff_altitudes == [(5.0, 0.25)]
    assert core.vehicle_level_state is VehicleLevelState.STAGING


def legacy_test_takeoff_retries_px4_takeoff_while_vehicle_still_grounded():
    clock = [10.0]
    px4_interface = FakePx4Interface(
        state=vehicle_state(
            z=-0.05,
            armed=False,
            navigation_state='auto_takeoff',
            landed=True,
        ),
    )
    core = make_core(px4_interface=px4_interface, now_s=lambda: clock[0])
    core.handle_staging_setpoint(staging_setpoint(z=-5.0))
    mission = MissionCommand()
    mission.command = MissionCommand.TAKEOFF
    core.handle_mission_command(mission)

    clock[0] = 10.5
    core.control_tick()
    clock[0] = 11.1
    core.control_tick()

    assert px4_interface.arm_calls == 2
    assert px4_interface.takeoff_altitudes == [(5.0, 0.0), (5.0, 0.0)]
    assert core.vehicle_level_state is VehicleLevelState.TAKING_OFF


def legacy_test_new_takeoff_after_land_waits_for_fresh_staging_setpoint():
    px4_interface = FakePx4Interface()
    core = make_core(px4_interface=px4_interface)
    takeoff = MissionCommand()
    takeoff.command = MissionCommand.TAKEOFF
    land = MissionCommand()
    land.command = MissionCommand.LAND

    core.handle_staging_setpoint(staging_setpoint(z=-5.0, yaw=0.25))
    core.handle_mission_command(takeoff)
    core.handle_mission_command(land)
    core.handle_mission_command(takeoff)

    assert px4_interface.takeoff_altitudes == [(5.0, 0.25)]
    assert core.vehicle_level_state is VehicleLevelState.ARMING

    core.handle_staging_setpoint(staging_setpoint(z=-6.0, yaw=0.5))

    assert px4_interface.takeoff_altitudes == [(5.0, 0.25), (6.0, 0.5)]
    assert core.vehicle_level_state is VehicleLevelState.TAKING_OFF


def legacy_test_takeoff_waits_for_altitude_before_offboard_warmup():
    px4_interface = FakePx4Interface(
        state=vehicle_state(z=-2.0, armed=True, navigation_state='auto_takeoff'),
    )
    core = make_core(px4_interface=px4_interface)
    core.handle_staging_setpoint(staging_setpoint(z=-5.0))
    mission = MissionCommand()
    mission.command = MissionCommand.TAKEOFF
    core.handle_mission_command(mission)

    core.control_tick()

    assert px4_interface.heartbeats == 0
    assert px4_interface.setpoints == []
    assert px4_interface.offboard_mode_calls == 0
    assert core.vehicle_level_state is VehicleLevelState.TAKING_OFF


def legacy_test_takeoff_warms_up_offboard_for_one_second_before_mode_switch():
    clock = [10.0]
    px4_interface = FakePx4Interface(
        state=vehicle_state(z=-4.2, armed=True, navigation_state='auto_takeoff'),
    )
    core = make_core(px4_interface=px4_interface, now_s=lambda: clock[0])
    core.handle_staging_setpoint(staging_setpoint(z=-5.0, yaw=0.25))
    mission = MissionCommand()
    mission.command = MissionCommand.TAKEOFF
    core.handle_mission_command(mission)

    core.control_tick()
    clock[0] = 10.9
    core.control_tick()
    clock[0] = 11.1
    core.control_tick()

    assert px4_interface.heartbeats == 3
    assert px4_interface.setpoints == [
        PositionYawSetpoint(0.0, 0.0, -5.0, 0.25),
        PositionYawSetpoint(0.0, 0.0, -5.0, 0.25),
        PositionYawSetpoint(0.0, 0.0, -5.0, 0.25),
    ]
    assert px4_interface.offboard_mode_calls == 1
    assert core.vehicle_level_state is VehicleLevelState.TAKING_OFF


def legacy_test_takeoff_retries_offboard_mode_until_px4_accepts():
    clock = [10.0]
    px4_interface = FakePx4Interface(
        state=vehicle_state(z=-4.2, armed=True, navigation_state='auto_takeoff'),
    )
    core = make_core(px4_interface=px4_interface, now_s=lambda: clock[0])
    core.handle_staging_setpoint(staging_setpoint(z=-5.0))
    mission = MissionCommand()
    mission.command = MissionCommand.TAKEOFF
    core.handle_mission_command(mission)

    core.control_tick()
    clock[0] = 11.1
    core.control_tick()
    clock[0] = 11.6
    core.control_tick()
    clock[0] = 12.2
    core.control_tick()

    assert px4_interface.offboard_mode_calls == 2


def legacy_test_takeoff_does_not_treat_offboard_available_as_mode_accepted():
    px4_interface = FakePx4Interface(
        state=vehicle_state(
            z=-4.8,
            armed=True,
            navigation_state='auto_takeoff',
            offboard_available=True,
        ),
    )
    core = make_core(px4_interface=px4_interface)
    core.handle_staging_setpoint(staging_setpoint(z=-5.0))
    mission = MissionCommand()
    mission.command = MissionCommand.TAKEOFF
    core.handle_mission_command(mission)

    core.control_tick()

    assert core.vehicle_level_state is VehicleLevelState.TAKING_OFF
    assert px4_interface.heartbeats == 1
    assert px4_interface.setpoints == [PositionYawSetpoint(0.0, 0.0, -5.0, 0.0)]


def legacy_test_staging_control_starts_only_after_px4_reports_offboard_accepted():
    px4_interface = FakePx4Interface(
        state=vehicle_state(
            z=-4.8,
            armed=True,
            navigation_state='offboard',
            offboard_available=True,
        ),
    )
    core = make_core(px4_interface=px4_interface)
    core.handle_staging_setpoint(staging_setpoint(z=-5.0))
    mission = MissionCommand()
    mission.command = MissionCommand.TAKEOFF
    core.handle_mission_command(mission)

    core.control_tick()

    assert px4_interface.heartbeats == 1
    assert px4_interface.setpoints == [PositionYawSetpoint(0.0, 0.0, -5.0, 0.0)]
    assert core.vehicle_level_state is VehicleLevelState.STAGING

    core.control_tick()

    assert px4_interface.heartbeats == 2
    assert px4_interface.setpoints[-1] == PositionYawSetpoint(0.0, 0.0, -5.0, 0.0)
    assert core.vehicle_level_state is VehicleLevelState.STAGING


def legacy_test_takeoff_does_not_accept_offboard_while_px4_still_reports_landed():
    px4_interface = FakePx4Interface(
        state=vehicle_state(
            z=-0.05,
            armed=False,
            navigation_state='offboard',
            offboard_available=True,
            landed=True,
        ),
    )
    core = make_core(px4_interface=px4_interface)
    core.handle_staging_setpoint(staging_setpoint(z=-5.0))
    mission = MissionCommand()
    mission.command = MissionCommand.TAKEOFF
    core.handle_mission_command(mission)

    core.control_tick()

    assert core.vehicle_level_state is VehicleLevelState.TAKING_OFF
    assert px4_interface.setpoints == []


def test_vehicle_ignores_staging_setpoint_for_another_vehicle():
    core = make_core()
    setpoint = VehicleSetpoint()
    setpoint.vehicle_id = 2
    setpoint.x = 9.0
    setpoint.y = 9.0
    setpoint.z = -9.0
    setpoint.yaw = 1.0

    core.handle_staging_setpoint(setpoint)

    assert core.active_setpoint == core.config.hold_setpoint


def test_land_mission_command_calls_px4_land_and_updates_state():
    px4_interface = FakePx4Interface()
    core = make_core(px4_interface=px4_interface)
    mission = MissionCommand()
    mission.command = MissionCommand.LAND

    core.handle_mission_command(mission)

    assert px4_interface.land_calls == 1
    assert core.vehicle_level_state is VehicleLevelState.LANDING


def test_landing_control_tick_does_not_republish_staging_setpoint_or_leave_landing():
    px4_interface = FakePx4Interface(state=vehicle_state(z=-5.0, armed=True))
    core = make_core(px4_interface=px4_interface)
    core.handle_staging_setpoint(staging_setpoint(z=-5.0))
    mission = MissionCommand()
    mission.command = MissionCommand.LAND
    core.handle_mission_command(mission)

    core.control_tick()

    assert px4_interface.heartbeats == 0
    assert px4_interface.setpoints == []
    assert core.vehicle_level_state is VehicleLevelState.LANDING


def legacy_test_landed_offboard_telemetry_runs_land_complete_recovery_once_without_control_outputs():
    px4_interface = FakePx4Interface(
        state=vehicle_state(
            z=-0.01,
            armed=False,
            navigation_state='offboard',
            landed=True,
            vehicle_level_state=VehicleLevelState.LANDING,
            pre_flight_checks_pass=False,
            offboard_control_signal_lost=True,
        ),
    )
    core = make_core(px4_interface=px4_interface)
    mission = MissionCommand()
    mission.command = MissionCommand.LAND
    core.handle_mission_command(mission)

    core.control_tick()
    core.control_tick()

    assert core.vehicle_level_state is VehicleLevelState.LANDED
    assert px4_interface.ground_safe_mode_calls == 1
    assert px4_interface.heartbeats == 0
    assert px4_interface.setpoints == []
    assert px4_interface.takeoff_altitudes == []


def test_landed_non_offboard_telemetry_does_not_run_land_complete_recovery():
    px4_interface = FakePx4Interface(
        state=vehicle_state(
            z=-0.01,
            armed=False,
            navigation_state='auto_loiter',
            landed=True,
            vehicle_level_state=VehicleLevelState.LANDING,
            pre_flight_checks_pass=True,
            offboard_control_signal_lost=False,
        ),
    )
    core = make_core(px4_interface=px4_interface)
    mission = MissionCommand()
    mission.command = MissionCommand.LAND
    core.handle_mission_command(mission)

    core.control_tick()
    core.control_tick()

    assert core.vehicle_level_state is VehicleLevelState.LANDED
    assert px4_interface.ground_safe_mode_calls == 0
    assert px4_interface.heartbeats == 0
    assert px4_interface.setpoints == []


def test_landed_px4_state_is_reported_as_landed_vehicle_status():
    state = vehicle_state(
        z=-0.01,
        armed=False,
        navigation_state='auto_land',
        landed=True,
        vehicle_level_state=VehicleLevelState.LANDING,
    )
    status_publisher = FakePublisher()
    core = make_core(
        px4_interface=FakePx4Interface(state=state),
        status_publisher=status_publisher,
    )

    core.publish_status()

    assert core.vehicle_level_state is VehicleLevelState.LANDED
    assert status_publisher.messages[-1].vehicle_state == 'landed'


def test_publish_status_exposes_arm_eligibility_health_fields():
    state = vehicle_state(
        z=-0.01,
        armed=False,
        navigation_state='offboard',
        landed=True,
        pre_flight_checks_pass=False,
        offboard_control_signal_lost=True,
    )
    status_publisher = FakePublisher()
    core = make_core(
        px4_interface=FakePx4Interface(state=state),
        status_publisher=status_publisher,
    )

    core.publish_status()

    status = status_publisher.messages[-1]
    assert status.pre_flight_checks_pass is False
    assert status.offboard_control_signal_lost is True


def test_airborne_px4_telemetry_clears_stale_internal_landed_status():
    state = vehicle_state(
        z=-4.0,
        armed=True,
        navigation_state='auto_takeoff',
        landed=False,
        vehicle_level_state=VehicleLevelState.LANDED,
    )
    status_publisher = FakePublisher()
    core = make_core(
        px4_interface=FakePx4Interface(state=state),
        status_publisher=status_publisher,
    )
    core.transition_to(VehicleLevelState.LANDED, 'previous land completed')

    core.publish_status()

    assert core.vehicle_level_state is not VehicleLevelState.LANDED
    assert status_publisher.messages[-1].vehicle_state != 'landed'


def legacy_test_takeoff_sequence_ignores_grounded_landed_telemetry_until_airborne():
    px4_interface = FakePx4Interface(
        state=vehicle_state(
            z=-0.05,
            armed=False,
            navigation_state='auto_takeoff',
            landed=True,
            vehicle_level_state=VehicleLevelState.TAKING_OFF,
        ),
    )
    core = make_core(px4_interface=px4_interface)
    core.handle_staging_setpoint(staging_setpoint(z=-5.0))
    mission = MissionCommand()
    mission.command = MissionCommand.TAKEOFF
    core.handle_mission_command(mission)

    core.control_tick()

    assert core.vehicle_level_state is VehicleLevelState.TAKING_OFF
    assert px4_interface.arm_calls == 1
    assert px4_interface.takeoff_altitudes == [(5.0, 0.0)]


def test_publish_status_uses_internal_vehicle_state_when_available():
    state = VehicleState(
        vehicle_id='MAV2',
        position=(1.0, 2.0, -3.0),
        yaw=0.4,
        velocity=(0.1, 0.2, -0.3),
        armed=True,
        navigation_state='offboard',
        offboard_available=True,
        telemetry_age_s=0.25,
        vehicle_level_state=VehicleLevelState.FOLLOWING,
    )
    status_publisher = FakePublisher()
    core = make_core(
        config=VehicleNodeConfig(
            role=VehicleRole.FOLLOWER,
            vehicle_id='MAV2',
            px4_namespace='/MAV2',
            px4_target_system=3,
            slot=Slot.FOLLOWER_LEFT,
            hold_setpoint=PositionYawSetpoint(0.0, 0.0, -2.0, 0.0),
        ),
        px4_interface=FakePx4Interface(state=state),
        status_publisher=status_publisher,
    )

    core.publish_status()

    msg = status_publisher.messages[-1]
    assert msg.vehicle_id == 2
    assert msg.role == 'follower'
    assert msg.px4_namespace == '/MAV2'
    assert msg.slot == 'follower_left'
    assert (msg.x, msg.y, msg.z, msg.yaw) == (1.0, 2.0, -3.0, 0.4)
    assert (msg.vx, msg.vy, msg.vz) == (0.1, 0.2, -0.3)
    assert msg.armed is True
    assert msg.nav_state == 'offboard'
    assert msg.offboard_available is True
    assert msg.last_telemetry_age_sec == 0.25
    assert msg.vehicle_state == 'following'


def test_publish_status_drops_mismatched_interface_vehicle_state():
    state = VehicleState(
        vehicle_id='MAV3',
        position=(9.0, 8.0, -7.0),
        yaw=0.4,
        velocity=(0.1, 0.2, -0.3),
        armed=True,
        navigation_state='offboard',
        offboard_available=True,
        telemetry_age_s=0.25,
        vehicle_level_state=VehicleLevelState.FOLLOWING,
    )
    status_publisher = FakePublisher()
    logger = FakeLogger()
    core = make_core(
        config=VehicleNodeConfig(
            role=VehicleRole.FOLLOWER,
            vehicle_id='MAV2',
            px4_namespace='/MAV2',
            px4_target_system=3,
            slot=Slot.FOLLOWER_LEFT,
            hold_setpoint=PositionYawSetpoint(0.0, 0.0, -2.0, 0.0),
        ),
        px4_interface=FakePx4Interface(state=state),
        status_publisher=status_publisher,
        logger=logger,
    )

    core.publish_status()

    assert status_publisher.messages == []
    assert logger.warnings == [
        'MAV2 ignored telemetry for MAV3 from /MAV2',
    ]


def test_state_transition_logging_is_not_repeated_for_same_state():
    logger = FakeLogger()
    core = make_core(logger=logger)

    core.transition_to(VehicleLevelState.HOLDING, 'test')
    core.transition_to(VehicleLevelState.HOLDING, 'test')

    assert core.vehicle_level_state is VehicleLevelState.HOLDING
    assert logger.infos == ['MAV1 state idle -> holding: test']


def test_vehicle_id_to_uint8_extracts_numeric_suffix():
    assert vehicle_id_to_uint8('MAV3') == 3
    assert vehicle_id_to_uint8('7') == 7


def test_takeoff_warms_local_vertical_target_before_offboard_then_arms():
    clock = [0.0]
    px4_interface = FakePx4Interface(
        state=vehicle_state(
            x=10.0, y=20.0, z=0.0, yaw=0.3, armed=False,
            navigation_state='auto_loiter',
        ),
    )
    core = make_core(px4_interface=px4_interface, now_s=lambda: clock[0])
    takeoff = MissionCommand()
    takeoff.command = MissionCommand.TAKEOFF

    core.handle_staging_setpoint(staging_setpoint(x=7.0, y=8.0, z=-5.0, yaw=0.9))
    core.handle_mission_command(takeoff)
    core.control_tick()

    vertical_target = PositionYawSetpoint(10.0, 20.0, -5.0, 0.3)
    assert px4_interface.setpoints == [vertical_target]
    assert px4_interface.offboard_mode_calls == 0
    assert px4_interface.arm_calls == 0

    clock[0] = 1.0
    core.control_tick()
    assert px4_interface.offboard_mode_calls == 1
    assert px4_interface.arm_calls == 0

    px4_interface.state = vehicle_state(
        x=10.0, y=20.0, z=0.0, yaw=0.3, armed=False, navigation_state='offboard',
    )
    core.control_tick()
    core.control_tick()
    assert px4_interface.arm_calls == 1


def test_takeoff_switches_to_full_staging_only_after_local_ned_height_gate():
    clock = [0.0]
    px4_interface = FakePx4Interface(
        state=vehicle_state(
            x=1.0, y=2.0, z=0.0, yaw=0.4, armed=False,
            navigation_state='auto_loiter',
        ),
    )
    core = make_core(px4_interface=px4_interface, now_s=lambda: clock[0])
    takeoff = MissionCommand()
    takeoff.command = MissionCommand.TAKEOFF
    staging = staging_setpoint(x=7.0, y=8.0, z=-5.0, yaw=0.9)
    core.handle_staging_setpoint(staging)
    core.handle_mission_command(takeoff)
    core.control_tick()
    clock[0] = 1.0
    core.control_tick()
    px4_interface.state = vehicle_state(
        x=1.0, y=2.0, z=-4.8, yaw=0.4, armed=False, navigation_state='offboard',
    )
    core.control_tick()
    core.control_tick()
    px4_interface.state = vehicle_state(
        x=1.0, y=2.0, z=-4.8, yaw=0.4, armed=True, navigation_state='offboard',
    )
    core.control_tick()
    assert px4_interface.setpoints[-1] == PositionYawSetpoint(1.0, 2.0, -5.0, 0.4)

    px4_interface.state = vehicle_state(
        x=1.0, y=2.0, z=-4.9, yaw=0.4, armed=True, navigation_state='offboard',
    )
    core.control_tick()
    assert px4_interface.setpoints[-1] == PositionYawSetpoint(7.0, 8.0, -5.0, 0.9)
    assert core.vehicle_level_state is VehicleLevelState.STAGING


def test_takeoff_waits_without_commands_until_local_position_is_ready():
    px4_interface = FakePx4Interface(
        state=vehicle_state(armed=False), local_ready=False,
    )
    core = make_core(px4_interface=px4_interface)
    takeoff = MissionCommand()
    takeoff.command = MissionCommand.TAKEOFF
    core.handle_staging_setpoint(staging_setpoint(z=-5.0))
    core.handle_mission_command(takeoff)
    core.control_tick()

    assert px4_interface.heartbeats == 0
    assert px4_interface.setpoints == []
    assert px4_interface.offboard_mode_calls == 0
    assert px4_interface.arm_calls == 0


def test_pause_cancels_takeoff_and_resume_does_not_restart_it():
    px4_interface = FakePx4Interface(
        state=vehicle_state(x=3.0, y=4.0, z=-1.0, yaw=0.5, armed=False),
    )
    core = make_core(px4_interface=px4_interface)
    takeoff = MissionCommand()
    takeoff.command = MissionCommand.TAKEOFF
    pause = MissionCommand()
    pause.command = MissionCommand.PAUSE
    resume = MissionCommand()
    resume.command = MissionCommand.RESUME
    core.handle_staging_setpoint(staging_setpoint(z=-5.0))
    core.handle_mission_command(takeoff)
    core.control_tick()
    core.handle_mission_command(pause)
    core.handle_mission_command(resume)
    core.control_tick()

    assert px4_interface.setpoints[-1] == PositionYawSetpoint(3.0, 4.0, -1.0, 0.5)
    assert px4_interface.offboard_mode_calls == 0
    assert px4_interface.arm_calls == 0


def vehicle_state(
    *,
    vehicle_id='MAV1',
    x=0.0,
    y=0.0,
    z=-5.0,
    yaw=0.0,
    armed=True,
    navigation_state='offboard',
    offboard_available=False,
    pre_flight_checks_pass=True,
    offboard_control_signal_lost=False,
    landed=False,
    telemetry_age_s=0.1,
    vehicle_level_state=VehicleLevelState.HOLDING,
):
    return VehicleState(
        vehicle_id=vehicle_id,
        position=(x, y, z),
        yaw=yaw,
        velocity=(0.0, 0.0, 0.0),
        armed=armed,
        navigation_state=navigation_state,
        offboard_available=offboard_available,
        pre_flight_checks_pass=pre_flight_checks_pass,
        offboard_control_signal_lost=offboard_control_signal_lost,
        telemetry_age_s=telemetry_age_s,
        vehicle_level_state=vehicle_level_state,
        landed=landed,
    )


def staging_setpoint(*, x=0.0, y=0.0, z=-5.0, yaw=0.0):
    setpoint = VehicleSetpoint()
    setpoint.vehicle_id = 1
    setpoint.x = x
    setpoint.y = y
    setpoint.z = z
    setpoint.yaw = yaw
    return setpoint


def make_core(config=None, px4_interface=None, status_publisher=None, logger=None, now_s=None):
    if config is None:
        config = VehicleNodeConfig(
            role=VehicleRole.LEADER,
            vehicle_id='MAV1',
            px4_namespace='/MAV1',
            px4_target_system=2,
            slot=Slot.LEADER,
            hold_setpoint=PositionYawSetpoint(0.0, 0.0, -2.0, 0.0),
        )
    if px4_interface is None:
        px4_interface = FakePx4Interface()
    if status_publisher is None:
        status_publisher = FakePublisher()
    if logger is None:
        logger = FakeLogger()
    return VehicleNodeCore(config, px4_interface, status_publisher, logger, now_s=now_s)


def follower_config(vehicle_id, slot):
    target_system = 3 if vehicle_id == 'MAV2' else 4
    return VehicleNodeConfig(
        role=VehicleRole.FOLLOWER,
        vehicle_id=vehicle_id,
        px4_namespace=f'/{vehicle_id}',
        px4_target_system=target_system,
        slot=slot,
        hold_setpoint=PositionYawSetpoint(0.0, 0.0, -2.0, 0.0),
    )


def leader_status(
    *,
    x,
    y,
    z,
    yaw,
    last_telemetry_age_sec=0.1,
    vehicle_state='following',
):
    status = SwarmVehicleStatus()
    status.vehicle_id = 1
    status.role = 'leader'
    status.x = x
    status.y = y
    status.z = z
    status.yaw = yaw
    status.vx = 0.0
    status.vy = 0.0
    status.vz = 0.0
    status.armed = True
    status.nav_state = 'offboard'
    status.offboard_available = True
    status.last_telemetry_age_sec = last_telemetry_age_sec
    status.vehicle_state = vehicle_state
    return status


def peer_status(
    vehicle_id,
    *,
    x,
    y,
    z=-1.0,
    yaw=0.0,
    last_telemetry_age_sec=0.1,
    slot='',
    vehicle_state='following',
):
    status = SwarmVehicleStatus()
    status.vehicle_id = vehicle_id
    status.role = 'follower'
    status.slot = slot
    status.x = x
    status.y = y
    status.z = z
    status.yaw = yaw
    status.armed = True
    status.nav_state = 'offboard'
    status.last_telemetry_age_sec = last_telemetry_age_sec
    status.vehicle_state = vehicle_state
    return status


def prepare_safe_follower_telemetry(core, px4_interface, vehicle_id):
    if vehicle_id == 'MAV2':
        own_y = 21.0
        peer_id = 3
        peer_y = 19.0
        peer_slot = 'follower_right'
    else:
        own_y = 19.0
        peer_id = 2
        peer_y = 21.0
        peer_slot = 'follower_left'
    px4_interface.state = vehicle_state(
        vehicle_id=vehicle_id,
        x=9.0,
        y=own_y,
        z=-5.0,
        vehicle_level_state=VehicleLevelState.FOLLOWING,
    )
    core.handle_peer_status(
        peer_status(peer_id, x=9.0, y=peer_y, z=-5.0, slot=peer_slot)
    )
