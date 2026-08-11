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
from px4_swarm_interfaces.msg import LeaderGoal, MissionCommand, VehicleSetpoint


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
    def __init__(self, state=None, stale=False):
        self.state = state
        self.stale = stale
        self.heartbeats = 0
        self.setpoints = []
        self.safe_hover_calls = 0
        self.offboard_mode_calls = 0
        self.arm_calls = 0
        self.takeoff_altitudes = []
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

    def arm(self):
        self.arm_calls += 1

    def takeoff(self, altitude_m, yaw=0.0):
        self.takeoff_altitudes.append((altitude_m, yaw))

    def land(self):
        self.land_calls += 1

    def vehicle_state(self):
        return self.state

    def is_telemetry_stale(self):
        return self.stale


def test_parse_vehicle_node_config_accepts_leader_and_follower_values():
    leader = parse_vehicle_node_config(
        {
            'role': 'leader',
            'vehicle_id': 'vehicle_1',
            'px4_namespace': '/vehicle_1',
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
            'vehicle_id': 'vehicle_2',
            'px4_namespace': 'vehicle_2',
            'slot': 'follower_left',
        },
    )

    assert leader.role is VehicleRole.LEADER
    assert leader.slot is Slot.LEADER
    assert leader.px4_target_system == 2
    assert leader.hold_setpoint == PositionYawSetpoint(1.0, 2.0, -3.0, 0.5)
    assert follower.role is VehicleRole.FOLLOWER
    assert follower.px4_namespace == '/vehicle_2'
    assert follower.px4_target_system == 3
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
        {'vehicle_id': 'vehicle_2', 'px4_namespace': '/vehicle_3', 'slot': 'follower_left'},
        {'vehicle_id': 'vehicle_2', 'px4_namespace': '/vehicle_2', 'slot': 'follower_right'},
        {'vehicle_id': 'vehicle_2', 'px4_namespace': '/vehicle_2', 'px4_target_system': 4, 'slot': 'follower_left'},
        {'vehicle_id': 'vehicle_3', 'px4_namespace': '/vehicle_3', 'role': 'leader'},
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
        ('vehicle_1', '/vehicle_1', 2, VehicleRole.LEADER, Slot.LEADER),
        ('vehicle_2', '/vehicle_2', 3, VehicleRole.FOLLOWER, Slot.FOLLOWER_LEFT),
        ('vehicle_3', '/vehicle_3', 4, VehicleRole.FOLLOWER, Slot.FOLLOWER_RIGHT),
    ]


def test_leader_goal_updates_only_leader_setpoint():
    leader_core = make_core(
        VehicleNodeConfig(
            role=VehicleRole.LEADER,
            vehicle_id='vehicle_1',
            px4_namespace='/vehicle_1',
            px4_target_system=2,
            slot=Slot.LEADER,
            hold_setpoint=PositionYawSetpoint(0.0, 0.0, -2.0, 0.0),
        ),
    )
    follower_core = make_core(
        VehicleNodeConfig(
            role=VehicleRole.FOLLOWER,
            vehicle_id='vehicle_2',
            px4_namespace='/vehicle_2',
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
    assert follower_core.logger.warnings == ['follower vehicle_2 ignored leader goal']


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
    assert core.vehicle_level_state is VehicleLevelState.HOLDING


def test_vehicle_staging_setpoint_and_takeoff_command_warm_up_offboard_without_qgc():
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


def test_takeoff_command_waits_until_staging_setpoint_has_arrived():
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
        'vehicle_1 received takeoff before staging setpoint; waiting for staging target'
    )

    core.handle_staging_setpoint(staging_setpoint(z=-5.0, yaw=0.25))

    assert px4_interface.arm_calls == 1
    assert px4_interface.takeoff_altitudes == [(5.0, 0.25)]
    assert core.vehicle_level_state is VehicleLevelState.TAKING_OFF


def test_takeoff_waits_for_altitude_before_offboard_warmup():
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


def test_takeoff_warms_up_offboard_for_one_second_before_mode_switch():
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


def test_takeoff_retries_offboard_mode_until_px4_accepts():
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


def test_staging_control_starts_only_after_px4_reports_offboard_accepted():
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


def test_publish_status_uses_internal_vehicle_state_when_available():
    state = VehicleState(
        vehicle_id='vehicle_2',
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
            vehicle_id='vehicle_2',
            px4_namespace='/vehicle_2',
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
    assert msg.px4_namespace == '/vehicle_2'
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
        vehicle_id='vehicle_3',
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
            vehicle_id='vehicle_2',
            px4_namespace='/vehicle_2',
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
        'vehicle_2 ignored telemetry for vehicle_3 from /vehicle_2',
    ]


def test_state_transition_logging_is_not_repeated_for_same_state():
    logger = FakeLogger()
    core = make_core(logger=logger)

    core.transition_to(VehicleLevelState.HOLDING, 'test')
    core.transition_to(VehicleLevelState.HOLDING, 'test')

    assert core.vehicle_level_state is VehicleLevelState.HOLDING
    assert logger.infos == ['vehicle_1 state idle -> holding: test']


def test_vehicle_id_to_uint8_extracts_numeric_suffix():
    assert vehicle_id_to_uint8('vehicle_3') == 3
    assert vehicle_id_to_uint8('7') == 7


def vehicle_state(
    *,
    z=-5.0,
    armed=True,
    navigation_state='offboard',
    offboard_available=False,
    landed=False,
    vehicle_level_state=VehicleLevelState.HOLDING,
):
    return VehicleState(
        vehicle_id='vehicle_1',
        position=(0.0, 0.0, z),
        yaw=0.0,
        velocity=(0.0, 0.0, 0.0),
        armed=armed,
        navigation_state=navigation_state,
        offboard_available=offboard_available,
        telemetry_age_s=0.1,
        vehicle_level_state=vehicle_level_state,
        landed=landed,
    )


def staging_setpoint(*, z=-5.0, yaw=0.0):
    setpoint = VehicleSetpoint()
    setpoint.vehicle_id = 1
    setpoint.x = 0.0
    setpoint.y = 0.0
    setpoint.z = z
    setpoint.yaw = yaw
    return setpoint


def make_core(config=None, px4_interface=None, status_publisher=None, logger=None, now_s=None):
    if config is None:
        config = VehicleNodeConfig(
            role=VehicleRole.LEADER,
            vehicle_id='vehicle_1',
            px4_namespace='/vehicle_1',
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
