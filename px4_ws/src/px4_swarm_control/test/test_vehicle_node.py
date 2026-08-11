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
from px4_swarm_interfaces.msg import LeaderGoal


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

    def publish_offboard_heartbeat(self):
        self.heartbeats += 1

    def publish_position_yaw_setpoint(self, setpoint):
        self.setpoints.append(setpoint)

    def publish_safe_hover_setpoint(self):
        self.safe_hover_calls += 1
        return True

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
    assert leader.hold_setpoint == PositionYawSetpoint(1.0, 2.0, -3.0, 0.5)
    assert follower.role is VehicleRole.FOLLOWER
    assert follower.px4_namespace == '/vehicle_2'
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
        {'vehicle_id': 'vehicle_3', 'px4_namespace': '/vehicle_3', 'role': 'leader'},
    ):
        try:
            parse_vehicle_node_config(values)
        except ValueError:
            continue
        raise AssertionError(f'expected ValueError for {values}')


def test_default_vehicle_node_configs_match_first_version_three_vehicle_layout():
    configs = default_vehicle_node_configs()

    assert [(config.vehicle_id, config.px4_namespace, config.role, config.slot) for config in configs] == [
        ('vehicle_1', '/vehicle_1', VehicleRole.LEADER, Slot.LEADER),
        ('vehicle_2', '/vehicle_2', VehicleRole.FOLLOWER, Slot.FOLLOWER_LEFT),
        ('vehicle_3', '/vehicle_3', VehicleRole.FOLLOWER, Slot.FOLLOWER_RIGHT),
    ]


def test_leader_goal_updates_only_leader_setpoint():
    leader_core = make_core(
        VehicleNodeConfig(
            role=VehicleRole.LEADER,
            vehicle_id='vehicle_1',
            px4_namespace='/vehicle_1',
            slot=Slot.LEADER,
            hold_setpoint=PositionYawSetpoint(0.0, 0.0, -2.0, 0.0),
        ),
    )
    follower_core = make_core(
        VehicleNodeConfig(
            role=VehicleRole.FOLLOWER,
            vehicle_id='vehicle_2',
            px4_namespace='/vehicle_2',
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


def make_core(config=None, px4_interface=None, status_publisher=None, logger=None):
    if config is None:
        config = VehicleNodeConfig(
            role=VehicleRole.LEADER,
            vehicle_id='vehicle_1',
            px4_namespace='/vehicle_1',
            slot=Slot.LEADER,
            hold_setpoint=PositionYawSetpoint(0.0, 0.0, -2.0, 0.0),
        )
    if px4_interface is None:
        px4_interface = FakePx4Interface()
    if status_publisher is None:
        status_publisher = FakePublisher()
    if logger is None:
        logger = FakeLogger()
    return VehicleNodeCore(config, px4_interface, status_publisher, logger)
