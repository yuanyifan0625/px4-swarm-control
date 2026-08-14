from math import isclose, pi

from px4_swarm_control import operator_console
from px4_swarm_control.operator_console import (
    ConsoleActionResult,
    ConsoleCommandDispatcher,
    FormationSettleGate,
    OperatorConsoleConfig,
    RosSwarmActionGateway,
    SwarmActionGateway,
    formation_settle_ready,
)
from px4_swarm_interfaces.msg import VehicleStatus


class FakeGateway(SwarmActionGateway):
    def __init__(self, leader_status=None, paused=False, action_results=None):
        self.leader_status = leader_status
        self.paused = paused
        self.action_results = action_results or {}
        self.calls = []

    def get_leader_status(self):
        return self.leader_status

    def is_paused(self):
        return self.paused

    def describe_status(self):
        return 'paused' if self.paused else 'running'

    def takeoff(self, altitude_m, timeout_sec):
        self.calls.append(('takeoff', altitude_m, timeout_sec))
        return self.action_results.get('takeoff', ConsoleActionResult(True, 'takeoff ok'))

    def arm(self, timeout_sec):
        self.calls.append(('arm', timeout_sec))
        return self.action_results.get('arm', ConsoleActionResult(True, 'arm ok'))

    def move_leader(
        self,
        x,
        y,
        z,
        yaw,
        position_tolerance_m,
        yaw_tolerance_rad,
        timeout_sec,
    ):
        self.calls.append(
            (
                'move_leader',
                x,
                y,
                z,
                yaw,
                position_tolerance_m,
                yaw_tolerance_rad,
                timeout_sec,
            )
        )
        return self.action_results.get('move_leader', ConsoleActionResult(True, 'move ok'))

    def change_formation(self, formation_mode, timeout_sec):
        self.calls.append(('change_formation', formation_mode, timeout_sec))
        return self.action_results.get(
            'change_formation',
            ConsoleActionResult(True, 'formation ok'),
        )

    def pause(self, pause, reason):
        self.calls.append(('pause', pause, reason))
        self.paused = pause
        return self.action_results.get('pause', ConsoleActionResult(True, 'pause ok'))

    def land(self, timeout_sec):
        self.calls.append(('land', timeout_sec))
        return self.action_results.get('land', ConsoleActionResult(True, 'land ok'))

    def wait_for_formation_settle(self, formation_mode, config):
        self.calls.append(
            (
                'settle',
                formation_mode,
                config.settle_stable_duration_s,
                config.settle_timeout_sec,
                config.settle_position_tolerance_m,
                config.settle_yaw_tolerance_rad,
            )
        )
        return self.action_results.get('settle', ConsoleActionResult(True, 'settle ok'))


def leader_status(x=10.0, y=20.0, z=-5.0, yaw=0.25):
    status = VehicleStatus()
    status.vehicle_id = 1
    status.x = x
    status.y = y
    status.z = z
    status.yaw = yaw
    status.vehicle_state = 'following'
    status.armed = True
    status.nav_state = 'offboard'
    status.last_telemetry_age_sec = 0.1
    return status


def follower_status(vehicle_id, slot, x, y, z=-5.0, yaw=0.0):
    status = VehicleStatus()
    status.vehicle_id = vehicle_id
    status.role = 'follower'
    status.slot = slot
    status.x = x
    status.y = y
    status.z = z
    status.yaw = yaw
    status.vehicle_state = 'following'
    status.armed = True
    status.nav_state = 'offboard'
    status.last_telemetry_age_sec = 0.1
    return status


def test_numeric_commands_call_existing_swarm_actions_with_configured_defaults():
    config = OperatorConsoleConfig(
        takeoff_altitude_m=6.0,
        default_timeout_sec=70.0,
        move_step_x_m=1.5,
        move_step_y_m=2.5,
        altitude_step_m=0.75,
        yaw_step_rad=0.5,
        move_position_tolerance_m=0.3,
        move_yaw_tolerance_rad=0.1,
    )
    gateway = FakeGateway(leader_status=leader_status())
    dispatcher = ConsoleCommandDispatcher(config, gateway)

    assert dispatcher.dispatch('1').success is True
    assert dispatcher.dispatch('0').success is True
    assert dispatcher.dispatch('6').success is True
    assert dispatcher.dispatch('7').success is True
    assert dispatcher.dispatch('8').success is True

    assert gateway.calls == [
        ('takeoff', 6.0, 70.0),
        ('arm', 70.0),
        ('change_formation', 'vee', 70.0),
        ('change_formation', 'line_abreast', 70.0),
        ('land', 70.0),
    ]


def test_ros_gateway_subscribes_to_mav_status_topics(monkeypatch):
    class FakeActionClient:
        def __init__(self, *args, **kwargs):
            pass

    class FakeNode:
        def __init__(self):
            self.subscriptions = []

        def create_subscription(self, msg_type, topic, callback, qos):
            self.subscriptions.append((msg_type, topic, callback, qos))
            return topic

    monkeypatch.setattr(operator_console, 'ActionClient', FakeActionClient)

    node = FakeNode()
    RosSwarmActionGateway(node, OperatorConsoleConfig())

    assert [subscription[1] for subscription in node.subscriptions] == [
        '/MAV1/status',
        '/MAV2/status',
        '/MAV3/status',
    ]


def test_relative_leader_jog_commands_convert_from_current_status_to_absolute_goal():
    config = OperatorConsoleConfig(
        move_step_x_m=1.0,
        move_step_y_m=1.0,
        altitude_step_m=1.0,
        yaw_step_rad=pi / 4.0,
        move_position_tolerance_m=0.3,
        move_yaw_tolerance_rad=0.2,
        default_timeout_sec=60.0,
    )
    gateway = FakeGateway(leader_status=leader_status(x=1.0, y=2.0, z=-5.0, yaw=0.0))
    dispatcher = ConsoleCommandDispatcher(config, gateway)

    dispatcher.dispatch('2')
    dispatcher.dispatch('3')
    dispatcher.dispatch('4')
    dispatcher.dispatch('5')

    move_calls = [call for call in gateway.calls if call[0] == 'move_leader']
    assert move_calls[0][1:5] == (2.0, 2.0, -5.0, 0.0)
    assert move_calls[1][1:5] == (1.0, 3.0, -5.0, 0.0)
    assert move_calls[2][1:5] == (1.0, 2.0, -6.0, 0.0)
    assert move_calls[3][1:4] == (1.0, 2.0, -5.0)
    assert isclose(move_calls[3][4], pi / 4.0)


def test_paused_console_allows_status_resume_and_land_but_blocks_motion_and_macro():
    gateway = FakeGateway(leader_status=leader_status(), paused=True)
    dispatcher = ConsoleCommandDispatcher(OperatorConsoleConfig(), gateway)

    assert dispatcher.dispatch('s').success is True
    assert dispatcher.dispatch('r').success is True
    gateway.paused = True
    assert dispatcher.dispatch('8').success is True
    assert dispatcher.dispatch('2').success is False
    assert dispatcher.dispatch('6').success is False
    assert dispatcher.dispatch('0').success is False
    assert dispatcher.dispatch('9').success is False

    assert ('pause', False, 'operator console resume') in gateway.calls
    assert ('land', 60.0) in gateway.calls
    assert all(call[0] != 'move_leader' for call in gateway.calls)
    assert all(call[0] != 'arm' for call in gateway.calls)


def test_settle_command_observes_current_formation_without_moving_followers():
    config = OperatorConsoleConfig(
        settle_stable_duration_s=1.25,
        settle_timeout_sec=12.0,
        settle_position_tolerance_m=0.4,
        settle_yaw_tolerance_rad=0.15,
    )
    gateway = FakeGateway(leader_status=leader_status())
    dispatcher = ConsoleCommandDispatcher(config, gateway)

    result = dispatcher.dispatch('settle')

    assert result.success is True
    assert gateway.calls == [('settle', 'vee', 1.25, 12.0, 0.4, 0.15)]


def test_demo_macro_runs_settle_steps_and_tracks_successful_formation_mode():
    config = OperatorConsoleConfig(demo_commands=('1', '7', 'settle', '6', 'settle', '8'))
    gateway = FakeGateway(leader_status=leader_status(x=0.0, y=0.0, z=-5.0, yaw=0.0))
    dispatcher = ConsoleCommandDispatcher(config, gateway)

    result = dispatcher.dispatch('9')

    assert result.success is True
    assert gateway.calls == [
        ('takeoff', 5.0, 60.0),
        ('change_formation', 'line_abreast', 60.0),
        ('settle', 'line_abreast', 1.0, 30.0, 0.5, 0.25),
        ('change_formation', 'vee', 60.0),
        ('settle', 'vee', 1.0, 30.0, 0.5, 0.25),
        ('land', 60.0),
    ]


def test_demo_macro_home_yaw_rotates_current_leader_pose_to_home_yaw_before_home():
    config = OperatorConsoleConfig(
        demo_commands=('1', '2', '5', 'home_yaw', 'settle', 'home', 'settle', '8')
    )
    gateway = FakeGateway(leader_status=leader_status(x=0.0, y=0.0, z=-5.0, yaw=0.0))
    dispatcher = ConsoleCommandDispatcher(config, gateway)

    def move_leader(x, y, z, yaw, position_tolerance_m, yaw_tolerance_rad, timeout_sec):
        FakeGateway.move_leader(
            gateway,
            x,
            y,
            z,
            yaw,
            position_tolerance_m,
            yaw_tolerance_rad,
            timeout_sec,
        )
        gateway.leader_status = leader_status(x=x, y=y, z=z, yaw=yaw)
        return ConsoleActionResult(True, 'move ok')

    def takeoff(altitude_m, timeout_sec):
        gateway.calls.append(('takeoff', altitude_m, timeout_sec))
        gateway.leader_status = leader_status(x=1.0, y=2.0, z=-5.0, yaw=0.0)
        return ConsoleActionResult(True, 'takeoff ok')

    gateway.takeoff = takeoff
    gateway.move_leader = move_leader

    result = dispatcher.dispatch('9')

    assert result.success is True
    move_calls = [call for call in gateway.calls if call[0] == 'move_leader']
    assert move_calls[0][1:5] == (4.0, 2.0, -5.0, 0.0)
    assert move_calls[1][1:4] == (4.0, 2.0, -5.0)
    assert isclose(move_calls[1][4], pi / 3.0)
    assert move_calls[2][1:5] == (4.0, 2.0, -5.0, 0.0)
    assert move_calls[3][1:5] == (1.0, 2.0, -5.0, 0.0)
    assert gateway.calls == [
        ('takeoff', 5.0, 60.0),
        ('move_leader', 4.0, 2.0, -5.0, 0.0, 0.3, 0.2, 60.0),
        ('move_leader', 4.0, 2.0, -5.0, pi / 3.0, 0.3, 0.2, 60.0),
        ('move_leader', 4.0, 2.0, -5.0, 0.0, 0.3, 0.2, 60.0),
        ('settle', 'vee', 1.0, 30.0, 0.5, 0.25),
        ('move_leader', 1.0, 2.0, -5.0, 0.0, 0.3, 0.2, 60.0),
        ('settle', 'vee', 1.0, 30.0, 0.5, 0.25),
        ('land', 60.0),
    ]


def test_demo_macro_stops_when_settle_times_out():
    gateway = FakeGateway(
        leader_status=leader_status(x=0.0, y=0.0, z=-5.0, yaw=0.0),
        action_results={'settle': ConsoleActionResult(False, 'formation settle timed out')},
    )
    dispatcher = ConsoleCommandDispatcher(
        OperatorConsoleConfig(demo_commands=('1', 'settle', '8')),
        gateway,
    )

    result = dispatcher.dispatch('9')

    assert result.success is False
    assert 'formation settle timed out' in result.message
    assert gateway.calls == [
        ('takeoff', 5.0, 60.0),
        ('settle', 'vee', 1.0, 30.0, 0.5, 0.25),
    ]


def test_demo_macro_stops_on_first_failed_action():
    gateway = FakeGateway(
        leader_status=leader_status(x=0.0, y=0.0, z=-5.0, yaw=0.0),
        action_results={'move_leader': ConsoleActionResult(False, 'move failed')},
    )
    dispatcher = ConsoleCommandDispatcher(OperatorConsoleConfig(), gateway)

    result = dispatcher.dispatch('9')

    assert result.success is False
    assert 'move failed' in result.message
    assert gateway.calls == [
        ('takeoff', 5.0, 60.0),
        ('move_leader', 3.0, 0.0, -5.0, 0.0, 0.3, 0.2, 60.0),
    ]


def test_unknown_command_returns_helpful_failure_without_calling_actions():
    gateway = FakeGateway(leader_status=leader_status())
    dispatcher = ConsoleCommandDispatcher(OperatorConsoleConfig(), gateway)

    result = dispatcher.dispatch('bad')

    assert result.success is False
    assert 'unknown command' in result.message
    assert gateway.calls == []


def test_formation_settle_ready_uses_leader_state_and_fixed_follower_slots():
    statuses = {
        1: leader_status(x=10.0, y=20.0, z=-5.0, yaw=0.0),
        2: follower_status(2, 'follower_left', x=7.0, y=24.0),
        3: follower_status(3, 'follower_right', x=7.0, y=16.0),
    }

    assert formation_settle_ready(statuses, 'vee', OperatorConsoleConfig()) is True


def test_formation_settle_ready_rejects_stale_or_misaligned_followers():
    stale_right = follower_status(3, 'follower_right', x=7.0, y=16.0)
    stale_right.last_telemetry_age_sec = 2.0
    statuses = {
        1: leader_status(x=10.0, y=20.0, z=-5.0, yaw=0.0),
        2: follower_status(2, 'follower_left', x=7.0, y=24.0),
        3: stale_right,
    }

    assert formation_settle_ready(statuses, 'vee', OperatorConsoleConfig()) is False

    statuses[3] = follower_status(3, 'follower_right', x=7.0, y=20.0)

    assert formation_settle_ready(statuses, 'vee', OperatorConsoleConfig()) is False


def test_formation_settle_gate_requires_continuous_stable_window():
    now = 0.0

    def now_s():
        return now

    config = OperatorConsoleConfig(settle_stable_duration_s=1.0)
    gate = FormationSettleGate(config, now_s=now_s)
    ready_statuses = {
        1: leader_status(x=10.0, y=20.0, z=-5.0, yaw=0.0),
        2: follower_status(2, 'follower_left', x=7.0, y=24.0),
        3: follower_status(3, 'follower_right', x=7.0, y=16.0),
    }

    assert gate.update(ready_statuses, 'vee') is False
    now = 0.5
    assert gate.update(ready_statuses, 'vee') is False
    now = 1.1
    assert gate.update(ready_statuses, 'vee') is True

    now = 1.2
    disturbed_statuses = {
        **ready_statuses,
        3: follower_status(3, 'follower_right', x=7.0, y=20.0),
    }
    assert gate.update(disturbed_statuses, 'vee') is False
    now = 2.3
    assert gate.update(ready_statuses, 'vee') is False
