from math import isnan
from types import SimpleNamespace

import pytest

from px4_msgs.msg import (
    FailsafeFlags,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)

from px4_swarm_control.models import (
    CommandStatus,
    PositionYawSetpoint,
    VehicleLevelState,
)
from px4_swarm_control.px4_vehicle_interface import PX4_CUSTOM_MAIN_MODE_AUTO
from px4_swarm_control.px4_vehicle_interface import PX4_CUSTOM_MAIN_MODE_OFFBOARD
from px4_swarm_control.px4_vehicle_interface import PX4_CUSTOM_MODE_ENABLED
from px4_swarm_control.px4_vehicle_interface import PX4_CUSTOM_SUB_MODE_AUTO_LOITER
from px4_swarm_control.px4_vehicle_interface import Px4VehicleInterface


class FakePublisher:
    def __init__(self, msg_type, topic, qos_profile):
        self.msg_type = msg_type
        self.topic = topic
        self.qos_profile = qos_profile
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeNode:
    def __init__(self, now_us=123456):
        self.now_us = now_us
        self.publishers = []
        self.subscriptions = []

    def create_publisher(self, msg_type, topic, qos_profile):
        publisher = FakePublisher(msg_type, topic, qos_profile)
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, msg_type, topic, callback, qos_profile):
        self.subscriptions.append((msg_type, topic, callback, qos_profile))
        return callback

    def px4_now_us(self):
        return self.now_us


def make_interface(now_us=123456, namespace='/MAV2', px4_target_system=3):
    return Px4VehicleInterface(
        node=FakeNode(now_us=now_us),
        vehicle_id='MAV2',
        px4_namespace=namespace,
        px4_target_system=px4_target_system,
        telemetry_timeout_s=0.5,
    )


def test_interface_creates_vehicle_namespaced_px4_topics():
    interface = make_interface(namespace='/MAV2')

    publisher_topics = [publisher.topic for publisher in interface.node.publishers]
    subscriber_topics = [subscription[1] for subscription in interface.node.subscriptions]

    assert publisher_topics == [
        '/MAV2/fmu/in/offboard_control_mode',
        '/MAV2/fmu/in/trajectory_setpoint',
        '/MAV2/fmu/in/vehicle_command',
    ]
    assert subscriber_topics == [
        '/MAV2/fmu/out/vehicle_local_position_v1',
        '/MAV2/fmu/out/vehicle_status_v1',
        '/MAV2/fmu/out/vehicle_command_ack',
        '/MAV2/fmu/out/vehicle_land_detected',
        '/MAV2/fmu/out/failsafe_flags',
    ]


def test_publish_offboard_heartbeat_enables_only_position_control():
    interface = make_interface(now_us=2000)

    interface.publish_offboard_heartbeat()

    msg = interface.offboard_control_mode_publisher.messages[-1]
    assert isinstance(msg, OffboardControlMode)
    assert msg.timestamp == 2000
    assert msg.position is True
    assert msg.velocity is False
    assert msg.acceleration is False
    assert msg.attitude is False
    assert msg.body_rate is False
    assert msg.thrust_and_torque is False
    assert msg.direct_actuator is False


def test_publish_position_yaw_setpoint_maps_internal_model_to_px4_message():
    interface = make_interface(now_us=3000)

    interface.publish_position_yaw_setpoint(
        PositionYawSetpoint(x=1.0, y=2.0, z=-3.0, yaw=0.75),
    )

    msg = interface.trajectory_setpoint_publisher.messages[-1]
    assert isinstance(msg, TrajectorySetpoint)
    assert msg.timestamp == 3000
    assert list(msg.position) == [1.0, 2.0, -3.0]
    assert msg.yaw == 0.75
    assert all(isnan(value) for value in msg.velocity)
    assert all(isnan(value) for value in msg.acceleration)
    assert all(isnan(value) for value in msg.jerk)
    assert isnan(msg.yawspeed)


def test_vehicle_commands_never_publish_nav_takeoff():
    interface = make_interface(now_us=4000)

    interface.arm()
    interface.disarm()
    interface.land()
    interface.set_offboard_mode()
    interface.set_ground_safe_mode()

    commands = interface.vehicle_command_publisher.messages
    assert [msg.command for msg in commands] == [
        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
        VehicleCommand.VEHICLE_CMD_NAV_LAND,
        VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
        VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
    ]
    assert commands[0].param1 == 1.0
    assert commands[1].param1 == 0.0
    assert isnan(commands[2].param5)
    assert isnan(commands[2].param6)
    assert commands[3].param1 == PX4_CUSTOM_MODE_ENABLED
    assert commands[3].param2 == PX4_CUSTOM_MAIN_MODE_OFFBOARD
    assert commands[4].param1 == PX4_CUSTOM_MODE_ENABLED
    assert commands[4].param2 == PX4_CUSTOM_MAIN_MODE_AUTO
    assert commands[4].param3 == PX4_CUSTOM_SUB_MODE_AUTO_LOITER
    assert all(msg.from_external for msg in commands)
    assert all(msg.timestamp == 4000 for msg in commands)
    assert all(msg.target_system == 3 for msg in commands)


def test_local_position_ready_requires_fresh_valid_finite_non_dead_reckoning_pose():
    interface = make_interface(now_us=1_500_000)
    local_position = VehicleLocalPosition()
    local_position.timestamp = 1_200_000
    local_position.x = 1.0
    local_position.y = 2.0
    local_position.z = -3.0
    local_position.heading = 0.5
    local_position.xy_valid = True
    local_position.z_valid = True
    local_position.dead_reckoning = False
    interface.handle_vehicle_local_position(local_position)

    assert interface.local_position_ready() is True

    local_position.dead_reckoning = True
    assert interface.local_position_ready() is False

    local_position.dead_reckoning = False
    local_position.xy_valid = False
    assert interface.local_position_ready() is False

    local_position.xy_valid = True
    local_position.z_valid = False
    assert interface.local_position_ready() is False

    local_position.z_valid = True
    local_position.heading = float('nan')
    assert interface.local_position_ready() is False


@pytest.mark.parametrize(
    ('attribute', 'value'),
    [
        ('x', float('nan')),
        ('y', float('inf')),
        ('z', float('-inf')),
        ('heading', float('inf')),
        ('xy_valid', False),
        ('z_valid', False),
        ('dead_reckoning', True),
        ('timestamp', 900_000),
    ],
)
def test_local_position_ready_rejects_each_required_invalid_combination(
    attribute,
    value,
):
    interface = make_interface(now_us=1_500_000)
    local_position = VehicleLocalPosition()
    local_position.timestamp = 1_200_000
    local_position.x = 1.0
    local_position.y = 2.0
    local_position.z = -3.0
    local_position.heading = 0.5
    local_position.xy_valid = True
    local_position.z_valid = True
    local_position.dead_reckoning = False
    setattr(local_position, attribute, value)
    interface.handle_vehicle_local_position(local_position)

    assert interface.local_position_ready() is False


def test_vehicle_command_target_system_can_be_broadcast_for_smoke_testing():
    interface = make_interface(now_us=4000, px4_target_system=0)

    interface.land()

    msg = interface.vehicle_command_publisher.messages[-1]
    assert msg.target_system == 0


def test_safe_hover_republishes_last_setpoint_when_available():
    interface = make_interface(now_us=5000)

    setpoint = PositionYawSetpoint(x=4.0, y=5.0, z=-6.0, yaw=-0.4)
    interface.publish_position_yaw_setpoint(setpoint)
    interface.node.now_us = 6000
    assert interface.publish_safe_hover_setpoint() is True

    msg = interface.trajectory_setpoint_publisher.messages[-1]
    assert msg.timestamp == 6000
    assert list(msg.position) == [4.0, 5.0, -6.0]
    assert msg.yaw == -0.4


def test_vehicle_state_converts_latest_px4_telemetry_to_internal_model():
    interface = make_interface(now_us=1_500_000)
    local_position = VehicleLocalPosition()
    local_position.timestamp = 1_000_000
    local_position.x = 1.0
    local_position.y = 2.0
    local_position.z = -3.0
    local_position.vx = 0.1
    local_position.vy = 0.2
    local_position.vz = -0.3
    local_position.heading = 0.5

    vehicle_status = SimpleNamespace(
        timestamp=1_100_000,
        arming_state=VehicleStatus.ARMING_STATE_ARMED,
        nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD,
        accepts_offboard_setpoints=True,
        pre_flight_checks_pass=False,
    )

    failsafe_flags = FailsafeFlags()
    failsafe_flags.offboard_control_signal_lost = True

    interface.handle_vehicle_local_position(local_position)
    interface.handle_vehicle_status(vehicle_status)
    interface.handle_failsafe_flags(failsafe_flags)

    state = interface.vehicle_state()
    assert state.vehicle_id == 'MAV2'
    assert state.position == (1.0, 2.0, -3.0)
    assert state.velocity == (0.1, 0.2, -0.3)
    assert state.yaw == 0.5
    assert state.armed is True
    assert state.navigation_state == 'offboard'
    assert state.offboard_available is True
    assert state.pre_flight_checks_pass is False
    assert state.offboard_control_signal_lost is True
    assert state.telemetry_age_s == 0.5
    assert state.vehicle_level_state is VehicleLevelState.IDLE
    assert state.landed is False


def test_vehicle_state_treats_missing_offboard_acceptance_as_unavailable():
    interface = make_interface(now_us=1_500_000)
    local_position = VehicleLocalPosition()
    local_position.timestamp = 1_000_000
    vehicle_status = SimpleNamespace(
        arming_state=VehicleStatus.ARMING_STATE_ARMED,
        nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD,
        pre_flight_checks_pass=True,
    )

    interface.handle_vehicle_local_position(local_position)
    interface.handle_vehicle_status(vehicle_status)

    state = interface.vehicle_state()

    assert state.offboard_available is False


def test_vehicle_state_reports_landed_from_px4_land_detected_topic():
    interface = make_interface(now_us=1_500_000)
    local_position = VehicleLocalPosition()
    local_position.timestamp = 1_400_000
    local_position.z = -0.02

    vehicle_status = VehicleStatus()
    vehicle_status.arming_state = VehicleStatus.ARMING_STATE_DISARMED
    vehicle_status.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LAND

    land_detected = VehicleLandDetected()
    land_detected.landed = True

    interface.handle_vehicle_local_position(local_position)
    interface.handle_vehicle_status(vehicle_status)
    interface.handle_vehicle_land_detected(land_detected)

    state = interface.vehicle_state()
    assert state.landed is True
    assert state.vehicle_level_state is VehicleLevelState.LANDED


def test_telemetry_is_stale_when_missing_or_older_than_timeout():
    interface = make_interface(now_us=2_000_000)

    assert interface.is_telemetry_stale() is True

    local_position = VehicleLocalPosition()
    local_position.timestamp = 1_400_000
    interface.handle_vehicle_local_position(local_position)

    assert interface.is_telemetry_stale() is True

    local_position.timestamp = 1_800_000
    interface.handle_vehicle_local_position(local_position)

    assert interface.is_telemetry_stale() is False


def test_command_ack_updates_normalized_command_result():
    interface = make_interface()
    ack = VehicleCommandAck()
    ack.result = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED

    interface.handle_vehicle_command_ack(ack)

    assert interface.last_command_result.status is CommandStatus.ACCEPTED
