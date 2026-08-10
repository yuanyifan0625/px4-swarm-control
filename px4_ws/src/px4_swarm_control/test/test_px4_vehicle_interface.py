from math import isnan

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLocalPosition,
    VehicleStatus,
)

from px4_swarm_control.models import (
    CommandStatus,
    PositionYawSetpoint,
    VehicleLevelState,
)
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


def make_interface(now_us=123456, namespace='/vehicle_2'):
    return Px4VehicleInterface(
        node=FakeNode(now_us=now_us),
        vehicle_id='vehicle_2',
        px4_namespace=namespace,
        telemetry_timeout_s=0.5,
    )


def test_interface_creates_vehicle_namespaced_px4_topics():
    interface = make_interface(namespace='/vehicle_2')

    publisher_topics = [publisher.topic for publisher in interface.node.publishers]
    subscriber_topics = [subscription[1] for subscription in interface.node.subscriptions]

    assert publisher_topics == [
        '/vehicle_2/fmu/in/offboard_control_mode',
        '/vehicle_2/fmu/in/trajectory_setpoint',
        '/vehicle_2/fmu/in/vehicle_command',
    ]
    assert subscriber_topics == [
        '/vehicle_2/fmu/out/vehicle_local_position',
        '/vehicle_2/fmu/out/vehicle_status',
        '/vehicle_2/fmu/out/vehicle_command_ack',
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


def test_vehicle_commands_include_expected_command_ids_and_params():
    interface = make_interface(now_us=4000)

    interface.arm()
    interface.disarm()
    interface.takeoff(altitude_m=8.0, yaw=1.2)
    interface.land()
    interface.set_offboard_mode()

    commands = interface.vehicle_command_publisher.messages
    assert [msg.command for msg in commands] == [
        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
        VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
        VehicleCommand.VEHICLE_CMD_NAV_LAND,
        VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
    ]
    assert commands[0].param1 == 1.0
    assert commands[1].param1 == 0.0
    assert commands[2].param4 == 1.2
    assert commands[2].param7 == 8.0
    assert commands[4].param1 == 1.0
    assert commands[4].param2 == 6.0
    assert all(msg.from_external for msg in commands)
    assert all(msg.timestamp == 4000 for msg in commands)


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

    vehicle_status = VehicleStatus()
    vehicle_status.timestamp = 1_100_000
    vehicle_status.arming_state = VehicleStatus.ARMING_STATE_ARMED
    vehicle_status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
    vehicle_status.accepts_offboard_setpoints = True

    interface.handle_vehicle_local_position(local_position)
    interface.handle_vehicle_status(vehicle_status)

    state = interface.vehicle_state()
    assert state.vehicle_id == 'vehicle_2'
    assert state.position == (1.0, 2.0, -3.0)
    assert state.velocity == (0.1, 0.2, -0.3)
    assert state.yaw == 0.5
    assert state.armed is True
    assert state.navigation_state == 'offboard'
    assert state.offboard_available is True
    assert state.telemetry_age_s == 0.5
    assert state.vehicle_level_state is VehicleLevelState.IDLE


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
