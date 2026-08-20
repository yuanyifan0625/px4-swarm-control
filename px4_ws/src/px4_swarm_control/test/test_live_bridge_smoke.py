from px4_swarm_control.live_bridge_smoke import (
    disconnected_agent_clients,
    expected_px4_instance_commands,
    expected_px4_prompt_setup,
    expected_ros2_topics,
    missing_gazebo_models,
    missing_ros2_publishers,
    models_without_separated_pose,
    publisher_endpoint_errors,
)


def test_expected_px4_instance_commands_keep_vehicle_namespaces_and_safe_spacing():
    commands = expected_px4_instance_commands()

    assert all(' -s ' not in command for command in commands)
    assert 'PX4_UXRCE_DDS_NS=MAV1' in commands[0]
    assert 'PX4_GZ_STANDALONE=1' not in commands[0]
    assert './build/px4_sitl_default/bin/px4 -i 1' in commands[0]
    assert 'PX4_UXRCE_DDS_NS=MAV2' in commands[1]
    assert 'PX4_GZ_STANDALONE=1' in commands[1]
    assert 'PX4_GZ_MODEL_POSE="0,2,0"' in commands[1]
    assert './build/px4_sitl_default/bin/px4 -i 2' in commands[1]
    assert 'PX4_UXRCE_DDS_NS=MAV3' in commands[2]
    assert 'PX4_GZ_STANDALONE=1' in commands[2]
    assert 'PX4_GZ_MODEL_POSE="0,-2,0"' in commands[2]
    assert './build/px4_sitl_default/bin/px4 -i 3' in commands[2]


def test_v117_sitl_prompt_setup_disables_gcs_requirement_explicitly():
    assert expected_px4_prompt_setup() == 'param set NAV_DLL_ACT 0'


def test_missing_gazebo_models_reports_only_absent_models():
    gz_topics = """
/model/x500_1/command/motor_speed
/world/default/model/x500_1/link/base_link/pose
/model/x500_3/command/motor_speed
"""

    assert missing_gazebo_models(gz_topics) == ['x500_2']


def test_missing_ros2_publishers_requires_px4_v117_output_topics():
    topic_info = {topic: 'Publisher count: 1\n' for topic in expected_ros2_topics()}
    topic_info['/MAV2/fmu/out/vehicle_local_position_v1'] = 'Publisher count: 0\n'

    assert missing_ros2_publishers(topic_info) == [
        '/MAV2/fmu/out/vehicle_local_position_v1',
    ]


def test_publisher_endpoint_contract_requires_bare_dds_and_v117_message_type():
    topic = '/MAV1/fmu/out/vehicle_status_v1'
    valid_info = """
Type: px4_msgs/msg/VehicleStatus
Publisher count: 1
Node name: _CREATED_BY_BARE_DDS_APP_
Endpoint type: PUBLISHER
"""

    assert publisher_endpoint_errors(
        topic,
        'px4_msgs/msg/VehicleStatus',
        valid_info,
    ) == ()

    invalid_info = valid_info.replace(
        'px4_msgs/msg/VehicleStatus',
        'px4_msgs/msg/VehicleStatusV4',
    ).replace('_CREATED_BY_BARE_DDS_APP_', 'vehicle_node')
    assert publisher_endpoint_errors(
        topic,
        'px4_msgs/msg/VehicleStatus',
        invalid_info,
    ) == (
        f'{topic}: expected type px4_msgs/msg/VehicleStatus',
        f'{topic}: publisher is not a bare DDS endpoint',
    )


def test_publisher_endpoint_contract_rejects_bare_dds_subscriber_only():
    topic = '/MAV1/fmu/out/vehicle_status_v1'
    mixed_info = """
Type: px4_msgs/msg/VehicleStatus
Publisher count: 1

Node name: vehicle_node
Endpoint type: PUBLISHER
GID: publisher

Subscription count: 1

Node name: _CREATED_BY_BARE_DDS_APP_
Endpoint type: SUBSCRIPTION
GID: subscriber
"""

    assert publisher_endpoint_errors(
        topic,
        'px4_msgs/msg/VehicleStatus',
        mixed_info,
    ) == (f'{topic}: publisher is not a bare DDS endpoint',)


def test_expected_ros2_topics_do_not_use_old_vehicle_prefixes():
    topics = expected_ros2_topics()

    assert '/MAV1/fmu/out/vehicle_local_position_v1' in topics
    assert '/MAV2/fmu/out/vehicle_status_v1' in topics
    assert '/MAV3/fmu/out/vehicle_command_ack' in topics
    assert '/MAV1/fmu/out/vehicle_land_detected' in topics
    assert '/MAV2/fmu/out/failsafe_flags' in topics
    assert all(not topic.startswith('/vehicle_') for topic in topics)


def test_disconnected_agent_clients_requires_three_established_sessions():
    agent_log = """
[1650000] info     | Root.cpp           | create_client
[1650001] info     | SessionManager.hpp | session established
[1650002] info     | SessionManager.hpp | session established
"""

    assert disconnected_agent_clients(agent_log) == ['missing session 3']
    assert disconnected_agent_clients(agent_log + 'session established\n') == []


def test_models_without_separated_pose_reports_missing_or_close_models():
    pose_info = """
pose {
  name: "x500_1"
  position {
    x: 0
    y: 0
  }
}
pose {
  name: "x500_2"
  position {
    x: 0
    y: 0.5
  }
}
"""

    assert models_without_separated_pose(pose_info) == [
        'x500_3',
        'x500_1',
        'x500_2',
    ]


def test_models_without_separated_pose_accepts_three_spaced_models():
    pose_info = """
pose {
  name: "x500_1"
  position {
    x: 0
    y: 0
  }
}
pose {
  name: "x500_2"
  position {
    x: 0
    y: 2
  }
}
pose {
  name: "x500_3"
  position {
    x: 0
    y: -2
  }
}
"""

    assert models_without_separated_pose(pose_info) == []
