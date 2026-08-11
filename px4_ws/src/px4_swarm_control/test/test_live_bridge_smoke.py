from px4_swarm_control.live_bridge_smoke import (
    disconnected_agent_clients,
    expected_px4_instance_commands,
    missing_gazebo_models,
    missing_ros2_publishers,
    models_without_separated_pose,
)


def test_expected_px4_instance_commands_keep_vehicle_namespaces_and_safe_spacing():
    commands = expected_px4_instance_commands()

    assert 'PX4_UXRCE_DDS_NS=vehicle_1' in commands[0]
    assert 'PX4_GZ_STANDALONE=1' not in commands[0]
    assert './build/px4_sitl_default/bin/px4 -i 1' in commands[0]
    assert 'PX4_UXRCE_DDS_NS=vehicle_2' in commands[1]
    assert 'PX4_GZ_STANDALONE=1' in commands[1]
    assert 'PX4_GZ_MODEL_POSE="0,2,0"' in commands[1]
    assert './build/px4_sitl_default/bin/px4 -i 2' in commands[1]
    assert 'PX4_UXRCE_DDS_NS=vehicle_3' in commands[2]
    assert 'PX4_GZ_STANDALONE=1' in commands[2]
    assert 'PX4_GZ_MODEL_POSE="0,-2,0"' in commands[2]
    assert './build/px4_sitl_default/bin/px4 -i 3' in commands[2]


def test_missing_gazebo_models_reports_only_absent_models():
    gz_topics = """
/model/x500_1/command/motor_speed
/world/default/model/x500_1/link/base_link/pose
/model/x500_3/command/motor_speed
"""

    assert missing_gazebo_models(gz_topics) == ['x500_2']


def test_missing_ros2_publishers_requires_versioned_px4_v118_topics():
    topic_info = {
        '/vehicle_1/fmu/out/vehicle_local_position_v1': 'Publisher count: 1\n',
        '/vehicle_1/fmu/out/vehicle_status_v4': 'Publisher count: 1\n',
        '/vehicle_1/fmu/out/vehicle_command_ack_v1': 'Publisher count: 1\n',
        '/vehicle_2/fmu/out/vehicle_local_position_v1': 'Publisher count: 0\n',
        '/vehicle_2/fmu/out/vehicle_status_v4': 'Publisher count: 1\n',
        '/vehicle_2/fmu/out/vehicle_command_ack_v1': 'Publisher count: 1\n',
        '/vehicle_3/fmu/out/vehicle_local_position_v1': 'Publisher count: 1\n',
        '/vehicle_3/fmu/out/vehicle_status_v4': 'Publisher count: 1\n',
        '/vehicle_3/fmu/out/vehicle_command_ack_v1': 'Publisher count: 1\n',
    }

    assert missing_ros2_publishers(topic_info) == [
        '/vehicle_2/fmu/out/vehicle_local_position_v1',
    ]


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
