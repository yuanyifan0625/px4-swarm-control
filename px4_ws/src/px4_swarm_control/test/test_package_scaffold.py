from pathlib import Path

from px4_swarm_control import __version__
from px4_swarm_control.package_info import (
    EXPECTED_ACTIONS,
    EXPECTED_MESSAGES,
    SWARM_NAMESPACE,
    VEHICLE_NAMESPACES,
)


def read_final_sitl_smoke_doc() -> str:
    smoke_doc = (
        Path(__file__).parents[1]
        / 'config'
        / 'final_sitl_manual_smoke.zh.md'
    )
    return smoke_doc.read_text()


def read_final_real_vehicle_doc() -> str:
    smoke_doc = (
        Path(__file__).parents[1]
        / 'config'
        / 'final_real_vehicle_ros2_manual.zh.md'
    )
    return smoke_doc.read_text()


def test_package_exposes_initial_contract_names():
    assert __version__ == "0.1.0"
    assert EXPECTED_ACTIONS == (
        "ArmSwarm",
        "TakeoffSwarm",
        "MoveLeader",
        "ChangeFormation",
        "PauseSwarm",
        "LandSwarm",
    )
    assert EXPECTED_MESSAGES == (
        "LeaderGoal",
        "FormationMode",
        "MissionCommand",
        "FailsafeCommand",
        "VehicleSetpoint",
        "VehicleStatus",
    )


def test_namespace_scaffold_uses_role_independent_vehicle_names():
    assert SWARM_NAMESPACE == "/swarm"
    assert VEHICLE_NAMESPACES == ("/MAV1", "/MAV2", "/MAV3")


def test_three_vehicle_manual_debug_docs_name_external_prerequisites_and_nodes():
    readme = Path(__file__).parents[1] / 'config' / 'README.md'
    text = readme.read_text()

    assert 'MicroXRCEAgent udp4 -p 8888' in text
    assert 'make px4_sitl' in text
    assert 'MAV1' in text
    assert 'MAV2' in text
    assert 'MAV3' in text
    assert '/fmu/out' in text


def test_three_vehicle_config_file_names_fixed_namespace_layout():
    config = Path(__file__).parents[1] / 'config' / 'three_vehicle_nodes.yaml'
    text = config.read_text()

    assert 'MAV1' in text
    assert 'role: leader' in text
    assert 'slot: leader' in text
    assert 'MAV2' in text
    assert 'slot: follower_left' in text
    assert 'MAV3' in text
    assert 'slot: follower_right' in text


def test_final_sitl_smoke_doc_describes_external_runtime_launch_and_console():
    text = read_final_sitl_smoke_doc()

    assert 'MicroXRCEAgent udp4 -p 8888' in text
    assert 'PX4_UXRCE_DDS_NS=MAV1' in text
    assert 'PX4_UXRCE_DDS_NS=MAV2' in text
    assert 'PX4_UXRCE_DDS_NS=MAV3' in text
    assert 'ros2 launch px4_swarm_control swarm_nodes.launch.py' in text
    assert 'formation_position_tolerance_m:=0.10' in text
    assert 'ros2 run px4_swarm_control operator_console' in text
    assert 'ros2 launch px4_swarm_control operator_console.launch.py' in text
    assert 'settle_position_tolerance_m:=0.10' in text
    assert 'settle_stable_duration_s:=1.5' in text
    assert '9' in text
    assert 'x' in text
    assert 'y' in text
    assert 'z' in text
    assert 'c' in text
    assert '/swarm/arm' in text
    assert '/MAV1/status' in text
    assert '/MAV2/status' in text
    assert '/MAV3/status' in text
    assert 'vehicle_state: landed' in text
    assert 'armed: false' in text


def test_final_sitl_smoke_doc_uses_in_container_commands():
    text = read_final_sitl_smoke_doc()

    assert 'docker compose exec ros2_jazzy bash -lc' not in text
    assert '已經進入 container' in text
    assert '/home/ncrl/docker_ubuntu24' in text
    assert 'cd /home/ncrl/docker_ubuntu24/px4_ws' in text


def test_final_real_vehicle_doc_names_distributed_launches_and_mav_contract():
    text = read_final_real_vehicle_doc()

    assert 'real_mav1_vehicle.launch.py' in text
    assert 'real_mav2_vehicle.launch.py' in text
    assert 'real_mav3_vehicle.launch.py' in text
    assert 'real_ground_station.launch.py' in text
    assert 'formation_position_tolerance_m:=0.10' in text
    assert 'ros2 run px4_swarm_control operator_console' in text
    assert 'ros2 launch px4_swarm_control operator_console.launch.py' in text
    assert 'settle_position_tolerance_m:=0.10' in text
    assert 'settle_stable_duration_s:=1.5' in text
    assert '0.12' in text
    assert '0.15' in text
    assert '/MAV1/fmu/out/vehicle_local_position_v1' in text
    assert '/MAV2/fmu/out/vehicle_status_v4' in text
    assert '/MAV3/fmu/out/vehicle_command_ack_v1' in text
    assert '/vehicle_1' in text


def test_coordinate_frame_probe_docs_and_executable_are_registered():
    package_root = Path(__file__).parents[1]
    setup_text = (package_root / 'setup.py').read_text()
    sitl_doc = (
        package_root / 'config' / 'final_sitl_coordinate_frame_command_probe.zh.md'
    ).read_text()
    real_doc = (
        package_root / 'config' / 'final_real_coordinate_frame_manual_probe.zh.md'
    ).read_text()

    assert 'coordinate_frame_probe = px4_swarm_control.coordinate_frame_probe:main' in setup_text
    assert 'ros2 run px4_swarm_control coordinate_frame_probe' in sitl_doc
    assert 'mode:=commanded' in sitl_doc
    assert '/home/ncrl/docker_ubuntu24' in sitl_doc
    assert 'PX4 +X' in sitl_doc
    assert 'Gazebo +Y' in sitl_doc
    assert 'ros2 run px4_swarm_control coordinate_frame_probe' in real_doc
    assert 'mode:=manual' in real_doc
    assert '/MAV1/fmu/out/vehicle_local_position_v1' in real_doc
    assert '/MAV1/status' in real_doc
    assert 'WARNING' in real_doc


def test_field_frame_console_docs_and_executable_are_registered():
    package_root = Path(__file__).parents[1]
    setup_text = (package_root / 'setup.py').read_text()
    sitl_doc = (
        package_root / 'config' / 'final_sitl_field_frame_console_command.zh.md'
    ).read_text()
    real_doc = (
        package_root / 'config' / 'final_real_field_frame_console_manual.zh.md'
    ).read_text()

    assert 'field_frame_console = px4_swarm_control.field_frame_console:main' in setup_text
    assert 'ros2 run px4_swarm_control field_frame_console' in sitl_doc
    assert 'field_x_axis:=px4_y' in sitl_doc
    assert 'field_y_axis:=px4_x' in sitl_doc
    assert 'field_up_sign:=negative' in sitl_doc
    assert 'Gazebo visual profile' in sitl_doc
    assert 's / status' in sitl_doc
    assert 'p / pause' in sitl_doc
    assert 'home_yaw' in sitl_doc
    assert '9 demo' in sitl_doc
    assert 'ros2 run px4_swarm_control field_frame_console' in real_doc
    assert 'coordinate_frame_probe' in real_doc
    assert 'field_x_sign' in real_doc
    assert 'Gazebo visual profile' in real_doc
    assert 's / status' in real_doc
    assert 'p / pause' in real_doc
    assert 'home_yaw' in real_doc
    assert 'operator_console' in real_doc
    assert 'raw PX4 local NED' in real_doc
