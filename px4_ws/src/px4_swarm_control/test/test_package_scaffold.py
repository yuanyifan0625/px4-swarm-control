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
    assert 'ros2 run px4_swarm_control operator_console' in text
    assert '9' in text
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
    assert 'ros2 run px4_swarm_control operator_console' in text
    assert '/MAV1/fmu/out/vehicle_local_position_v1' in text
    assert '/MAV2/fmu/out/vehicle_status_v4' in text
    assert '/MAV3/fmu/out/vehicle_command_ack_v1' in text
    assert '/vehicle_1' in text
