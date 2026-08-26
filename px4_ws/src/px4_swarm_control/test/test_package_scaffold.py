from pathlib import Path

from px4_swarm_control import __version__
from px4_swarm_control.package_info import (
    EXPECTED_ACTIONS,
    EXPECTED_MESSAGES,
    SWARM_NAMESPACE,
    VEHICLE_NAMESPACES,
)


def read_final_manual() -> str:
    manual = (
        Path(__file__).parents[1]
        / 'config'
        / 'FINAL_MANUAL.zh.md'
    )
    return manual.read_text()


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


def test_config_installs_one_final_manual_only():
    config = Path(__file__).parents[1] / 'config'

    assert sorted(path.name for path in config.glob('*.md')) == ['FINAL_MANUAL.zh.md']


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


def test_operator_console_is_the_only_installed_manual_control_entrypoint():
    setup_text = (Path(__file__).parents[1] / 'setup.py').read_text()

    assert 'operator_console = px4_swarm_control.operator_console:main' in setup_text
    assert 'coordinate_frame_probe = ' not in setup_text
    assert 'field_frame_console = ' not in setup_text


def test_final_manual_covers_fixed_sitl_baseline_and_takeoff_safety():
    text = read_final_manual()

    assert 'MicroXRCEAgent udp4 -p 8888' in text
    assert 'PX4_UXRCE_DDS_NS=MAV1' in text
    assert "./build/px4_sitl_default/bin/px4 -d -i 0" in text
    assert "PX4_GZ_MODEL_POSE='-1,1,0'" in text
    assert "./build/px4_sitl_default/bin/px4 -d -i 1" in text
    assert "PX4_GZ_MODEL_POSE='-1,-1,0'" in text
    assert "./build/px4_sitl_default/bin/px4 -d -i 2" in text
    assert 'GZ_IP=127.0.0.1' in text
    assert 'param set NAV_DLL_ACT 0' in text
    assert 'ros2 launch px4_swarm_control swarm_nodes.launch.py' in text
    assert 'ros2 run px4_swarm_control operator_console' in text
    assert '/MAV1/fmu/out/vehicle_local_position_v1' in text
    assert '/MAV1/status' in text
    assert 'command 22' in text
    assert 'fresh staging anchor' in text
