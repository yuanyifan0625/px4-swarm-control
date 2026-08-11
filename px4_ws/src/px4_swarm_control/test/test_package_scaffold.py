from pathlib import Path

from px4_swarm_control import __version__
from px4_swarm_control.package_info import (
    EXPECTED_ACTIONS,
    EXPECTED_MESSAGES,
    SWARM_NAMESPACE,
    VEHICLE_NAMESPACES,
)


def test_package_exposes_initial_contract_names():
    assert __version__ == "0.1.0"
    assert EXPECTED_ACTIONS == (
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
        "VehicleStatus",
    )


def test_namespace_scaffold_uses_role_independent_vehicle_names():
    assert SWARM_NAMESPACE == "/swarm"
    assert VEHICLE_NAMESPACES == ("/vehicle_1", "/vehicle_2", "/vehicle_3")


def test_three_vehicle_manual_debug_docs_name_external_prerequisites_and_nodes():
    readme = Path(__file__).parents[1] / 'config' / 'README.md'
    text = readme.read_text()

    assert 'MicroXRCEAgent udp4 -p 8888' in text
    assert 'make px4_sitl' in text
    assert 'vehicle_1' in text
    assert 'vehicle_2' in text
    assert 'vehicle_3' in text
    assert '/fmu/out' in text


def test_three_vehicle_config_file_names_fixed_namespace_layout():
    config = Path(__file__).parents[1] / 'config' / 'three_vehicle_nodes.yaml'
    text = config.read_text()

    assert 'vehicle_1' in text
    assert 'role: leader' in text
    assert 'slot: leader' in text
    assert 'vehicle_2' in text
    assert 'slot: follower_left' in text
    assert 'vehicle_3' in text
    assert 'slot: follower_right' in text
