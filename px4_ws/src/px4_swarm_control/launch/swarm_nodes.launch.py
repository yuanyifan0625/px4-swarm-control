"""Launch the first-version ROS swarm nodes only."""

from __future__ import annotations

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from px4_swarm_control.bridge_config import FIRST_VERSION_VEHICLES


def _config_path() -> Path:
    package_root = Path(__file__).resolve().parents[1]
    return package_root / 'config' / 'three_vehicle_nodes.yaml'


def _load_vehicle_parameters(config_path: Path) -> dict[str, dict[str, object]]:
    data = yaml.safe_load(config_path.read_text(encoding='utf-8')) or {}
    return {
        vehicle_id: node_config['ros__parameters']
        for vehicle_id, node_config in data.items()
    }


def generate_launch_description() -> LaunchDescription:
    vehicle_parameters = _load_vehicle_parameters(_config_path())
    nodes = []
    for vehicle in FIRST_VERSION_VEHICLES:
        # 由同一份 MAV YAML 取 role/slot/target，保護 launch 時不把三台飛機接錯。
        nodes.append(
            Node(
                package='px4_swarm_control',
                executable='vehicle_node',
                namespace=vehicle.namespace,
                name='vehicle_node',
                parameters=[vehicle_parameters[vehicle.vehicle_id]],
                output='screen',
            ),
        )
    nodes.append(
        Node(
            package='px4_swarm_control',
            executable='ground_station_node',
            output='screen',
        ),
    )
    return LaunchDescription(nodes)
