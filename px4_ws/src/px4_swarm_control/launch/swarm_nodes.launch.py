"""Launch the first-version ROS swarm nodes only."""

from __future__ import annotations

from pathlib import Path

import yaml
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from px4_swarm_control.bridge_config import FIRST_VERSION_VEHICLES
from px4_swarm_control.operation_profile import FORMATION_POSITION_TOLERANCE_M


def _config_path() -> Path:
    package_root = Path(__file__).resolve().parents[1]
    return package_root / 'config' / 'three_vehicle_nodes.yaml'


def _load_vehicle_parameters(config_path: Path) -> dict[str, dict[str, object]]:
    data = yaml.safe_load(config_path.read_text(encoding='utf-8')) or {}
    return {
        vehicle_id: node_config['ros__parameters']
        for vehicle_id, node_config in data.items()
        if vehicle_id.startswith('MAV')
    }


def _load_ground_station_parameters(config_path: Path) -> dict[str, object]:
    data = yaml.safe_load(config_path.read_text(encoding='utf-8')) or {}
    try:
        return data['ground_station_node']['ros__parameters']
    except KeyError as exc:
        raise ValueError(
            'three_vehicle_nodes.yaml must define '
            'ground_station_node.ros__parameters',
        ) from exc


def generate_launch_description() -> LaunchDescription:
    config_path = _config_path()
    vehicle_parameters = _load_vehicle_parameters(config_path)
    ground_station_parameters = _load_ground_station_parameters(config_path)
    launch_entities = [
        DeclareLaunchArgument(
            'formation_position_tolerance_m',
            default_value=f'{FORMATION_POSITION_TOLERANCE_M:.2f}',
            description='Ground-station formation completion position tolerance.',
        ),
    ]
    for vehicle in FIRST_VERSION_VEHICLES:
        # 由同一份 MAV YAML 取 role/slot/target，保護 launch 時不把三台飛機接錯。
        launch_entities.append(
            Node(
                package='px4_swarm_control',
                executable='vehicle_node',
                namespace=vehicle.namespace,
                name='vehicle_node',
                parameters=[
                    vehicle_parameters[vehicle.vehicle_id],
                    {'coordinate_profile': 'gazebo_enu_common_world'},
                ],
                output='screen',
            ),
        )
    launch_entities.append(
        Node(
            package='px4_swarm_control',
            executable='ground_station_node',
            parameters=[
                ground_station_parameters,
                {
                    'formation_position_tolerance_m': LaunchConfiguration(
                        'formation_position_tolerance_m',
                    ),
                    'vertical_axis_up': True,
                },
            ],
            output='screen',
        ),
    )
    return LaunchDescription(launch_entities)
