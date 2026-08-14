"""Launch only the real-deployment ground-station node."""

from __future__ import annotations

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


def _config_path() -> Path:
    package_root = Path(__file__).resolve().parents[1]
    return package_root / 'config' / 'three_vehicle_nodes.yaml'


def generate_launch_description() -> LaunchDescription:
    data = yaml.safe_load(_config_path().read_text(encoding='utf-8')) or {}
    parameters = data['ground_station_node']['ros__parameters']
    return LaunchDescription(
        [
            Node(
                package='px4_swarm_control',
                executable='ground_station_node',
                parameters=[parameters],
                output='screen',
            ),
        ],
    )
