"""Launch only the real-deployment MAV1 vehicle node."""

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
    parameters = data['MAV1']['ros__parameters']
    return LaunchDescription(
        [
            Node(
                package='px4_swarm_control',
                executable='vehicle_node',
                namespace='/MAV1',
                name='vehicle_node',
                parameters=[
                    parameters,
                    {
                        'coordinate_profile': 'raw_px4_local',
                        'common_origin_e_m': 0.0,
                        'common_origin_n_m': 0.0,
                        'common_origin_u_m': 0.0,
                        'hold_z': -1.0,
                    },
                ],
                output='screen',
            ),
        ],
    )
