"""Launch only the operator console with optional settle overrides."""

from __future__ import annotations

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from px4_swarm_control.operation_profile import FORMATION_POSITION_TOLERANCE_M
from px4_swarm_control.operation_profile import SETTLE_STABLE_DURATION_S


def _config_path() -> Path:
    package_root = Path(__file__).resolve().parents[1]
    return package_root / 'config' / 'operator_console.yaml'


def _load_operator_console_parameters(config_path: Path) -> dict[str, object]:
    data = yaml.safe_load(config_path.read_text(encoding='utf-8')) or {}
    try:
        return data['operator_console']['ros__parameters']
    except KeyError as exc:
        raise ValueError(
            'operator_console.yaml must define '
            'operator_console.ros__parameters',
        ) from exc


def generate_launch_description() -> LaunchDescription:
    parameters = _load_operator_console_parameters(_config_path())
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'settle_position_tolerance_m',
                default_value=f'{FORMATION_POSITION_TOLERANCE_M:.2f}',
                description='Operator settle position tolerance.',
            ),
            DeclareLaunchArgument(
                'settle_stable_duration_s',
                default_value=f'{SETTLE_STABLE_DURATION_S:.1f}',
                description='Seconds that formation must remain settled.',
            ),
            Node(
                package='px4_swarm_control',
                executable='operator_console',
                parameters=[
                    parameters,
                    {
                        'settle_position_tolerance_m': LaunchConfiguration(
                            'settle_position_tolerance_m',
                        ),
                        'settle_stable_duration_s': LaunchConfiguration(
                            'settle_stable_duration_s',
                        ),
                    },
                ],
                output='screen',
            ),
        ],
    )
