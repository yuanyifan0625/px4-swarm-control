"""Manual smoke checks for live PX4 Gz multi-vehicle bridge state."""

from __future__ import annotations

from argparse import ArgumentParser, ArgumentTypeError
from math import hypot
from re import search
from subprocess import CompletedProcess, TimeoutExpired, run
from typing import Mapping, Sequence

from px4_swarm_control.bridge_config import (
    FIRST_VERSION_VEHICLES,
    PX4_V118_OUT_TOPIC_SUFFIXES,
)


def expected_px4_instance_commands() -> tuple[str, str, str]:
    """Return copy-paste PX4 commands that keep project namespaces stable."""
    commands = []
    for vehicle in FIRST_VERSION_VEHICLES:
        env = [
            f'PX4_UXRCE_DDS_NS={vehicle.namespace.lstrip("/")}',
            'PX4_SYS_AUTOSTART=4001',
            'PX4_SIM_MODEL=gz_x500',
        ]
        if vehicle.spawn_pose is not None:
            env.insert(1, 'PX4_GZ_STANDALONE=1')
            env.insert(3, f'PX4_GZ_MODEL_POSE="{vehicle.spawn_pose}"')
        command = ' '.join(
            [*env, f'./build/px4_sitl_default/bin/px4 -i {vehicle.px4_instance}'],
        )
        commands.append(command)
    return tuple(commands)


def expected_ros2_topics() -> tuple[str, ...]:
    """Return PX4 v1.18 runtime out topics that must have bridge publishers."""
    return tuple(
        f'{vehicle.namespace}{suffix}'
        for vehicle in FIRST_VERSION_VEHICLES
        for suffix in PX4_V118_OUT_TOPIC_SUFFIXES
    )


def missing_gazebo_models(gz_topic_list_output: str) -> list[str]:
    return [
        vehicle.model_name
        for vehicle in FIRST_VERSION_VEHICLES
        if f'/model/{vehicle.model_name}/' not in gz_topic_list_output
        and f'/model/{vehicle.model_name}' not in gz_topic_list_output
    ]


def missing_ros2_publishers(topic_info_by_topic: Mapping[str, str]) -> list[str]:
    return [
        topic
        for topic in expected_ros2_topics()
        if _publisher_count(topic_info_by_topic.get(topic, '')) < 1
    ]


def disconnected_agent_clients(agent_log_output: str) -> list[str]:
    session_count = agent_log_output.count('session established')
    if session_count >= len(FIRST_VERSION_VEHICLES):
        return []
    missing_count = len(FIRST_VERSION_VEHICLES) - session_count
    return [
        f'missing session {session_count + i + 1}'
        for i in range(missing_count)
    ]


def models_without_separated_pose(pose_info_output: str) -> list[str]:
    poses = _model_xy_poses(pose_info_output)
    missing = [
        vehicle.model_name
        for vehicle in FIRST_VERSION_VEHICLES
        if vehicle.model_name not in poses
    ]
    too_close = set()
    vehicles = list(FIRST_VERSION_VEHICLES)
    for index, left in enumerate(vehicles):
        if left.model_name not in poses:
            continue
        for right in vehicles[index + 1:]:
            if right.model_name not in poses:
                continue
            distance = hypot(
                poses[left.model_name][0] - poses[right.model_name][0],
                poses[left.model_name][1] - poses[right.model_name][1],
            )
            if distance < 1.0:
                too_close.update((left.model_name, right.model_name))
    return [*missing, *sorted(too_close)]


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    print('Expected manual PX4 commands:')
    for index, command in enumerate(expected_px4_instance_commands(), start=1):
        print(f'  Terminal PX4-{index}: {command}')

    gz_result = _run_command(('gz', 'topic', '-l'))
    pose_result = _run_command(
        ('gz', 'topic', '-e', '-t', '/world/default/pose/info', '-n', '1'),
    )
    topic_info = {
        topic: _run_command(('ros2', 'topic', 'info', '-v', topic)).stdout
        for topic in expected_ros2_topics()
    }
    agent_log = args.agent_log.read_text() if args.agent_log else ''

    missing_models = missing_gazebo_models(gz_result.stdout)
    missing_pose = models_without_separated_pose(pose_result.stdout)
    missing_topics = missing_ros2_publishers(topic_info)
    missing_agent_sessions = (
        disconnected_agent_clients(agent_log) if args.agent_log else []
    )

    if missing_models:
        print('Missing Gazebo models: ' + ', '.join(missing_models))
    else:
        print('Gazebo models OK: x500_1, x500_2, x500_3')

    if missing_pose:
        print('Missing or unseparated Gazebo model poses: ' + ', '.join(missing_pose))
    else:
        print('Gazebo model pose separation OK')

    if missing_topics:
        print('Missing ROS 2 PX4 publishers:')
        for topic in missing_topics:
            print(f'  {topic}')
    else:
        print('ROS 2 PX4 publishers OK for all vehicle telemetry topics')

    if args.agent_log:
        if missing_agent_sessions:
            print('Micro XRCE-DDS Agent did not log three established sessions')
        else:
            print('Micro XRCE-DDS Agent sessions OK')
    else:
        print('Agent log not checked; pass --agent-log to verify three sessions')

    if missing_models or missing_pose or missing_topics or missing_agent_sessions:
        return 1
    return 0


def _run_command(command: Sequence[str]) -> CompletedProcess[str]:
    # Smoke check 只讀取現場狀態，保護手動 PX4/Gazebo 流程不被測試腳本改動。
    try:
        return run(command, check=False, capture_output=True, text=True, timeout=5)
    except TimeoutExpired as exc:
        return CompletedProcess(command, 124, exc.stdout or '', exc.stderr or '')


def _publisher_count(topic_info_output: str) -> int:
    for line in topic_info_output.splitlines():
        stripped = line.strip()
        if stripped.startswith('Publisher count:'):
            return int(stripped.split(':', maxsplit=1)[1].strip())
    return 0


def _model_xy_poses(pose_info_output: str) -> dict[str, tuple[float, float]]:
    poses = {}
    current_name = None
    in_position = False
    x = None
    y = None
    for line in pose_info_output.splitlines():
        name_match = search(r'name:\s+"([^"]+)"', line)
        if name_match:
            current_name = name_match.group(1)
            in_position = False
            x = None
            y = None
            continue
        if current_name is None:
            continue
        stripped = line.strip()
        if stripped == 'position {':
            in_position = True
            continue
        if not in_position:
            continue
        if stripped == '}':
            in_position = False
            continue
        if stripped.startswith('x:'):
            x = float(stripped.split(':', maxsplit=1)[1].strip())
        if stripped.startswith('y:'):
            y = float(stripped.split(':', maxsplit=1)[1].strip())
        if x is not None and y is not None:
            poses[current_name] = (x, y)
            in_position = False
    return poses


def _parse_args(argv: Sequence[str] | None):
    parser = ArgumentParser(
        description='Check live PX4 Gz three-vehicle bridge state.',
    )
    parser.add_argument(
        '--agent-log',
        type=_readable_path,
        help='MicroXRCEAgent log file to check for three established sessions.',
    )
    return parser.parse_args(argv)


def _readable_path(value: str):
    from pathlib import Path

    path = Path(value)
    if not path.is_file():
        raise ArgumentTypeError(f'agent log does not exist: {value}')
    return path


if __name__ == '__main__':
    raise SystemExit(main())
