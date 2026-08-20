"""Read-only compatibility checks for the pinned PX4 v1.17 bridge contract."""

from __future__ import annotations

from argparse import ArgumentParser
from dataclasses import dataclass
from pathlib import Path
from subprocess import run
from typing import Mapping, Sequence

from px4_msgs import msg as px4_msg

from px4_swarm_control.bridge_config import PX4_V117, versioned_topic_suffix


@dataclass(frozen=True)
class CompatibilityResult:
    compatible: bool
    errors: tuple[str, ...]
    topic_suffixes: tuple[str, ...]
    summary: str


def inspect_runtime_contract(
    message_types: Mapping[str, type] | None = None,
) -> CompatibilityResult:
    """Inspect generated px4_msgs classes without exposing them to upper layers."""
    types = message_types or {
        contract.message_type: getattr(px4_msg, contract.message_type)
        for contract in PX4_V117.message_contracts
    }
    errors = []
    topics = []
    lines = [
        f'profile={PX4_V117.name}',
        f'px4_firmware_commit={PX4_V117.px4_firmware_commit}',
        f'px4_msgs_commit={PX4_V117.px4_msgs_commit}',
        f'px4_msgs_module={px4_msg.__file__}',
    ]
    for contract in PX4_V117.message_contracts:
        message_type = types[contract.message_type]
        instance = message_type()
        actual_version = int(getattr(message_type, 'MESSAGE_VERSION', 0))
        topic = versioned_topic_suffix(contract.topic_suffix, message_type)
        topics.append(topic)
        lines.append(
            f'{contract.message_type}: version={actual_version}, topic={topic}',
        )
        if actual_version != contract.message_version:
            errors.append(
                f'{contract.message_type} version expected '
                f'{contract.message_version}, actual {actual_version}',
            )
        for field in contract.required_fields:
            present = hasattr(instance, field)
            state = 'present' if present else 'missing'
            lines.append(f'{contract.message_type}.{field}={state}')
            if not present:
                errors.append(f'{contract.message_type} field is required: {field}')
        for field in contract.absent_fields:
            present = hasattr(instance, field)
            state = 'present' if present else 'absent'
            lines.append(f'{contract.message_type}.{field}={state}')
            if present:
                errors.append(f'{contract.message_type} field must be absent: {field}')
    return CompatibilityResult(not errors, tuple(errors), tuple(topics), '\n'.join(lines))


def compare_message_definitions(
    px4_definitions: Mapping[str, str],
    px4_msgs_definitions: Mapping[str, str],
) -> tuple[str, ...]:
    """Compare all message definitions named by the compatibility profile."""
    return tuple(
        f'{contract.message_type} definition differs between PX4 and px4_msgs'
        for contract in PX4_V117.message_contracts
        if px4_definitions.get(contract.message_type)
        != px4_msgs_definitions.get(contract.message_type)
    )


def read_fixed_git_definitions(
    repository: Path,
    revision: str,
    px4_layout: bool,
) -> dict[str, str]:
    definitions = {}
    for contract in PX4_V117.message_contracts:
        path = (
            contract.px4_definition_path
            if px4_layout
            else f'msg/{contract.message_type}.msg'
        )
        completed = run(
            ('git', '-C', str(repository), 'show', f'{revision}:{path}'),
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode != 0:
            raise RuntimeError(
                f'cannot read {revision}:{path} from {repository}: '
                f'{completed.stderr.strip()}',
            )
        definitions[contract.message_type] = completed.stdout
    return definitions


def read_git_head(repository: Path) -> str:
    completed = run(
        ('git', '-C', str(repository), 'rev-parse', 'HEAD'),
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f'cannot read HEAD from {repository}: {completed.stderr.strip()}',
        )
    return completed.stdout.strip()


def main(argv: Sequence[str] | None = None) -> int:
    parser = ArgumentParser(description='Check the pinned PX4 v1.17 ROS contract.')
    parser.add_argument('--px4-source', type=Path, default=Path('PX4-Autopilot'))
    parser.add_argument(
        '--px4-msgs-source',
        type=Path,
        default=Path('px4_ws/src/px4_msgs'),
    )
    args = parser.parse_args(argv)

    result = inspect_runtime_contract()
    print(result.summary)
    source_errors = []
    try:
        px4_head = read_git_head(args.px4_source)
        print(f'px4_source_head={px4_head}')
        if px4_head != PX4_V117.px4_firmware_commit:
            source_errors.append(
                f'PX4 source HEAD expected {PX4_V117.px4_firmware_commit}, '
                f'actual {px4_head}',
            )
        px4_msgs_head = read_git_head(args.px4_msgs_source)
        print(f'px4_msgs_source_head={px4_msgs_head}')
        if px4_msgs_head != PX4_V117.px4_msgs_commit:
            source_errors.append(
                f'px4_msgs source HEAD expected {PX4_V117.px4_msgs_commit}, '
                f'actual {px4_msgs_head}',
            )
        px4_definitions = read_fixed_git_definitions(
            args.px4_source,
            PX4_V117.px4_firmware_commit,
            px4_layout=True,
        )
        px4_msgs_definitions = read_fixed_git_definitions(
            args.px4_msgs_source,
            PX4_V117.px4_msgs_commit,
            px4_layout=False,
        )
        definition_errors = compare_message_definitions(
            px4_definitions,
            px4_msgs_definitions,
        )
    except RuntimeError as exc:
        definition_errors = (str(exc),)

    errors = (*result.errors, *source_errors, *definition_errors)
    if definition_errors:
        print('message_definitions=incompatible')
    else:
        print('message_definitions=matched (8/8)')
    for error in errors:
        print(f'ERROR: {error}')
    return 1 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
