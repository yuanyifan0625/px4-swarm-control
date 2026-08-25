"""PX4 speed-profile check and explicit apply command workflow."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from math import isfinite
from numbers import Real
from pathlib import Path
from typing import Iterable, Mapping, Protocol

import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - only used when running outside ROS install.
    get_package_share_directory = None


REQUIRED_PX4_SPEED_PARAMS = frozenset(
    {
        'MPC_XY_VEL_MAX',
        'MPC_Z_VEL_MAX_UP',
        'MPC_YAWRAUTO_MAX',
    }
)
SUPPORTED_PX4_SPEED_PARAMS = REQUIRED_PX4_SPEED_PARAMS
DEFAULT_VEHICLE_IDS = ('MAV1', 'MAV2', 'MAV3')


class ExplicitApplyRequiredError(ValueError):
    """Raised when an apply path is called without deliberate confirmation."""


class Px4ParameterClient(Protocol):
    """Small seam for future live PX4 parameter clients and current unit tests."""

    def get_param(self, name: str) -> float:
        """Return the current PX4 parameter value."""

    def set_param(self, name: str, value: float) -> None:
        """Set a PX4 parameter value."""


@dataclass(frozen=True)
class Px4SpeedProfile:
    """Versioned PX4 runtime parameter profile for smooth demo or bring-up."""

    version: int
    name: str
    description: str
    intended_use: str
    parameters: dict[str, float]


@dataclass(frozen=True)
class Px4ParameterDiff:
    """One current-vs-desired PX4 parameter comparison."""

    vehicle_id: str
    parameter: str
    current: float | None
    desired: float
    matches: bool


def load_speed_profile(path: str | Path) -> Px4SpeedProfile:
    profile_path = Path(path)
    if not profile_path.exists():
        raise FileNotFoundError(profile_path)
    data = yaml.safe_load(profile_path.read_text(encoding='utf-8')) or {}
    if not isinstance(data, dict):
        raise ValueError('speed profile YAML must be a mapping')

    raw_parameters = data.get('parameters')
    if not isinstance(raw_parameters, dict) or not raw_parameters:
        raise ValueError('speed profile must define at least one parameter')

    profile = Px4SpeedProfile(
        version=int(data.get('version', 0)),
        name=str(data.get('name', '')).strip(),
        description=str(data.get('description', '')).strip(),
        intended_use=str(data.get('intended_use', '')).strip(),
        parameters={name: float(value) for name, value in raw_parameters.items()},
    )
    validate_speed_profile(profile)
    return profile


def validate_speed_profile(profile: Px4SpeedProfile) -> None:
    if profile.version != 1:
        raise ValueError('speed profile version must be 1')
    if not profile.name:
        raise ValueError('speed profile name is required')
    if not profile.parameters:
        raise ValueError('speed profile must define at least one parameter')

    missing_parameters = REQUIRED_PX4_SPEED_PARAMS.difference(profile.parameters)
    if missing_parameters:
        missing = ', '.join(sorted(missing_parameters))
        raise ValueError(f'missing required PX4 speed parameters: {missing}')

    for parameter_name, value in profile.parameters.items():
        if parameter_name not in SUPPORTED_PX4_SPEED_PARAMS:
            raise ValueError(f'unsupported PX4 speed parameter: {parameter_name}')
        if not isinstance(value, Real):
            raise ValueError(f'PX4 speed parameter must be numeric: {parameter_name}')
        if not isfinite(float(value)) or float(value) <= 0.0:
            raise ValueError(
                f'PX4 speed parameter must be finite and greater than zero: '
                f'{parameter_name}'
            )


def load_current_values(path: str | Path) -> dict[str, dict[str, float]]:
    current_path = Path(path)
    if not current_path.exists():
        raise FileNotFoundError(current_path)
    data = yaml.safe_load(current_path.read_text(encoding='utf-8')) or {}
    if not isinstance(data, dict):
        raise ValueError('current values YAML must be a vehicle mapping')

    current_values: dict[str, dict[str, float]] = {}
    for vehicle_id, values in data.items():
        if not isinstance(values, dict):
            raise ValueError(f'current values for {vehicle_id} must be a mapping')
        current_values[str(vehicle_id)] = {
            str(name): float(value) for name, value in values.items()
        }
    return current_values


def check_profile(
    profile: Px4SpeedProfile,
    clients: Mapping[str, Px4ParameterClient],
) -> list[Px4ParameterDiff]:
    current_values = {
        vehicle_id: {
            parameter_name: client.get_param(parameter_name)
            for parameter_name in profile.parameters
        }
        for vehicle_id, client in clients.items()
    }
    return diff_profile(profile, current_values)


def diff_profile(
    profile: Px4SpeedProfile,
    current_values_by_vehicle: Mapping[str, Mapping[str, float]],
    *,
    vehicle_ids: Iterable[str] | None = None,
) -> list[Px4ParameterDiff]:
    rows: list[Px4ParameterDiff] = []
    report_vehicle_ids = (
        sorted(set(vehicle_ids))
        if vehicle_ids is not None
        else sorted(current_values_by_vehicle)
    )
    for vehicle_id in report_vehicle_ids:
        current_values = current_values_by_vehicle.get(vehicle_id, {})
        for parameter_name, desired_value in profile.parameters.items():
            current_value = current_values.get(parameter_name)
            rows.append(
                Px4ParameterDiff(
                    vehicle_id=vehicle_id,
                    parameter=parameter_name,
                    current=current_value,
                    desired=desired_value,
                    matches=(
                        current_value is not None
                        and abs(float(current_value) - desired_value) <= 1e-6
                    ),
                )
            )
    return rows


def render_diff_report(rows: Iterable[Px4ParameterDiff]) -> str:
    lines = []
    for row in rows:
        match = 'yes' if row.matches else 'no'
        current = 'missing' if row.current is None else _format_value(row.current)
        desired = _format_value(row.desired)
        lines.append(
            f'{row.vehicle_id} {row.parameter} '
            f'current={current} desired={desired} match={match}'
        )
    return '\n'.join(lines)


def build_pxh_check_commands(
    profile: Px4SpeedProfile,
    vehicle_ids: Iterable[str] = DEFAULT_VEHICLE_IDS,
) -> str:
    blocks = []
    for vehicle_id in vehicle_ids:
        commands = '\n'.join(
            f'param show {parameter_name}' for parameter_name in profile.parameters
        )
        blocks.append(f'# {vehicle_id} PX4 shell\n{commands}')
    return '\n\n'.join(blocks)


def build_pxh_apply_commands(
    profile: Px4SpeedProfile,
    vehicle_ids: Iterable[str] = DEFAULT_VEHICLE_IDS,
    *,
    explicit_apply: bool,
    save: bool = True,
) -> str:
    if not explicit_apply:
        raise ExplicitApplyRequiredError(
            'PX4 speed profile apply requires explicit operator confirmation'
        )
    blocks = []
    for vehicle_id in vehicle_ids:
        # 這裡只產生 PX4 runtime param 指令，保護 ROS setpoint/follower 邏輯不被速度 profile 牽動。
        commands = [
            f'param set {parameter_name} {_format_value(value)}'
            for parameter_name, value in profile.parameters.items()
        ]
        if save:
            commands.append('param save')
        blocks.append(f'# {vehicle_id} PX4 shell\n' + '\n'.join(commands))
    return '\n\n'.join(blocks)


def resolve_profile_path(profile_name: str, profile_dir: str | Path | None = None) -> Path:
    if profile_dir is not None:
        return Path(profile_dir) / f'{profile_name}.yaml'

    local_config = Path(__file__).parents[1] / 'config' / 'px4_speed_profiles'
    if (local_config / f'{profile_name}.yaml').exists():
        return local_config / f'{profile_name}.yaml'

    if get_package_share_directory is not None:
        share_dir = Path(get_package_share_directory('px4_swarm_control'))
        return share_dir / 'config' / 'px4_speed_profiles' / f'{profile_name}.yaml'

    return local_config / f'{profile_name}.yaml'


def _format_value(value: float) -> str:
    if float(value).is_integer():
        return f'{value:.1f}'
    return str(float(value))


def _vehicles_from_arg(raw: str) -> tuple[str, ...]:
    vehicle_ids = tuple(item.strip() for item in raw.split(',') if item.strip())
    if not vehicle_ids:
        raise ValueError('at least one vehicle id is required')
    return vehicle_ids


def main(args: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Check or explicitly apply PX4 speed parameter profiles.',
    )
    parser.add_argument(
        'mode',
        choices=('check', 'apply', 'print-check-commands'),
        help='check compares values, apply prints confirmed pxh commands',
    )
    parser.add_argument('--profile', default='slow_demo')
    parser.add_argument('--profile-dir')
    parser.add_argument('--current-values')
    parser.add_argument('--vehicles', default=','.join(DEFAULT_VEHICLE_IDS))
    parser.add_argument(
        '--yes',
        action='store_true',
        help='required for apply mode',
    )
    parsed = parser.parse_args(list(args) if args is not None else None)

    profile = load_speed_profile(resolve_profile_path(parsed.profile, parsed.profile_dir))
    vehicle_ids = _vehicles_from_arg(parsed.vehicles)

    if parsed.mode == 'check':
        if parsed.current_values:
            rows = diff_profile(
                profile,
                load_current_values(parsed.current_values),
                vehicle_ids=vehicle_ids,
            )
            print(render_diff_report(rows))
            return 0
        print('目前沒有 live PX4 parameter client；請把下列 param show 指令貼到各 PX4 shell 檢查。')
        print(build_pxh_check_commands(profile, vehicle_ids))
        return 0

    if parsed.mode == 'print-check-commands':
        print(build_pxh_check_commands(profile, vehicle_ids))
        return 0

    if parsed.mode == 'apply':
        try:
            commands = build_pxh_apply_commands(
                profile,
                vehicle_ids,
                explicit_apply=parsed.yes,
            )
        except ExplicitApplyRequiredError as exc:
            parser.error(str(exc) + '; add --yes when you intentionally want apply commands')
        print('警告：這會修改 PX4 runtime parameters，請只在起飛前或安全測試流程中執行。')
        print(commands)
        return 0

    raise AssertionError(f'unhandled mode: {parsed.mode}')


if __name__ == '__main__':
    raise SystemExit(main())
