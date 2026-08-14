"""Coordinate-frame preflight probe for PX4 DDS, swarm status, and Gazebo."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, sqrt
import subprocess
from time import monotonic, sleep
from typing import Callable, Iterable

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleLocalPosition
from px4_swarm_interfaces.action import MoveLeader
from px4_swarm_interfaces.msg import LeaderGoal
from px4_swarm_interfaces.msg import VehicleStatus


Vector3 = tuple[float, float, float]
PoseYaw = tuple[float, float, float, float]


@dataclass(frozen=True)
class ProbeThresholds:
    """Thresholds used to classify manual axis motion."""

    dominant_delta_m: float = 0.30
    cross_axis_delta_m: float = 0.10
    stable_duration_s: float = 1.0
    timeout_s: float = 15.0


@dataclass(frozen=True)
class AxisExpectation:
    """Expected PX4 local NED axis/sign for a manual field-frame motion."""

    axis: str
    sign: int
    label: str


@dataclass(frozen=True)
class PositionConsistencyResult:
    """Result of comparing raw PX4 local position with swarm status."""

    level: str
    distance_m: float
    message: str


@dataclass(frozen=True)
class AxisClassificationResult:
    """Result of classifying an observed position delta."""

    level: str
    expected_label: str
    expected_axis: str
    dominant_axis: str
    delta: Vector3
    message: str


def position_consistency(
    *,
    raw_position: Vector3,
    status_position: Vector3,
    tolerance_m: float,
) -> PositionConsistencyResult:
    """Compare raw PX4 local position against the package status projection."""
    distance_m = sqrt(
        (raw_position[0] - status_position[0]) ** 2
        + (raw_position[1] - status_position[1]) ** 2
        + (raw_position[2] - status_position[2]) ** 2,
    )
    distance_m = round(distance_m, 10)
    if distance_m <= tolerance_m:
        return PositionConsistencyResult(
            level='PASS',
            distance_m=distance_m,
            message='status matches PX4 local position',
        )
    return PositionConsistencyResult(
        level='ERROR',
        distance_m=distance_m,
        message='status does not match PX4 local position',
    )


def classify_axis_delta(
    *,
    delta: Vector3,
    expected: AxisExpectation,
    thresholds: ProbeThresholds,
) -> AxisClassificationResult:
    """Classify whether a measured delta matches the expected axis/sign."""
    expected_index = _axis_index(expected.axis)
    expected_signed_delta = delta[expected_index] * expected.sign
    dominant_index = max(range(3), key=lambda index: abs(delta[index]))
    dominant_axis = _signed_axis_label(dominant_index, delta[dominant_index])
    expected_axis = _axis_label(expected.axis, expected.sign)
    cross_axis = max(
        abs(delta[index])
        for index in range(3)
        if index != expected_index
    )

    if expected_signed_delta >= thresholds.dominant_delta_m:
        if cross_axis <= thresholds.cross_axis_delta_m:
            return AxisClassificationResult(
                level='PASS',
                expected_label=expected.label,
                expected_axis=expected_axis,
                dominant_axis=dominant_axis,
                delta=delta,
                message=f'{expected.label} matched {expected_axis}',
            )
        return AxisClassificationResult(
            level='WARNING',
            expected_label=expected.label,
            expected_axis=expected_axis,
            dominant_axis=dominant_axis,
            delta=delta,
            message='cross-axis motion too large',
        )

    if expected_signed_delta <= -thresholds.dominant_delta_m:
        return AxisClassificationResult(
            level='WARNING',
            expected_label=expected.label,
            expected_axis=expected_axis,
            dominant_axis=dominant_axis,
            delta=delta,
            message=f'{expected.label} moved in the opposite direction',
        )

    if abs(delta[dominant_index]) >= thresholds.dominant_delta_m:
        return AxisClassificationResult(
            level='WARNING',
            expected_label=expected.label,
            expected_axis=expected_axis,
            dominant_axis=dominant_axis,
            delta=delta,
            message=f'{expected.label} dominant axis is {dominant_axis}',
        )

    return AxisClassificationResult(
        level='ERROR',
        expected_label=expected.label,
        expected_axis=expected_axis,
        dominant_axis=dominant_axis,
        delta=delta,
        message=(
            f'{expected.label} did not reach '
            f'{thresholds.dominant_delta_m:.2f} m before timeout'
        ),
    )


def classify_manual_axis_sample(
    *,
    baseline_raw_position: Vector3,
    raw_position: Vector3,
    status_position: Vector3,
    expected: AxisExpectation,
    thresholds: ProbeThresholds,
    status_position_tolerance_m: float,
) -> AxisClassificationResult:
    """Classify one manual sample only after status/raw consistency passes."""
    consistency = position_consistency(
        raw_position=raw_position,
        status_position=status_position,
        tolerance_m=status_position_tolerance_m,
    )
    delta = _position_delta(baseline_raw_position, raw_position)
    if consistency.level != 'PASS':
        return AxisClassificationResult(
            level='ERROR',
            expected_label=expected.label,
            expected_axis=_axis_label(expected.axis, expected.sign),
            dominant_axis=_dominant_axis_label(delta),
            delta=delta,
            message=consistency.message,
        )
    return classify_axis_delta(
        delta=delta,
        expected=expected,
        thresholds=thresholds,
    )


def gazebo_mapping_observation(px4_label: str, gazebo_delta: Vector3) -> str:
    """Describe which Gazebo world axis most visibly changed."""
    dominant_index = max(range(3), key=lambda index: abs(gazebo_delta[index]))
    return f'{px4_label} appears as Gazebo {_signed_axis_label(dominant_index, gazebo_delta[dominant_index])}'


def format_axis_result(result: AxisClassificationResult) -> str:
    """Format a manual-axis result for terminal output."""
    return (
        f'{result.level}: {result.expected_label} expected {result.expected_axis}; '
        f'dominant={result.dominant_axis}; delta={_format_vector(result.delta)}; '
        f'{result.message}'
    )


def _axis_index(axis: str) -> int:
    normalized = axis.lower().removeprefix('px4_')
    if normalized == 'x':
        return 0
    if normalized == 'y':
        return 1
    if normalized == 'z':
        return 2
    raise ValueError(f'unsupported axis: {axis}')


def _axis_label(axis: str, sign: int) -> str:
    normalized = axis.lower().removeprefix('px4_').upper()
    sign_label = '+' if sign >= 0 else '-'
    return f'{sign_label}{normalized}'


def _dominant_axis_label(delta: Vector3) -> str:
    dominant_index = max(range(3), key=lambda index: abs(delta[index]))
    return _signed_axis_label(dominant_index, delta[dominant_index])


def _signed_axis_label(index: int, value: float) -> str:
    axis = ('X', 'Y', 'Z')[index]
    sign_label = '+' if value >= 0.0 else '-'
    return f'{sign_label}{axis}'


def _format_vector(values: Iterable[float]) -> str:
    x, y, z = values
    return f'({x:+.3f}, {y:+.3f}, {z:+.3f})'


@dataclass(frozen=True)
class ProbeConfig:
    """Runtime parameters for the coordinate frame probe."""

    px4_namespace: str = '/MAV1'
    mode: str = 'commanded'
    axis_step_m: float = 0.30
    up_step_m: float = 0.20
    status_position_tolerance_m: float = 0.05
    gazebo_pose_topic: str = '/world/default/dynamic_pose/info'
    gazebo_model_name: str = 'x500_1'
    thresholds: ProbeThresholds = ProbeThresholds()


class CoordinateFrameProbeNode(Node):
    """ROS 2 node that runs coordinate-frame checks on demand."""

    def __init__(
        self,
        *,
        gazebo_pose_reader: Callable[[str, str], Vector3 | None] | None = None,
    ) -> None:
        super().__init__('coordinate_frame_probe')
        self._config = self._read_config()
        self._latest_status: VehicleStatus | None = None
        self._latest_local_position: VehicleLocalPosition | None = None
        self._latest_trajectory_setpoint: TrajectorySetpoint | None = None
        self._latest_leader_goal: LeaderGoal | None = None
        self._gazebo_pose_reader = gazebo_pose_reader or read_gazebo_model_pose

        namespace = self._config.px4_namespace.rstrip('/')
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            VehicleLocalPosition,
            f'{namespace}/fmu/out/vehicle_local_position_v1',
            self._handle_local_position,
            px4_qos,
        )
        self.create_subscription(
            VehicleStatus,
            f'{namespace}/status',
            self._handle_status,
            10,
        )
        self.create_subscription(
            TrajectorySetpoint,
            f'{namespace}/fmu/in/trajectory_setpoint',
            self._handle_trajectory_setpoint,
            px4_qos,
        )
        self.create_subscription(
            LeaderGoal,
            '/swarm/leader_goal',
            self._handle_leader_goal,
            10,
        )
        self._move_leader_client = ActionClient(self, MoveLeader, '/swarm/move_leader')

    def _read_config(self) -> ProbeConfig:
        self.declare_parameter('px4_namespace', '/MAV1')
        self.declare_parameter('mode', 'commanded')
        self.declare_parameter('axis_step_m', 0.30)
        self.declare_parameter('up_step_m', 0.20)
        self.declare_parameter('dominant_delta_m', 0.30)
        self.declare_parameter('cross_axis_delta_m', 0.10)
        self.declare_parameter('stable_duration_s', 1.0)
        self.declare_parameter('timeout_s', 15.0)
        self.declare_parameter('status_position_tolerance_m', 0.05)
        self.declare_parameter('gazebo_pose_topic', '/world/default/dynamic_pose/info')
        self.declare_parameter('gazebo_model_name', 'x500_1')
        return ProbeConfig(
            px4_namespace=str(self.get_parameter('px4_namespace').value),
            mode=str(self.get_parameter('mode').value),
            axis_step_m=float(self.get_parameter('axis_step_m').value),
            up_step_m=float(self.get_parameter('up_step_m').value),
            status_position_tolerance_m=float(
                self.get_parameter('status_position_tolerance_m').value,
            ),
            gazebo_pose_topic=str(self.get_parameter('gazebo_pose_topic').value),
            gazebo_model_name=str(self.get_parameter('gazebo_model_name').value),
            thresholds=ProbeThresholds(
                dominant_delta_m=float(
                    self.get_parameter('dominant_delta_m').value,
                ),
                cross_axis_delta_m=float(
                    self.get_parameter('cross_axis_delta_m').value,
                ),
                stable_duration_s=float(self.get_parameter('stable_duration_s').value),
                timeout_s=float(self.get_parameter('timeout_s').value),
            ),
        )

    def _handle_status(self, msg: VehicleStatus) -> None:
        self._latest_status = msg

    def _handle_local_position(self, msg: VehicleLocalPosition) -> None:
        self._latest_local_position = msg

    def _handle_trajectory_setpoint(self, msg: TrajectorySetpoint) -> None:
        self._latest_trajectory_setpoint = msg

    def _handle_leader_goal(self, msg: LeaderGoal) -> None:
        self._latest_leader_goal = msg

    def run(self) -> int:
        if self._config.mode == 'commanded':
            return 0 if self._run_commanded_mode() else 1
        if self._config.mode == 'manual':
            return 0 if self._run_manual_mode() else 1
        self.get_logger().error(
            f'unsupported mode: {self._config.mode}; expected commanded or manual',
        )
        return 2

    def _run_commanded_mode(self) -> bool:
        if self._config.px4_namespace.rstrip('/') != '/MAV1':
            self.get_logger().warning(
                'commanded mode moves the swarm leader through /swarm/move_leader; '
                'use /MAV1 unless intentionally probing the leader status topic.',
            )
        if not self._wait_for_baseline():
            return False
        baseline = self._snapshot('BASELINE')
        if baseline is None:
            return False
        self._print_snapshot(baseline)
        if baseline.consistency.level != 'PASS':
            self.get_logger().error(baseline.consistency.message)
            return False
        home = baseline.status_pose
        tests = (
            ('PX4 +X', (home[0] + self._config.axis_step_m, home[1], home[2], home[3])),
            ('PX4 +Y', (home[0], home[1] + self._config.axis_step_m, home[2], home[3])),
            ('PX4 -Z', (home[0], home[1], home[2] - self._config.up_step_m, home[3])),
        )
        success = True
        for label, target in tests:
            result = self._call_move_leader(target)
            after = self._snapshot(label)
            if after is not None:
                self.get_logger().info(f'{label} action: {result.level} {result.message}')
                self._print_delta(label, baseline, after)
                success = (
                    success
                    and result.level == 'PASS'
                    and after.consistency.level == 'PASS'
                )
            else:
                success = False
            return_result = self._call_move_leader(home)
            returned = self._snapshot(f'RETURN AFTER {label}')
            if returned is not None:
                self.get_logger().info(
                    f'return after {label}: {return_result.level} {return_result.message}',
                )
                self._print_delta(f'RETURN AFTER {label}', baseline, returned)
                success = (
                    success
                    and return_result.level == 'PASS'
                    and returned.consistency.level == 'PASS'
                )
            else:
                success = False
        return success

    def _run_manual_mode(self) -> bool:
        if not self._wait_for_baseline():
            return False
        expectations = (
            AxisExpectation(axis='x', sign=1, label='field +X'),
            AxisExpectation(axis='y', sign=1, label='field +Y'),
            AxisExpectation(axis='z', sign=-1, label='field up'),
        )
        all_passed = True
        for expectation in expectations:
            baseline = self._snapshot('MANUAL BASELINE')
            if baseline is None:
                all_passed = False
                continue
            if baseline.consistency.level != 'PASS':
                self.get_logger().error(baseline.consistency.message)
                all_passed = False
                continue
            self.get_logger().info(f'Move vehicle along {expectation.label}...')
            result = self._observe_manual_axis(baseline.raw_position, expectation)
            self.get_logger().info(format_axis_result(result))
            all_passed = all_passed and result.level == 'PASS'
        self.get_logger().info('coordinate frame manual probe complete')
        return all_passed

    def _observe_manual_axis(
        self,
        baseline_position: Vector3,
        expectation: AxisExpectation,
    ) -> AxisClassificationResult:
        stable_since_s: float | None = None
        last_result: AxisClassificationResult | None = None
        deadline = monotonic() + self._config.thresholds.timeout_s
        while monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_local_position is None or self._latest_status is None:
                continue
            result = classify_manual_axis_sample(
                baseline_raw_position=baseline_position,
                raw_position=_local_position_tuple(self._latest_local_position),
                status_position=_status_pose_tuple(self._latest_status)[:3],
                expected=expectation,
                thresholds=self._config.thresholds,
                status_position_tolerance_m=self._config.status_position_tolerance_m,
            )
            last_result = result
            if result.level == 'PASS':
                now_s = monotonic()
                if stable_since_s is None:
                    stable_since_s = now_s
                if now_s - stable_since_s >= self._config.thresholds.stable_duration_s:
                    return result
            else:
                stable_since_s = None
                if result.level == 'WARNING':
                    return result
        if last_result is not None:
            return AxisClassificationResult(
                level='ERROR',
                expected_label=last_result.expected_label,
                expected_axis=last_result.expected_axis,
                dominant_axis=last_result.dominant_axis,
                delta=last_result.delta,
                message=(
                    f'{expectation.label} did not reach '
                    f'{self._config.thresholds.dominant_delta_m:.2f} m before timeout'
                ),
            )
        return AxisClassificationResult(
            level='ERROR',
            expected_label=expectation.label,
            expected_axis=_axis_label(expectation.axis, expectation.sign),
            dominant_axis='+X',
            delta=(0.0, 0.0, 0.0),
            message='no PX4 local position samples before timeout',
        )

    def _wait_for_baseline(self) -> bool:
        deadline = monotonic() + self._config.thresholds.timeout_s
        while monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_local_position is not None and self._latest_status is not None:
                return True
        self.get_logger().error(
            'timed out waiting for PX4 local position and /status samples',
        )
        return False

    def _snapshot(self, label: str) -> '_Snapshot | None':
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._latest_local_position is None or self._latest_status is None:
            self.get_logger().error('missing PX4 local position or /status sample')
            return None
        raw_position = _local_position_tuple(self._latest_local_position)
        status_pose = _status_pose_tuple(self._latest_status)
        consistency = position_consistency(
            raw_position=raw_position,
            status_position=status_pose[:3],
            tolerance_m=self._config.status_position_tolerance_m,
        )
        gazebo_position = self._read_gazebo_pose()
        return _Snapshot(
            label=label,
            raw_position=raw_position,
            status_pose=status_pose,
            consistency=consistency,
            leader_goal=_leader_goal_tuple(self._latest_leader_goal),
            trajectory_setpoint=_trajectory_tuple(self._latest_trajectory_setpoint),
            gazebo_position=gazebo_position,
        )

    def _read_gazebo_pose(self) -> Vector3 | None:
        try:
            return self._gazebo_pose_reader(
                self._config.gazebo_pose_topic,
                self._config.gazebo_model_name,
            )
        except Exception as exc:  # noqa: BLE001 - preflight tool reports optional Gazebo failure.
            self.get_logger().warning(f'Gazebo pose unavailable: {exc}')
            return None

    def _print_snapshot(self, snapshot: '_Snapshot') -> None:
        self.get_logger().info(snapshot.label)
        self.get_logger().info(
            f'  PX4 raw local xyz: {_format_vector(snapshot.raw_position)}',
        )
        self.get_logger().info(
            f'  /status xyz: {_format_vector(snapshot.status_pose[:3])}',
        )
        self.get_logger().info(
            f'  status consistency: {snapshot.consistency.level} '
            f'distance={snapshot.consistency.distance_m:.3f} m',
        )
        if snapshot.trajectory_setpoint is not None:
            self.get_logger().info(
                '  trajectory setpoint xyz: '
                f'{_format_vector(snapshot.trajectory_setpoint[:3])}',
            )
        if snapshot.leader_goal is not None:
            self.get_logger().info(
                f'  leader goal xyz: {_format_vector(snapshot.leader_goal[:3])}',
            )
        if snapshot.gazebo_position is not None:
            self.get_logger().info(
                f'  Gazebo world xyz: {_format_vector(snapshot.gazebo_position)}',
            )

    def _print_delta(self, label: str, before: '_Snapshot', after: '_Snapshot') -> None:
        raw_delta = _position_delta(before.raw_position, after.raw_position)
        status_delta = _position_delta(before.status_pose[:3], after.status_pose[:3])
        self.get_logger().info(f'{label} delta')
        self.get_logger().info(f'  PX4 raw delta: {_format_vector(raw_delta)}')
        self.get_logger().info(f'  status delta: {_format_vector(status_delta)}')
        self.get_logger().info(
            f'  status consistency: {after.consistency.level} '
            f'distance={after.consistency.distance_m:.3f} m',
        )
        if after.gazebo_position is not None and before.gazebo_position is not None:
            gazebo_delta = _position_delta(before.gazebo_position, after.gazebo_position)
            self.get_logger().info(f'  Gazebo world delta: {_format_vector(gazebo_delta)}')
            self.get_logger().info(
                f'  observed: {gazebo_mapping_observation(label, gazebo_delta)}',
            )

    def _call_move_leader(self, target: PoseYaw) -> '_ActionResult':
        if not self._move_leader_client.wait_for_server(timeout_sec=5.0):
            return _ActionResult('ERROR', '/swarm/move_leader unavailable')
        goal = MoveLeader.Goal()
        goal.x = float(target[0])
        goal.y = float(target[1])
        goal.z = float(target[2])
        goal.yaw = float(target[3])
        goal.position_tolerance_m = 0.08
        goal.yaw_tolerance_rad = 0.2
        goal.timeout_sec = 25.0
        send_future = self._move_leader_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return _ActionResult('ERROR', '/swarm/move_leader goal rejected')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        action_result = result_future.result().result
        return _ActionResult(
            'PASS' if bool(action_result.success) else 'ERROR',
            action_result.message,
        )


@dataclass(frozen=True)
class _ActionResult:
    level: str
    message: str


@dataclass(frozen=True)
class _Snapshot:
    label: str
    raw_position: Vector3
    status_pose: PoseYaw
    consistency: PositionConsistencyResult
    leader_goal: PoseYaw | None
    trajectory_setpoint: PoseYaw | None
    gazebo_position: Vector3 | None


def read_gazebo_model_pose(topic: str, model_name: str) -> Vector3 | None:
    """Read one Gazebo pose sample for a model, returning None if unavailable."""
    output = subprocess.check_output(
        ['timeout', '5s', 'gz', 'topic', '-e', '-t', topic, '-n', '1'],
        text=True,
    )
    return parse_gazebo_model_pose(output, model_name)


def parse_gazebo_model_pose(text: str, model_name: str) -> Vector3 | None:
    """Parse a Gazebo pose/info message emitted by ``gz topic -e``."""
    for block in _pose_blocks(text):
        name: str | None = None
        section: str | None = None
        position: dict[str, float] = {}
        for line in block:
            stripped = line.strip()
            if stripped.startswith('name: '):
                name = stripped.split('name: ', 1)[1].strip().strip('"')
            elif stripped == 'position {':
                section = 'position'
            elif stripped.endswith('{'):
                section = None
            elif stripped == '}':
                section = None
            elif section == 'position' and ': ' in stripped:
                key, value = stripped.split(': ', 1)
                if key in {'x', 'y', 'z'}:
                    position[key] = float(value)
        if name == model_name and {'x', 'y', 'z'} <= position.keys():
            return (position['x'], position['y'], position['z'])
    return None


def _pose_blocks(text: str) -> Iterable[list[str]]:
    lines = text.splitlines()
    index = 0
    while index < len(lines):
        if lines[index].strip() != 'pose {':
            index += 1
            continue
        depth = 1
        block: list[str] = []
        index += 1
        while index < len(lines) and depth > 0:
            stripped = lines[index].strip()
            if stripped.endswith('{'):
                depth += 1
            elif stripped == '}':
                depth -= 1
                if depth == 0:
                    break
            block.append(lines[index])
            index += 1
        yield block
        index += 1


def _local_position_tuple(msg: VehicleLocalPosition) -> Vector3:
    return (float(msg.x), float(msg.y), float(msg.z))


def _status_pose_tuple(msg: VehicleStatus) -> PoseYaw:
    return (float(msg.x), float(msg.y), float(msg.z), float(msg.yaw))


def _trajectory_tuple(msg: TrajectorySetpoint | None) -> PoseYaw | None:
    if msg is None:
        return None
    return (
        float(msg.position[0]),
        float(msg.position[1]),
        float(msg.position[2]),
        float(msg.yaw),
    )


def _leader_goal_tuple(msg: LeaderGoal | None) -> PoseYaw | None:
    if msg is None:
        return None
    return (float(msg.x), float(msg.y), float(msg.z), float(msg.yaw))


def _position_delta(before: Vector3, after: Vector3) -> Vector3:
    return (after[0] - before[0], after[1] - before[1], after[2] - before[2])


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CoordinateFrameProbeNode()
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
