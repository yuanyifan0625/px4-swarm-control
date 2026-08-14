from px4_swarm_control.coordinate_frame_probe import (
    AxisExpectation,
    ProbeConfig,
    ProbeThresholds,
    classify_axis_delta,
    classify_manual_axis_sample,
    CoordinateFrameProbeNode,
    format_axis_result,
    gazebo_mapping_observation,
    position_consistency,
)


def test_status_position_consistency_accepts_fresh_matching_px4_local_position():
    result = position_consistency(
        raw_position=(1.0, 2.0, -1.5),
        status_position=(1.02, 1.99, -1.48),
        tolerance_m=0.05,
    )

    assert result.level == 'PASS'
    assert result.distance_m < 0.05


def test_status_position_consistency_rejects_stale_or_mismatched_status():
    result = position_consistency(
        raw_position=(1.0, 2.0, -1.5),
        status_position=(1.10, 2.0, -1.5),
        tolerance_m=0.05,
    )

    assert result.level == 'ERROR'
    assert result.distance_m == 0.10
    assert 'status does not match PX4 local position' in result.message


def test_axis_delta_passes_when_expected_axis_is_dominant_and_cross_axes_stay_small():
    result = classify_axis_delta(
        delta=(0.34, 0.02, -0.01),
        expected=AxisExpectation(axis='x', sign=1, label='field +X'),
        thresholds=ProbeThresholds(
            dominant_delta_m=0.30,
            cross_axis_delta_m=0.10,
            stable_duration_s=1.0,
            timeout_s=15.0,
        ),
    )

    assert result.level == 'PASS'
    assert result.dominant_axis == '+X'
    assert result.expected_axis == '+X'


def test_axis_delta_warns_on_reversed_sign_and_keeps_moving_to_next_axis():
    result = classify_axis_delta(
        delta=(-0.36, 0.02, 0.01),
        expected=AxisExpectation(axis='x', sign=1, label='field +X'),
        thresholds=ProbeThresholds(
            dominant_delta_m=0.30,
            cross_axis_delta_m=0.10,
            stable_duration_s=1.0,
            timeout_s=15.0,
        ),
    )

    assert result.level == 'WARNING'
    assert result.dominant_axis == '-X'
    assert 'opposite direction' in result.message


def test_axis_delta_warns_when_another_axis_is_dominant():
    result = classify_axis_delta(
        delta=(0.04, 0.42, 0.02),
        expected=AxisExpectation(axis='x', sign=1, label='field +X'),
        thresholds=ProbeThresholds(
            dominant_delta_m=0.30,
            cross_axis_delta_m=0.10,
            stable_duration_s=1.0,
            timeout_s=15.0,
        ),
    )

    assert result.level == 'WARNING'
    assert result.dominant_axis == '+Y'
    assert 'dominant axis is +Y' in result.message


def test_axis_delta_warns_when_cross_axis_motion_is_too_large():
    result = classify_axis_delta(
        delta=(0.35, 0.16, 0.01),
        expected=AxisExpectation(axis='x', sign=1, label='field +X'),
        thresholds=ProbeThresholds(
            dominant_delta_m=0.30,
            cross_axis_delta_m=0.10,
            stable_duration_s=1.0,
            timeout_s=15.0,
        ),
    )

    assert result.level == 'WARNING'
    assert 'cross-axis motion too large' in result.message


def test_axis_delta_errors_when_timeout_happens_before_threshold():
    result = classify_axis_delta(
        delta=(0.12, 0.03, 0.01),
        expected=AxisExpectation(axis='x', sign=1, label='field +X'),
        thresholds=ProbeThresholds(
            dominant_delta_m=0.30,
            cross_axis_delta_m=0.10,
            stable_duration_s=1.0,
            timeout_s=15.0,
        ),
    )

    assert result.level == 'ERROR'
    assert 'did not reach 0.30 m' in result.message


def test_gazebo_mapping_observation_names_dominant_gazebo_axis():
    assert (
        gazebo_mapping_observation('PX4 +X', (0.04, 0.29, 0.02))
        == 'PX4 +X appears as Gazebo +Y'
    )
    assert (
        gazebo_mapping_observation('PX4 -Z', (-0.01, 0.03, 0.20))
        == 'PX4 -Z appears as Gazebo +Z'
    )


def test_axis_result_format_includes_level_delta_and_message():
    result = classify_axis_delta(
        delta=(0.34, 0.02, -0.01),
        expected=AxisExpectation(axis='x', sign=1, label='field +X'),
        thresholds=ProbeThresholds(),
    )

    text = format_axis_result(result)

    assert 'PASS' in text
    assert 'delta=(+0.340, +0.020, -0.010)' in text
    assert 'field +X' in text


def test_manual_axis_sample_errors_when_status_no_longer_matches_raw_position():
    result = classify_manual_axis_sample(
        baseline_raw_position=(1.0, 2.0, -1.5),
        raw_position=(1.34, 2.02, -1.51),
        status_position=(1.10, 2.02, -1.51),
        expected=AxisExpectation(axis='x', sign=1, label='field +X'),
        thresholds=ProbeThresholds(),
        status_position_tolerance_m=0.05,
    )

    assert result.level == 'ERROR'
    assert 'status does not match PX4 local position' in result.message


class _CommandedProbeFake:
    def __init__(self):
        self._config = ProbeConfig(axis_step_m=0.30, up_step_m=0.20)
        self.calls = []
        self.snapshots = [
            _FakeSnapshot(status_pose=(1.0, 2.0, -1.5, 0.0), consistency_level='PASS'),
            None,
            None,
            None,
            None,
            None,
            None,
        ]

    def get_logger(self):
        return self

    def warning(self, message):
        self.calls.append(('warning', message))

    def error(self, message):
        self.calls.append(('error', message))

    def info(self, message):
        self.calls.append(('info', message))

    def _wait_for_baseline(self):
        return True

    def _snapshot(self, label):
        self.calls.append(('snapshot', label))
        return self.snapshots.pop(0)

    def _print_snapshot(self, snapshot):
        self.calls.append(('print_snapshot', snapshot.status_pose))

    def _call_move_leader(self, target):
        self.calls.append(('move_leader', target))
        return _FakeActionResult(level='PASS', message='leader reached target')

    def _print_delta(self, label, baseline, after):
        self.calls.append(('print_delta', label))


class _FakeSnapshot:
    def __init__(self, *, status_pose, consistency_level):
        self.status_pose = status_pose
        self.consistency = _FakeActionResult(
            level=consistency_level,
            message='status matches PX4 local position',
        )


class _FakeActionResult:
    def __init__(self, *, level, message):
        self.level = level
        self.message = message


def test_commanded_probe_returns_home_even_when_after_snapshot_fails():
    fake = _CommandedProbeFake()

    success = CoordinateFrameProbeNode._run_commanded_mode(fake)

    assert success is False
    assert ('move_leader', (1.3, 2.0, -1.5, 0.0)) in fake.calls
    assert ('move_leader', (1.0, 2.0, -1.5, 0.0)) in fake.calls
