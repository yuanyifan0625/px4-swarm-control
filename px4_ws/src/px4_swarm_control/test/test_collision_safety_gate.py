from math import pi

from px4_swarm_control.collision_safety_gate import (
    CollisionSafetyConfig,
    CollisionSafetyGate,
    evaluate_leader_movement,
    VehicleObservation,
)
from px4_swarm_control.models import PositionYawSetpoint, Slot


def observation(vehicle_id, x, y, *, yaw=0.0, age_s=0.1):
    return VehicleObservation(
        vehicle_id=vehicle_id,
        x=x,
        y=y,
        z=-1.0,
        yaw=yaw,
        telemetry_age_s=age_s,
    )


def test_follower_enters_hold_only_after_one_second_below_minimum_distance():
    gate = CollisionSafetyGate(
        CollisionSafetyConfig(
            minimum_horizontal_distance_m=0.7,
            transition_duration_s=1.0,
            fallback_step_m=0.3,
            telemetry_timeout_s=0.5,
        )
    )
    safe_target = PositionYawSetpoint(0.0, 1.0, -1.0, 0.0)
    moving_target = PositionYawSetpoint(1.0, 1.0, -1.0, 0.0)
    own = observation(2, 0.0, 0.0)

    initial = gate.evaluate(
        candidate_target=safe_target,
        own_observation=own,
        peer_observations={1: observation(1, 2.0, 0.0)},
        slot=Slot.FOLLOWER_LEFT,
        leader_yaw=0.0,
        now_s=0.0,
    )
    before_debounce = gate.evaluate(
        candidate_target=moving_target,
        own_observation=own,
        peer_observations={1: observation(1, 0.6, 0.0)},
        slot=Slot.FOLLOWER_LEFT,
        leader_yaw=0.0,
        now_s=1.0,
    )
    still_before_debounce = gate.evaluate(
        candidate_target=moving_target,
        own_observation=own,
        peer_observations={1: observation(1, 0.6, 0.0)},
        slot=Slot.FOLLOWER_LEFT,
        leader_yaw=0.0,
        now_s=1.9,
    )
    held = gate.evaluate(
        candidate_target=moving_target,
        own_observation=own,
        peer_observations={1: observation(1, 0.6, 0.0)},
        slot=Slot.FOLLOWER_LEFT,
        leader_yaw=0.0,
        now_s=2.0,
    )

    assert initial.holding is False
    assert before_debounce.holding is False
    assert still_before_debounce.holding is False
    assert held.holding is True
    assert held.target == safe_target
    assert 'horizontal distance' in held.reason


def test_follower_resumes_only_after_one_second_back_above_minimum_distance():
    gate = CollisionSafetyGate(CollisionSafetyConfig())
    first_target = PositionYawSetpoint(0.0, 1.0, -1.0, 0.0)
    resumed_target = PositionYawSetpoint(1.0, 1.0, -1.0, 0.0)
    own = observation(2, 0.0, 0.0)
    safe_peers = {1: observation(1, 2.0, 0.0)}
    unsafe_peers = {1: observation(1, 0.6, 0.0)}

    gate.evaluate(
        candidate_target=first_target,
        own_observation=own,
        peer_observations=safe_peers,
        slot=Slot.FOLLOWER_LEFT,
        leader_yaw=0.0,
        now_s=0.0,
    )
    gate.evaluate(
        candidate_target=resumed_target,
        own_observation=own,
        peer_observations=unsafe_peers,
        slot=Slot.FOLLOWER_LEFT,
        leader_yaw=0.0,
        now_s=1.0,
    )
    gate.evaluate(
        candidate_target=resumed_target,
        own_observation=own,
        peer_observations=unsafe_peers,
        slot=Slot.FOLLOWER_LEFT,
        leader_yaw=0.0,
        now_s=2.0,
    )

    still_held = gate.evaluate(
        candidate_target=resumed_target,
        own_observation=own,
        peer_observations=safe_peers,
        slot=Slot.FOLLOWER_LEFT,
        leader_yaw=0.0,
        now_s=2.1,
    )
    resumed = gate.evaluate(
        candidate_target=resumed_target,
        own_observation=own,
        peer_observations=safe_peers,
        slot=Slot.FOLLOWER_LEFT,
        leader_yaw=0.0,
        now_s=3.1,
    )

    assert still_held.holding is True
    assert still_held.target == first_target
    assert resumed.holding is False
    assert resumed.target == resumed_target


def test_missing_last_safe_target_uses_slot_direction_fallback():
    left_gate = CollisionSafetyGate(CollisionSafetyConfig())
    right_gate = CollisionSafetyGate(CollisionSafetyConfig())
    own = observation(2, 1.0, 2.0, yaw=0.4)
    stale_peer = {1: observation(1, 2.0, 2.0, age_s=5.0)}
    candidate = PositionYawSetpoint(9.0, 9.0, -1.0, 1.0)

    left = left_gate.evaluate(
        candidate_target=candidate,
        own_observation=own,
        peer_observations=stale_peer,
        slot=Slot.FOLLOWER_LEFT,
        leader_yaw=0.0,
        now_s=0.0,
    )
    right = right_gate.evaluate(
        candidate_target=candidate,
        own_observation=own,
        peer_observations=stale_peer,
        slot=Slot.FOLLOWER_RIGHT,
        leader_yaw=pi / 2.0,
        now_s=0.0,
    )

    assert left.holding is True
    assert left.target == PositionYawSetpoint(1.0, 2.3, -1.0, 0.4)
    assert right.holding is True
    assert right.target == PositionYawSetpoint(1.3, 2.0, -1.0, 0.4)


def test_follower_never_accepts_candidate_target_inside_peer_minimum_distance():
    gate = CollisionSafetyGate(CollisionSafetyConfig())
    own = observation(2, 0.0, 0.0)
    peers = {1: observation(1, 2.0, 0.0)}
    last_safe = PositionYawSetpoint(0.0, 1.0, -1.0, 0.0)
    unsafe_candidate = PositionYawSetpoint(2.1, 0.0, -1.0, 0.0)

    gate.evaluate(
        candidate_target=last_safe,
        own_observation=own,
        peer_observations=peers,
        slot=Slot.FOLLOWER_LEFT,
        leader_yaw=0.0,
        now_s=0.0,
    )
    decision = gate.evaluate(
        candidate_target=unsafe_candidate,
        own_observation=own,
        peer_observations=peers,
        slot=Slot.FOLLOWER_LEFT,
        leader_yaw=0.0,
        now_s=0.1,
    )

    assert decision.holding is True
    assert decision.target == last_safe
    assert 'candidate target' in decision.reason


def test_leader_movement_guard_checks_goal_against_actual_follower_positions():
    actual = {
        1: observation(1, 0.0, 0.0),
        2: observation(2, 2.2, 0.0),
        3: observation(3, -1.0, -1.0),
    }
    theoretical = {
        1: PositionYawSetpoint(2.0, 0.0, -1.0, 0.0),
        2: PositionYawSetpoint(1.0, 1.0, -1.0, 0.0),
        3: PositionYawSetpoint(1.0, -1.0, -1.0, 0.0),
    }

    decision = evaluate_leader_movement(
        actual_observations=actual,
        theoretical_positions=theoretical,
        config=CollisionSafetyConfig(),
    )

    assert decision.allowed is False
    assert 'leader goal' in decision.reason
    assert 'MAV2 actual position' in decision.reason


def test_leader_movement_guard_checks_all_theoretical_formation_positions():
    actual = {
        1: observation(1, 0.0, 0.0),
        2: observation(2, -1.0, 1.0),
        3: observation(3, -1.0, -1.0),
    }
    theoretical = {
        1: PositionYawSetpoint(2.0, 0.0, -1.0, 0.0),
        2: PositionYawSetpoint(2.5, 0.0, -1.0, 0.0),
        3: PositionYawSetpoint(1.0, -1.0, -1.0, 0.0),
    }

    decision = evaluate_leader_movement(
        actual_observations=actual,
        theoretical_positions=theoretical,
        config=CollisionSafetyConfig(),
    )

    assert decision.allowed is False
    assert 'theoretical positions' in decision.reason
    assert 'MAV1' in decision.reason
    assert 'MAV2' in decision.reason


def test_leader_movement_guard_checks_theoretical_targets_against_other_actual_vehicles():
    actual = {
        1: observation(1, 0.0, 0.0),
        2: observation(2, -1.0, 1.0),
        3: observation(3, -1.0, -1.0),
    }
    theoretical = {
        1: PositionYawSetpoint(4.0, 0.0, -1.0, 0.0),
        2: PositionYawSetpoint(3.0, 1.0, -1.0, 0.0),
        3: PositionYawSetpoint(-1.1, 1.0, -1.0, 0.0),
    }

    decision = evaluate_leader_movement(
        actual_observations=actual,
        theoretical_positions=theoretical,
        config=CollisionSafetyConfig(),
    )

    assert decision.allowed is False
    assert 'MAV3 theoretical position' in decision.reason
    assert 'MAV2 actual position' in decision.reason


def test_leader_movement_guard_fails_closed_for_stale_or_missing_telemetry():
    theoretical = {
        1: PositionYawSetpoint(2.0, 0.0, -1.0, 0.0),
        2: PositionYawSetpoint(1.0, 1.0, -1.0, 0.0),
        3: PositionYawSetpoint(1.0, -1.0, -1.0, 0.0),
    }
    stale_actual = {
        1: observation(1, 0.0, 0.0),
        2: observation(2, -1.0, 1.0),
        3: observation(3, -1.0, -1.0, age_s=5.0),
    }
    missing_actual = {1: stale_actual[1], 2: stale_actual[2]}

    stale = evaluate_leader_movement(
        actual_observations=stale_actual,
        theoretical_positions=theoretical,
        config=CollisionSafetyConfig(),
    )
    missing = evaluate_leader_movement(
        actual_observations=missing_actual,
        theoretical_positions=theoretical,
        config=CollisionSafetyConfig(),
    )

    assert stale.allowed is False
    assert 'MAV3 telemetry' in stale.reason
    assert missing.allowed is False
    assert 'MAV3 telemetry' in missing.reason


def test_leader_movement_guard_rejects_existing_actual_separation_violation():
    actual = {
        1: observation(1, 0.0, 0.0),
        2: observation(2, -1.0, 0.2),
        3: observation(3, -1.0, -0.2),
    }
    theoretical = {
        1: PositionYawSetpoint(2.0, 0.0, -1.0, 0.0),
        2: PositionYawSetpoint(1.0, 1.0, -1.0, 0.0),
        3: PositionYawSetpoint(1.0, -1.0, -1.0, 0.0),
    }

    decision = evaluate_leader_movement(
        actual_observations=actual,
        theoretical_positions=theoretical,
        config=CollisionSafetyConfig(),
    )

    assert decision.allowed is False
    assert 'actual positions' in decision.reason
    assert 'MAV2' in decision.reason
    assert 'MAV3' in decision.reason
