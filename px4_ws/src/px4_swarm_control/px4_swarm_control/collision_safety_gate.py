"""Collision safety decisions for follower targets and leader movement goals."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, isfinite, sin
from typing import Mapping

from px4_swarm_control.models import PositionYawSetpoint, Slot


@dataclass(frozen=True)
class CollisionSafetyConfig:
    """Shared horizontal-separation and telemetry trust settings."""

    minimum_horizontal_distance_m: float = 0.7
    transition_duration_s: float = 1.0
    fallback_step_m: float = 0.3
    telemetry_timeout_s: float = 0.5


@dataclass(frozen=True)
class VehicleObservation:
    """Frame-independent vehicle pose and freshness used by safety decisions."""

    vehicle_id: int
    x: float
    y: float
    z: float
    yaw: float
    telemetry_age_s: float


@dataclass(frozen=True)
class FollowerSafetyDecision:
    """Target selected by the follower safety gate."""

    target: PositionYawSetpoint | None
    holding: bool
    reason: str = ''


@dataclass(frozen=True)
class MovementSafetyDecision:
    """Ground-station decision for a proposed leader movement."""

    allowed: bool
    reason: str = ''


class CollisionSafetyGate:
    """Debounce separation state and retain the last target known to be safe."""

    def __init__(self, config: CollisionSafetyConfig) -> None:
        self.config = config
        self._holding = False
        self._unsafe_since_s: float | None = None
        self._safe_since_s: float | None = None
        self._last_safe_target: PositionYawSetpoint | None = None

    def evaluate(
        self,
        *,
        candidate_target: PositionYawSetpoint,
        own_observation: VehicleObservation | None,
        peer_observations: Mapping[int, VehicleObservation | None],
        slot: Slot,
        leader_yaw: float,
        now_s: float,
    ) -> FollowerSafetyDecision:
        observations = [own_observation, *peer_observations.values()]
        if not all(_observation_is_trusted(item, self.config) for item in observations):
            self._holding = True
            self._unsafe_since_s = None
            self._safe_since_s = None
            fallback = self._last_safe_target or _slot_fallback_target(
                own_observation,
                slot,
                leader_yaw,
                self.config,
            )
            return FollowerSafetyDecision(
                target=fallback,
                holding=True,
                reason='telemetry unavailable or stale',
            )

        assert own_observation is not None
        candidate_unsafe = any(
            hypot(candidate_target.x - peer.x, candidate_target.y - peer.y)
            < self.config.minimum_horizontal_distance_m
            for peer in peer_observations.values()
            if peer is not None
        )
        if candidate_unsafe:
            self._holding = True
            self._unsafe_since_s = None
            self._safe_since_s = None
            fallback = self._last_safe_target or _slot_fallback_target(
                own_observation,
                slot,
                leader_yaw,
                self.config,
            )
            return FollowerSafetyDecision(
                target=fallback,
                holding=True,
                reason='candidate target below minimum peer distance',
            )

        unsafe = any(
            _horizontal_distance(own_observation, peer)
            < self.config.minimum_horizontal_distance_m
            for peer in peer_observations.values()
            if peer is not None
        )

        if not self._holding:
            if unsafe:
                if self._unsafe_since_s is None:
                    self._unsafe_since_s = now_s
                if now_s - self._unsafe_since_s >= self.config.transition_duration_s:
                    self._holding = True
                    self._safe_since_s = None
                    return FollowerSafetyDecision(
                        target=self._last_safe_target or _slot_fallback_target(
                            own_observation,
                            slot,
                            leader_yaw,
                            self.config,
                        ),
                        holding=True,
                        reason='horizontal distance below minimum',
                    )
                return FollowerSafetyDecision(candidate_target, False)

            self._unsafe_since_s = None
            self._last_safe_target = candidate_target
            return FollowerSafetyDecision(candidate_target, False)

        if unsafe:
            self._safe_since_s = None
            return FollowerSafetyDecision(
                target=self._last_safe_target or _slot_fallback_target(
                    own_observation,
                    slot,
                    leader_yaw,
                    self.config,
                ),
                holding=True,
                reason='horizontal distance below minimum',
            )

        if self._safe_since_s is None:
            self._safe_since_s = now_s
        if now_s - self._safe_since_s >= self.config.transition_duration_s:
            self._holding = False
            self._unsafe_since_s = None
            self._safe_since_s = None
            self._last_safe_target = candidate_target
            return FollowerSafetyDecision(candidate_target, False)
        return FollowerSafetyDecision(
            target=self._last_safe_target or _slot_fallback_target(
                own_observation,
                slot,
                leader_yaw,
                self.config,
            ),
            holding=True,
            reason='waiting for safe-distance debounce',
        )


def evaluate_leader_movement(
    *,
    actual_observations: Mapping[int, VehicleObservation | None],
    theoretical_positions: Mapping[int, PositionYawSetpoint],
    config: CollisionSafetyConfig,
    leader_id: int = 1,
) -> MovementSafetyDecision:
    """Reject a leader goal that conflicts with an actual follower position."""
    leader_goal = theoretical_positions.get(leader_id)
    if leader_goal is None:
        return MovementSafetyDecision(False, 'leader theoretical position missing')
    for vehicle_id in sorted(theoretical_positions):
        if not _observation_is_trusted(actual_observations.get(vehicle_id), config):
            return MovementSafetyDecision(
                False,
                f'MAV{vehicle_id} telemetry unavailable or stale',
            )
    actual_ids = sorted(theoretical_positions)
    for index, first_id in enumerate(actual_ids):
        first = actual_observations[first_id]
        assert first is not None
        for second_id in actual_ids[index + 1:]:
            second = actual_observations[second_id]
            assert second is not None
            if (
                _horizontal_distance(first, second)
                < config.minimum_horizontal_distance_m
            ):
                return MovementSafetyDecision(
                    False,
                    f'MAV{first_id} and MAV{second_id} actual positions '
                    'below minimum distance',
                )
    theoretical_ids = sorted(theoretical_positions)
    for index, first_id in enumerate(theoretical_ids):
        first = theoretical_positions[first_id]
        for second_id in theoretical_ids[index + 1:]:
            second = theoretical_positions[second_id]
            if (
                hypot(first.x - second.x, first.y - second.y)
                < config.minimum_horizontal_distance_m
            ):
                return MovementSafetyDecision(
                    False,
                    f'MAV{first_id} and MAV{second_id} theoretical positions '
                    'below minimum distance',
                )
    for target_id, target in sorted(theoretical_positions.items()):
        for actual_id, observation in sorted(actual_observations.items()):
            if target_id == actual_id or observation is None:
                continue
            if (
                hypot(target.x - observation.x, target.y - observation.y)
                < config.minimum_horizontal_distance_m
            ):
                target_name = (
                    'leader goal'
                    if target_id == leader_id
                    else f'MAV{target_id} theoretical position'
                )
                return MovementSafetyDecision(
                    False,
                    f'{target_name} below minimum distance from '
                    f'MAV{actual_id} actual position',
                )
    return MovementSafetyDecision(True)


def _observation_is_trusted(
    observation: VehicleObservation | None,
    config: CollisionSafetyConfig,
) -> bool:
    if observation is None:
        return False
    return (
        all(
            isfinite(value)
            for value in (
                observation.x,
                observation.y,
                observation.z,
                observation.yaw,
                observation.telemetry_age_s,
            )
        )
        and observation.telemetry_age_s <= config.telemetry_timeout_s
    )


def _horizontal_distance(
    first: VehicleObservation,
    second: VehicleObservation,
) -> float:
    return hypot(first.x - second.x, first.y - second.y)


def _slot_fallback_target(
    own_observation: VehicleObservation | None,
    slot: Slot,
    leader_yaw: float,
    config: CollisionSafetyConfig,
) -> PositionYawSetpoint | None:
    if (
        not _observation_is_trusted(own_observation, config)
        or not isfinite(leader_yaw)
        or slot not in (Slot.FOLLOWER_LEFT, Slot.FOLLOWER_RIGHT)
    ):
        return None
    assert own_observation is not None
    body_left_sign = 1.0 if slot is Slot.FOLLOWER_LEFT else -1.0
    delta_x = -sin(leader_yaw) * body_left_sign * config.fallback_step_m
    delta_y = cos(leader_yaw) * body_left_sign * config.fallback_step_m
    if abs(delta_x) < 1e-12:
        delta_x = 0.0
    if abs(delta_y) < 1e-12:
        delta_y = 0.0
    return PositionYawSetpoint(
        own_observation.x + delta_x,
        own_observation.y + delta_y,
        own_observation.z,
        own_observation.yaw,
    )
