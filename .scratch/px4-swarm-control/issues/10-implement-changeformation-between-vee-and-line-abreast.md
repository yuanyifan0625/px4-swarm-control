# 10 - Implement `ChangeFormation` between `vee` and `line_abreast`

**What to build:** Allow the operator to switch formation mode between `vee` and `line_abreast`, with followers transitioning between body-frame slots and the ground station reporting formation completion.

**Blocked by:** 09 - Implement follower fixed-slot following from leader state.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

First-version formation changes should stay simple: only `vee` and `line_abreast`, only fixed slots, and only body-frame slot transitions. The ground station chooses/broadcasts formation mode, while followers compute their own setpoints.

Ticket 07c showed that mission completion must be tied to fresh vehicle status and real tolerance checks, not command publication alone. `ChangeFormation` must therefore report success only after the vehicles actually reach the new mode's slots.

This ticket must preserve the distributed follower-control boundary introduced by ticket 09. A formation change changes the mode and therefore the follower slot offsets; it must not turn the ground station into a continuous publisher of absolute follower target positions.

## Scope

- Implement `ChangeFormation` action behavior.
- Broadcast target formation mode from the ground station.
- Have follower nodes consume formation mode and switch their body-frame target offsets.
- Detect formation establishment from vehicle status and tolerances.
- Log `formation established` from the ground station.
- Add action feedback/result for formation change.
- Add tests for formation mode switching and completion detection.
- Add a manual verification card that proves a full SITL mission can be completed: clean runtime, takeoff to staging, follow leader, change formation, confirm new slots, then land all.

## Non-goals

- Do not implement `column`.
- Do not implement custom arbitrary offsets.
- Do not implement dynamic slot assignment.
- Do not implement follower-follower coordination.
- Do not compute or continuously publish absolute follower target positions from the ground station.
- Do not change vehicle identity or swap follower slots during formation change.

## Implementation notes

- Formation mode changes should alter desired slots, not vehicle identity.
- Ground station broadcasts only the target formation mode; each follower converts its own slot offset to a world-frame setpoint from leader state.
- Apply the 07c lesson: formation completion must use fresh vehicle status and tolerance checks, not the fact that the mode command was published.
- Add a one-line Chinese comment near formation completion logic explaining that it protects the operator from assuming the mode changed before vehicles reached the new slots.

## Design decisions before implementation

- `vee` body-frame offsets:
  - `follower_left`: `(-trail_spacing_m, +lateral_spacing_m, 0)`
  - `follower_right`: `(-trail_spacing_m, -lateral_spacing_m, 0)`
- `line_abreast` body-frame offsets:
  - `follower_left`: `(0, +lateral_spacing_m, 0)`
  - `follower_right`: `(0, -lateral_spacing_m, 0)`
- Follower yaw follows leader yaw in both formation modes.
- `ChangeFormation` does not move the leader. The leader keeps holding or tracking its current leader goal.
- First version only allows `ChangeFormation` while the swarm mission state is `following`; otherwise reject clearly.
- Do not change the `ChangeFormation.action` interface in this ticket. Formation completion uses ground-station config values:
  - `formation_position_tolerance_m = 0.3`
  - `formation_yaw_tolerance_rad = 0.2`
- During a pending `ChangeFormation`, the ground station may republish `/swarm/formation_mode` to avoid a fragile single-shot topic.
- During a pending `ChangeFormation`, the ground station must not publish `/vehicle_2/staging_setpoint` or `/vehicle_3/staging_setpoint` as follower formation targets.
- Formation completion requires:
  - leader status is fresh, armed, Offboard, and `vehicle_state == following`
  - `vehicle_2` remains `follower_left`
  - `vehicle_3` remains `follower_right`
  - follower statuses are fresh, armed, and Offboard
  - followers are within formation position/yaw tolerance of targets derived from leader state, target formation mode, and their fixed slots
- Feedback `progress` is follower completion ratio:
  - `0.0`: no follower reached the new slot
  - `0.5`: one follower reached the new slot
  - `1.0`: both followers reached the new slots
- Do not add custom trajectory interpolation in this ticket. Followers switch their derived setpoint directly and PX4 Offboard position control handles motion.

## Acceptance criteria

- [ ] `ChangeFormation` accepts `vee` and `line_abreast`.
- [ ] Unsupported formation modes are rejected clearly.
- [ ] `ChangeFormation` rejects requests when the swarm is not in `following`.
- [ ] Ground station broadcasts the target formation mode.
- [ ] Ground station republishes only `/swarm/formation_mode` while waiting for completion.
- [ ] Followers compute new body-frame slot setpoints from the target mode.
- [ ] Ground station does not compute or continuously publish absolute follower target positions during formation change.
- [ ] Ground station does not publish `/vehicle_2/staging_setpoint` or `/vehicle_3/staging_setpoint` as follower formation targets during formation change.
- [ ] Follower slot identity remains fixed: `vehicle_2` is follower-left and `vehicle_3` is follower-right.
- [ ] Completion uses `formation_position_tolerance_m = 0.3` and `formation_yaw_tolerance_rad = 0.2`.
- [ ] Completion requires fresh leader and follower statuses, and followers must be within tolerance of targets derived from leader state, target formation mode, and fixed slots.
- [ ] Feedback `progress` reports follower completion ratio as `0.0`, `0.5`, or `1.0`.
- [ ] Ground station logs `formation established` after all vehicles meet formation tolerance.
- [ ] `ChangeFormation` action succeeds only after fresh vehicle status shows the formation reached tolerance.
- [ ] Tests cover both formation modes, invalid mode rejection, distributed follower setpoint derivation, and completion behavior.
- [ ] Manual SITL verification completes one full mission card: `TakeoffSwarm -> MoveLeader/following -> verify vee -> ChangeFormation line_abreast -> verify line_abreast -> ChangeFormation vee -> verify vee -> LandSwarm`, with followers reaching the new formation before success.

## Testing approach

- Unit-test slot offsets for `vee` and `line_abreast`.
- Unit-test that formation mode changes update follower-local offsets without ground-station absolute follower targets.
- Unit-test that formation completion rejects stale vehicle status.
- Unit-test that `ChangeFormation` only accepts requests in `following`.
- Unit-test invalid mode rejection.
- Unit-test stale leader status and stale follower status prevent completion.
- Unit-test followers outside `0.3m` position tolerance or `0.2rad` yaw tolerance do not complete, and followers inside tolerance do complete.
- Unit-test feedback progress values for zero, one, and two followers inside tolerance.
- Run ROS 2 package commands from the `px4_ws` workspace, not the outer Docker workspace.
- Use fake follower status to test action feedback/result and completion detection.
- Start manual SITL verification from a clean runtime, using the cleanup commands documented in the ticket 07 manual smoke file.
- Manually verify in SITL that followers move between `vee` and `line_abreast` while leader continues to define heading, then `LandSwarm` succeeds.

## Blocking edges

- Blocked by 09 - Implement follower fixed-slot following from leader state.
- Blocks 12 - Add swarm launch and SITL smoke acceptance workflow.
