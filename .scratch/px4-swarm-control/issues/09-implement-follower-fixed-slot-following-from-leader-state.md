# 09 - Implement follower fixed-slot following from leader state

**What to build:** Make `vehicle_2` and `vehicle_3` follow the leader by subscribing to leader position, yaw, velocity, and status, then deriving their own fixed-slot position+yaw setpoints.

**Blocked by:** 08 - Implement leader movement by absolute world position plus yaw.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

The architecture is logically distributed: each follower computes its own setpoint from leader state and its configured slot. The ground station does not compute continuous follower setpoints.

Ticket 07c showed that stale status, stale mission state, and one-shot topic publication can make a mission appear complete when the vehicles have not actually reached the intended state. Follower following must therefore use fresh leader state, must avoid stale leader commands on timeout, and must remain repeatable after takeoff/land cycles.

Ticket 08 is intentionally leader-only. This ticket is the first place followers may move in response to leader movement, and they may do so only by subscribing to leader state and formation mode, then computing their own fixed-slot setpoint locally.

## Scope

- Publish or expose leader state so followers can subscribe to position, yaw, velocity, and status.
- Publish or expose formation mode so followers can select the active slot offset locally.
- Implement follower setpoint derivation for fixed slots using leader body-frame offsets.
- Start with `vee` formation as the default following mode.
- Ensure `vehicle_2` remains follower-left slot 1 and `vehicle_3` remains follower-right slot 2.
- Have each follower output only its own position+yaw setpoint through `Px4VehicleInterface`.
- Ensure followers never treat `/swarm/leader_goal` or any ground-station absolute leader target as their own target.
- Add tests for follower setpoint derivation and leader timeout hover behavior if not deferred to ticket 11.
- Add a manual verification card that proves a full SITL mission can be completed: clean runtime, takeoff to staging, move leader, followers maintain `vee` slots, then land all.

## Non-goals

- Do not implement `ChangeFormation` yet.
- Do not implement follower-follower coordination.
- Do not implement dynamic slot assignment.
- Do not implement automatic leader reassignment.
- Do not let the ground station compute or continuously publish absolute follower target positions during following.
- Do not use operator leader goals as direct follower goals.

## Implementation notes

- Body-frame offsets protect formation orientation as leader yaw changes.
- Followers derive world-frame setpoints by rotating their body-frame slot offset by current leader yaw, then adding it to current leader world position.
- Apply the 07c lesson: follower outputs must be based on fresh leader status; leader timeout or stale telemetry should cause a safe hover/hold behavior rather than chasing old data.
- Takeoff/staging remains the only exception where the ground station publishes per-vehicle absolute staging positions for collision safety.
- Add one-line Chinese comments near follower setpoint derivation explaining why the offset is rotated by leader yaw.

## Acceptance criteria

- [ ] Followers subscribe to leader position, yaw, velocity, and status.
- [ ] Followers subscribe to current formation mode and use it to select their local fixed-slot offset.
- [ ] `vehicle_2` computes the follower-left slot setpoint from leader state.
- [ ] `vehicle_3` computes the follower-right slot setpoint from leader state.
- [ ] Followers publish only their own position+yaw setpoints.
- [ ] Ground station does not compute continuous follower setpoints.
- [ ] Ground station does not publish continuous absolute follower target positions during following.
- [ ] Followers do not consume `/swarm/leader_goal` as their own target.
- [ ] With leader movement active, followers track their fixed `vee` slots.
- [ ] Tests verify setpoint derivation for both follower slots and several leader yaw values.
- [ ] Manual SITL verification completes one full mission card: `TakeoffSwarm -> MoveLeader with follower following -> LandSwarm`, with followers maintaining default `vee` offsets.

## Testing approach

- Unit-test follower setpoint derivation from internal models.
- Unit-test that followers ignore absolute leader goals and use leader state plus formation mode instead.
- Unit-test leader stale/timeout behavior so followers hold or hover instead of chasing stale leader state.
- Run ROS 2 package commands from the `px4_ws` workspace, not the outer Docker workspace.
- Use fake leader state streams to verify follower outputs.
- Start manual SITL verification from a clean runtime, using the cleanup commands documented in the ticket 07 manual smoke file.
- Manually verify in SITL that followers follow leader movement while maintaining default `vee` offsets, then `LandSwarm` succeeds.

## Blocking edges

- Blocked by 08 - Implement leader movement by absolute world position plus yaw.
- Blocks 10 - Implement `ChangeFormation` between `vee` and `line_abreast`.
- Blocks 11 - Add minimal pause and failsafe behavior.
