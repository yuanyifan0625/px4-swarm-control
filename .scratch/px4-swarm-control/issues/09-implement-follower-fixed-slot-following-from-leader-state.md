# 09 - Implement follower fixed-slot following from leader state

**What to build:** Make `vehicle_2` and `vehicle_3` follow the leader by subscribing to leader position, yaw, velocity, and status, then deriving their own fixed-slot position+yaw setpoints.

**Blocked by:** 08 - Implement leader movement by absolute world position plus yaw.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

The architecture is logically distributed: each follower computes its own setpoint from leader state and its configured slot. The ground station does not compute continuous follower setpoints.

## Scope

- Publish or expose leader state so followers can subscribe to position, yaw, velocity, and status.
- Implement follower setpoint derivation for fixed slots using leader body-frame offsets.
- Start with `vee` formation as the default following mode.
- Ensure `vehicle_2` remains follower-left slot 1 and `vehicle_3` remains follower-right slot 2.
- Have each follower output only its own position+yaw setpoint through `Px4VehicleInterface`.
- Add tests for follower setpoint derivation and leader timeout hover behavior if not deferred to ticket 11.

## Non-goals

- Do not implement `ChangeFormation` yet.
- Do not implement follower-follower coordination.
- Do not implement dynamic slot assignment.
- Do not implement automatic leader reassignment.

## Implementation notes

- Body-frame offsets protect formation orientation as leader yaw changes.
- Add one-line Chinese comments near follower setpoint derivation explaining why the offset is rotated by leader yaw.

## Acceptance criteria

- [ ] Followers subscribe to leader position, yaw, velocity, and status.
- [ ] `vehicle_2` computes the follower-left slot setpoint from leader state.
- [ ] `vehicle_3` computes the follower-right slot setpoint from leader state.
- [ ] Followers publish only their own position+yaw setpoints.
- [ ] Ground station does not compute continuous follower setpoints.
- [ ] With leader movement active, followers track their fixed `vee` slots.
- [ ] Tests verify setpoint derivation for both follower slots and several leader yaw values.

## Testing approach

- Unit-test follower setpoint derivation from internal models.
- Run ROS 2 package commands from the `px4_ws` workspace, not the outer Docker workspace.
- Use fake leader state streams to verify follower outputs.
- Manually verify in SITL that followers follow leader movement while maintaining default `vee` offsets.

## Blocking edges

- Blocked by 08 - Implement leader movement by absolute world position plus yaw.
- Blocks 10 - Implement `ChangeFormation` between `vee` and `line_abreast`.
- Blocks 11 - Add minimal pause and failsafe behavior.
