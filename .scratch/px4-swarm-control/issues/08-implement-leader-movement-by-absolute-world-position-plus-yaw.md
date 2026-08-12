# 08 - Implement leader movement by absolute world position plus yaw

**What to build:** Allow the operator to move the leader to an absolute world-frame position and yaw through `MoveLeader`, while followers remain staged or holding.

**Blocked by:** 07 - Deliver synchronized takeoff to staging and land-all milestone; 07b - Fix takeoff-to-offboard staging sequencing; 07c - Fix repeatable TakeoffSwarm/LandSwarm mission cycles and stale landed/staging state.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

Leader movement is the next vertical slice after takeoff/staging/landing works. The first version uses absolute world position plus yaw because it is easier to inspect in Gazebo/RViz than relative commands. QGC may be used as an optional monitoring and manual safety observation tool, but it is not the control entrypoint.

Ticket 07c showed that action success, stale status, and topic timing must be treated carefully. `MoveLeader` must use fresh leader status before reporting success, must not reuse stale mission state from takeoff/land cycles, and must remain repeatable after a clean runtime or after a previous mission cycle.

This ticket also protects the distributed follower-control boundary: `MoveLeader` is an operator command for the leader only. Followers must remain staged or holding in this ticket and must not treat `/swarm/leader_goal` or any ground-station absolute position as their own target.

## Scope

- Implement `MoveLeader` action behavior in the ground station.
- Publish leader goal internally as absolute world position plus yaw.
- Have the leader vehicle node consume the leader goal and command PX4 through `Px4VehicleInterface`.
- Keep followers staged or holding; do not make them follow yet.
- Ensure follower vehicle nodes ignore leader goals as movement targets in this ticket.
- Provide action feedback/result based on leader progress.
- Add tests for leader goal publication, leader acceptance, and completion conditions.
- Add a manual verification card that proves a full SITL mission can be completed: clean runtime, takeoff to staging, move only the leader, confirm followers hold, then land all.

## Non-goals

- Do not implement relative movement commands.
- Do not implement follower following.
- Do not implement formation change.
- Do not reassign leader.
- Do not publish or compute continuous absolute follower target positions from the ground station.
- Do not let followers consume `/swarm/leader_goal` as their own movement target.

## Implementation notes

- Keep the leader goal world-frame and explicit; avoid hidden relative movement semantics.
- Apply the 07c lesson: action completion must be based on fresh telemetry and tolerance, not old cached status or command publication alone.
- Treat takeoff/staging per-vehicle setpoints as the only first-version exception where the ground station may publish per-vehicle absolute positions for collision-safe staging.
- Add one-line Chinese comments near goal validation and completion tolerance explaining what operator ambiguity or oscillation they prevent.

## Acceptance criteria

- [ ] `MoveLeader` accepts absolute target x/y/z/yaw.
- [ ] The ground station publishes the leader goal through an internal topic.
- [ ] The `/vehicle_1` leader node consumes the goal and sends position+yaw setpoints.
- [ ] The action reports progress and succeeds only when fresh `/vehicle_1/status` shows the leader reached tolerance.
- [ ] Followers do not start formation following as part of this ticket.
- [ ] Followers ignore `/swarm/leader_goal` as their own movement target and remain staged or holding while the leader moves.
- [ ] Ground station does not publish continuous absolute follower targets during `MoveLeader`.
- [ ] Tests cover accepted/rejected leader goals and completion behavior.
- [ ] Manual SITL verification completes one full mission card: `TakeoffSwarm -> MoveLeader -> LandSwarm`, with only the leader moving during `MoveLeader`.

## Testing approach

- Unit-test leader goal validation and completion tolerance.
- Unit-test that follower roles ignore leader goals as follower movement targets.
- Unit-test that `MoveLeader` completion requires fresh leader status and does not complete from stale cached status.
- Run ROS 2 package commands from the `px4_ws` workspace, not the outer Docker workspace.
- Use fake vehicle status to test action feedback/result.
- Start manual SITL verification from a clean runtime, using the cleanup commands documented in the ticket 07 manual smoke file.
- Manually verify in SITL that the leader moves after staging while followers hold, then `LandSwarm` succeeds.

## Blocking edges

- Blocked by 07 - Deliver synchronized takeoff to staging and land-all milestone.
- Blocked by 07b - Fix takeoff-to-offboard staging sequencing.
- Blocked by 07c - Fix repeatable TakeoffSwarm/LandSwarm mission cycles and stale landed/staging state.
- Blocks 09 - Implement follower fixed-slot following from leader state.
