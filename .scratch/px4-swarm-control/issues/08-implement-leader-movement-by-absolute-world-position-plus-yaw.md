# 08 - Implement leader movement by absolute world position plus yaw

**What to build:** Allow the operator to move the leader to an absolute world-frame position and yaw through `MoveLeader`, while followers remain staged or holding.

**Blocked by:** 07 - Deliver synchronized takeoff to staging and land-all milestone.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

Leader movement is the next vertical slice after takeoff/staging/landing works. The first version uses absolute world position plus yaw because it is easier to inspect in Gazebo/RViz than relative commands. QGC may be used as an optional monitoring and manual safety observation tool, but it is not the control entrypoint.

## Scope

- Implement `MoveLeader` action behavior in the ground station.
- Publish leader goal internally as absolute world position plus yaw.
- Have the leader vehicle node consume the leader goal and command PX4 through `Px4VehicleInterface`.
- Keep followers staged or holding; do not make them follow yet.
- Provide action feedback/result based on leader progress.
- Add tests for leader goal publication, leader acceptance, and completion conditions.

## Non-goals

- Do not implement relative movement commands.
- Do not implement follower following.
- Do not implement formation change.
- Do not reassign leader.

## Implementation notes

- Keep the leader goal world-frame and explicit; avoid hidden relative movement semantics.
- Add one-line Chinese comments near goal validation and completion tolerance explaining what operator ambiguity or oscillation they prevent.

## Acceptance criteria

- [ ] `MoveLeader` accepts absolute target x/y/z/yaw.
- [ ] The ground station publishes the leader goal through an internal topic.
- [ ] The `/vehicle_1` leader node consumes the goal and sends position+yaw setpoints.
- [ ] The action reports progress and succeeds when the leader reaches tolerance.
- [ ] Followers do not start formation following as part of this ticket.
- [ ] Tests cover accepted/rejected leader goals and completion behavior.

## Testing approach

- Unit-test leader goal validation and completion tolerance.
- Run ROS 2 package commands from the `px4_ws` workspace, not the outer Docker workspace.
- Use fake vehicle status to test action feedback/result.
- Manually verify in SITL that the leader moves after staging while followers hold.

## Blocking edges

- Blocked by 07 - Deliver synchronized takeoff to staging and land-all milestone.
- Blocks 09 - Implement follower fixed-slot following from leader state.
