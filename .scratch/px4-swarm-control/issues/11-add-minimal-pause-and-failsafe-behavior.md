# 11 - Add minimal pause and failsafe behavior

**What to build:** Add the first-version safety behavior: pause, land-all, vehicle telemetry timeout hover/hold, and leader timeout causing followers to hover.

**Blocked by:** 09 - Implement follower fixed-slot following from leader state.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

The spec intentionally limits first-version failsafe behavior. The system should avoid stale commands and give the operator a simple intervention path, but it should not implement complex autonomous recovery.

## Scope

- Implement `PauseSwarm` behavior that holds safe setpoints.
- Ensure `LandSwarm` remains available as operator recovery.
- Detect vehicle telemetry timeout and make the affected vehicle hover or keep its last safe setpoint.
- Detect leader telemetry/status timeout and make followers hover.
- Reflect pause/failsafe states in vehicle status and ground-station mission state.
- Add tests for timeout and pause/land behavior.

## Non-goals

- Do not implement automatic leader reassignment.
- Do not implement dynamic slot reassignment.
- Do not implement complex autonomous recovery.
- Do not implement collision avoidance beyond fixed spacing/staging assumptions.

## Implementation notes

- Timeout guards protect against stale leader or vehicle data producing unsafe setpoints.
- Add one-line Chinese comments near timeout thresholds and hover/hold fallbacks explaining what stale-data risk they protect against.

## Acceptance criteria

- [ ] `PauseSwarm` causes vehicles to hold safe setpoints rather than continue mission progression.
- [ ] `LandSwarm` remains available from paused/failsafe states where practical.
- [ ] A vehicle telemetry timeout causes that vehicle to hover or hold its last safe setpoint.
- [ ] Leader telemetry/status timeout causes followers to hover.
- [ ] Ground station and vehicle statuses expose pause/failsafe state.
- [ ] Tests cover timeout, pause, and land-all paths.

## Testing approach

- Unit-test timeout state transitions using controlled timestamps.
- Run ROS 2 package commands from the `px4_ws` workspace, not the outer Docker workspace.
- Use fake telemetry/status streams to verify followers stop following on leader timeout.
- Manually test pause and land-all in SITL after follower following is available.

## Blocking edges

- Blocked by 09 - Implement follower fixed-slot following from leader state.
- Blocks 12 - Add swarm launch and SITL smoke acceptance workflow.
