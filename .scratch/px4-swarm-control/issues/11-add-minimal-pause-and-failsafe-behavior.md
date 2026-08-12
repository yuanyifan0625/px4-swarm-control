# 11 - Add minimal pause and failsafe behavior

**What to build:** Add the first-version safety behavior: pause without restarting runtime, land-all recovery, vehicle telemetry timeout hover/hold, and leader timeout causing followers to hover.

**Blocked by:** 09 - Implement follower fixed-slot following from leader state.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

The spec intentionally limits first-version failsafe behavior. The system should avoid stale commands and give the operator a simple intervention path, but it should not implement complex autonomous recovery.

Ticket 07c proved that mission state must stay repeatable without restarting DDS, PX4, Gazebo, vehicle nodes, or the ground station. The same rule applies to pause/failsafe: a normal pause is a runtime state transition, not a reason to restart the simulation stack.

## Scope

- Implement `PauseSwarm` behavior that holds safe setpoints without restarting Micro XRCE-DDS Agent, PX4 SITL, Gazebo, vehicle nodes, or the ground station.
- Implement resume semantics for `PauseSwarm(pause=false)`: resume leaves the swarm in a safe holding state, not automatic continuation of the old move/formation command.
- Ensure `LandSwarm` remains available as operator recovery.
- Allow `LandSwarm` while paused or in failsafe where PX4 telemetry and command paths are still available.
- Detect vehicle telemetry timeout and make the affected vehicle hover or keep its last safe setpoint.
- Detect leader telemetry/status timeout and make followers hover.
- Reflect pause/failsafe states in vehicle status and ground-station mission state.
- Restrict paused-state operator behavior: allow status observation, resume, and land-all; reject new leader movement, formation change, and demo/macro progression while paused.
- Add tests for timeout and pause/land behavior.

## Non-goals

- Do not implement automatic leader reassignment.
- Do not implement dynamic slot reassignment.
- Do not implement complex autonomous recovery.
- Do not implement collision avoidance beyond fixed spacing/staging assumptions.
- Do not add the short-number operator console in this ticket; that belongs to ticket 11b.
- Do not make pause restart or respawn DDS, PX4, Gazebo, vehicle nodes, or the ground station.
- Do not automatically continue a stale pre-pause mission after resume.

## Implementation notes

- Timeout guards protect against stale leader or vehicle data producing unsafe setpoints.
- Pause is an operator-requested hold; failsafe is a system-detected stale-data hold. They may share hover/hold behavior, but status/logs must preserve the different reasons.
- Resume should clear the pause command and return vehicles to a safe holding state. The operator can then issue a fresh `MoveLeader`, `ChangeFormation`, or `LandSwarm` command.
- While paused, continuing old leader goals or formation changes would surprise the operator. Reject movement and formation actions until resume succeeds.
- Add one-line Chinese comments near timeout thresholds and hover/hold fallbacks explaining what stale-data risk they protect against.

## Acceptance criteria

- [ ] `PauseSwarm` causes vehicles to hold safe setpoints rather than continue mission progression.
- [ ] `PauseSwarm` does not require restarting Micro XRCE-DDS Agent, PX4 SITL, Gazebo, vehicle nodes, or the ground station.
- [ ] `PauseSwarm(pause=false)` resumes into a safe holding state and does not automatically continue a stale pre-pause movement or formation action.
- [ ] While paused, the system allows status observation, resume, and `LandSwarm`.
- [ ] While paused, `MoveLeader`, `ChangeFormation`, and any demo/macro progression are rejected clearly.
- [ ] `LandSwarm` remains available from paused/failsafe states where practical.
- [ ] A vehicle telemetry timeout causes that vehicle to hover or hold its last safe setpoint.
- [ ] Leader telemetry/status timeout causes followers to hover.
- [ ] Ground station and vehicle statuses expose pause/failsafe state.
- [ ] Tests cover timeout, pause, and land-all paths.
- [ ] Manual SITL verification completes without QGC as the control entrypoint: `TakeoffSwarm -> MoveLeader/following -> PauseSwarm -> verify hold -> reject MoveLeader/ChangeFormation while paused -> resume to holding -> send a fresh MoveLeader or ChangeFormation -> PauseSwarm -> LandSwarm`, without restarting runtime.

## Testing approach

- Unit-test timeout state transitions using controlled timestamps.
- Run ROS 2 package commands from the `px4_ws` workspace, not the outer Docker workspace.
- Use fake telemetry/status streams to verify followers stop following on leader timeout.
- Unit-test paused-state action gating: movement and formation are rejected while status/resume/land remain available.
- Unit-test resume semantics so old pre-pause goals are not continued automatically.
- Manually test pause, resume, rejected commands while paused, fresh command after resume, and land-all in SITL after follower following is available.

## Blocking edges

- Blocked by 09 - Implement follower fixed-slot following from leader state.
- Blocks 12 - Add swarm launch and SITL smoke acceptance workflow.
