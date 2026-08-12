# 11b - Add configurable operator short-command console

**What to build:** Add a first-version terminal operator console that maps short numeric commands to the existing swarm actions, so manual SITL testing is faster without changing the action APIs or follower-control architecture.

**Blocked by:** 11 - Add minimal pause and failsafe behavior.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

Manual `ros2 action send_goal ...` commands are long. They are useful for debugging each action directly, but slow for repeated SITL operation and demos. The console should reduce typing while keeping the existing architecture intact.

The console is an operator convenience layer. It must call the same `TakeoffSwarm`, `MoveLeader`, `ChangeFormation`, `PauseSwarm`, and `LandSwarm` actions that the operator can already call manually. It must not publish follower setpoints, bypass the ground station, or turn followers into direct operator targets.

## Scope

- Add a terminal console or CLI executable for manual SITL operation.
- Map short commands to existing swarm actions:
  - `1`: takeoff using configured default altitude and timeout.
  - `2`: move leader by configured `+x` step.
  - `3`: move leader by configured `+y` step.
  - `4`: move leader by configured altitude step.
  - `5`: rotate leader yaw by configured step, default `45 deg`.
  - `6`: change formation to `vee`.
  - `7`: change formation to `line_abreast`.
  - `8`: land all.
  - `9`: run a configured demo macro: takeoff, move, yaw change, formation change, return to initial leader position, land.
- Read leader status before relative movement commands and convert relative deltas into the existing absolute `MoveLeader` goal.
- Put step sizes, takeoff altitude, yaw step, tolerances, timeouts, and demo macro sequence in ROS 2 parameters or YAML config.
- Keep the console optional. Existing manual action commands must continue to work.
- Add tests for command mapping, parameter usage, relative-to-absolute leader movement conversion, pause-state behavior, and demo macro sequencing.
- Add a Chinese manual verification document for the console.

## Non-goals

- Do not change the existing action message definitions.
- Do not add direct follower absolute target commands.
- Do not publish `/vehicle_2/staging_setpoint` or `/vehicle_3/staging_setpoint` for formation movement.
- Do not bypass `ground_station_node`.
- Do not replace manual `ros2 action send_goal` commands.
- Do not implement a full GUI.
- Do not implement arbitrary trajectory planning or future autonomy algorithms in this ticket.

## Implementation notes

- The console should be a thin client over the operator-facing actions. This protects the ground-station action boundary and keeps future algorithm work from depending on key numbers.
- Relative movement commands are console-only convenience. Internally they should read the latest leader status and send the existing absolute world-frame `MoveLeader` action.
- Demo macro should be declarative or parameterized so future formation settings and algorithm experiments can change the sequence without editing core control logic.
- If the swarm is paused, the console should follow ticket 11 behavior: allow status/resume/land and block movement, formation change, and macro progression.
- Add one-line Chinese comments near the relative-to-absolute conversion and macro safety gate explaining what architecture or operator-safety rule they protect.

## Acceptance criteria

- [ ] The console can be started from the ROS 2 workspace inside the container.
- [ ] `1` sends `TakeoffSwarm` with configured default altitude and timeout.
- [ ] `2`, `3`, and `4` read leader status, convert relative movement into an absolute `MoveLeader` goal, and move only the leader.
- [ ] `5` reads leader yaw, converts the configured yaw delta into an absolute `MoveLeader` yaw goal, and moves only the leader yaw target.
- [ ] `6` sends `ChangeFormation(vee)`.
- [ ] `7` sends `ChangeFormation(line_abreast)`.
- [ ] `8` sends `LandSwarm`.
- [ ] `9` runs the configured demo macro end to end and stops on the first failed action.
- [ ] Console defaults are configurable through ROS 2 parameters or YAML, not hard-coded into control logic.
- [ ] The console never sends direct follower targets and never bypasses `ground_station_node`.
- [ ] Existing manual `ros2 action send_goal` workflows still work.
- [ ] While paused, the console allows status/resume/land and blocks movement, formation change, and demo macro progression.
- [ ] Tests cover command mapping, relative leader movement conversion, yaw delta conversion, configurable defaults, blocked paused-state commands, and macro failure stop.
- [ ] Manual SITL verification completes without QGC as the control entrypoint: use console commands to run `takeoff -> leader relative move -> yaw delta -> line_abreast -> vee -> pause -> rejected move while paused -> resume -> fresh move -> land`, without restarting runtime.

## Testing approach

- Unit-test command parsing and mapping without live ROS 2 actions.
- Unit-test relative movement conversion using fake leader status.
- Unit-test yaw delta conversion using fake leader status.
- Unit-test that console config controls altitude, step size, yaw step, tolerances, timeouts, and macro sequence.
- Unit-test paused-state command gating.
- Use fake action clients to verify the console calls only the existing swarm action surface.
- Run ROS 2 package commands from the `px4_ws` workspace, not the outer Docker workspace.
- Manually verify the console in SITL after ticket 11 pause/resume behavior is available.

## Blocking edges

- Blocked by 11 - Add minimal pause and failsafe behavior.
- Optionally informs 12 - Add swarm launch and SITL smoke acceptance workflow if the final smoke workflow includes the operator console.
