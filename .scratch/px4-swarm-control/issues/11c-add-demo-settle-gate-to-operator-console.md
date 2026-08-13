# 11c - Add demo settle gate to operator console

**What to build:** Improve the operator console demo macro so it waits for the followers to settle into the current formation before sending the next demo action, making SITL demos easier to observe without changing the swarm-control architecture.

**Blocked by:** 11b - Add configurable operator short-command console.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

Ticket 11b added a short-command operator console and a configurable demo macro. In manual SITL demos, the macro can still look visually rushed because each step waits for its own action result, but a leader movement action only proves that the leader reached its goal. The followers may still be converging to the leader body-frame offset when the next yaw or formation action starts, which makes the demo look shaky even when the distributed follower-control architecture is working correctly.

This ticket should improve demo pacing only. It should not change the core vehicle control logic, follower controller, PX4-Autopilot, or the existing action APIs.

## Scope

- Add a demo-only `settle` macro step that can appear in the configured `demo_commands` sequence.
- Define `settle` as an observation gate: the operator console watches the latest leader and follower statuses and waits until followers are within the current formation tolerance for a configured stable duration, defaulting to about `1.0 s`.
- Keep the example macro sequence configurable, for example: `takeoff -> leader move -> settle -> yaw -> settle -> line_abreast -> settle -> vee -> settle -> home -> settle -> land`.
- Let `settle` understand the current formation mode used by the console demo. It may track the last successful `vee`/`line_abreast` command and/or observe the formation mode topic, but it must not publish formation targets.
- Add configurable settle parameters such as stable duration, timeout, position tolerance, yaw tolerance, lateral spacing, and trail spacing where needed.
- Add TDD coverage for successful settle, timeout/failure, paused-state behavior, and the demo macro stopping if settle fails.
- Update the Chinese manual SITL verification document for the operator console demo flow.

## Non-goals

- Do not modify `vehicle_node`.
- Do not modify `follower_controller`.
- Do not modify PX4-Autopilot or PX4 internal control logic.
- Do not add new follower controller ROS nodes.
- Do not publish direct follower absolute targets from the operator console or ground station during demo settle.
- Do not change `TakeoffSwarm`, `MoveLeader`, `ChangeFormation`, `PauseSwarm`, or `LandSwarm` action definitions.
- Do not implement full trajectory planning or velocity limiting in this ticket.
- Do not tune PX4 controller parameters in this ticket.

## Implementation notes

- `settle` exists only to make human-visible demos calmer. It is not a new mission command and should not be required for normal manual actions.
- The console may compute expected follower positions locally for observation, using the same fixed first-version formation geometry and current formation mode. That computation is allowed only as a readiness check; it must not become a follower command path.
- If the swarm is paused, `settle` and demo macro progression should follow ticket 11/11b behavior and stop clearly rather than continuing.
- Add one-line Chinese comments near the settle gate explaining that it protects the demo from issuing the next action while followers are still converging.

## Acceptance criteria

- [ ] The configured demo macro accepts a `settle` step.
- [ ] `settle` waits until both followers are within configured position/yaw tolerance of the current formation target for the configured stable duration before returning success.
- [ ] `settle` only observes `/vehicle_1/status`, `/vehicle_2/status`, `/vehicle_3/status`, and the current formation mode state needed for readiness checks.
- [ ] `settle` never publishes `/vehicle_2/staging_setpoint`, `/vehicle_3/staging_setpoint`, or any direct follower absolute target.
- [ ] The console demo macro stops on settle timeout or stale telemetry and reports a clear failure message.
- [ ] Paused-state behavior remains intact: status/resume/land are allowed, while movement, formation change, settle progression, and demo macro progression are blocked or stopped clearly.
- [ ] Existing commands `1` through `8` continue to work as before.
- [ ] Existing manual `ros2 action send_goal` workflows continue to work.
- [ ] Tests cover successful settle, settle timeout, stale status handling, paused-state blocking, and macro sequencing with `settle`.
- [ ] Manual SITL verification completes without QGC as the control entrypoint: use console command `9` to run `takeoff -> leader move -> settle -> yaw -> settle -> line_abreast -> settle -> vee -> settle -> home -> settle -> land`, and verify in Gazebo that followers visibly settle before the next demo step starts.

## Testing approach

- Unit-test the settle readiness calculation using fake leader/follower statuses and fixed formation geometry.
- Unit-test that `settle` requires a continuous stable window, not just one matching status sample.
- Unit-test stale telemetry and timeout failures.
- Unit-test demo macro sequencing to verify `settle` is executed between actions and stops the macro on failure.
- Unit-test that the fake gateway records no follower target publications or bypass commands.
- Run ROS 2 package tests from the `px4_ws` workspace inside the container.
- Manually verify the updated console demo in three-vehicle SITL after the package tests pass.

## Blocking edges

- Blocked by 11b - Add configurable operator short-command console.
- Optionally informs 12 - Add swarm launch and SITL smoke acceptance workflow if the final smoke workflow includes the operator console demo.
