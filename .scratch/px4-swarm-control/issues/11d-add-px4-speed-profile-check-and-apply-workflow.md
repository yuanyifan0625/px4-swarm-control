# 11d - Add PX4 speed profile check and explicit apply workflow

**What to build:** Add a clean PX4 speed-profile workflow so the operator can check and explicitly apply slow, smooth PX4 flight-control parameters for SITL demos and future real-vehicle deployment without modifying PX4-Autopilot, `px4_msgs`, `vehicle_node`, or follower-control logic.

**Blocked by:** 11b - Add configurable operator short-command console.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container. PX4/Gazebo/runtime commands must run inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

The swarm currently sends high-level position plus yaw setpoints to PX4. It does not send velocity setpoints or implement a ROS-side speed limiter. That means the visible movement speed, acceleration, jerk, and yaw rate are primarily controlled by PX4 runtime parameters such as `MPC_XY_VEL_MAX`, `MPC_ACC_HOR`, `MPC_JERK_AUTO`, and `MPC_YAWRAUTO_MAX`.

The user wants slower and smoother movement for demos while keeping the architecture clean and preserving future real-vehicle deployment. Speed parameters should not be placed in `three_vehicle_nodes.yaml`, because that file configures ROS 2 vehicle-node identity, namespace, loop rate, timeout, and formation spacing. PX4 speed and acceleration parameters belong to a PX4 speed profile.

Future real deployment may use three Raspberry Pi computers plus one local ground-station computer. Each Pi represents one aircraft. The same speed-profile concept should work for both SITL and real vehicles, but both environments should use a check plus explicit apply workflow rather than silently changing flight-controller parameters.

## Scope

- Define a versioned PX4 speed-profile format for flight-control parameters.
- Add at least a `slow_demo` profile for SITL demos.
- Add a conservative `real_cautious` profile draft for future real-vehicle bring-up.
- Include parameters such as:
  - `MPC_XY_VEL_MAX`
  - `MPC_Z_VEL_MAX_UP`
  - `MPC_Z_VEL_MAX_DN`
  - `MPC_ACC_HOR`
  - `MPC_JERK_AUTO`
  - `MPC_YAWRAUTO_MAX`
  - `MPC_YAWRAUTO_ACC`
- Provide a workflow to check current PX4 values against a selected profile for all three vehicles.
- Provide a workflow to explicitly apply a selected profile only after the operator asks for it.
- Show clear terminal output before and after applying parameters, including current value, desired value, and per-vehicle result.
- Support SITL first, while keeping the file format and workflow suitable for future per-vehicle Raspberry Pi deployment.
- Add tests for profile parsing, validation, diff generation, and explicit apply gating where feasible without live PX4.
- Add Chinese manual documentation for checking and applying speed profiles in three-vehicle SITL.

## Non-goals

- Do not modify PX4-Autopilot source code.
- Do not modify `px4_msgs`.
- Do not modify `vehicle_node`.
- Do not modify `follower_controller`.
- Do not implement a ROS-side velocity limiter or trajectory planner in this ticket.
- Do not change action definitions.
- Do not silently apply PX4 parameters when the swarm starts.
- Do not apply parameters while vehicles are already flying unless the manual procedure explicitly marks that as safe for a specific future use case.
- Do not treat QGC as the control entrypoint.

## Implementation notes

- Keep `three_vehicle_nodes.yaml` focused on ROS 2 node identity and runtime behavior. Do not add PX4 controller parameters there.
- Store PX4 speed profiles separately from ROS 2 node configuration, for example under a dedicated config subdirectory.
- The first workflow can be a small ROS 2 CLI, Python helper, or documented script, but it must make the difference between "check" and "apply" explicit.
- The speed-profile source of truth should live in the repository. For future real-vehicle deployment, the same profile can be copied to each Raspberry Pi or checked centrally by the ground-station workflow.
- The apply workflow should print a Chinese warning explaining that it changes PX4 runtime parameters and should be run before flight.
- Add short Chinese comments near any command-generation boundary explaining that this applies PX4 runtime parameters rather than changing ROS setpoint logic.

## Suggested starting profile values

Use these as conservative SITL demo starting values, then refine during manual verification:

- `MPC_XY_VEL_MAX`: `2.0`
- `MPC_Z_VEL_MAX_UP`: `1.0`
- `MPC_Z_VEL_MAX_DN`: `0.8`
- `MPC_ACC_HOR`: `2.0`
- `MPC_JERK_AUTO`: `1.0`
- `MPC_YAWRAUTO_MAX`: `25`
- `MPC_YAWRAUTO_ACC`: `10`

If SITL still looks too fast, try lowering horizontal speed to around `1.5 m/s` and yaw rate to around `20 deg/s`, while increasing action timeouts where needed.

## Acceptance criteria

- [ ] PX4 speed profiles are stored separately from ROS 2 vehicle-node configuration.
- [ ] `slow_demo` and `real_cautious` profiles exist and document their intended use.
- [ ] The workflow can check current PX4 parameter values for all three vehicles against a selected profile.
- [ ] The workflow can explicitly apply a selected profile after an operator-visible confirmation or deliberate apply command.
- [ ] Terminal output clearly shows per-vehicle current value, desired value, and whether each parameter matches or was applied.
- [ ] No PX4 parameters are changed silently during normal `vehicle_node`, `ground_station_node`, or `operator_console` startup.
- [ ] Existing swarm control behavior remains unchanged: position+yaw setpoints still flow through `Px4VehicleInterface`, followers remain distributed, and the operator console does not command followers directly.
- [ ] Tests cover profile parsing, supported parameter validation, profile diff output, missing/invalid profile handling, and explicit apply gating.
- [ ] Manual SITL verification shows that all three PX4 instances can be checked, explicitly updated to `slow_demo`, verified with parameter output, then used with `operator_console` demo without QGC as the control entrypoint.

## Testing approach

- Unit-test profile parsing from YAML.
- Unit-test validation of required/supported PX4 parameter names.
- Unit-test diff/report generation using fake current PX4 parameter values.
- Unit-test that apply mode is separate from check mode and cannot happen accidentally.
- Run ROS 2 package tests from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container.
- Manually verify in SITL:
  1. Start Micro XRCE-DDS Agent and three PX4/Gazebo vehicles.
  2. Check the selected speed profile against all three PX4 instances.
  3. Explicitly apply the selected profile.
  4. Re-check and confirm all configured values match.
  5. Run `operator_console` demo and observe slower, smoother movement.

## Blocking edges

- Blocked by 11b - Add configurable operator short-command console.
- Complements 11c - Add demo settle gate to operator console.
- Should inform 12 - Add swarm launch and SITL smoke acceptance workflow, because the launch/smoke workflow should reference the speed-profile check/apply step when demos require slower motion.
