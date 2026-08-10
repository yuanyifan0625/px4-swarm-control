# 04 - Build single parameterized `vehicle_node` for one vehicle

**What to build:** Create one reusable ROS 2 `vehicle_node` executable that can represent a leader or follower vehicle through parameters and can control one PX4 vehicle through `Px4VehicleInterface`.

**Blocked by:** 02 - Add internal models, state enums, and formation geometry; 03 - Implement `Px4VehicleInterface` PX4 topic boundary.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

The runtime design uses three instances of the same `vehicle_node`, not separate leader/follower executables. Each instance is configured by `role`, `vehicle_id`, `px4_namespace`, and `slot`. This ticket creates the single-vehicle vertical path before scaling to three vehicles.

## Scope

- Implement a parameterized `vehicle_node` executable.
- Accept parameters for role, vehicle ID, PX4 namespace, slot, control loop rate, staging/hold setpoint defaults, and initial yaw handling as needed.
- Use `Px4VehicleInterface` to publish Offboard heartbeat and position+yaw setpoints.
- Publish the vehicle status summary topic.
- For a leader role, accept leader goal topic input.
- For follower role, initialize follower behavior structure but do not implement full following yet.
- Maintain vehicle-level state and log local state transitions.
- Add tests that validate parameter parsing, status publication shape, and basic state transitions.

## Non-goals

- Do not run all three vehicles yet.
- Do not implement synchronized takeoff.
- Do not implement follower following.
- Do not implement ground-station actions.
- Do not manage PX4 SITL or Micro XRCE-DDS Agent.

## Implementation notes

- Keep the executable role-parameterized rather than branching into separate leader/follower entrypoints.
- Add concise Chinese comments where state guards prevent unsafe command publication or protect Offboard timing assumptions.

## Acceptance criteria

- [ ] One executable can start as a leader by parameters.
- [ ] One executable can start as a follower by parameters.
- [ ] The node uses `Px4VehicleInterface` rather than raw PX4 topic logic spread through controller code.
- [ ] The node publishes a status summary with role, vehicle ID, pose/yaw/velocity if available, armed/nav state, Offboard availability, telemetry age, and vehicle-level state.
- [ ] The node can hold or command a position+yaw setpoint for one vehicle when PX4 bridge prerequisites are running.
- [ ] Local state transitions are logged without flooding the terminal.
- [ ] Tests cover parameter validation and basic node behavior.

## Testing approach

- Run unit tests for parameter handling and vehicle-level state transitions.
- Run ROS 2 package commands from the `px4_ws` workspace, not the outer Docker workspace.
- Run ROS 2 node-level tests with a fake or simulated `Px4VehicleInterface` if practical.
- Optionally perform a manual single-vehicle SITL smoke check in the container after PX4 SITL and Micro XRCE-DDS Agent are running.

## Blocking edges

- Blocked by 02 - Add internal models, state enums, and formation geometry.
- Blocked by 03 - Implement `Px4VehicleInterface` PX4 topic boundary.
- Blocks 05 - Validate three-vehicle namespaces and telemetry flow.
