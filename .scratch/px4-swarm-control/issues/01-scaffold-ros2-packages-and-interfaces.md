# 01 - Scaffold ROS 2 packages and interfaces

**What to build:** Establish a buildable ROS 2 Jazzy foundation for the swarm-control work: one Python control package, one interfaces package, and the first action/message contracts needed by the ground station and vehicle nodes.

**Blocked by:** None - can start immediately.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

The first version will run in SITL with one `ground_station_node` and three parameterized `vehicle_node` instances. Operator commands should enter through ROS 2 actions, while internal node communication uses topics. Generated ROS 2 interfaces should be separated from Python node logic.

## Scope

- Create a Python/rclpy control package for nodes, shared modules, launch, config, and tests.
- Create a separate ROS 2 interfaces package for actions and messages.
- Define the initial operator actions: `TakeoffSwarm`, `MoveLeader`, `ChangeFormation`, `PauseSwarm`, and `LandSwarm`.
- Define initial internal messages for leader goal, formation mode, mission/failsafe command, and vehicle status summary.
- Ensure both packages build inside the Docker container.
- Add package-level test scaffolding so later tickets can add unit and integration tests.

## Non-goals

- Do not implement PX4 communication yet.
- Do not implement vehicle control behavior yet.
- Do not modify PX4-Autopilot or upstream ROS-PX4 packages.
- Do not launch PX4 SITL or Micro XRCE-DDS Agent from this ticket.

## Implementation notes

- Keep interface names stable and role-independent; leadership should be represented as state/parameters, not by hard-coding topic names around `leader`.
- Add short Chinese comments only around non-obvious interface choices if needed. Comments should explain why the contract exists or what ambiguity it prevents.

## Acceptance criteria

- [ ] The control package builds successfully in the container.
- [ ] The interfaces package builds successfully in the container.
- [ ] The five operator actions exist with fields sufficient for takeoff, absolute leader movement plus yaw, formation change, pause, and land-all.
- [ ] Internal messages exist for leader goal, formation mode, mission/failsafe command, and vehicle status summary.
- [ ] Package tests can be invoked through the container workflow.
- [ ] No PX4-Autopilot files are changed.

## Testing approach

- Run package build commands inside the container.
- Run ROS 2 package commands from the `px4_ws` workspace, not the outer Docker workspace.
- Run the initial package test suite, even if it only verifies scaffolding/imports.
- Inspect generated interfaces with ROS 2 tooling inside the container to confirm actions/messages are available.

## Blocking edges

- Blocks 02 - Add internal models, state enums, and formation geometry.
- Blocks 03 - Implement `Px4VehicleInterface` PX4 topic boundary.
- Blocks 06 - Build `ground_station_node` action surface and swarm topics.
