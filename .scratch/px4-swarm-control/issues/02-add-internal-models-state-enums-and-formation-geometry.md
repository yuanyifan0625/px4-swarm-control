# 02 - Add internal models, state enums, and formation geometry

**What to build:** Add the internal domain models and geometry helpers that let vehicle and ground-station logic talk in swarm-control concepts instead of raw PX4 messages.

**Blocked by:** 01 - Scaffold ROS 2 packages and interfaces.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

The spec requires `px4_msgs` to stay behind `Px4VehicleInterface`. Control logic should use internal concepts such as `VehicleState`, `PositionYawSetpoint`, formation modes, vehicle roles, mission states, and vehicle states. Formation geometry must preserve left/right slot meaning and separate world-frame staging from leader body-frame following.

## Scope

- Define internal data models for vehicle state, position+yaw setpoints, command results, roles, slots, mission states, and vehicle-level states.
- Implement formation geometry for `vee` and `line_abreast`.
- Implement world-frame staging position calculation from leader initial yaw.
- Implement leader body-frame offset transformation from current leader yaw.
- Preserve `vehicle_2` as follower-left slot 1 and `vehicle_3` as follower-right slot 2.
- Add unit tests for geometry and enum/model behavior.

## Non-goals

- Do not publish or subscribe to ROS 2 topics yet.
- Do not command PX4.
- Do not implement follower control loops yet.
- Do not add `column` formation.
- Do not implement dynamic leader or slot reassignment.

## Implementation notes

- Add one-line Chinese comments near coordinate-frame and yaw transformations explaining what safety/consistency property the transformation protects.
- Prefer simple pure functions for geometry so tests do not need ROS 2 runtime.

## Acceptance criteria

- [ ] Internal models can represent role, vehicle ID, PX4 namespace, slot, vehicle state, mission state, command result, and position+yaw setpoint.
- [ ] `vee` and `line_abreast` slots produce deterministic offsets for left and right followers.
- [ ] Staging positions use world-frame positions derived from leader initial yaw.
- [ ] Following positions use leader body-frame offsets derived from current leader yaw.
- [ ] Unit tests verify left/right sign conventions, leader initial yaw staging, and leader current yaw body-frame following.
- [ ] No raw `px4_msgs` types are required by geometry/controller-facing model tests.

## Testing approach

- Run unit tests for geometry without PX4, Gazebo, or Micro XRCE-DDS Agent.
- Run ROS 2 package commands from the `px4_ws` workspace, not the outer Docker workspace.
- Include cases for zero yaw, positive yaw, negative yaw, and left/right follower slots.
- Include a regression case proving follower-left starts on the leader's left relative to leader initial heading.

## Blocking edges

- Blocked by 01 - Scaffold ROS 2 packages and interfaces.
- Blocks 03 - Implement `Px4VehicleInterface` PX4 topic boundary.
- Blocks 04 - Build single parameterized `vehicle_node` for one vehicle.
- Blocks 06 - Build `ground_station_node` action surface and swarm topics.
