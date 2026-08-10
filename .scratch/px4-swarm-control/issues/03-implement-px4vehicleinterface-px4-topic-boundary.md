# 03 - Implement `Px4VehicleInterface` PX4 topic boundary

**What to build:** Add the shared internal interface that encapsulates PX4 `px4_msgs` topics for each vehicle while exposing internal swarm-control models to the rest of the code.

**Blocked by:** 01 - Scaffold ROS 2 packages and interfaces; 02 - Add internal models, state enums, and formation geometry.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

Micro XRCE-DDS Agent is the external ROS 2-PX4 bridge. The application should not replace it or wrap it as another node. Instead, each `vehicle_node` should use a shared `Px4VehicleInterface` class/module to own `px4_msgs` topic details, QoS, timestamps, Offboard heartbeat, commands, and telemetry conversion.

## Scope

- Implement a reusable `Px4VehicleInterface` class/module used inside vehicle nodes.
- Encapsulate `px4_msgs` publishers/subscribers for Offboard control mode, trajectory setpoint, vehicle command, local position/odometry/status, command ack where useful, and telemetry freshness.
- Convert PX4 telemetry into internal `VehicleState`.
- Convert internal `PositionYawSetpoint` into PX4 Offboard position+yaw setpoint publication.
- Provide methods for Offboard heartbeat, arm/disarm, takeoff, land, and safe hover/hold setpoint.
- Handle per-vehicle PX4 namespace configuration.
- Add focused tests for conversion and topic-boundary behavior without requiring live PX4.

## Non-goals

- Do not implement leader/follower behavior.
- Do not implement mission-level actions.
- Do not launch or manage Micro XRCE-DDS Agent.
- Do not modify PX4-Autopilot.

## Implementation notes

- Keep raw `px4_msgs` imports localized to the PX4 boundary.
- Add short Chinese comments at risky conversions and command publications, explaining why the conversion/heartbeat/command guard exists.
- The comments should not restate the code line by line.

## Acceptance criteria

- [ ] Controller-facing code can use internal models without importing raw `px4_msgs`.
- [ ] `Px4VehicleInterface` publishes position+yaw setpoints and Offboard heartbeat through PX4 topics.
- [ ] `Px4VehicleInterface` can send arm, disarm, takeoff, and land commands.
- [ ] Telemetry conversion populates internal vehicle state including position, yaw, velocity, armed/nav state, Offboard availability, and telemetry age.
- [ ] Namespace handling supports three separate PX4 vehicle topic roots.
- [ ] Unit tests cover PX4-to-internal and internal-to-PX4 mapping.
- [ ] Critical conversion/command paths include meaningful one-line Chinese comments.

## Testing approach

- Use unit tests with fake ROS 2 publishers/subscribers or isolated conversion helpers where possible.
- Run ROS 2 package commands from the `px4_ws` workspace, not the outer Docker workspace.
- Verify generated PX4 messages have expected position, yaw, timestamps, and command fields.
- Verify stale telemetry detection without requiring live PX4.
- Leave SITL validation to later tickets.

## Blocking edges

- Blocked by 01 - Scaffold ROS 2 packages and interfaces.
- Blocked by 02 - Add internal models, state enums, and formation geometry.
- Blocks 04 - Build single parameterized `vehicle_node` for one vehicle.
