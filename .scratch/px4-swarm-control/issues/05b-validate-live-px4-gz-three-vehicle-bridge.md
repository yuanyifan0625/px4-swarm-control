# 05b - Validate live PX4 Gz three-vehicle bridge

**What to build:** Prove that the first-version namespace design works against real PX4 Gz SITL, not only against three ROS 2 `vehicle_node` instances. A developer should be able to start three real `gz_x500` PX4 SITL instances, see three vehicles in Gazebo, confirm all three PX4 clients connect through one Micro XRCE-DDS Agent, and confirm ROS 2 telemetry topics have real PX4 publishers.

**Blocked by:** 05 - Validate three-vehicle namespaces and telemetry flow.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, ROS 2 launch commands, PX4 SITL commands, Gazebo commands, and Micro XRCE-DDS Agent commands inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

Ticket 05 validated the ROS 2 side of three configured `vehicle_node` instances, but a later diagnosis found that `make px4_sitl gz_x500` starts only one PX4/Gazebo vehicle. Three `vehicle_node` instances can also make ROS 2 topics appear through subscriptions even when PX4 is not publishing telemetry. Before implementing swarm takeoff, the project needs a live SITL proof that three PX4 instances, three Gazebo models, Micro XRCE-DDS Agent, and the ROS 2 topic boundary all agree on namespaces and topic names.

## Scope

- Document a manual three-vehicle PX4 Gz startup workflow that does not modify PX4-Autopilot source.
- Start three PX4 SITL `gz_x500` instances with unique PX4 instance IDs and separated spawn positions.
- Use `PX4_UXRCE_DDS_NS` so the ROS 2 namespaces remain `/vehicle_1`, `/vehicle_2`, and `/vehicle_3` instead of switching the project to PX4's default `/px4_1`, `/px4_2`, `/px4_3` convention.
- Confirm that one `MicroXRCEAgent udp4 -p 8888` process accepts all three PX4 clients.
- Confirm Gazebo exposes three `x500` models, not one model plus three ROS 2 nodes.
- Confirm ROS 2 topic inspection shows real PX4 publishers for all three vehicles, not only subscribers created by `vehicle_node`.
- Update the PX4 topic-boundary expectations for PX4 v1.18 runtime output topic names, including the observed version suffixes on `vehicle_local_position`, `vehicle_status`, and `vehicle_command_ack`.
- Add tests or smoke-check documentation that fails clearly when the ROS 2 code listens to an unversioned PX4 output topic while PX4 publishes a versioned one.
- Identify and document the required `VehicleCommand.target_system` strategy for multi-vehicle commands before takeoff/land implementation begins.

## Non-goals

- Do not implement swarm takeoff, staging, landing, leader movement, follower following, or formation change.
- Do not make ROS 2 launch manage PX4 SITL or Micro XRCE-DDS Agent as the normal workflow.
- Do not use QGC as the first-version control entrypoint.
- Do not modify PX4-Autopilot cooperative-control behavior.
- Do not hide multi-vehicle startup behind a combined launch file before the manual workflow is proven.

## Implementation notes

- Treat Micro XRCE-DDS Agent as an external bridge, not as a ROS 2 package node.
- Prefer keeping `/vehicle_1..3` as the project namespace convention by explicitly configuring PX4's DDS namespace.
- Keep PX4 message types behind `Px4VehicleInterface`; only the boundary should know the concrete `px4_msgs` topic names.
- Make versioned PX4 output topic names configurable or centralized so a future PX4 message-version change has one obvious update point.
- Add concise one-line Chinese comments near namespace guards, versioned-topic mapping, and command target-system mapping to explain which cross-vehicle or bridge mismatch risk is being protected.

## Acceptance criteria

- [ ] A developer can manually start three real PX4 Gz `gz_x500` SITL instances without editing PX4-Autopilot source.
- [ ] Gazebo shows or exposes three separate `x500` models with separated initial positions.
- [ ] One `MicroXRCEAgent udp4 -p 8888` process establishes sessions for all three PX4 clients.
- [ ] ROS 2 topic inspection for each `/vehicle_1`, `/vehicle_2`, and `/vehicle_3` PX4 telemetry namespace shows `Publisher count: 1` from the DDS bridge for local position.
- [ ] ROS 2 topic inspection also confirms real PX4 publishers for vehicle status and command acknowledgement topics using the PX4 v1.18 runtime topic names.
- [ ] The project documents whether `/vehicle_1..3` or `/px4_1..3` is the intended namespace convention, and explains how PX4 is configured to make that true.
- [ ] The ROS 2 PX4 boundary no longer treats subscriber-created topics as evidence of live PX4 telemetry.
- [ ] Multi-vehicle `VehicleCommand.target_system` behavior is documented or parameterized before ticket 07 can command takeoff/land.
- [ ] No QGC dependency is introduced for automated or required acceptance.

## Testing approach

- Run the manual startup workflow inside the container with one terminal for Micro XRCE-DDS Agent and one terminal per PX4 instance.
- Use Gazebo topic/model inspection to confirm three real simulated vehicles exist.
- Use ROS 2 topic inspection with verbose endpoint details to distinguish real bridge publishers from local ROS 2 subscribers.
- Source the ROS 2 workspace before using `ros2 topic echo` so `px4_msgs` types resolve correctly.
- Run package tests from `px4_ws` after updating PX4 topic-boundary defaults or configuration.
- Keep the smoke check focused on bridge truth: three PX4 clients, three Gazebo models, three ROS 2 telemetry publishers.

## Blocking edges

- Blocked by 05 - Validate three-vehicle namespaces and telemetry flow.
- Blocks 07 - Deliver synchronized takeoff to staging and land-all milestone.
- Blocks later SITL smoke acceptance work that assumes three real PX4 vehicles exist.
