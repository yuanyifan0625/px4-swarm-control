# 05 - Validate three-vehicle namespaces and telemetry flow

**What to build:** Run three configured `vehicle_node` instances for `/vehicle_1`, `/vehicle_2`, and `/vehicle_3`, and verify each receives/publishes the correct vehicle-specific telemetry/status without namespace cross-talk.

**Blocked by:** 04 - Build single parameterized `vehicle_node` for one vehicle.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

The first infrastructure risk is multi-vehicle namespace correctness. Before building swarm actions, each `vehicle_node` instance must map cleanly to its PX4 vehicle namespace and publish its own status summary.

## Scope

- Configure three vehicle node instances:
  - `/vehicle_1`: leader
  - `/vehicle_2`: follower-left slot 1
  - `/vehicle_3`: follower-right slot 2
- Verify each instance uses its configured PX4 namespace.
- Verify each status summary topic updates from its own telemetry stream.
- Add manual startup documentation or commands for debugging the three-node setup.
- Add tests or smoke checks that catch swapped namespaces or mixed vehicle IDs.

## Non-goals

- Do not implement ground-station actions yet.
- Do not implement synchronized takeoff.
- Do not implement follower following.
- Do not implement launch-all as the normal workflow yet.

## Implementation notes

- Keep namespace and role assignment explicit in configuration.
- Add short Chinese comments near any namespace-to-vehicle mapping guard explaining that it prevents cross-vehicle command/telemetry mixups.

## Acceptance criteria

- [ ] Three `vehicle_node` instances can run concurrently with distinct role, vehicle ID, PX4 namespace, and slot parameters.
- [ ] `/vehicle_1` publishes leader status.
- [ ] `/vehicle_2` publishes follower-left slot 1 status.
- [ ] `/vehicle_3` publishes follower-right slot 2 status.
- [ ] Telemetry/status from one vehicle does not appear under another vehicle's status topic.
- [ ] Manual debug steps identify PX4 SITL and Micro XRCE-DDS Agent as external prerequisites.
- [ ] No PX4-Autopilot cooperative-control changes are made.

## Testing approach

- Use ROS 2 topic inspection inside the container to confirm three status streams.
- Run package and topic-inspection commands from the `px4_ws` workspace, not the outer Docker workspace.
- Use fake interfaces or controlled telemetry where possible to detect namespace swaps.
- With SITL running, verify each status topic corresponds to the expected vehicle instance.

## Blocking edges

- Blocked by 04 - Build single parameterized `vehicle_node` for one vehicle.
- Blocks 07 - Deliver synchronized takeoff to staging and land-all milestone.
