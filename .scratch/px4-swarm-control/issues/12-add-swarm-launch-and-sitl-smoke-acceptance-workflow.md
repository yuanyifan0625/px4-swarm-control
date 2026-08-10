# 12 - Add swarm launch and SITL smoke acceptance workflow

**What to build:** Provide the normal ROS 2 launch path for the four swarm nodes and document/verify the first-version SITL smoke workflow from external prerequisites through takeoff, leader move, following, formation change, pause/failsafe, and land.

**Blocked by:** 10 - Implement `ChangeFormation` between `vee` and `line_abreast`; 11 - Add minimal pause and failsafe behavior.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

Manual startup should come first for debugging, but the completed first version needs a repeatable launch path for the ROS 2 swarm nodes. PX4 SITL and Micro XRCE-DDS Agent remain external prerequisites in this version.

## Scope

- Add a launch workflow that starts three parameterized `vehicle_node` instances and one `ground_station_node`.
- Provide configuration for roles, vehicle IDs, PX4 namespaces, slots, control rates, staging positions, tolerances, and formation defaults.
- Document or encode the smoke workflow prerequisites: external PX4 SITL multi-vehicle environment and `MicroXRCEAgent udp4 -p 8888`.
- Keep QGC optional for monitoring/manual safety observation; do not require it for the smoke workflow control path.
- Verify the full first-version behavior: takeoff to staging, leader move, follower fixed-slot following, formation change, pause/failsafe path, and land-all.
- Add smoke-test checklist or automation where practical.

## Non-goals

- Do not launch PX4 SITL from the swarm launch file.
- Do not launch Micro XRCE-DDS Agent from the swarm launch file.
- Do not support real vehicles.
- Do not add `column`, dynamic slots, or automatic leader reassignment.

## Implementation notes

- Keep launch configuration explicit so namespace/slot mistakes are easy to inspect.
- Add concise Chinese comments only where launch/config logic protects against role-slot mismatches or unsafe defaults.

## Acceptance criteria

- [ ] A normal launch path starts `/swarm`, `/vehicle_1`, `/vehicle_2`, and `/vehicle_3` ROS 2 nodes.
- [ ] Configuration maps `/vehicle_1` to leader, `/vehicle_2` to follower-left slot 1, and `/vehicle_3` to follower-right slot 2.
- [ ] Documentation/checklist states that PX4 SITL and Micro XRCE-DDS Agent must be started externally.
- [ ] The smoke workflow reaches staging and logs `all vehicles reached staging positions`.
- [ ] The smoke workflow moves the leader and followers maintain fixed-slot following.
- [ ] The smoke workflow changes between `vee` and `line_abreast` and logs `formation established`.
- [ ] The smoke workflow exercises pause/failsafe behavior and ends with land-all.

## Testing approach

- Run launch syntax/import tests inside the container.
- Run package and launch commands from the `px4_ws` workspace, not the outer Docker workspace.
- Run ROS 2 tests for launch configuration if practical.
- Perform a manual SITL smoke test with external PX4 SITL and Micro XRCE-DDS Agent.
- Record the exact command sequence needed for reproducibility using the workspace container command pattern.

## Blocking edges

- Blocked by 10 - Implement `ChangeFormation` between `vee` and `line_abreast`.
- Blocked by 11 - Add minimal pause and failsafe behavior.
