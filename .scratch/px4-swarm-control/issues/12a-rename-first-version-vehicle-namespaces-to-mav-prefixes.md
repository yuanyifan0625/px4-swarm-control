# 12a - Rename first-version vehicle namespaces to `/MAV1`, `/MAV2`, `/MAV3`

**What to build:** Rename the first-version ROS 2 vehicle namespace and topic-prefix contract from `/vehicle_1`, `/vehicle_2`, `/vehicle_3` to `/MAV1`, `/MAV2`, `/MAV3` across the swarm package, configuration, tests, and manual verification docs, while preserving the same leader/follower roles and distributed follower-control behavior.

**Blocked by:** None - can start immediately.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, ROS 2 package tests, ROS 2 launch commands, PX4/Gazebo/runtime commands from inside the container using the workspace command pattern in `AGENTS.md`. Do not modify PX4-Autopilot or `px4_msgs`.

## Background

The first-version code currently uses `/vehicle_1`, `/vehicle_2`, and `/vehicle_3` as role-independent ROS 2 vehicle namespaces. The user now wants the vehicle namespace/topic prefix contract to be `/MAV1`, `/MAV2`, and `/MAV3`, which better matches the intended vehicle naming style for later operator workflows and smoke verification.

This is a topic-contract rename, not a control-behavior change. It affects bridge expectations, vehicle configuration, subscriptions, publishers, tests, and manual docs. It should be completed before adding the normal swarm launch file so ticket 12 can launch the final namespace contract directly.

## Scope

- Rename the first-version ROS 2 vehicle namespaces/topic prefixes:
  - `/vehicle_1` -> `/MAV1`
  - `/vehicle_2` -> `/MAV2`
  - `/vehicle_3` -> `/MAV3`
- Preserve first-version roles and slots:
  - `/MAV1` is leader
  - `/MAV2` is follower-left
  - `/MAV3` is follower-right
- Update vehicle configuration so each `vehicle_node` uses the new namespace and still targets the correct PX4 instance/system ID.
- Update bridge expectations and live bridge smoke checks so three PX4 SITL instances are expected to publish `/MAV1/fmu/out/...`, `/MAV2/fmu/out/...`, and `/MAV3/fmu/out/...`.
- Update ground-station status subscriptions, staging setpoint publishers, and any other swarm topics that still hard-code the old `/vehicle_N` names.
- Update follower subscriptions so followers subscribe to the leader status at `/MAV1/status`.
- Update operator console status observation and formation-settle checks so it observes `/MAV1/status`, `/MAV2/status`, and `/MAV3/status`.
- Update tests and manual documentation that refer to `/vehicle_1`, `/vehicle_2`, `/vehicle_3`.
- Keep the distributed follower-control architecture unchanged: followers derive their own setpoints from leader state, formation mode, and slot.

## Non-goals

- Do not modify PX4-Autopilot.
- Do not modify `px4_msgs`.
- Do not add a launch file in this ticket; ticket 12 owns launch.
- Do not change action definitions.
- Do not change `operator_console` command meanings.
- Do not change speed profile behavior or put speed-profile parameters into vehicle-node YAML.
- Do not introduce real-vehicle deployment support.
- Do not add dynamic leader reassignment, dynamic slots, new formations, or path planning.

## Implementation notes

- Treat `/MAV1`, `/MAV2`, `/MAV3` as the canonical first-version vehicle namespace contract after this ticket.
- Keep `vehicle_id` values and display strings consistent with the new namespace naming unless tests or interface constraints require an explicit compatibility decision.
- The PX4 SITL startup commands must set `PX4_UXRCE_DDS_NS=MAV1`, `PX4_UXRCE_DDS_NS=MAV2`, and `PX4_UXRCE_DDS_NS=MAV3` so Micro XRCE-DDS exposes the new ROS 2 topic prefixes.
- Add short Chinese comments only where a hard-coded namespace mapping protects against role/slot/vehicle mismatch.
- This is a wide topic-contract rename; keep behavior changes out so failures are attributable to naming only.

## Acceptance criteria

- [ ] ROS 2 topics use `/MAV1`, `/MAV2`, and `/MAV3` as the first-version vehicle prefixes.
- [ ] `/MAV1/fmu/out/vehicle_local_position_v1`, `/MAV2/fmu/out/vehicle_local_position_v1`, and `/MAV3/fmu/out/vehicle_local_position_v1` are the expected live PX4 telemetry topics.
- [ ] `/MAV1/status`, `/MAV2/status`, and `/MAV3/status` are the expected swarm status topics.
- [ ] `/MAV1` remains leader, `/MAV2` remains follower-left, and `/MAV3` remains follower-right.
- [ ] Followers subscribe to `/MAV1/status` for leader state and do not treat `/swarm/leader_goal` as their own target.
- [ ] Ground station does not continuously publish absolute follower targets during following or formation change.
- [ ] Existing takeoff/staging, land, MoveLeader, follower following, formation change, pause/failsafe, operator console, and speed-profile tests are updated and pass under the `/MAV*` namespace contract.
- [ ] Live bridge smoke documentation and tooling instruct the operator to start PX4 SITL with `PX4_UXRCE_DDS_NS=MAV1`, `MAV2`, and `MAV3`.
- [ ] A manual or smoke check confirms that the old `/vehicle_1`, `/vehicle_2`, `/vehicle_3` prefixes are no longer the primary expected first-version topics.

## Testing approach

- Run unit tests for package scaffold, bridge config, vehicle node, ground station, operator console, follower controller, and live bridge smoke helpers after the rename.
- Run full `px4_swarm_control` package tests inside the container from `/home/ncrl/docker_ubuntu24/px4_ws`.
- Run CLI/topic-name checks where practical to ensure generated or expected topics use `/MAV1`, `/MAV2`, `/MAV3`.
- Manually validate with three PX4 SITL instances and Micro XRCE-DDS Agent:
  1. Start the agent on UDP port 8888.
  2. Start PX4 instances with `PX4_UXRCE_DDS_NS=MAV1`, `MAV2`, and `MAV3`.
  3. Run the live bridge smoke check.
  4. Confirm ROS 2 telemetry publishers and status topics exist under `/MAV1`, `/MAV2`, `/MAV3`.

## Blocking edges

- None - can start immediately.
- Blocks 12 - Add swarm launch and SITL smoke acceptance workflow.
