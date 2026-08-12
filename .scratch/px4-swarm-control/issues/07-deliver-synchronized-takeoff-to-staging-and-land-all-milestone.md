# 07 - Deliver synchronized takeoff to staging and land-all milestone

**What to build:** Complete the first milestone: the operator can trigger three-vehicle takeoff to separated staging positions, see a staging-complete progress message, and trigger land-all.

**Blocked by:** 05 - Validate three-vehicle namespaces and telemetry flow; 06 - Build `ground_station_node` action surface and swarm topics.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

The first milestone validates the full infrastructure path before formation following: multi-vehicle SITL, ROS 2-PX4 bridge, three vehicle nodes, ground-station actions, staging geometry, status aggregation, and safe land-all.

The first-version SITL milestone must not depend on QGC being open. QGC may still be used as an optional monitoring/safety observation tool, but `TakeoffSwarm` acceptance must prove the ROS 2/PX4 path can arm and take off without QGC acting as a hidden runtime prerequisite.

## Scope

- Implement whole-swarm takeoff coordination through the ground station.
- Have all three vehicles arm/take off together to the configured altitude.
- Ensure first-version SITL `TakeoffSwarm` does not require QGC to be open.
- Command separated horizontal staging positions:
  - leader centered
  - follower-left behind-left relative to leader initial yaw
  - follower-right behind-right relative to leader initial yaw
- Detect when all vehicles reach staging positions.
- Log `all vehicles reached staging positions` from the ground station.
- Implement land-all action behavior for all three vehicles.
- Add action feedback/result for takeoff and land-all.
- Add tests for staging completion and land-all state transitions.

## Non-goals

- Do not implement leader movement beyond staging.
- Do not implement follower following.
- Do not implement formation change.
- Do not add dynamic slot assignment.
- Do not manage PX4 SITL or Micro XRCE-DDS Agent from launch.
- Do not make QGC part of the required control or startup path.

## Implementation notes

- Staging uses world-frame positions to protect takeoff/landing separation.
- Add concise Chinese comments near staging completion checks explaining what collision/sequencing risk they protect against.
- If no-QGC takeoff requires changes to PX4 parameters, ROS 2 Offboard sequencing, command `target_system`, or the manual startup flow, print one clear Chinese terminal log line explaining the adopted fix. The log should name the category of fix, for example PX4 parameter/preflight behavior, Offboard heartbeat warmup, target-system mapping, or startup sequencing.
- If no-QGC takeoff fails during implementation, diagnose it with a tight feedback loop before adding workaround logic. Useful evidence includes PX4 commander/preflight output, `VehicleCommandAck`, `VehicleStatus`, arming state, Offboard availability, and Micro XRCE-DDS/PX4 namespace checks.

## Acceptance criteria

- [ ] `TakeoffSwarm` arms/takes off all three vehicles and commands their staging positions.
- [ ] `TakeoffSwarm` can be manually demonstrated in SITL with QGC closed; QGC is not a required runtime dependency for arm/takeoff.
- [ ] Vehicles use the same target altitude and different horizontal staging positions.
- [ ] `vehicle_2` stages behind-left relative to leader initial yaw.
- [ ] `vehicle_3` stages behind-right relative to leader initial yaw.
- [ ] Ground station logs `all vehicles reached staging positions` once all three are staged.
- [ ] `LandSwarm` commands all three vehicles to land.
- [ ] The first milestone can be manually demonstrated in SITL with Micro XRCE-DDS Agent running.
- [ ] If implementation changes PX4 parameters, ROS 2 Offboard sequencing, command `target_system`, or startup flow to remove QGC dependence, the terminal logs one concise Chinese explanation of the selected fix.

## Testing approach

- Unit-test staging geometry and staging-complete tolerance checks.
- Run ROS 2 package commands from the `px4_ws` workspace, not the outer Docker workspace.
- Use fake vehicle status streams to test ground-station takeoff and land-all action behavior.
- Run a manual SITL smoke test inside the container once PX4 SITL and Micro XRCE-DDS Agent are externally running.
- Run the manual SITL smoke test with QGC closed and record the no-QGC arm/takeoff result.
- If no-QGC arm/takeoff fails, use `$diagnosing-bugs` before changing control logic so the failure mode is captured and verified.

## Blocking edges

- Blocked by 05 - Validate three-vehicle namespaces and telemetry flow.
- Blocked by 06 - Build `ground_station_node` action surface and swarm topics.
- Blocks 08 - Implement leader movement by absolute world position plus yaw.
