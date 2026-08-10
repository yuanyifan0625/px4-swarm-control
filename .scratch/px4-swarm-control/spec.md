# PX4 Swarm Control Build Spec

Status: ready-for-agent

## Problem Statement

The user wants to build a first-version three-aircraft cooperative control system using PX4, ROS 2 Jazzy, and Gazebo SITL. The system should let three `gz_x500` vehicles take off together, designate one vehicle as the leader, let two followers track the leader and change formation, and land the whole swarm together.

The user wants this work to happen in the outer Docker workspace and ROS 2 workspace, not by changing PX4 internals. PX4-Autopilot should remain an upstream dependency for cooperative-control work. ROS 2 should command PX4 through `px4_msgs` topics over Micro XRCE-DDS, using high-level Offboard position and yaw setpoints plus takeoff and land commands. PX4 should continue to own low-level stabilization, attitude control, motor control, and flight safety behavior.

The user also wants the architecture to be easy to debug and extend. The first version is SITL-only, but ROS 2 interfaces and namespace conventions should avoid simulation-only assumptions so the design can later move toward real vehicles.

## Solution

Build a ROS 2 Jazzy swarm-control system for three PX4 SITL `gz_x500` vehicles. The runtime design has four primary ROS 2 nodes:

- One `ground_station_node`
- Three instances of one parameterized `vehicle_node` executable

The `ground_station_node` provides the operator-facing action interface, monitors swarm state, sends leader goals, broadcasts formation mode, coordinates takeoff and landing, and handles pause/failsafe commands. It does not compute the followers' continuous formation setpoints.

Each `vehicle_node` represents one aircraft and is parameterized by role, vehicle ID, PX4 namespace, and formation slot. The leader instance receives leader goals. Each follower instance subscribes to leader state and formation mode, then computes its own position and yaw setpoint from its fixed slot. The followers do not command each other and do not directly receive operator goals.

Micro XRCE-DDS Agent is treated as the external ROS 2-PX4 bridge. The ROS 2 package should communicate with PX4 through `px4_msgs` topics. A shared internal `Px4VehicleInterface` class/module should encapsulate `px4_msgs` publishers/subscribers, QoS, timestamps, namespace handling, Offboard heartbeat, takeoff, land, arm/disarm, and position+yaw setpoint publication. Control logic outside that interface should use internal concepts such as `VehicleState`, `PositionYawSetpoint`, and `VehicleCommandResult`.

The first milestone should validate the complete foundation before formation following: three vehicles spawn, telemetry reaches ROS 2, a ground-station action triggers synchronized takeoff to separated staging positions, terminal output reports that all vehicles reached staging positions, and a ground-station action lands all three vehicles safely.

## User Stories

1. As the developer, I want the first version to be limited to PX4 SITL, Gazebo, and ROS 2 Jazzy, so that I can build and debug the system without real-flight risk.
2. As the developer, I want the ROS 2 interfaces to avoid SITL-only assumptions, so that future real-vehicle integration remains possible.
3. As the developer, I want PX4-Autopilot treated as an upstream dependency, so that cooperative control changes do not fork PX4 control logic.
4. As the developer, I want to avoid modifying PX4-Autopilot for swarm-control behavior, so that all cooperative logic stays in my ROS 2 workspace.
5. As the developer, I want PX4 bug fixes to be allowed only when clearly necessary, so that real upstream/environment bugs can still be handled deliberately.
6. As the developer, I want Micro XRCE-DDS Agent treated as the ROS 2-PX4 bridge, so that ROS 2 nodes can communicate with PX4 through `px4_msgs` topics.
7. As the operator, I want a ground-station action interface, so that I can issue high-level commands without directly publishing low-level PX4 topics.
8. As the operator, I want to trigger whole-swarm takeoff, so that all three vehicles enter the mission together.
9. As the operator, I want to trigger whole-swarm landing, so that all three vehicles can be recovered together.
10. As the operator, I want to send a leader goal as an absolute world position plus yaw, so that I can move the formation target predictably in Gazebo/RViz and optionally observe it in QGC.
11. As the operator, I want to change formation mode, so that the followers can rearrange around the leader.
12. As the operator, I want to pause the swarm, so that I can stop progression during debugging or unsafe behavior.
13. As the operator, I want a failsafe/land-all command, so that I can recover from bad states during testing.
14. As the operator, I want terminal progress messages for major mission transitions, so that I can understand what the system is doing without inspecting every topic.
15. As the operator, I want the ground station to report when all vehicles reached staging positions, so that I know takeoff staging completed.
16. As the operator, I want the ground station to report when formation is established, so that I know the system entered formation behavior.
17. As the developer, I want each vehicle represented by one parameterized `vehicle_node`, so that leader and follower behavior share one implementation shape.
18. As the developer, I want `role`, `vehicle_id`, `px4_namespace`, and `slot` parameters, so that one executable can represent all three vehicles.
19. As the developer, I want `vehicle_1` to be the leader namespace, so that the first version has a stable, inspectable leader assignment.
20. As the developer, I want `vehicle_2` to be follower-left slot 1, so that slot naming matches the vehicle's initial geometric position.
21. As the developer, I want `vehicle_3` to be follower-right slot 2, so that left/right slot assignments are not ambiguous.
22. As the developer, I want follower-left and follower-right staging positions defined relative to the leader's initial yaw, so that "left" and "right" remain geometrically meaningful.
23. As the developer, I want staging to use world-frame positions, so that takeoff and landing can be kept collision-resistant.
24. As the developer, I want formation following to use leader body-frame offsets, so that the formation rotates with leader heading during cruise/follow behavior.
25. As the developer, I want first-version formation changes to switch between body-frame slots, so that formation behavior stays simple enough to debug.
26. As the developer, I want `vee` formation support, so that the followers can sit behind the leader on left and right slots.
27. As the developer, I want `line_abreast` formation support, so that the followers can sit left and right of the leader.
28. As the developer, I want `column` deferred, so that first-version control does not start with the delay-sensitive formation.
29. As the developer, I want followers to subscribe to leader position, yaw, velocity, and status, so that each follower can compute its own setpoint from leader state.
30. As the developer, I want followers to avoid follower-follower coordination in the first version, so that fixed spacing and staging handle collision risk initially.
31. As the developer, I want fixed follower slots in the first version, so that dynamic slot assignment does not complicate the state machine.
32. As the developer, I want the operator to command only leader goal, formation mode, takeoff, land, pause, and failsafe, so that followers remain derived-control participants.
33. As the developer, I want each follower to output only its own position and yaw setpoint, so that nodes do not command other vehicles.
34. As the developer, I want `Px4VehicleInterface` to encapsulate `px4_msgs`, so that formation logic does not depend directly on PX4 message shapes.
35. As the developer, I want control logic to use internal models like `VehicleState`, so that future bridge or namespace changes are localized.
36. As the developer, I want internal models like `PositionYawSetpoint`, so that controller code expresses the control intent directly.
37. As the developer, I want important conversions and critical functions to have one-line comments, so that later inspection and debugging are easier.
38. As the developer, I want comments around `px4_msgs` conversion, coordinate conversion, yaw conversion, Offboard command publishing, takeoff, and landing, so that risky transformations are visible.
39. As the developer, I want Offboard control limited to high-level position plus yaw, so that PX4 remains responsible for attitude and motor control.
40. As the developer, I want action APIs only at the operator-to-ground-station boundary, so that long-running commands can expose progress and result.
41. As the developer, I want ground-station-to-vehicle communication to use topics, so that the internal ROS 2 graph remains observable and simple.
42. As the developer, I want vehicle status topics, so that the ground station and operator can monitor vehicle role, position, yaw, velocity, arming state, navigation state, Offboard availability, telemetry age, and local control state.
43. As the developer, I want a mission-level state machine in the ground station, so that multi-vehicle sequencing is explicit.
44. As the developer, I want each vehicle node to maintain its own vehicle-level state, so that vehicle-specific transitions and timeouts are isolated.
45. As the developer, I want first-version mission states including idle, arming, taking off, staging, forming, following, reconfiguring, paused, landing, done, failsafe, and error, so that major behavior is inspectable.
46. As the developer, I want three vehicles to arm and take off together, so that the first version validates synchronized swarm startup.
47. As the developer, I want each vehicle to use a different horizontal staging position, so that vehicles do not start too close together.
48. As the developer, I want the leader centered in staging and followers behind-left and behind-right, so that staging matches the default `vee` intuition.
49. As the developer, I want first-version failsafe to hover on vehicle telemetry timeout, so that transient data loss does not cause uncontrolled motion.
50. As the developer, I want followers to hover on leader telemetry/status timeout, so that leader loss does not produce stale follow commands.
51. As the operator, I want pause to hold safe setpoints, so that I can stop motion without immediately landing.
52. As the operator, I want land-all available as an action, so that I can recover the full swarm from the ground station.
53. As the developer, I want Python/rclpy for the first version, so that iteration speed and logging are better during architecture debugging.
54. As the developer, I want a main ROS 2 package for control nodes, launch, config, and shared modules, so that the first version is easy to navigate.
55. As the developer, I want a separate interfaces package for actions and messages, so that generated ROS 2 interfaces do not mix with Python node logic.
56. As the developer, I want control loops initially at 20 Hz, so that Offboard setpoints and follower control are frequent enough for SITL debugging.
57. As the developer, I want ground-station state loops at 5-10 Hz, so that mission state monitoring is responsive without excessive logs.
58. As the developer, I want manual node startup supported first, so that early debugging can isolate PX4, Micro XRCE-DDS, vehicle nodes, and ground station.
59. As the developer, I want a swarm launch file after manual startup works, so that normal operation can start all ROS 2 swarm nodes together.
60. As the developer, I want Micro XRCE-DDS Agent and PX4 SITL started externally in the first version, so that launch complexity does not hide bridge or simulator failures.
61. As the developer, I want the first milestone to prove three-vehicle staging and landing before follow behavior, so that the hardest infrastructure risks are handled first.
62. As the developer, I want later milestones to add leader move, follower following, formation change, and failsafe incrementally, so that failures are attributable to one layer at a time.

## Implementation Decisions

- First-version scope is PX4 SITL, Gazebo, and ROS 2 Jazzy only. The architecture should avoid simulation-only naming and assumptions where practical, but real-vehicle deployment is not part of this build spec.
- PX4-Autopilot must not be modified for cooperative-control behavior. Any PX4-Autopilot edit is out of the normal path and should be limited to clearly identified environment, compatibility, build, or upstream bug fixes.
- The ROS 2 system commands PX4 through high-level Offboard position plus yaw setpoints, plus takeoff and land commands. It does not send attitude, thrust, actuator, or motor commands.
- Micro XRCE-DDS Agent is the external ROS 2-PX4 bridge. Development and testing require the agent to be running on UDP port 8888 before expecting `px4_msgs` traffic.
- Build new ROS 2 functionality in a new Python/rclpy control package and a separate ROS 2 interfaces package. Existing upstream packages remain dependencies.
- New project ROS 2 packages live under the ROS 2 workspace `px4_ws/src/`, not directly under the outer Docker workspace.
- Runtime topology has four primary ROS 2 nodes: one ground-station node and three instances of a parameterized vehicle node.
- The vehicle node executable is shared by all three vehicles and switches behavior by parameters: role, vehicle ID, PX4 namespace, and formation slot.
- Vehicle namespace convention is stable and role-independent: `/vehicle_1` is the first-version leader, `/vehicle_2` is follower-left slot 1, `/vehicle_3` is follower-right slot 2, and `/swarm` is the swarm-level namespace.
- Do not use `leader` as a vehicle namespace. Leadership is a role parameter so that future leader reassignment remains possible without renaming topics.
- The ground-station node owns the operator-facing action interface, mission-level state, swarm monitoring, leader goal publication, formation-mode publication, whole-swarm takeoff/land, pause, and failsafe commands.
- The ground-station node does not compute continuous follower setpoints. Follower setpoint generation is distributed into each follower vehicle node.
- Operator-to-ground-station commands are ROS 2 actions: `TakeoffSwarm`, `MoveLeader`, `ChangeFormation`, `PauseSwarm`, and `LandSwarm`.
- Ground-station-to-vehicle communication uses ROS 2 topics for leader goal, formation mode, mission/failsafe command, and status aggregation.
- Each vehicle node publishes a status summary topic. The status model should include role, vehicle ID, position, yaw, velocity, armed state, navigation state, Offboard availability, telemetry age, and vehicle-level state.
- Each follower subscribes to leader position, yaw, velocity, and status, then computes its own position plus yaw setpoint from its fixed slot and current formation mode.
- Followers do not directly receive operator movement goals. User movement intent reaches followers only through leader state and formation mode.
- Followers do not command each other or perform follower-follower coordination in the first version.
- Follower slots are fixed in the first version. Dynamic slot assignment is deferred.
- `vehicle_2` must start as follower-left slot 1, and its initial staging position must be on the leader's left side.
- `vehicle_3` must start as follower-right slot 2, and its initial staging position must be on the leader's right side.
- Left/right staging directions are defined relative to the leader's initial yaw/heading. Staging points are computed as world-frame positions derived from that initial heading.
- Takeoff, landing, and staging use world-frame positions to reduce collision risk.
- Cruise/follow formation uses leader body-frame offsets so that slots rotate with current leader yaw/heading.
- Formation changes in the first version transition between body-frame slots only.
- First-version formation modes are `vee` and `line_abreast`. `column` is deferred.
- Whole-swarm takeoff arms and takes off all three vehicles together to the same altitude while maintaining separated horizontal staging positions.
- The ground-station node logs mission-level progress lines, including when all vehicles reach staging positions and when formation is established.
- Vehicle nodes log only their own state transitions and relevant local events to avoid noisy terminal output.
- The ground station owns mission states including idle, arming, taking off, staging, forming, following, reconfiguring, paused, landing, done, failsafe, and error.
- Each vehicle node owns vehicle-level state for arming, Offboard availability, local setpoint tracking, telemetry freshness, pause/hover, landing, and error handling.
- `MoveLeader` uses absolute world position plus yaw in the first version. Relative movement commands are deferred.
- First-version failsafe behavior is intentionally minimal: a vehicle telemetry timeout causes that vehicle to hover or keep its last safe setpoint; leader telemetry/status timeout causes followers to hover; operator can trigger pause or land-all.
- Do not implement automatic leader reassignment, dynamic slot reassignment, or complex autonomous recovery in the first version.
- `Px4VehicleInterface` is an internal shared class/module used by each vehicle node. It is not a ROS 2 node and does not replace Micro XRCE-DDS Agent.
- `Px4VehicleInterface` owns `px4_msgs` publishers/subscribers, QoS choices, namespace handling, timestamps, Offboard heartbeat, arm/disarm, takeoff, land, position+yaw setpoint publication, telemetry subscriptions, and relevant command result tracking.
- Control logic outside `Px4VehicleInterface` should use internal models such as `VehicleState`, `PositionYawSetpoint`, and `VehicleCommandResult`, not raw `px4_msgs`.
- Important transformations and critical functions should have short one-line comments, especially PX4 message conversion, coordinate-frame conversion, yaw conversion, Offboard heartbeat, takeoff command, and land command.
- Vehicle control and follower formation loops start at 20 Hz. Ground-station mission monitoring starts at 5-10 Hz.
- Manual startup should be supported before a combined launch file. After manual startup works, provide a launch file to start all three vehicle nodes and the ground station together.
- PX4 SITL and Micro XRCE-DDS Agent remain externally started prerequisites for the first version rather than being managed by the swarm launch file.
- Development and verification commands for ROS 2, PX4, build, test, and runtime must run inside the Docker container using the workspace command pattern from the agent instructions.
- ROS 2 package commands such as `colcon build`, `colcon test`, `colcon test-result`, `ros2 interface show`, and ROS 2 launch commands should run from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container.
- QGC is optional for monitoring and manual safety observation during development. It is not the first-version control entrypoint for swarm takeoff, leader movement, formation change, pause, failsafe, or land-all.

## Testing Decisions

- The highest-value test seam is the operator-facing action interface observed through vehicle status topics, ground-station progress logs, and SITL vehicle behavior. Tests should validate externally visible outcomes rather than internal callback structure.
- First milestone acceptance test: with three PX4 SITL `gz_x500` vehicles and Micro XRCE-DDS Agent running, invoking the swarm takeoff action causes all three vehicles to arm, take off to the configured altitude, reach separated staging positions, emit the ground-station progress line for staging completion, and then land safely when the land-all action is invoked.
- The first milestone should not assert formation following. It should prove multi-vehicle namespace handling, bridge traffic, Offboard command publication, vehicle status aggregation, action handling, staging geometry, and land-all behavior.
- Add unit tests for geometry transformations: leader initial yaw to world-frame staging positions; leader current yaw to body-frame formation offsets; follower-left and follower-right slot sign conventions.
- Add unit tests for mission state transitions at the ground-station level: takeoff request, all vehicles staged, formation established, pause, land, timeout, failsafe, and error transitions.
- Add unit tests for follower setpoint derivation using internal models: given leader state, formation mode, and fixed slot, the follower computes the expected position plus yaw setpoint.
- Add unit tests for internal model conversion at the PX4 topic boundary with `Px4VehicleInterface` isolated from controller logic. These tests should verify mapping behavior without requiring live PX4.
- Add integration-style ROS 2 tests that instantiate the ground station and vehicle nodes with fake or simulated vehicle interfaces where possible. These should verify action feedback/results, status publication, and mission-command topics.
- Add SITL smoke tests only after the manual workflow is stable. These tests should exercise the real Micro XRCE-DDS bridge and PX4 topics but remain focused on broad behavior rather than message-by-message implementation details.
- Prior art exists in the upstream ROS-PX4 bridge examples and tests: a Python Offboard example shows position setpoint and command publication, while bridge tests demonstrate checking data flow through ROS 2/PX4 topics. These should guide behavior expectations, not become copied product architecture.
- Do not test raw implementation details such as private timer counters, exact method names, or internal callback ordering unless they are the public seam of a state machine.
- Test logs should require mission-level progress messages from the ground station for major transitions so the operator experience remains verifiable.
- Tests should include negative scenarios for missing or stale telemetry, especially leader timeout causing followers to hover.
- ROS 2 build and test commands must be run inside the container. Do not assume ROS 2 Jazzy, PX4, or Gazebo tools exist on the host.
- ROS 2 workspace verification must run from `px4_ws` in the container to avoid creating unrelated outer-workspace `build/`, `install/`, or `log/` artifacts.
- Automated acceptance should not depend on QGC being open. Verify through ROS 2 actions, topics, logs, and SITL behavior; use QGC only as optional observation.

## Out of Scope

- Real-vehicle deployment.
- Modifying PX4-Autopilot to implement cooperative control.
- Low-level PX4 control changes, including attitude, thrust, actuator, or motor control.
- Replacing or wrapping Micro XRCE-DDS Agent as a custom bridge.
- Dynamic leader reassignment.
- Dynamic follower slot assignment.
- Follower-follower coordination.
- Complex autonomous fault recovery.
- `column` formation.
- Relative leader movement commands.
- Managing PX4 SITL and Micro XRCE-DDS Agent from the first-version swarm launch file.
- Using QGC as the first-version operator control entrypoint.
- Full production-grade UI beyond operator actions and terminal/log feedback.

## Further Notes

- The first build should optimize for inspectability and staged debugging over clever autonomy.
- The first successful milestone should be treated as infrastructure validation: three vehicles, namespaces, ROS 2-PX4 bridge, actions, status, staging geometry, and land-all.
- The recommended milestone order is:
  1. Multi-vehicle SITL, Micro XRCE-DDS Agent, and three-vehicle ROS 2 telemetry namespace validation.
  2. A parameterized vehicle node controlling one PX4 vehicle to position+yaw hover.
  3. Three vehicle node instances simultaneously arm, take off, and reach staging positions.
  4. Ground-station actions for takeoff, land, and pause.
  5. Leader movement with followers not yet following.
  6. Followers compute fixed-slot setpoints from leader state.
  7. Formation change between `vee` and `line_abreast`.
  8. Minimal failsafe.
- Keep `PX4-Autopilot` as upstream dependency code unless the user explicitly approves a targeted fix.
- Terminal output should be useful but not noisy: mission-level progress belongs in the ground station; vehicle-local transitions belong in each vehicle node.
