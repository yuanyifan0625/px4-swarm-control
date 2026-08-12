# 07c - Fix repeatable TakeoffSwarm/LandSwarm mission cycles and stale landed/staging state

**What to build:** Make the first-version SITL swarm mission repeatable from the operator action interface: after DDS, three PX4 Gz vehicles, vehicle nodes, and ground station are started once, one `TakeoffSwarm` command must take all three vehicles to staging, one `LandSwarm` command must land them, and the same takeoff/land cycle must work again without restarting runtime processes.

**Blocked by:** 07b - Fix takeoff-to-offboard staging sequencing.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, ROS 2 package tests, ROS 2 launch commands, PX4 SITL commands, Gazebo commands, and Micro XRCE-DDS Agent commands from inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

Ticket 07b fixed the intended takeoff-to-Offboard staging sequence, but live diagnosis after a clean runtime found the mission is not yet repeatable or self-consistent from the operator's perspective.

The diagnosed behavior was:

- Three PX4 Gz vehicles, Micro XRCE-DDS Agent, PX4 publishers, and ROS 2 vehicle status topics were all present and correctly namespaced.
- From a clean runtime, the first `TakeoffSwarm` caused PX4 to arm and climb to about `z=-4`, but the ROS 2 vehicle status still reported `vehicle_state: landed`.
- Sending the same `TakeoffSwarm` a second time caused all three vehicles to enter Offboard and reach the expected staging positions.
- After `LandSwarm`, sending another single `TakeoffSwarm` without restarting runtime left the ROS 2 state in `landed` and did not complete the staging mission.
- PX4 `vehicle_land_detected` telemetry showed the vehicles were not landed once airborne, so the stale `landed` state is coming from the ROS 2 mission/state boundary rather than Gazebo, DDS, PX4 publisher discovery, namespace mapping, or QGC.

The likely root causes are:

- Vehicle-level `landed` state is treated as evidence that the vehicle is still landed, so an old internal state can reinforce itself after PX4 telemetry has changed.
- Ground station completion logic can use cached `landed` statuses from the previous mission and mark the current takeoff mission `done` before fresh takeoff/staging statuses arrive.
- `TakeoffSwarm` currently reports success when commands are accepted/published, not when all three vehicles have actually reached staging.
- Staging setpoints are too dependent on one-shot delivery, making startup order and topic timing more fragile than necessary.

This ticket closes the gap between command acceptance and real mission completion before leader movement and follower formation behavior are added.

## Scope

- Keep PX4-Autopilot unmodified.
- Keep QGC out of the required control path.
- Preserve `TakeoffSwarm` and `LandSwarm` as the operator-facing action commands.
- Fix vehicle-level landed-state handling so fresh PX4 landed telemetry, arming state, altitude, and active mission phase determine whether the vehicle can report `landed`.
- Clear or version stale vehicle mission state when a new takeoff or land command starts.
- Prevent an old internal `landed` state from overriding fresh airborne PX4 telemetry.
- Prevent ground station from using cached landed statuses to mark a new `TakeoffSwarm` mission complete.
- Make ground station evaluate `all vehicles reached staging positions` only for the current takeoff mission generation.
- Make `TakeoffSwarm` action completion mean the staging milestone completed, or return a timeout/failure if staging does not complete.
- Make `LandSwarm` action completion mean all three vehicles reached a confirmed landed state for the current land mission, or return a timeout/failure.
- Improve staging setpoint delivery so every vehicle has the current staging target before and during the takeoff-to-Offboard sequence, instead of depending on a single fragile publish.
- Keep the first-version staging geometry unchanged: world-frame staging, leader centered, `vehicle_2` behind-left, `vehicle_3` behind-right.
- Add concise one-line Chinese comments near key state/epoch/telemetry checks explaining which stale-state or sequencing risk they protect.
- Update the Chinese manual smoke document so it verifies a repeated takeoff-land-takeoff-land cycle, not only a single takeoff and land.

## Non-goals

- Do not modify PX4-Autopilot source or PX4 internal controllers.
- Do not require QGC for arming, takeoff, Offboard transition, staging, landing, or repeat cycles.
- Do not change the `/vehicle_1`, `/vehicle_2`, `/vehicle_3` namespace convention.
- Do not change PX4 target-system mapping unless a focused test proves the mapping is directly wrong.
- Do not implement leader movement beyond staging.
- Do not implement follower following.
- Do not implement formation changes.
- Do not add collision avoidance, custom takeoff control, or custom landing control.
- Do not add a full launch orchestration system for PX4/Gazebo/DDS unless it is strictly needed to make the repeatability tests executable.

## Acceptance criteria

- [ ] From a clean runtime, one `TakeoffSwarm` action with `altitude_m: 5.0` causes all three vehicles to arm, climb, enter Offboard staging control, and reach the expected staging positions without sending the action a second time.
- [ ] While any vehicle is armed and airborne, or PX4 landed telemetry says `landed=false`, that vehicle's ROS 2 status must not report `vehicle_state: landed`.
- [ ] Ground station does not transition a takeoff mission to `done` using cached `landed` statuses from startup or a previous landing.
- [ ] Ground station logs `all vehicles reached staging positions` only after all three current-mission statuses have real telemetry, are armed, have Offboard accepted or active, and are within staging tolerance.
- [ ] `TakeoffSwarm` action result is success only after all three vehicles reach staging for the current takeoff mission.
- [ ] `TakeoffSwarm` action result is failure or timeout if staging is not reached before the requested timeout.
- [ ] `LandSwarm` action result is success only after all three vehicles report confirmed landed state for the current land mission.
- [ ] `LandSwarm` action result is failure or timeout if all three vehicles do not land before the requested timeout.
- [ ] After a successful `LandSwarm`, another single `TakeoffSwarm` works without restarting Micro XRCE-DDS Agent, PX4 SITL instances, Gazebo, vehicle nodes, or ground station.
- [ ] The repeated live SITL cycle `TakeoffSwarm -> LandSwarm -> TakeoffSwarm -> LandSwarm` keeps Gazebo motion, PX4 telemetry, and `/vehicle_*/status` consistent at every milestone.
- [ ] Staging target delivery is robust to normal startup/topic timing: a vehicle must not miss its current staging target because it received the mission command before the setpoint message.
- [ ] The Chinese manual smoke document includes explicit pass conditions for first takeoff, first landing, second takeoff without restart, and second landing.

## Testing approach

- Add TDD coverage at the vehicle-node core level for the diagnosed stale landed bug:
  - a vehicle that previously reported `landed` must stop reporting `landed` when fresh PX4 telemetry says it is airborne;
  - an active takeoff mission must not be immediately overwritten by an old internal `landed` state;
  - after a confirmed landing, a new takeoff command must clear the stale landed condition and progress through takeoff staging again.
- Add TDD coverage for ground-station mission generation or equivalent freshness handling:
  - cached landed statuses from startup do not complete a new takeoff mission;
  - cached landed statuses from a previous land mission do not complete a new takeoff mission;
  - staging completion requires fresh current-mission vehicle statuses;
  - landing completion requires fresh current-mission landed statuses.
- Add action-level tests proving:
  - `TakeoffSwarm` does not return success until staging is complete;
  - `TakeoffSwarm` times out when staging completion never arrives;
  - `LandSwarm` does not return success until all vehicles are landed;
  - `LandSwarm` times out when landing completion never arrives.
- Add tests for staging setpoint robustness so the current staging target is available to each vehicle before the takeoff-to-Offboard sequence depends on it.
- Run ROS 2 build and package tests from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container.
- Manually verify in SITL with Micro XRCE-DDS Agent and three external PX4 Gz instances:
  - `check_live_px4_gz_bridge --agent-log ...` passes;
  - send one `TakeoffSwarm` and confirm all three vehicles reach staging;
  - send one `LandSwarm` and confirm all three vehicles land;
  - without restarting runtime, send one more `TakeoffSwarm` and confirm all three vehicles reach staging again;
  - send one more `LandSwarm` and confirm all three vehicles land again;
  - confirm `/vehicle_*/status` never reports `landed` while the simulator and PX4 telemetry show the vehicle is airborne.

## Blocking edges

- Blocked by 07b - Fix takeoff-to-offboard staging sequencing.
- Blocks 08 - Implement leader movement by absolute world position plus yaw.
- Blocks 09 - Implement follower fixed-slot following from leader state.
- Blocks 10 - Implement ChangeFormation between vee and line-abreast.
- Blocks 12 - Add swarm launch and SITL smoke acceptance workflow.
