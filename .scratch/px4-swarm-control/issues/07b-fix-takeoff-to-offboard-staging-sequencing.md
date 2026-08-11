# 07b - Fix takeoff-to-offboard staging sequencing

**What to build:** Make `TakeoffSwarm` reliably transition all three vehicles from PX4-managed takeoff into ROS 2 Offboard staging control, then make `LandSwarm` hand control back to PX4 landing without ROS 2 continuing to fight the descent.

**Blocked by:** 07 - Deliver synchronized takeoff to staging and land-all milestone.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, ROS 2 package tests, ROS 2 launch commands, PX4 SITL commands, Gazebo commands, and Micro XRCE-DDS Agent commands from inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

Ticket 07 implemented the first `TakeoffSwarm` path, but live SITL diagnosis showed that sending `DO_SET_MODE Offboard` at the same moment as arm/takeoff is too early to be reliable. PX4 can arm and climb through `NAV_TAKEOFF`, while `trajectory_setpoint` messages are visible on the ROS 2 graph, but PX4 may remain in an AUTO navigation state with `flag_control_offboard_enabled=false` and `accepts_offboard_setpoints=false`.

A later no-code diagnosis verified the intended sequencing: after all three vehicles climb to a safe altitude, repeatedly publishing Offboard heartbeat plus trajectory setpoint for at least one second, then sending `DO_SET_MODE Offboard`, lets PX4 accept Offboard mode. With that sequence, all three vehicles reached the expected staging targets:

- `vehicle_1`: near `(0, 0, -5)`
- `vehicle_2`: near `(-3, 4, -5)`
- `vehicle_3`: near `(-3, -4, -5)`

This ticket turns that verified runtime sequence into the product behavior.

A separate landing diagnosis found a related sequencing bug in the same vehicle-level state machine: after a `LAND` mission command, `vehicle_node` sends PX4 `NAV_LAND` and enters `landing`, but the next control tick transitions back to `holding` and publishes the old staging setpoint again. That means ROS 2 may keep sending Offboard heartbeat/trajectory targets while PX4 is trying to land. Before tuning PX4 landing parameters or adding a custom landing algorithm, this ticket should first stop ROS 2 from publishing staging setpoints during `LANDING` and wait for PX4 landed detection.

## Scope

- Keep PX4-Autopilot unmodified.
- Keep QGC out of the required startup/control path.
- Preserve `TakeoffSwarm` as the operator-facing action.
- Update vehicle-level takeoff behavior so each `vehicle_node`:
  - stores its own staging target before takeoff control begins;
  - arms and sends PX4 `NAV_TAKEOFF` first;
  - waits until local altitude is safely near the requested staging altitude;
  - publishes Offboard heartbeat and the staging trajectory setpoint continuously for at least one second;
  - sends `DO_SET_MODE Offboard` only after that warmup;
  - confirms PX4 actually entered Offboard before treating staging control as active;
  - keeps publishing the staging position plus yaw setpoint while moving to the staging slot.
- Update ground-station staging completion so `all vehicles reached staging positions` is emitted only after all three vehicles have real telemetry, are armed, are in or accepting Offboard control, and are within staging position tolerance.
- Update vehicle-level landing behavior so each `vehicle_node`:
  - sends PX4 `NAV_LAND` on `LandSwarm`;
  - enters and stays in `landing` instead of being reset to `holding` by the next control tick;
  - stops publishing staging `trajectory_setpoint` and Offboard heartbeat while PX4 landing owns the descent, unless a deliberate safe-hold/failsafe path is active;
  - observes PX4 landed state through available telemetry before reporting `landed`.
- Update ground-station land-all completion so the mission reaches `done` only after all three vehicles report `landed`, not merely after the `LandSwarm` command is accepted.
- Keep the first-version staging geometry unchanged: world-frame staging, leader centered, `vehicle_2` behind-left, `vehicle_3` behind-right.
- Add concise one-line Chinese comments near the sequencing checks explaining which PX4 readiness or collision/sequencing risk they protect.
- Update the Chinese manual smoke document for the new expected Offboard transition and landing checks.

## Non-goals

- Do not modify PX4-Autopilot source or PX4 internal controllers.
- Do not require QGC for arming, takeoff, Offboard transition, staging, or landing.
- Do not implement leader movement beyond staging.
- Do not implement follower following.
- Do not implement formation change.
- Do not implement a custom landing controller or collision-avoidance landing algorithm unless the sequencing fix and PX4 landing telemetry prove it is still needed.
- Do not tune PX4 landing parameters as the first fix; parameter tuning can be a follow-up after ROS 2 stops publishing conflicting staging setpoints during landing.
- Do not add a combined launch file or make ROS 2 launch manage PX4 SITL/Micro XRCE-DDS Agent.
- Do not change namespace conventions, target-system mapping, or staging slot geometry unless the diagnosis proves a direct bug in those boundaries.

## Acceptance criteria

- [ ] `TakeoffSwarm` still succeeds from the operator action interface with QGC closed.
- [ ] All three vehicles arm and climb using PX4-managed takeoff before Offboard staging control is considered active.
- [ ] All three vehicles reach a safe altitude threshold near the requested takeoff altitude, for example `z < -4.0` when `altitude_m: 5.0`.
- [ ] Each vehicle publishes Offboard heartbeat plus its staging `trajectory_setpoint` continuously for at least one second before sending `DO_SET_MODE Offboard`.
- [ ] Each vehicle confirms PX4 Offboard acceptance using `flag_control_offboard_enabled=true`, `nav_state=offboard`, or an equivalent internal model field before treating staging control as active.
- [ ] All three vehicles continue publishing their own staging position plus yaw setpoint after Offboard is accepted.
- [ ] In live SITL, all three vehicles reach the expected staging targets within tolerance:
  - `vehicle_1` near `(0, 0, -5)`
  - `vehicle_2` near `(-3, 4, -5)`
  - `vehicle_3` near `(-3, -4, -5)`
- [ ] Ground station logs `all vehicles reached staging positions` only after all three vehicles have real telemetry, are armed, are in or accepting Offboard control, and are within staging tolerance.
- [ ] `LandSwarm` still commands all three vehicles to land after staging.
- [ ] After `LandSwarm`, each vehicle remains in `landing` or progresses to `landed`; the regular control tick must not transition it back to `holding`.
- [ ] After `LandSwarm`, vehicle nodes stop publishing staging trajectory setpoints that would command the vehicle to stay at the airborne staging position.
- [ ] Vehicle status reports `landed` only after PX4 landed telemetry indicates the vehicle is actually landed.
- [ ] Ground station transitions the mission to `done` only after all three vehicles report `landed`.
- [ ] Live SITL landing verification shows all three vehicles descend under PX4 landing control and settle without ROS 2 continuing to command staging hover.
- [ ] The Chinese manual smoke document explains the expected `z`, Offboard mode, and staging-position pass conditions.
- [ ] The Chinese manual smoke document explains the expected `LandSwarm` checks, including landing/landed states and absence of continued staging setpoint commands during landing.

## Testing approach

- Add TDD coverage at the vehicle-node core level for the sequencing state machine:
  - takeoff command starts PX4 arm plus `NAV_TAKEOFF`;
  - Offboard mode is not requested before the altitude threshold is reached;
  - Offboard heartbeat plus staging setpoint warmup must run for at least one second;
  - `DO_SET_MODE Offboard` is requested after warmup;
  - staging control is active only after Offboard acceptance is observed.
- Add tests for stale/missing telemetry so the vehicle does not claim Offboard staging readiness without valid PX4 state.
- Add ground-station tests proving staging completion requires position tolerance plus armed/telemetry/Offboard readiness, not position alone.
- Add TDD coverage for landing sequencing:
  - `LAND` command calls PX4 `NAV_LAND` and enters `landing`;
  - subsequent control ticks do not publish the old staging setpoint while in `landing`;
  - subsequent control ticks do not transition `landing` back to `holding`;
  - PX4 landed telemetry transitions vehicle state to `landed`;
  - ground station waits for all three `landed` statuses before mission `done`.
- Add a regression test for the diagnosed bug: after `LAND`, one control tick must not publish a staging setpoint.
- Run ROS 2 build and package tests from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container.
- Manually verify in SITL with Micro XRCE-DDS Agent and three external PX4 Gz instances:
  - `check_live_px4_gz_bridge --agent-log ...` passes;
  - `TakeoffSwarm` with `altitude_m: 5.0` causes all vehicles to reach `z < -4.0`;
  - all vehicles report Offboard accepted (`flag_control_offboard_enabled=true` or `nav_state=offboard`);
  - all vehicles reach the staging coordinates within tolerance;
  - `LandSwarm` switches all three vehicles into landing/landed progression;
  - vehicle nodes do not keep commanding airborne staging setpoints during landing;
  - all three vehicles settle on the ground and report `landed`.

## Blocking edges

- Blocked by 07 - Deliver synchronized takeoff to staging and land-all milestone.
- Blocks 08 - Implement leader movement by absolute world position plus yaw.
- Blocks 09 - Implement follower fixed-slot following from leader state.
- Blocks 12 - Add swarm launch and SITL smoke acceptance workflow.
