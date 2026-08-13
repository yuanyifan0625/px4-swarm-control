# 12 - Add swarm launch and SITL smoke acceptance workflow

**What to build:** Provide the normal ROS 2 launch path for the four swarm nodes after the first-version vehicle namespaces have been renamed to `/MAV1`, `/MAV2`, and `/MAV3`, then document and manually verify a complete SITL smoke workflow from clean runtime through slow-demo speed-profile check/apply/re-check, ROS launch, operator-console demo, final landed status, and runtime cleanup.

**Blocked by:** 12a - Rename first-version vehicle namespaces to `/MAV1`, `/MAV2`, `/MAV3`.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container. PX4/Gazebo/runtime commands must run inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

Manual startup was useful while building the swarm control stack, but the completed first-version SITL workflow needs a repeatable launch path for the ROS 2 swarm nodes. PX4 SITL, Gazebo, and Micro XRCE-DDS Agent remain external prerequisites in this version. The launch file should make the ROS node topology repeatable without hiding simulator, bridge, or operator-console failures.

Tickets 11b, 11c, and 11d added two operator-side tools that must be part of the smoke workflow but not part of the core ROS launch:

- `operator_console` is a short-command wrapper around the existing `/swarm` action surface.
- `px4_speed_profile` is a PX4 runtime parameter check/apply workflow for profiles such as `slow_demo`.

Neither tool should be silently started or applied by the swarm launch file. The launch file should start the control nodes only; the manual smoke document should explain when and how the operator uses the console and speed-profile workflow.

## Scope

- Add a ROS 2 launch workflow that starts exactly three parameterized `vehicle_node` instances and one `ground_station_node`.
- Use the post-12a namespace contract: `/MAV1` is leader, `/MAV2` is follower-left, and `/MAV3` is follower-right.
- Use the existing vehicle-node YAML configuration as the source of truth for roles, vehicle IDs, PX4 namespaces, PX4 target systems, slots, control rates, hold setpoints, and related runtime settings.
- Keep launch configuration explicit so role/slot/namespace mistakes are easy to inspect.
- Document the external prerequisites for SITL smoke verification:
  - clean runtime
  - `MicroXRCEAgent udp4 -p 8888`
  - three PX4/Gazebo `gz_x500` instances using `/MAV1`, `/MAV2`, `/MAV3`
  - live bridge smoke check
- Document the `slow_demo` PX4 speed-profile workflow as a manual preflight step:
  - check first
  - apply only with explicit `apply --yes`
  - re-check after applying
- Document `operator_console` as a separate terminal workflow, not as a process started by the swarm launch file.
- Add a concise Chinese manual SITL smoke document that includes all commands from clean runtime through successful demo and cleanup.
- Perform one full manual SITL verification after implementation and update the manual document with the final working command sequence.

## Non-goals

- Do not launch PX4 SITL from the swarm launch file.
- Do not launch Gazebo from the swarm launch file.
- Do not launch Micro XRCE-DDS Agent from the swarm launch file.
- Do not launch QGC from the swarm launch file.
- Do not launch `operator_console` from the swarm launch file.
- Do not check or apply `px4_speed_profile` from the swarm launch file.
- Do not silently apply PX4 speed parameters during any normal launch.
- Do not support real-vehicle deployment in this ticket.
- Do not add `column`, dynamic slots, automatic leader reassignment, or new swarm behaviors.
- Do not modify PX4-Autopilot, `px4_msgs`, `vehicle_node` control behavior, or follower-control logic except where a launch/test/doc issue exposes a narrow bug in the existing launch path.

## Implementation notes

- The recommended launch file name is `swarm_nodes.launch.py`, because it starts ROS swarm nodes only and does not start PX4/Gazebo/DDS.
- Keep `operator_console` as an explicit separate command so terminal input and demo macro execution remain under operator control.
- Keep `slow_demo` speed profile outside launch. It is a PX4 runtime parameter workflow, not ROS node configuration.
- Include concise Chinese comments only where launch/config logic protects against role-slot-namespace mismatch or unsafe defaults.
- If the manual smoke workflow needs cleanup commands, prefer commands that do not match and kill their own shell wrapper.

## Acceptance criteria

- [ ] A normal launch path starts one `/swarm` `ground_station_node` and three `vehicle_node` instances for `/MAV1`, `/MAV2`, and `/MAV3`.
- [ ] Launch/config maps `/MAV1` to leader, `/MAV2` to follower-left slot, and `/MAV3` to follower-right slot.
- [ ] The launch file does not start PX4 SITL, Gazebo, Micro XRCE-DDS Agent, QGC, `operator_console`, or `px4_speed_profile`.
- [ ] Documentation states that PX4 SITL/Gazebo and Micro XRCE-DDS Agent must be started externally before launching swarm nodes.
- [ ] Documentation includes the `slow_demo` preflight sequence: check, explicit `apply --yes`, and re-check.
- [ ] Documentation includes a separate `operator_console --command 9` demo command and explains that `operator_console` wraps existing `/swarm` actions.
- [ ] Manual SITL smoke verification reaches staging and logs or reports `all vehicles reached staging positions`.
- [ ] Manual SITL smoke verification moves the leader and followers maintain fixed-slot following from leader state and formation mode.
- [ ] Manual SITL smoke verification changes between `vee` and `line_abreast` and reports formation completion.
- [ ] Manual SITL smoke verification ends with `operator_console` reporting `OK: demo macro completed`.
- [ ] Manual SITL smoke verification shows `/MAV1/status`, `/MAV2/status`, and `/MAV3/status` all end with `vehicle_state: landed` and `armed: false`.
- [ ] Manual SITL smoke verification confirms the demo does not create new continuous follower absolute target flow from the operator or ground station.
- [ ] Runtime is clean after verification; no Micro XRCE-DDS Agent, PX4 SITL, Gazebo, swarm ROS nodes, or operator console processes are left running.

## Testing approach

- Run ROS 2 package tests inside the container from `/home/ncrl/docker_ubuntu24/px4_ws`.
- Add launch syntax/import tests if practical.
- Add tests or checks that the launch description contains exactly the expected four swarm nodes and does not include PX4/Gazebo/DDS/operator-console/speed-profile processes.
- Run `check_live_px4_gz_bridge` against externally started three-vehicle SITL before launching swarm nodes.
- Run `px4_speed_profile` check/apply/re-check manually before the demo when slow demo motion is desired.
- Run the launch file and then run `operator_console --command 9` from a separate terminal.
- Verify final status topics, action/console output, and cleanup state.

## Blocking edges

- Blocked by 12a - Rename first-version vehicle namespaces to `/MAV1`, `/MAV2`, `/MAV3`.
- Continues from 11b - Add configurable operator short-command console.
- Continues from 11c - Add demo settle gate to operator console.
- Continues from 11d - Add PX4 speed profile check and explicit apply workflow.
