# 06 - Build `ground_station_node` action surface and swarm topics

**What to build:** Create the ground-station node that exposes operator actions, owns mission-level state, and communicates internally with vehicle nodes through topics.

**Blocked by:** 01 - Scaffold ROS 2 packages and interfaces; 02 - Add internal models, state enums, and formation geometry.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

Operator commands should enter through ROS 2 actions. Internal communication from the ground station to vehicle nodes should use topics for observability and simplicity. The ground station supervises the swarm but does not compute continuous follower setpoints.

## Scope

- Implement `ground_station_node`.
- Add action servers for `TakeoffSwarm`, `MoveLeader`, `ChangeFormation`, `PauseSwarm`, and `LandSwarm`.
- Publish mission command, leader goal, formation mode, and failsafe/pause commands as internal topics.
- Subscribe to vehicle status summaries.
- Maintain mission-level state.
- Provide action feedback/results appropriate for long-running commands.
- Log mission-level progress without noisy per-vehicle spam.
- Add tests for action acceptance/rejection, topic publication, and mission-state transitions.

## Non-goals

- Do not compute follower continuous setpoints.
- Do not implement actual takeoff completion logic yet.
- Do not command PX4 directly.
- Do not manage PX4 SITL or Micro XRCE-DDS Agent.

## Implementation notes

- Mission-level state belongs here; vehicle-level state belongs in vehicle nodes.
- Add one-line Chinese comments near state-transition guards explaining which invalid mission transition they prevent.

## Acceptance criteria

- [ ] The ground-station node starts under the `/swarm` namespace.
- [ ] All five operator actions exist and return clear feedback/results.
- [ ] Ground-station-to-vehicle outputs are topics, not downstream actions.
- [ ] Vehicle status summaries are consumed and reflected in mission-level state.
- [ ] The ground station logs mission-level transitions only.
- [ ] Tests cover action command handling and mission-state transitions.

## Testing approach

- Use ROS 2 action clients or tests to send each action and verify feedback/result behavior.
- Run ROS 2 package commands from the `px4_ws` workspace, not the outer Docker workspace.
- Use fake vehicle status publishers to exercise mission-state transitions.
- Verify expected internal topics are published when actions are invoked.

## Blocking edges

- Blocked by 01 - Scaffold ROS 2 packages and interfaces.
- Blocked by 02 - Add internal models, state enums, and formation geometry.
- Blocks 07 - Deliver synchronized takeoff to staging and land-all milestone.
