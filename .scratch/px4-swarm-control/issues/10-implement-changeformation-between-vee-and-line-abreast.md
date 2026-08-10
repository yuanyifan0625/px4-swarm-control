# 10 - Implement `ChangeFormation` between `vee` and `line_abreast`

**What to build:** Allow the operator to switch formation mode between `vee` and `line_abreast`, with followers transitioning between body-frame slots and the ground station reporting formation completion.

**Blocked by:** 09 - Implement follower fixed-slot following from leader state.

**Status:** ready-for-agent

**Workspace note:** Project ROS 2 packages live under `px4_ws/src/`. Run `colcon`, `ros2 interface`, ROS 2 package tests, and ROS 2 launch commands from `/home/ncrl/docker_ubuntu24/px4_ws` inside the container. QGC is optional for monitoring/manual safety observation and is not the first-version control entrypoint.

## Background

First-version formation changes should stay simple: only `vee` and `line_abreast`, only fixed slots, and only body-frame slot transitions. The ground station chooses/broadcasts formation mode, while followers compute their own setpoints.

## Scope

- Implement `ChangeFormation` action behavior.
- Broadcast target formation mode from the ground station.
- Have follower nodes consume formation mode and switch their body-frame target offsets.
- Detect formation establishment from vehicle status and tolerances.
- Log `formation established` from the ground station.
- Add action feedback/result for formation change.
- Add tests for formation mode switching and completion detection.

## Non-goals

- Do not implement `column`.
- Do not implement custom arbitrary offsets.
- Do not implement dynamic slot assignment.
- Do not implement follower-follower coordination.

## Implementation notes

- Formation mode changes should alter desired slots, not vehicle identity.
- Add a one-line Chinese comment near formation completion logic explaining that it protects the operator from assuming the mode changed before vehicles reached the new slots.

## Acceptance criteria

- [ ] `ChangeFormation` accepts `vee` and `line_abreast`.
- [ ] Unsupported formation modes are rejected clearly.
- [ ] Ground station broadcasts the target formation mode.
- [ ] Followers compute new body-frame slot setpoints from the target mode.
- [ ] Ground station logs `formation established` after all vehicles meet formation tolerance.
- [ ] Tests cover both formation modes and invalid mode rejection.

## Testing approach

- Unit-test slot offsets for `vee` and `line_abreast`.
- Run ROS 2 package commands from the `px4_ws` workspace, not the outer Docker workspace.
- Use fake follower status to test action feedback/result and completion detection.
- Manually verify in SITL that followers move between the two modes while leader continues to define heading.

## Blocking edges

- Blocked by 09 - Implement follower fixed-slot following from leader state.
- Blocks 12 - Add swarm launch and SITL smoke acceptance workflow.
