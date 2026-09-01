# 01 — SITL common ENU coordinate profile and independent PX4 local origins

Type: task

Status: ready-for-human

Blocked by: none

## What to build

Make the `main`-branch Gazebo SITL swarm operate in one canonical **Gazebo ENU
common world** while keeping each PX4 instance's raw local frame at one boundary
only. The result must let the operator use intuitive Gazebo-world movement,
formation geometry, and collision fallback even though MAV1/MAV2/MAV3 have
independent PX4 local origins.

This is SITL-only. The real-hardware coordinate contract remains the existing
`raw_px4_local` profile. `operator_console.py` remains the only user-facing
ROS 2 command entry point.

## Verified coordinate contract

### Canonical SITL frame

- Position is Gazebo ENU: `E` (East), `N` (North), `U` (Up).
- Common yaw is Gazebo ENU yaw: `0 = East`, positive yaw turns toward North.
- Static common origins, in metres:

  | vehicle | origin `(E, N, U)` |
  | --- | --- |
  | MAV1 | `(0, 0, 0)` |
  | MAV2 | `(-1, +1, 0)` |
  | MAV3 | `(-1, -1, 0)` |

### Validated profile conversion

For a vehicle origin `O=(E0,N0,U0)` and PX4 raw local `(x,y,z,heading)`:

```text
E = E0 + y                 x = N - N0
N = N0 + x                 y = E - E0
U = U0 - z                 z = U0 - U

common_yaw = wrap(pi/2 - raw_heading + 0.102)
raw_heading = wrap(pi/2 + 0.102 - common_yaw)
```

`0.102 rad` is one explicit `gazebo_enu_common_world` profile calibration
parameter. It is neither a per-MAV trim nor a real-hardware setting.

### Validation evidence

- MAV1/2/3 physical Gazebo spawn positions differed while all PX4 raw local
  x/y positions initialized near zero. Raw origins are independent.
- Gazebo physical displacement matched `E=origin_E+raw_y` and
  `N=origin_N+raw_x` for all three MAVs with horizontal residuals below 5 cm.
- MAV2 at raw `z=-0.5016` had Gazebo `z=+0.4860`; `U=origin_U-raw_z` passed
  the 5 cm target.
- MAV1 raw yaw samples at `0`, `+pi/2`, `pi`, `-pi/2` matched the formula
  above. MAV2 raw yaw zero independently confirmed the same `~0.102 rad`
  offset.
- MAV3 yaw must still be checked during the post-change three-vehicle Gazebo
  acceptance run; it is not evidence for a per-MAV calibration.

## Architecture boundary

`frame_transform.py` owns a pure, configured `CoordinateProfile` conversion.
`Px4VehicleInterface` is the only PX4/raw boundary:

```text
PX4 VehicleLocalPosition --raw-> interface --common ENU-> VehicleState/status
operator / formation / follower / safety --common ENU-> interface --raw-> PX4 TrajectorySetpoint
```

`VehicleCommand` has no coordinates and remains unchanged. Above the
interface, no module may branch on Gazebo/PX4 semantics or duplicate axis
swaps, signs, offsets, or yaw conversion.

In canonical ENU geometry, use one shared primitive:

```text
forward = (cos(yaw), sin(yaw))
left    = (-sin(yaw), cos(yaw))
world   = leader + forward * forward_m + left * left_m
```

Positive `left_m` means physical left.

## Ordered implementation steps and exact change points

### 1. Add the profile seam and deterministic conversion tests

Change:

- `px4_swarm_control/frame_transform.py`
- `test/test_frame_transform.py` (add a focused profile test file only if it
  makes the tests clearer)

Implement the two named profiles:

- `gazebo_enu_common_world`: the validated position, yaw, and static-origin
  conversion above.
- `raw_px4_local`: identity position/yaw conversion with no Gazebo origin.

The profile API must convert complete position/yaw data in both directions,
validate finite inputs/profile names, wrap yaw, and remain ROS/Gazebo-free.
Test round trips, all three origins, yaw wrap edges, the `0.102` offset, and
unknown-profile rejection.

### 2. Put all raw/common conversion at the PX4 interface and select it per launch

Change:

- `px4_swarm_control/px4_vehicle_interface.py`
- `px4_swarm_control/vehicle_node.py`
- `config/three_vehicle_nodes.yaml`
- `launch/swarm_nodes.launch.py`
- `launch/real_mav1_vehicle.launch.py`
- `launch/real_mav2_vehicle.launch.py`
- `launch/real_mav3_vehicle.launch.py`
- `launch/real_ground_station.launch.py`
- `test/test_px4_vehicle_interface.py`
- `test/test_vehicle_node.py`
- `test/test_swarm_launch.py`
- configuration/package-scaffold tests that assert launch parameters

`Px4VehicleInterface` receives one immutable, per-vehicle profile instance.
It converts raw telemetry before constructing internal `VehicleState`, and
converts canonical `PositionYawSetpoint` immediately before publishing
`TrajectorySetpoint`. `publish_safe_hover_setpoint` stores canonical data and
therefore reuses the same outbound conversion.

`vehicle_node.py` reads profile/origin parameters once and injects the profile.
`swarm_nodes.launch.py` explicitly selects `gazebo_enu_common_world`; real
launches explicitly retain `raw_px4_local` and cannot consume Gazebo origins.
Do not infer the profile from namespace, environment, or `PX4_GZ_MODEL_POSE`.

Tests must prove raw telemetry becomes common status, common targets become the
correct MAV-specific raw trajectory target, staging/pause/takeoff holds do not
double-transform, and real launch parameters do not receive SITL offsets.

### 3. Migrate operator, formation, follower, and safety behaviour to canonical ENU

Change:

- `px4_swarm_control/operator_console.py`
- `px4_swarm_control/geometry.py`
- `px4_swarm_control/collision_safety_gate.py`
- `px4_swarm_control/follower_controller.py` only if a test proves a stale raw
  assumption
- `px4_swarm_control/ground_station_node.py` only for stale frame labels or a
  demonstrated raw-frame assumption
- `test/test_operator_console.py`
- `test/test_models_geometry.py`
- `test/test_follower_controller.py`
- `test/test_collision_safety_gate.py`
- `test/test_ground_station_node.py`

Operator `+/-X`, `+/-Y`, up/down, and yaw commands must create common ENU
action goals; help text must state the Gazebo directions, not raw NED axes.
`geometry.py` owns the single physical-left primitive used by VEE and
LINE_ABREAST. Collision fallback calls the same primitive, so it moves outward
at yaw `0`, `+pi/2`, `pi`, and `-pi/2` rather than retaining its own
`sin/cos`.

Test VEE/LINE_ABREAST left/right and collision fallback at all four headings,
with MAV2/MAV3 nonzero origins. `ground_station_node.py` and
`follower_controller.py` must remain profile-unaware canonical-frame consumers.

### 4. Make the 0.5 m operator path and runtime cleanup unambiguous

Change:

- `px4_swarm_control/operation_profile.py`
- `config/operator_console.yaml`
- `launch/operator_console.launch.py`
- `config/FINAL_MANUAL.zh.md`
- `launch/README.md`
- associated operator-console and launch tests

The currently documented `ros2 run ... operator_console` path bypasses
`operator_console.yaml` and falls back to `TAKEOFF_ALTITUDE_M = 1.5`. Make all
supported/documented console start paths use a tested 0.5 m default. Update the
manual to export `ROS_DOMAIN_ID=42`, start the configured console, monitor the
PX4 raw topics plus common status, and verify cleanup of launch descendants
(`vehicle_node`, `ground_station_node`) rather than only the `ros2 launch`
parent.

This ticket must not add a normal direct-PX4 command path; the direct diagnostic
used to validate yaw was a one-off simulation procedure only.

### 5. Remove obsolete duplicate consoles/probes and publish the frame architecture

Change:

- remove `px4_swarm_control/field_frame_console.py`
- remove `test/test_field_frame_console.py`
- remove `px4_swarm_control/coordinate_frame_probe.py`
- remove `test/test_coordinate_frame_probe.py`
- update `test/test_package_scaffold.py` and every remaining import/reference
- add `docs/coordinate-frame-architecture.md`
- update `CONTEXT.md`, `config/FINAL_MANUAL.zh.md`, and `launch/README.md`

`field_frame_console.py` and `coordinate_frame_probe.py` are non-installed,
duplicate/outdated surfaces. Migrate any still-useful command coverage to
`test_operator_console.py`; retain manual diagnostics as documented topic
observations, not executable command surfaces. Verify setup metadata, launch
files, imports, tests, and documentation contain no remaining reference.

The architecture document must distinguish the SITL ENU profile from the real
raw-PX4 profile, show the interface seam and data flow, and name the ownership
of each conversion. `CONTEXT.md` only receives glossary terms and a link.

## Required end-to-end acceptance

- [ ] Focused tests, package tests, and full test suite pass; code review is
  completed before merge.
- [ ] Normal SITL startup uses the documented 0.5 m operator path and discovers
  actions in ROS domain 42.
- [ ] In Gazebo, `operator_console` `+/-X`, `+/-Y`, up/down visibly follow ENU
  and produce the validated PX4 raw axis/sign on trajectory and telemetry.
- [ ] Common yaw agrees with PX4 raw heading through the validated profile;
  MAV1/MAV2/MAV3 each pass a post-change yaw observation.
- [ ] VEE and LINE_ABREAST preserve physical left/right as leader yaw changes,
  despite distinct MAV spawn origins.
- [ ] Collision fallback visibly moves outward and never toward the peer.
- [ ] `/MAVx/status`, `/swarm/leader_goal`, staging, follower, and safety data
  are common ENU; raw PX4 coordinates exist only under `/MAVx/fmu/*`.
- [ ] Real launch behaviour and real coordinate semantics remain unchanged.
- [ ] Only `operator_console` remains as a user-facing ROS command surface.

## Non-goals

- Changing the real-hardware coordinate contract.
- Runtime Gazebo origin queries, automatic calibration, or per-MAV yaw trims.
- Adding interactive `home`/`home_yaw` behaviour.
- A normal direct-PX4 control interface.

## Implementation notes

- Do not start source changes until the current branch is confirmed `main` and
  clean except for this ticket.
- Implement and test each ordered step before moving to the next; retain a
  green suite at every step where practical.
- The old baseline staging timeout is expected to disappear only after Steps 1–3
  make all control layers use the common ENU frame.

## Implementation record

Implemented on `main` with TDD coverage for profile math, MAV-specific origin
conversion, interface telemetry/setpoints, configuration parsing, launch
overrides, canonical formation geometry, and staging direction. The package
build and full test suite pass (`193 passed`).

The remaining `ready-for-human` work is the post-change Gazebo acceptance in
the checklist above: visually verify all operator directions, yaw for all
three MAVs, formations across headings, and live collision fallback. No SITL
or PX4 process was left running by this implementation pass.
