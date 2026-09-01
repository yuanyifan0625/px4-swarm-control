# Coordinate-frame architecture

## One seam, two profiles

All swarm-control code operates on its launch-selected canonical control
frame. Only `Px4VehicleInterface` may convert between that frame and PX4 DDS
raw local coordinates.

```text
PX4 VehicleLocalPosition (raw) -> Px4VehicleInterface -> VehicleState/status
operator_console / ground station / follower / safety -> interface -> PX4 TrajectorySetpoint (raw)
```

`VehicleCommand` contains no coordinates, so it crosses the seam unchanged.
Formation geometry and collision fallback receive canonical position/yaw only;
they contain no PX4, Gazebo, origin, axis-swap, or yaw-calibration branch.

## `gazebo_enu_common_world` (SITL)

`swarm_nodes.launch.py` explicitly selects this profile. The canonical frame
is Gazebo ENU: `+E` East, `+N` North, `+U` Up; yaw `0` faces East and positive
yaw turns toward North. Its configured static origins are:

| MAV | common origin `(E, N, U)` |
| --- | --- |
| MAV1 | `(0, 0, 0)` |
| MAV2 | `(-1, +1, 0)` |
| MAV3 | `(-1, -1, 0)` |

For raw PX4 local pose `(x, y, z, heading)` and origin `(E0, N0, U0)`:

```text
common position = (E0 + y, N0 + x, U0 - z)
common yaw      = wrap(pi/2 - heading + 0.102)
```

The interface applies the exact inverse before publishing a trajectory
setpoint. `0.102 rad` is one shared SITL calibration, never a per-MAV trim.
The SITL ground-station launch also selects `vertical_axis_up=true`, so a
positive takeoff altitude moves upward in this canonical frame.

The shared ENU body primitive is `forward=(cos(yaw), sin(yaw))` and
`left=(-sin(yaw), cos(yaw))`; a positive lateral offset is always physical
left. VEE, LINE_ABREAST, and collision fallback call this one primitive.

## `raw_px4_local` (real deployment)

Each `real_mav*_vehicle.launch.py` explicitly overrides the profile to
`raw_px4_local`, zeros all common-origin parameters, and keeps the existing
real setpoint semantics. `real_ground_station.launch.py` retains the prior
downward-positive staging altitude behavior (`vertical_axis_up=false`). Thus
Gazebo spawn offsets and ENU-only vertical behavior cannot leak into the real
launch path.

## Configuration ownership

- `config/three_vehicle_nodes.yaml` owns SITL MAV origins and the SITL hold
  altitude (0.5 m in common ENU).
- `frame_transform.py` owns pure profile math and yaw wrapping.
- `vehicle_node.py` parses profile/origin parameters once and injects an
  immutable profile into its interface.
- `operator_console.py` is the only user-facing ROS 2 command entry point.
- `config/FINAL_MANUAL.zh.md` owns the human startup, monitoring, and cleanup
  procedure.
