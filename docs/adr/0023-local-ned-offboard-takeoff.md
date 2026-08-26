# ADR 0023: Use local-NED Offboard position control for takeoff

## Status

Accepted

## Decision

The swarm vehicle node starts every takeoff from fresh PX4 local-NED position
telemetry. It warms up a local vertical `TrajectorySetpoint`, requests
Offboard, waits for PX4 telemetry to report Offboard, then requests ARM. After
the local-NED height gate, it switches to the full staging target.

`VEHICLE_CMD_NAV_TAKEOFF` is not used. PX4 v1.17 interprets its altitude as
AMSL, which is unavailable on local-only real vehicles. Keeping one local-NED
path therefore gives SITL and real vehicles the same ROS 2 control behavior.

Landing continues to use PX4 `VEHICLE_CMD_NAV_LAND`: PX4 retains ownership of
the landing-mode and landing-detection safety behavior.

## Consequences

Each vehicle uses its own local origin; this decision does not align local
origins or assert that raw coordinates are shared between vehicles. A takeoff
timeout sends PAUSE, which holds the latest local pose or last safe setpoint
and requires a new staging anchor for another attempt.

## TDD record

The PX4-boundary test was first made to fail for the missing
`local_position_ready` contract, then implemented. Vehicle-node tests then
specified the warmup, Offboard-before-ARM, local vertical target, height gate,
and PAUSE/RESUME behavior. Focused tests and a package build passed; the full
package run also exposed pre-existing unrelated formation/collision regressions
outside this ticket's permitted scope.
