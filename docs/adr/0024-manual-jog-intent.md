# ADR 0024: Represent keyboard jog as an expiring GroundStation intent

## Status

Accepted

The operator console remains the only control entrypoint and retains its
discrete swarm actions.  Its keyboard jog mode publishes an expiring
world-frame manual jog intent to GroundStation rather than repeatedly invoking
the completion-oriented `MoveLeader` action.  GroundStation remains the owner
of motion-state validation, formation ownership, and collision safety.

At most one cardinal direction is live; the latest directional input replaces
the prior direction. A rejected incremental target holds the last target that
passed collision safety and requires a release before another jog is accepted.

Two deadman intervals make a held-key control safe in a terminal environment:
the console infers key release after 150 ms without a directional repeat, and
GroundStation holds after 250 ms without an intent heartbeat.  Keyboard jog is
available only after successful takeoff, accepts single-axis arrows only, and
leaves the mode after Escape, pause, land, or formation change.  No geofence
is added in the first version; existing collision safety remains in force.

## Considered Options

- Map arrows to the existing discrete `MoveLeader` action. Rejected because it
  waits for arrival and cannot express release-to-hold safely.
- Add a second direct PX4 or follower-control entrypoint. Rejected because it
  bypasses the existing GroundStation safety and formation-ownership boundary.
