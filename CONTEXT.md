# Context

## Glossary

- **World-frame jog**: An operator command that moves the leader along the world
  x/y/z axes, independent of the leader yaw.
- **Body-frame jog**: An operator command that moves relative to the leader
  heading, such as forward/back/left/right from the aircraft's current yaw.
- **Formation completion tolerance**: The maximum allowed position error for a
  vehicle to count as established in a requested formation.
- **Settle stable duration**: The continuous time a formation must remain inside
  tolerance before the operator workflow treats it as settled.
- **Launch-time override**: A ROS 2 launch argument that changes a runtime
  parameter for one launch invocation without editing the shared YAML defaults.
- **PX4 raw local frame**: The per-vehicle coordinate frame exposed by PX4
  telemetry over DDS. It is only consumed and produced by the selected
  coordinate profile at `Px4VehicleInterface`.
- **Gazebo world frame**: The simulator visual/world coordinate frame, which is
  not the same axis convention as the PX4 local NED frame.
- **Canonical control frame**: The profile-selected frame used by the swarm
  controller for positions, formation geometry, staging, and vehicle
  setpoints. See `docs/coordinate-frame-architecture.md`.
- **Coordinate profile**: The explicit mapping between a PX4 raw local frame
  and the canonical control frame.
- **Staging anchor**: The leader position and yaw used as the reference for
  placing the vehicles into their initial VEE arrangement before a takeoff
  sequence is considered staged.
- **Collision safety hold**: A conservative state that republishes the last
  position target that passed the separation check instead of accepting a new
  formation target while the safety condition is unresolved.
- **Formation slot offset**: A follower's relative position from the leader,
  expressed in the leader body frame and rotated into the canonical control
  frame using the leader yaw.
- **Offboard-setpoint acceptance**: PX4's explicit indication that the current
  flight mode accepts Offboard trajectory setpoints. It is not eligibility to
  enter Offboard mode and not the result of pre-flight checks.
- **Local-position readiness**: A fresh PX4 local-NED pose with valid xy/z,
  finite x/y/z/heading, and no dead reckoning. It is the takeoff-targeting
  contract and does not require a global altitude reference.
- **SITL handoff**: The documented, repeatable path by which a new user obtains
  the required simulator sources, builds the declared environment, and observes
  the three-vehicle demonstration without treating it as a real-flight claim.
- **Control entrypoint**: The supported operator surface that initiates swarm
  actions. For this system it is the ROS 2 operator console; a ground-control
  application may observe telemetry but is not a control entrypoint.
- **GCS heartbeat observer**: A ground-control application that remains
  connected to every simulated vehicle so PX4's communication-link health
  check passes. It observes the swarm but does not initiate swarm actions.
- **Keyboard jog mode**: A modal operator-console surface in which directional
  key input expresses a continuous world-frame planar motion intent for the
  leader and its formation.
- **Command mode**: The default operator-console surface for the existing
  discrete swarm commands, including the one-step world-frame jog commands.
- **Deadman interval**: The short interval after the last directional-input
  repeat during which a keyboard jog remains live; expiry means the operator
  has released the control and the swarm must hold.
- **Manual jog intent**: A short-lived world-frame planar motion request from
  keyboard jog mode, which expires unless the operator console renews it.
- **Following-ready leader status**: A fresh MAV1 status that reports armed,
  Offboard, and `following`; it permits a staged follower to begin calculating
  its formation slot offset.
