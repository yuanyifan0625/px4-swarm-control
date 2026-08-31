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
- **Measured PX4 local frame**: The real-vehicle coordinate frame exposed by
  PX4 telemetry over DDS: x is East, y is South, and z is Down. MAV1, MAV2,
  and MAV3 share one origin and this same axis convention.
- **Gazebo world frame**: The simulator visual/world coordinate frame, which is
  not the same axis convention as the PX4 local NED frame.
- **Field frame**: The human-defined test-field coordinate frame used by the
  operator to describe field +X, field +Y, and field up. It is fixed to the
  field and does not rotate when the leader changes yaw.
- **Canonical control frame**: The measured PX4 local frame used by the swarm
  controller for positions, formation geometry, staging, and vehicle
  setpoints. Operator-facing field commands are converted before entering
  this frame.
- **Staging anchor**: The leader position and yaw used as the reference for
  placing the vehicles into their initial VEE arrangement before a takeoff
  sequence is considered staged.
- **Collision safety hold**: A conservative state that republishes the last
  position target that passed the separation check instead of accepting a new
  formation target while the safety condition is unresolved.
- **Fixed field-to-PX4 mapping**: The stable signed axis mapping from the
  operator's field frame into the measured PX4 local frame: North maps to -y,
  West maps to -x, and up maps to -z. It is not recalculated from leader yaw.
- **Formation slot offset**: A follower's relative position from the leader,
  expressed in the leader body frame and rotated into the canonical control
  frame using the leader yaw.
- **Axis probe**: A preflight diagnostic that observes manual or commanded
  motion and verifies which PX4 local NED axis and sign changed.
- **Offboard-setpoint acceptance**: PX4's explicit indication that the current
  flight mode accepts Offboard trajectory setpoints. It is not eligibility to
  enter Offboard mode and not the result of pre-flight checks.
- **Local-position readiness**: A fresh PX4 local-NED pose with valid xy/z,
  finite x/y/z/heading, and no dead reckoning. It is the takeoff-targeting
  contract and does not require a global altitude reference.
