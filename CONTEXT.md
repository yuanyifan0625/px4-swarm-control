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
- **PX4 local NED frame**: The local coordinate frame exposed by PX4 telemetry
  over DDS, where x/y are local horizontal axes and z is positive downward.
- **Gazebo world frame**: The simulator visual/world coordinate frame, which is
  not the same axis convention as the PX4 local NED frame.
- **Field frame**: The human-defined test-field coordinate frame used by the
  operator to describe field +X, field +Y, and field up.
- **Axis probe**: A preflight diagnostic that observes manual or commanded
  motion and verifies which PX4 local NED axis and sign changed.
- **Field-frame console**: An operator console that accepts human field-frame
  movement commands and translates them into the existing swarm action goals.
- **Offboard-setpoint acceptance**: PX4's explicit indication that the current
  flight mode accepts Offboard trajectory setpoints. It is not eligibility to
  enter Offboard mode and not the result of pre-flight checks.
