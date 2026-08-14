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
