# px4_swarm_control config

The single manual workflow is
`final_operator_console_sitl_real_manual.zh.md`. It covers container SITL,
distributed real-vehicle launch, PX4 speed checks, raw NED/status evidence,
the complete operator-console flow, and a second takeoff/land cycle without
restarting Gazebo, PX4, DDS, or swarm nodes.

`operator_console` is the only installed manual swarm-control entrypoint.
Field commands are converted to canonical PX4 local NED at that boundary;
ground station, staging, formation, followers, and the PX4 adapter remain NED.

## Runtime configuration

- `three_vehicle_nodes.yaml`: vehicle identities, formation geometry, safety
  distance/debounce, telemetry freshness, and ground-station tolerances.
- `operator_console.yaml`: takeoff altitude, field jog steps, settle criteria,
  and demo commands.
- `px4_speed_profiles/slow_demo.yaml`: SITL speed limits.
- `px4_speed_profiles/real_cautious.yaml`: cautious real-vehicle speed limits.

ROS 2 does not rate-limit normal position setpoints. Check PX4's position
controller before every demo:

```bash
ros2 run px4_swarm_control px4_speed_profile check --profile slow_demo
```

The agreed limits are `MPC_XY_VEL_MAX=0.3 m/s`,
`MPC_Z_VEL_MAX_UP=0.3 m/s`, and `MPC_YAWRAUTO_MAX=30 deg/s`. `apply` only
prints commands and requires explicit confirmation:

```bash
ros2 run px4_swarm_control px4_speed_profile apply --profile slow_demo --yes
```

For exact DDS, PX4/Gazebo, ROS 2 launch, validation, and replay commands, use
the unified manual instead of copying commands from this README.
