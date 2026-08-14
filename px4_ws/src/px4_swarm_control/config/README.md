# px4_swarm_control config

Final manual verification guides:

- `final_sitl_manual_smoke.zh.md`: SITL/manual ROS 2 smoke flow from inside the container.
- `final_real_vehicle_ros2_manual.zh.md`: real-vehicle ROS 2 deployment and operation guide.

## Ticket 05b live PX4 Gz bridge smoke flow

PX4 SITL, Gazebo, and Micro XRCE-DDS Agent are external prerequisites for the
first-version swarm workflow. The ROS 2 launch files do not start or stop them
yet, so keep this flow manual while validating the bridge.

`make px4_sitl gz_x500` starts only one `gz_x500`. To see three aircraft in
Gazebo, build PX4 SITL once, then run three PX4 instances with unique instance
IDs, separated spawn positions, and explicit DDS namespaces.

All commands below are intended to run inside the `ros2_jazzy` container.

### Terminal 1: Micro XRCE-DDS Agent

```bash
cd /home/ncrl/docker_ubuntu24
MicroXRCEAgent udp4 -p 8888 | tee /tmp/microxrceagent_8888.log
```

Expected result: the agent stays running and later logs one session per PX4
client. One agent on port `8888` can serve all three PX4 instances.

### Terminal 2: build PX4 SITL once

```bash
cd /home/ncrl/docker_ubuntu24/PX4-Autopilot
make px4_sitl
```

Expected result: `build/px4_sitl_default/bin/px4` exists.

### Terminal 3: MAV1 starts Gazebo

```bash
cd /home/ncrl/docker_ubuntu24/PX4-Autopilot
PX4_UXRCE_DDS_NS=MAV1 \
PX4_SYS_AUTOSTART=4001 \
PX4_SIM_MODEL=gz_x500 \
./build/px4_sitl_default/bin/px4 -i 1
```

Expected result: Gazebo starts and model `x500_1` is spawned. The explicit
`PX4_UXRCE_DDS_NS=MAV1` keeps ROS 2 topics under `/MAV1`.

### Terminal 4: MAV2 joins the same Gazebo world

```bash
cd /home/ncrl/docker_ubuntu24/PX4-Autopilot
PX4_UXRCE_DDS_NS=MAV2 \
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="0,2,0" \
PX4_SIM_MODEL=gz_x500 \
./build/px4_sitl_default/bin/px4 -i 2
```

Expected result: model `x500_2` joins the existing Gazebo world at a separated
horizontal position.

### Terminal 5: MAV3 joins the same Gazebo world

```bash
cd /home/ncrl/docker_ubuntu24/PX4-Autopilot
PX4_UXRCE_DDS_NS=MAV3 \
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="0,-2,0" \
PX4_SIM_MODEL=gz_x500 \
./build/px4_sitl_default/bin/px4 -i 3
```

Expected result: model `x500_3` joins the existing Gazebo world at a separated
horizontal position.

### Terminal 6: inspect live bridge truth

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
gz topic -l | grep -E '/model/x500_[123]'
gz topic -e -t /world/default/pose/info -n 1
ros2 topic info -v /MAV1/fmu/out/vehicle_local_position_v1
ros2 topic info -v /MAV2/fmu/out/vehicle_local_position_v1
ros2 topic info -v /MAV3/fmu/out/vehicle_local_position_v1
ros2 topic info -v /MAV1/fmu/out/vehicle_status_v4
ros2 topic info -v /MAV2/fmu/out/vehicle_status_v4
ros2 topic info -v /MAV3/fmu/out/vehicle_status_v4
ros2 topic info -v /MAV1/fmu/out/vehicle_command_ack_v1
ros2 topic info -v /MAV2/fmu/out/vehicle_command_ack_v1
ros2 topic info -v /MAV3/fmu/out/vehicle_command_ack_v1
```

Expected result: each ROS 2 topic inspection shows `Publisher count: 1`. A topic
that only has subscribers from `vehicle_node` is not evidence of live PX4
telemetry. The Gazebo pose output should include `x500_1`, `x500_2`, and
`x500_3` with separated horizontal positions.

After rebuilding the package, the same checks can be summarized by:

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run px4_swarm_control check_live_px4_gz_bridge
```

The command exits `0` only when all three Gazebo models and all required PX4
v1.18 versioned telemetry publishers are present.

If Terminal 1 wrote `/tmp/microxrceagent_8888.log`, also verify that one Agent
accepted three PX4 client sessions:

```bash
ros2 run px4_swarm_control check_live_px4_gz_bridge \
  --agent-log /tmp/microxrceagent_8888.log
```

## Namespace and target-system convention

The project namespace convention remains `/MAV1`, `/MAV2`, and
`/MAV3`. PX4's default multi-vehicle namespaces may be `/px4_1`,
`/px4_2`, and `/px4_3`, so this workflow uses `PX4_UXRCE_DDS_NS=MAVN` to
make PX4 publish under the project namespaces.

When PX4 is launched with `-i 1`, `-i 2`, and `-i 3`, first-version command
targets are configured as:

- `MAV1`: `px4_target_system=2`
- `MAV2`: `px4_target_system=3`
- `MAV3`: `px4_target_system=4`

This keeps future takeoff/land commands addressed to the intended PX4 instance.
