# px4_swarm_control config

## Ticket 05 manual namespace and telemetry debug flow

PX4 SITL and Micro XRCE-DDS Agent are external prerequisites for this ticket. Start
them before expecting `px4_msgs` telemetry topics.

Terminal 1:

```bash
cd /home/ncrl/docker_ubuntu24
MicroXRCEAgent udp4 -p 8888
```

Terminal 2:

```bash
cd /home/ncrl/docker_ubuntu24/PX4-Autopilot
make px4_sitl gz_x500
```

For the three-vehicle ticket 05 check, use the PX4 multi-vehicle SITL setup that
publishes vehicle-specific telemetry under the configured prefixes:

```text
/vehicle_1/fmu/out/...
/vehicle_2/fmu/out/...
/vehicle_3/fmu/out/...
```

If the current PX4 multi-vehicle command publishes different prefixes, update the
`px4_namespace` parameters before running the three `vehicle_node` instances.

Terminal 3:

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run px4_swarm_control vehicle_node --ros-args \
  -r __ns:=/vehicle_1 \
  -p role:=leader \
  -p vehicle_id:=vehicle_1 \
  -p px4_namespace:=/vehicle_1 \
  -p slot:=leader
```

Terminal 4:

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run px4_swarm_control vehicle_node --ros-args \
  -r __ns:=/vehicle_2 \
  -p role:=follower \
  -p vehicle_id:=vehicle_2 \
  -p px4_namespace:=/vehicle_2 \
  -p slot:=follower_left
```

Terminal 5:

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run px4_swarm_control vehicle_node --ros-args \
  -r __ns:=/vehicle_3 \
  -p role:=follower \
  -p vehicle_id:=vehicle_3 \
  -p px4_namespace:=/vehicle_3 \
  -p slot:=follower_right
```

Terminal 6:

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic list | grep -E '/vehicle_[123]/(status|fmu/out)'
ros2 topic echo --once /vehicle_1/status
ros2 topic echo --once /vehicle_2/status
ros2 topic echo --once /vehicle_3/status
```
