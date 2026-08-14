# px4_swarm_control launch

`swarm_nodes.launch.py` starts only the first-version ROS 2 swarm nodes:

- `/MAV1/vehicle_node`
- `/MAV2/vehicle_node`
- `/MAV3/vehicle_node`
- `/swarm/ground_station_node`

PX4 SITL, Gazebo, Micro XRCE-DDS Agent, QGC, `operator_console`, and PX4 speed
profile commands are external workflow steps, not launch-file children.

Real-deployment launch files start one ROS 2 node each:

- `real_mav1_vehicle.launch.py`
- `real_mav2_vehicle.launch.py`
- `real_mav3_vehicle.launch.py`
- `real_ground_station.launch.py`
