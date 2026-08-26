# px4_swarm_control launch

`swarm_nodes.launch.py` starts only the first-version ROS 2 swarm nodes:

- `/MAV1/vehicle_node`
- `/MAV2/vehicle_node`
- `/MAV3/vehicle_node`
- `/swarm/ground_station_node`

PX4 SITL, Gazebo, Micro XRCE-DDS Agent, QGC, `operator_console`, and PX4 speed
profile commands are external workflow steps, not launch-file children.

Use `config/final_operator_console_sitl_real_manual.zh.md` for their exact
startup order and the replayable two-cycle demo. `operator_console` is the
only installed manual control entrypoint.

Real-deployment launch files start one ROS 2 node each:

- `real_mav1_vehicle.launch.py`
- `real_mav2_vehicle.launch.py`
- `real_mav3_vehicle.launch.py`
- `real_ground_station.launch.py`

`swarm_nodes.launch.py` and `real_ground_station.launch.py` accept
`formation_position_tolerance_m:=...` for field tuning. `operator_console.launch.py`
starts only `operator_console` and accepts `settle_position_tolerance_m:=...` plus
`settle_stable_duration_s:=...`; the shortest console entrypoint remains
`ros2 run px4_swarm_control operator_console`.

Before accepting takeoff, the ground station requires a fresh MAV1 position
and yaw. That pose anchors VEE staging, so a second takeoff after land does not
return the swarm to a fixed `(0, 0)` origin.
