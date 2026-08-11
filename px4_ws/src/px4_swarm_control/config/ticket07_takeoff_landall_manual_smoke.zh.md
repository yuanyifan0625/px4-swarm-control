# Ticket 07：TakeoffSwarm 到 staging 與 LandSwarm 手動驗證

目標：在第一版 SITL 中，不開啟 QGC，也能透過 ROS 2 `TakeoffSwarm` 讓三台 `gz_x500` arm/takeoff 到分開的 staging positions，最後用 `LandSwarm` 讓三台降落。

所有指令都假設你已經進入 container，且 workspace 位於：

```bash
/home/ncrl/docker_ubuntu24
```

## 前置條件

- QGC 關閉。
- 三機 PX4 Gz bridge 已照 `live_px4_gz_bridge_smoke.zh.md` 驗證通過。
- `MicroXRCEAgent udp4 -p 8888` 正在執行。
- Gazebo 中可以看到 `x500_1`、`x500_2`、`x500_3`。
- ROS 2 可看到 `/vehicle_1..3/fmu/out/*` 的 PX4 telemetry publishers。

## Terminal 1：確認 bridge 與 QGC-free 前置狀態

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run px4_swarm_control check_live_px4_gz_bridge --agent-log /tmp/microxrceagent_8888.log
```

通過條件：

- `Gazebo models OK: x500_1, x500_2, x500_3`
- `Gazebo model pose separation OK`
- `ROS 2 PX4 publishers OK for all vehicle telemetry topics`
- `Micro XRCE-DDS Agent sessions OK`
- command exit code 為 `0`

## Terminal 2：啟動 vehicle_1

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run px4_swarm_control vehicle_node --ros-args \
  -r __ns:=/vehicle_1 \
  -p role:=leader \
  -p vehicle_id:=vehicle_1 \
  -p px4_namespace:=/vehicle_1 \
  -p px4_target_system:=2 \
  -p slot:=leader
```

## Terminal 3：啟動 vehicle_2

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run px4_swarm_control vehicle_node --ros-args \
  -r __ns:=/vehicle_2 \
  -p role:=follower \
  -p vehicle_id:=vehicle_2 \
  -p px4_namespace:=/vehicle_2 \
  -p px4_target_system:=3 \
  -p slot:=follower_left
```

## Terminal 4：啟動 vehicle_3

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run px4_swarm_control vehicle_node --ros-args \
  -r __ns:=/vehicle_3 \
  -p role:=follower \
  -p vehicle_id:=vehicle_3 \
  -p px4_namespace:=/vehicle_3 \
  -p px4_target_system:=4 \
  -p slot:=follower_right
```

## Terminal 5：啟動 ground station

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run px4_swarm_control ground_station_node
```

通過條件：ground station 沒有 crash，且可以另開 terminal 看到 `/swarm` actions/topics。

## Terminal 6：確認 actions/topics

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 action list | grep /swarm
ros2 topic list | grep -E '/swarm|/vehicle_[123]/staging_setpoint|/vehicle_[123]/status'
```

通過條件：

- actions 包含 `/swarm/takeoff` 和 `/swarm/land`
- topics 包含 `/vehicle_1/staging_setpoint`、`/vehicle_2/staging_setpoint`、`/vehicle_3/staging_setpoint`
- topics 包含 `/vehicle_1/status`、`/vehicle_2/status`、`/vehicle_3/status`

## Terminal 6：送出 TakeoffSwarm

```bash
ros2 action send_goal /swarm/takeoff px4_swarm_interfaces/action/TakeoffSwarm \
  "{altitude_m: 5.0, timeout_sec: 60.0}" --feedback
```

通過條件：

- action goal accepted
- feedback 顯示 `current_state: taking_off`
- result 顯示 `success: true`
- vehicle node terminal 會輸出中文說明：

```text
不依賴 QGC：起飛前先發布 Offboard heartbeat/setpoint，再切換 Offboard、arm 並送出 takeoff command。
```

- Gazebo 中三台飛機起飛，並往分開的 staging positions 移動：
  - `vehicle_1` staging target 約為 `(0.0, 0.0, -5.0)`
  - `vehicle_2` staging target 約為 `(-3.0, 4.0, -5.0)`
  - `vehicle_3` staging target 約為 `(-3.0, -4.0, -5.0)`
- ground station terminal 最後會輸出：

```text
all vehicles reached staging positions
```

## Terminal 6：觀察 status

```bash
ros2 topic echo --once /vehicle_1/status
ros2 topic echo --once /vehicle_2/status
ros2 topic echo --once /vehicle_3/status
```

通過條件：

- 三台 status 都有有限的 `x/y/z/yaw`，不是 `nan`
- `armed: true`
- `vehicle_state` 至少進入 `taking_off`、`staging` 或 `holding`

## Terminal 6：送出 LandSwarm

```bash
ros2 action send_goal /swarm/land px4_swarm_interfaces/action/LandSwarm \
  "{timeout_sec: 60.0}" --feedback
```

通過條件：

- action goal accepted
- feedback 顯示 `current_state: landing`
- result 顯示 `success: true`
- Gazebo 中三台飛機開始降落
- 三台 vehicle status 最後進入 landing/landed 相關狀態

## 如果未開 QGC 起飛失敗

不要先開 QGC 當 workaround。請保留各 terminal 輸出，改用 `$diagnosing-bugs`，至少蒐集：

- `VehicleCommandAck`
- `/vehicle_1/status`、`/vehicle_2/status`、`/vehicle_3/status`
- PX4 terminal 中 commander / preflight / arming 相關訊息
- Micro XRCE-DDS Agent log
- `ros2 topic info -v` 對 PX4 input/output topics 的 publisher/subscriber 狀態
