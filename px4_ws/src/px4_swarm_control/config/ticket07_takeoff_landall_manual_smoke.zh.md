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

## Terminal 6：送出第一次 TakeoffSwarm

```bash
ros2 action send_goal /swarm/takeoff px4_swarm_interfaces/action/TakeoffSwarm \
  "{altitude_m: 5.0, timeout_sec: 60.0}" --feedback
```

通過條件：

- action goal accepted
- feedback 顯示 `current_state: taking_off`
- action 不會在命令剛送出時立刻 success；它會等三台都抵達 staging，或 timeout 後回報 failure
- result 顯示 `success: true` 時，代表三台已經真的抵達 staging
- vehicle node terminal 會輸出中文說明：

```text
不依賴 QGC：先用 PX4 NAV_TAKEOFF 到安全高度，再 warm up Offboard 後切換 staging control。
```

- Gazebo 中三台飛機先垂直起飛到安全高度，接著才切入 Offboard 並往分開的 staging positions 移動：
  - `vehicle_1` staging target 約為 `(0.0, 0.0, -5.0)`
  - `vehicle_2` staging target 約為 `(-3.0, 4.0, -5.0)`
  - `vehicle_3` staging target 約為 `(-3.0, -4.0, -5.0)`
- ground station terminal 最後會輸出：

```text
all vehicles reached staging positions
```

## Terminal 6：觀察 takeoff、Offboard、staging status

```bash
ros2 topic echo --once /vehicle_1/status
ros2 topic echo --once /vehicle_2/status
ros2 topic echo --once /vehicle_3/status
```

通過條件：

- 三台 status 都有有限的 `x/y/z/yaw`，不是 `nan`
- `armed: true`
- 起飛初期三台先到安全高度，例如 `altitude_m: 5.0` 時 `z < -4.0`
- staging 完成前，三台都要看到 `nav_state: offboard` 或 `offboard_available: true`
- staging 完成後，三台 `vehicle_state` 會進入 `staging`
- staging 完成後位置應接近：
  - `vehicle_1`: `x ~= 0.0`, `y ~= 0.0`, `z ~= -5.0`
  - `vehicle_2`: `x ~= -3.0`, `y ~= 4.0`, `z ~= -5.0`
  - `vehicle_3`: `x ~= -3.0`, `y ~= -4.0`, `z ~= -5.0`
- ground station 只有在三台都有 telemetry、armed、Offboard ready、且位置到 tolerance 內時，才會輸出 `all vehicles reached staging positions`

## Terminal 6：送出 LandSwarm

```bash
ros2 action send_goal /swarm/land px4_swarm_interfaces/action/LandSwarm \
  "{timeout_sec: 60.0}" --feedback
```

通過條件：

- action goal accepted
- feedback 顯示 `current_state: landing`
- action 不會在 land command 剛送出時立刻 success；它會等三台都回報 landed，或 timeout 後回報 failure
- result 顯示 `success: true` 時，代表三台都已經透過 PX4 landed telemetry 確認 landed
- Gazebo 中三台飛機開始降落
- 送出 land 後，三台 `/vehicle_N/status` 的 `vehicle_state` 應保持 `landing`，不能被一般 control tick 改回 `holding`
- 三台最後必須透過 PX4 landed telemetry 進入 `vehicle_state: landed`
- ground station terminal 最後會輸出類似：

```text
swarm mission landing -> done: all vehicles reported landed
```

可用下列指令重複觀察：

```bash
ros2 topic echo --once /vehicle_1/status
ros2 topic echo --once /vehicle_2/status
ros2 topic echo --once /vehicle_3/status
```

如果想確認 land 後 vehicle node 沒有繼續發空中 staging setpoint，可以在送出 LandSwarm 後另開 terminal 觀察：

```bash
ros2 topic hz /vehicle_1/fmu/in/trajectory_setpoint --window 10
ros2 topic hz /vehicle_2/fmu/in/trajectory_setpoint --window 10
ros2 topic hz /vehicle_3/fmu/in/trajectory_setpoint --window 10
```

通過條件：進入 `landing` 後不應再看到持續的 20 Hz staging setpoint；三台最後在 Gazebo 地面穩定停住並回報 `landed`。

## Terminal 6：不重開 runtime，送出第二次 TakeoffSwarm

第一輪 `LandSwarm` 成功後，不要重開 Micro XRCE-DDS Agent、PX4 SITL、Gazebo、vehicle nodes 或 ground station，直接送第二次 takeoff：

```bash
ros2 action send_goal /swarm/takeoff px4_swarm_interfaces/action/TakeoffSwarm \
  "{altitude_m: 5.0, timeout_sec: 60.0}" --feedback
```

通過條件：

- 不需要送第二次相同 takeoff action
- 三台飛機再次 arm/takeoff，並進入 Offboard staging control
- `/vehicle_1/status`、`/vehicle_2/status`、`/vehicle_3/status` 在飛機已離地時不會顯示 `vehicle_state: landed`
- 三台再次抵達 staging：
  - `vehicle_1`: `x ~= 0.0`, `y ~= 0.0`, `z ~= -5.0`
  - `vehicle_2`: `x ~= -3.0`, `y ~= 4.0`, `z ~= -5.0`
  - `vehicle_3`: `x ~= -3.0`, `y ~= -4.0`, `z ~= -5.0`
- ground station 再次輸出：

```text
all vehicles reached staging positions
```

- action result 顯示 `success: true`

## Terminal 6：不重開 runtime，送出第二次 LandSwarm

```bash
ros2 action send_goal /swarm/land px4_swarm_interfaces/action/LandSwarm \
  "{timeout_sec: 60.0}" --feedback
```

通過條件：

- 三台飛機再次進入 landing，最後在 Gazebo 地面穩定停住
- 三台 `/vehicle_N/status` 最後都顯示 `vehicle_state: landed`
- ground station 再次輸出 `swarm mission landing -> done: all vehicles reported landed`
- action result 顯示 `success: true`

## repeated cycle 總通過條件

完整 manual smoke 要能在同一組 runtime 中完成：

```text
TakeoffSwarm -> LandSwarm -> TakeoffSwarm -> LandSwarm
```

通過條件：

- 第一次和第二次 `TakeoffSwarm` 都只需要送一次 action
- 第一次和第二次 `TakeoffSwarm` 都要等三台抵達 staging 後才回 `success: true`
- 第一次和第二次 `LandSwarm` 都要等三台 confirmed landed 後才回 `success: true`
- Gazebo 畫面、PX4 telemetry、`/vehicle_*/status` 在每個 milestone 都一致

## 如果未開 QGC 起飛失敗

不要先開 QGC 當 workaround。請保留各 terminal 輸出，改用 `$diagnosing-bugs`，至少蒐集：

- `VehicleCommandAck`
- `/vehicle_1/status`、`/vehicle_2/status`、`/vehicle_3/status`
- PX4 terminal 中 commander / preflight / arming 相關訊息
- Micro XRCE-DDS Agent log
- `ros2 topic info -v` 對 PX4 input/output topics 的 publisher/subscriber 狀態
