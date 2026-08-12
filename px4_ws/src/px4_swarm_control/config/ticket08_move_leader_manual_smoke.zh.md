# Ticket 08：MoveLeader 手動 smoke 驗證

目標：第一版 SITL 不依賴 QGC，三台 `gz_x500` 完成 staging 後，operator 只透過 `MoveLeader` 移動 leader，followers 保持 staged 或 holding，不把 `/swarm/leader_goal` 當成自己的目標，最後用 `LandSwarm` 收尾。

完整任務卡：

```text
clean runtime -> TakeoffSwarm -> MoveLeader -> LandSwarm
```

所有指令都假設你已經進入 container，workspace 在：

```bash
/home/ncrl/docker_ubuntu24
```

## 0. 清乾淨舊 runtime

先照 ticket 07 manual smoke 文件的 cleanup commands 清掉舊的 ROS control nodes、Micro XRCE-DDS Agent、PX4 SITL、Gazebo：

```bash
pgrep -af '[M]icroXRCEAgent|[g]round_station_node|[v]ehicle_node|[p]x4 -i|[g]z sim|[g]zserver|[g]zclient'
pkill -TERM -x MicroXRCEAgent || true
pkill -TERM -x px4 || true
pkill -TERM -x gz || true
pkill -TERM -x gzserver || true
pkill -TERM -x gzclient || true
pgrep -f 'px4_swarm_control/lib/px4_swarm_control/[v]ehicle_node' | xargs -r kill
pgrep -f 'px4_swarm_control/lib/px4_swarm_control/[g]round_station_node' | xargs -r kill
pgrep -f '[g]z sim' | xargs -r kill
sleep 2
pgrep -af '[M]icroXRCEAgent|[g]round_station_node|[v]ehicle_node|[p]x4 -i|[g]z sim|[g]zserver|[g]zclient' || true
```

通過條件：最後一個 `pgrep` 不應看到舊的 runtime process。

## 1. 啟動三機 PX4 Gz bridge

照 `live_px4_gz_bridge_smoke.zh.md` 開啟：

```text
MicroXRCEAgent -> PX4 instance 1 -> PX4 instance 2 -> PX4 instance 3
```

確認 bridge：

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run px4_swarm_control check_live_px4_gz_bridge --agent-log /tmp/microxrceagent_8888.log
```

通過條件：

- `Gazebo models OK: x500_1, x500_2, x500_3`
- `ROS 2 PX4 publishers OK for all vehicle telemetry topics`
- `Micro XRCE-DDS Agent sessions OK`

## 2. 啟動 vehicle nodes 和 ground station

照 `ticket07_takeoff_landall_manual_smoke.zh.md` 啟動三個 `vehicle_node` 和一個 `ground_station_node`。

確認 actions/topics：

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 action list | grep /swarm
ros2 topic list | grep -E '/swarm/leader_goal|/swarm/move_leader|/vehicle_[123]/status'
```

通過條件：

- actions 包含 `/swarm/takeoff`、`/swarm/move_leader`、`/swarm/land`
- topics 包含 `/swarm/leader_goal`
- topics 包含 `/vehicle_1/status`、`/vehicle_2/status`、`/vehicle_3/status`

## 3. TakeoffSwarm 到 staging

```bash
ros2 action send_goal /swarm/takeoff \ px4_swarm_interfaces/action/TakeoffSwarm \
  "{altitude_m: 5.0, timeout_sec: 120.0}" --feedback
```

通過條件：

- result 顯示 `success: true`
- message 是 `all vehicles reached staging positions`
- 三台 status 都是 fresh telemetry
- 三台都到 staging：
  - `vehicle_1`: 約 `(0, 0, -5)`
  - `vehicle_2`: 約 `(-3, 4, -5)`
  - `vehicle_3`: 約 `(-3, -4, -5)`

## 4. 記錄 MoveLeader 前的 follower 位置

```bash
ros2 topic echo --once /vehicle_2/status
ros2 topic echo --once /vehicle_3/status
```

記下 `vehicle_2` 和 `vehicle_3` 的 `x/y/z/yaw`，後面用來確認 followers 沒有跟著 leader goal 移動。

## 5. 送出 MoveLeader

```bash
ros2 action send_goal /swarm/move_leader px4_swarm_interfaces/action/MoveLeader \
  "{x: 2.0, y: 0.0, z: -5.0, yaw: 0.5, position_tolerance_m: 0.5, yaw_tolerance_rad: 0.2, timeout_sec: 120.0}" --feedback
```

通過條件：

- action goal accepted。
- feedback 會回報 `current_state: following`、`remaining_distance_m`、`yaw_error_rad`。
- result 不會在命令剛發布時立刻成功；必須等 leader fresh status 顯示 `armed: true`、`nav_state: offboard`，且 position/yaw 都到 tolerance 內。
- result 最後顯示：

```text
success: true
message: leader reached target
```

## 6. 確認只有 leader 移動

```bash
ros2 topic echo --once /vehicle_1/status
ros2 topic echo --once /vehicle_2/status
ros2 topic echo --once /vehicle_3/status
```

通過條件：

- `vehicle_1` 接近 MoveLeader 目標：
  - `x ~= 2.0`
  - `y ~= 0.0`
  - `z ~= -5.0`
  - `yaw ~= 0.5`
- `vehicle_1` 的 `vehicle_state` 應是 `following` 或仍在 Offboard control 下的等價狀態。
- `vehicle_2` 和 `vehicle_3` 不應追 `/swarm/leader_goal`。
- `vehicle_2` 和 `vehicle_3` 應大致維持 staging/holding 位置，不會移動到 `(2.0, 0.0, -5.0)`。

這一步保護 distributed follower-control 邊界：ticket 08 只能移動 leader；followers 要等 ticket 09 才能根據 leader state 和 formation mode 本地計算跟隨 setpoint。

## 7. LandSwarm 收尾

```bash
ros2 action send_goal /swarm/land px4_swarm_interfaces/action/LandSwarm \
  "{timeout_sec: 120.0}" --feedback
```

通過條件：

- result 顯示 `success: true`
- message 是 `all vehicles reported landed`
- 三台 `/vehicle_N/status` 最後都是 `vehicle_state: landed`

## 整體通過條件

- QGC 全程不作為控制入口。
- `TakeoffSwarm` 完成後三台到 staging。
- `MoveLeader` 只讓 leader 到 absolute world-frame target。
- Followers 不把 `/swarm/leader_goal` 當成自己的 target。
- `MoveLeader` success 只在 fresh leader status 到 tolerance 後出現。
- `MoveLeader` success 還要求 leader 仍是 armed 且在 Offboard control。
- `LandSwarm` 最後讓三台 landed。
