# Ticket 11：Pause / Failsafe 手動 smoke 驗證

目標：第一版 SITL 不依賴 QGC，三台 `gz_x500` 在完成 takeoff、leader move、followers following 後，可以用 `PauseSwarm` 暫停並 hold safe setpoint。Pause 不需要重啟 Micro XRCE-DDS Agent、PX4 SITL、Gazebo、vehicle nodes 或 ground station。Paused 狀態下只允許 status observation、resume、LandSwarm，並拒絕新的 MoveLeader / ChangeFormation。

完整任務卡：

```text
clean runtime -> TakeoffSwarm -> MoveLeader -> PauseSwarm -> verify hold
-> reject MoveLeader/ChangeFormation while paused -> Resume -> fresh MoveLeader
-> PauseSwarm -> LandSwarm
```

所有指令都假設你已經進入 container，workspace 在：

```bash
/home/ncrl/docker_ubuntu24
```

## 0. 清乾淨舊 runtime

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

通過條件：最後一個 `pgrep` 不應看到舊的 `MicroXRCEAgent`、`px4 -i 1/2/3`、`gz sim`、`vehicle_node` 或 `ground_station_node`。

## 1. 啟動 bridge、PX4 Gz、vehicle nodes、ground station

照 `live_px4_gz_bridge_smoke.zh.md` 啟動：

```text
MicroXRCEAgent -> PX4 instance 1 -> PX4 instance 2 -> PX4 instance 3
```

照 `ticket09_follower_following_manual_smoke.zh.md` 啟動：

```text
MAV1 node -> MAV2 node -> MAV3 node -> ground_station_node
```

確認 bridge：

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run px4_swarm_control check_live_px4_gz_bridge --agent-log /tmp/microxrceagent_8888.log
```

通過條件：

- Gazebo 可看到 `x500_1`、`x500_2`、`x500_3`。
- `ROS 2 PX4 publishers OK for all vehicle telemetry topics`。
- `Micro XRCE-DDS Agent sessions OK`。

## 2. TakeoffSwarm 到 staging

```bash
ros2 action send_goal /swarm/takeoff px4_swarm_interfaces/action/TakeoffSwarm \
  "{altitude_m: 5.0, timeout_sec: 120.0}" --feedback
```

通過條件：

- result 顯示 `success: true`。
- message 是 `all vehicles reached staging positions`。
- 三台 `/vehicle_N/status` 都應是 `armed: true`、`nav_state: offboard`，高度約 `z=-5`。

## 3. MoveLeader 進入 following

```bash
ros2 action send_goal /swarm/move_leader px4_swarm_interfaces/action/MoveLeader \
  "{x: 3.0, y: 2.0, z: -5.0, yaw: 0.0, position_tolerance_m: 0.5, yaw_tolerance_rad: 0.2, timeout_sec: 120.0}" --feedback
```

通過條件：

- result 顯示 `success: true`。
- message 是 `leader reached target`。
- 三台 `/vehicle_N/status` 應逐步進入 `vehicle_state: following`。

## 4. PauseSwarm 並確認 hold

```bash
ros2 action send_goal /swarm/pause px4_swarm_interfaces/action/PauseSwarm \
  "{pause: true, reason: 'operator pause smoke'}" --feedback
```

通過條件：

- result 顯示 `success: true`。
- feedback 顯示 `current_state: paused`、`paused: true`。
- 三台 `/vehicle_N/status` 應顯示 `vehicle_state: paused`。
- Gazebo 中三台應維持目前位置附近 hover，不應繼續追新的 leader / formation 目標。

確認 status：

```bash
ros2 topic echo --once /MAV1/status
ros2 topic echo --once /MAV2/status
ros2 topic echo --once /MAV3/status
```

## 5. Paused 狀態拒絕 MoveLeader

```bash
ros2 action send_goal /swarm/move_leader px4_swarm_interfaces/action/MoveLeader \
  "{x: 6.0, y: 2.0, z: -5.0, yaw: 0.0, position_tolerance_m: 0.5, yaw_tolerance_rad: 0.2, timeout_sec: 20.0}" --feedback
```

通過條件：

- action 應回失敗或 aborted。
- result message 應包含 paused，例如 `MoveLeader rejected while swarm is paused`。
- `/MAV1/status` 不應開始追 `(6, 2, -5)`。
- Gazebo 中三台仍 hold。

## 6. Paused 狀態拒絕 ChangeFormation

```bash
ros2 action send_goal /swarm/change_formation px4_swarm_interfaces/action/ChangeFormation \
  "{formation_mode: line_abreast, timeout_sec: 20.0}" --feedback
```

通過條件：

- action 應回失敗或 aborted。
- result message 應包含 paused，例如 `ChangeFormation rejected while swarm is paused`。
- followers 不應開始切換到新的 formation slot。

## 7. Resume 回到 safe holding

```bash
ros2 action send_goal /swarm/pause px4_swarm_interfaces/action/PauseSwarm \
  "{pause: false, reason: 'resume from pause smoke'}" --feedback
```

通過條件：

- result 顯示 `success: true`。
- feedback 顯示 `current_state: holding`、`paused: false`。
- 三台 `/vehicle_N/status` 不應自動繼續 pause 前的舊 MoveLeader / ChangeFormation action。

## 8. Resume 後送 fresh MoveLeader

```bash
ros2 action send_goal /swarm/move_leader px4_swarm_interfaces/action/MoveLeader \
  "{x: 4.0, y: 2.0, z: -5.0, yaw: 0.0, position_tolerance_m: 0.5, yaw_tolerance_rad: 0.2, timeout_sec: 120.0}" --feedback
```

通過條件：

- result 顯示 `success: true`。
- message 是 `leader reached target`。
- leader 移到 fresh target 附近。
- followers 只根據 leader status 和 formation mode 本地跟隨，不直接吃 operator absolute target。

## 9. 再次 Pause 後 LandSwarm recovery

```bash
ros2 action send_goal /swarm/pause px4_swarm_interfaces/action/PauseSwarm \
  "{pause: true, reason: 'pause before land recovery'}" --feedback
```

接著不重啟任何 runtime，直接送 LandSwarm：

```bash
ros2 action send_goal /swarm/land px4_swarm_interfaces/action/LandSwarm \
  "{timeout_sec: 120.0}" --feedback
```

通過條件：

- `LandSwarm` result 顯示 `success: true`。
- message 是 `all vehicles reported landed`。
- 三台 `/vehicle_N/status` 最後都是 `vehicle_state: landed`、`armed: false`。

## 10. Optional：timeout/failsafe observation

Timeout/failsafe 的主要驗證由 unit tests 使用 controlled timestamps 完成。若要人工觀察，可在完成主要任務卡後，重新啟動一次 runtime，讓系統進入 following，再刻意停止某個 vehicle node 或 PX4 telemetry source，觀察對應 vehicle 是否進入 `vehicle_state: failsafe` 並 hover/hold。

此 optional 驗證會破壞 runtime，不屬於「不重啟完成 pause/resume/land」主流程。

## 整體通過條件

- QGC 全程不作為控制入口。
- Pause 不需要重啟 Micro XRCE-DDS Agent、PX4 SITL、Gazebo、vehicle nodes 或 ground station。
- Pause 後三台 hold safe setpoint，不繼續 mission progression。
- Paused 狀態允許 status observation、resume、LandSwarm。
- Paused 狀態拒絕 MoveLeader 和 ChangeFormation。
- Resume 後回到 safe holding，不自動續跑 pause 前舊目標。
- Resume 後可以送 fresh MoveLeader。
- Paused 狀態下仍可用 LandSwarm recovery，且最後三台 landed。
