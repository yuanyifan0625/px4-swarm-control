# Ticket 10：ChangeFormation 手動 smoke 驗證

目標：第一版 SITL 不依賴 QGC，三台 `gz_x500` 完成 staging 並進入 leader following 後，operator 只透過 `/swarm/change_formation` 切換 `vee` 與 `line_abreast`。Ground station 只能發布 `/swarm/formation_mode`；followers 必須在各自 `vehicle_node` 內根據 `/vehicle_1/status`、formation mode、自己的 fixed slot 計算 setpoint。

完整任務卡：

```text
clean runtime -> TakeoffSwarm -> MoveLeader -> verify vee -> ChangeFormation line_abreast -> verify line_abreast -> ChangeFormation vee -> verify vee -> LandSwarm
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
vehicle_1 node -> vehicle_2 node -> vehicle_3 node -> ground_station_node
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

確認 actions/topics：

```bash
ros2 action list | grep /swarm
ros2 topic list | grep -E '/swarm/(leader_goal|formation_mode)|/vehicle_[123]/status|/vehicle_[123]/staging_setpoint'
```

通過條件：

- actions 包含 `/swarm/takeoff`、`/swarm/move_leader`、`/swarm/change_formation`、`/swarm/land`。
- topics 包含 `/swarm/formation_mode`。
- topics 包含 `/vehicle_1/status`、`/vehicle_2/status`、`/vehicle_3/status`。

## 2. TakeoffSwarm 到 staging

```bash
ros2 action send_goal /swarm/takeoff px4_swarm_interfaces/action/TakeoffSwarm \
  "{altitude_m: 5.0, timeout_sec: 120.0}" --feedback
```

通過條件：

- result 顯示 `success: true`。
- message 是 `all vehicles reached staging positions`。
- 三台都到 staging：
  - `vehicle_1`: 約 `(0, 0, -5)`
  - `vehicle_2`: 約 `(-3, 4, -5)`
  - `vehicle_3`: 約 `(-3, -4, -5)`

## 3. MoveLeader 進入 following

```bash
ros2 action send_goal /swarm/move_leader px4_swarm_interfaces/action/MoveLeader \
  "{x: 3.0, y: 2.0, z: -5.0, yaw: 0.0, position_tolerance_m: 0.5, yaw_tolerance_rad: 0.2, timeout_sec: 120.0}" --feedback
```

通過條件：

- result 顯示 `success: true`。
- message 是 `leader reached target`。
- `/vehicle_1/status` 接近 `(3, 2, -5)`，`yaw` 接近 `0.0`。

## 4. 確認預設 vee

```bash
ros2 topic echo --once /vehicle_1/status
ros2 topic echo --once /vehicle_2/status
ros2 topic echo --once /vehicle_3/status
```

若 leader 約 `(3, 2, -5, yaw=0)`，預設 `vee` 通過條件約為：

```text
vehicle_1: x ~= 3, y ~= 2,  z ~= -5
vehicle_2: x ~= 0, y ~= 6,  z ~= -5
vehicle_3: x ~= 0, y ~= -2, z ~= -5
```

Gazebo 中應看到兩台 followers 在 leader 後方左右兩側。

## 5. ChangeFormation 切到 line_abreast

```bash
ros2 action send_goal /swarm/change_formation px4_swarm_interfaces/action/ChangeFormation \
  "{formation_mode: line_abreast, timeout_sec: 120.0}" --feedback
```

通過條件：

- feedback 一開始是 `current_state: reconfiguring`。
- feedback `active_formation: line_abreast`。
- feedback `progress` 會依 follower 到位比例顯示 `0.0`、`0.5` 或 `1.0`。
- result 不會在 mode 剛發布時立刻成功；必須等 followers 到新 slot tolerance 內。
- result 最後顯示：

```text
success: true
message: formation established
```

Ground station terminal 應輸出：

```text
formation established
```

## 6. 確認 line_abreast 位置

```bash
ros2 topic echo --once /vehicle_1/status
ros2 topic echo --once /vehicle_2/status
ros2 topic echo --once /vehicle_3/status
```

若 leader 約 `(3, 2, -5, yaw=0)`，`line_abreast` 通過條件約為：

```text
vehicle_1: x ~= 3, y ~= 2,  z ~= -5
vehicle_2: x ~= 3, y ~= 6,  z ~= -5
vehicle_3: x ~= 3, y ~= -2, z ~= -5
```

允許約 `0.3m` position tolerance 和約 `0.2rad` yaw tolerance。Gazebo 中應看到兩台 followers 移到 leader 左右同一排。

## 7. 確認沒有作弊

觀察 formation mode：

```bash
ros2 topic echo --once /swarm/formation_mode
```

通過條件：

- `/swarm/formation_mode` 顯示 `mode: line_abreast`。
- Ground station 在 formation change 階段只發布 mode，不應把 follower 新位置當成 `/vehicle_2/staging_setpoint` 或 `/vehicle_3/staging_setpoint` 持續發布。
- `vehicle_2` 應維持 `slot: follower_left`。
- `vehicle_3` 應維持 `slot: follower_right`。
- followers 不應飛到 leader 的 absolute goal，而是飛到 leader state 加上自己的 mode/slot offset。

## 8. ChangeFormation 切回 vee

```bash
ros2 action send_goal /swarm/change_formation px4_swarm_interfaces/action/ChangeFormation \
  "{formation_mode: vee, timeout_sec: 120.0}" --feedback
```

通過條件：

- result 最後顯示 `success: true`。
- message 是 `formation established`。
- 若 leader 約 `(3, 2, -5, yaw=0)`，followers 回到：

```text
vehicle_2: x ~= 0, y ~= 6,  z ~= -5
vehicle_3: x ~= 0, y ~= -2, z ~= -5
```

## 9. 附加 yaw 旋轉驗證

可再送一次 MoveLeader，使用非零 yaw：

```bash
ros2 action send_goal /swarm/move_leader px4_swarm_interfaces/action/MoveLeader \
  "{x: 3.0, y: 2.0, z: -5.0, yaw: 0.5, position_tolerance_m: 0.5, yaw_tolerance_rad: 0.2, timeout_sec: 120.0}" --feedback
```

通過條件：followers 的左右/後方位置會跟 leader yaw 一起旋轉，不會固定在 world-frame 的左右方向。

## 10. LandSwarm 收尾

```bash
ros2 action send_goal /swarm/land px4_swarm_interfaces/action/LandSwarm \
  "{timeout_sec: 120.0}" --feedback
```

通過條件：

- result 顯示 `success: true`。
- message 是 `all vehicles reported landed`。
- 三台 `/vehicle_N/status` 最後都是 `vehicle_state: landed`。

## 整體通過條件

- QGC 全程不作為控制入口。
- `ChangeFormation` 只接受 `vee` 和 `line_abreast`。
- `ChangeFormation` 不移動 leader。
- Ground station 只發布 `/swarm/formation_mode`，不替 followers 持續發布 absolute formation target。
- Followers 由各自 `vehicle_node` 根據 `/vehicle_1/status`、formation mode、fixed slot 本地計算 setpoint。
- `vehicle_2` 維持 `follower_left`，`vehicle_3` 維持 `follower_right`。
- `ChangeFormation` success 必須等 followers 實際到新 formation tolerance 內才出現。
- `LandSwarm` 最後讓三台 landed。
