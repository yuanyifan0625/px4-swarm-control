# Ticket 09：Follower 固定 slot 跟隨手動 smoke 驗證

目標：第一版 SITL 不依賴 QGC，三台 `gz_x500` 完成 staging 後，operator 只下 `MoveLeader` 給 leader；`vehicle_2` 和 `vehicle_3` 只根據 `/vehicle_1/status`、目前 formation mode、自己的固定 slot，在各自 `vehicle_node` 內本地計算自己的 `position + yaw` setpoint。

完整任務卡：

```text
clean runtime -> TakeoffSwarm -> MoveLeader -> followers follow vee offsets -> LandSwarm
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

## 1. 啟動 MicroXRCEAgent 與三台 PX4 Gz

照 `live_px4_gz_bridge_smoke.zh.md` 開啟四個 terminal：

```text
MicroXRCEAgent -> PX4 instance 1 -> PX4 instance 2 -> PX4 instance 3
```

三台 PX4 啟動時請維持：

```text
vehicle_1: PX4_UXRCE_DDS_NS=vehicle_1, -i 1, x500_1
vehicle_2: PX4_UXRCE_DDS_NS=vehicle_2, -i 2, x500_2
vehicle_3: PX4_UXRCE_DDS_NS=vehicle_3, -i 3, x500_3
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
- `Gazebo models OK: x500_1, x500_2, x500_3`
- `ROS 2 PX4 publishers OK for all vehicle telemetry topics`
- `Micro XRCE-DDS Agent sessions OK`

## 2. 啟動三個 vehicle_node

Terminal vehicle_1：

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

Terminal vehicle_2：

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

Terminal vehicle_3：

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

通過條件：

- `vehicle_2` 的 terminal 參數是 `role=follower`、`slot=follower_left`。
- `vehicle_3` 的 terminal 參數是 `role=follower`、`slot=follower_right`。
- 不需要、也不應該啟動額外 follower controller node。

## 3. 啟動 ground station

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run px4_swarm_control ground_station_node
```

另開 terminal 確認 actions/topics：

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 action list | grep /swarm
ros2 topic list | grep -E '/swarm/(takeoff|move_leader|land|leader_goal|formation_mode)|/vehicle_[123]/status|/vehicle_[123]/staging_setpoint'
```

通過條件：

- actions 包含 `/swarm/takeoff`、`/swarm/move_leader`、`/swarm/land`。
- topics 包含 `/swarm/leader_goal`、`/swarm/formation_mode`。
- topics 包含 `/vehicle_1/status`、`/vehicle_2/status`、`/vehicle_3/status`。

## 4. TakeoffSwarm 到 staging

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

## 5. 送出 MoveLeader

```bash
ros2 action send_goal /swarm/move_leader px4_swarm_interfaces/action/MoveLeader \
  "{x: 3.0, y: 2.0, z: -5.0, yaw: 0.0, position_tolerance_m: 0.5, yaw_tolerance_rad: 0.2, timeout_sec: 120.0}" --feedback
```

通過條件：

- action goal accepted。
- result 最後顯示：

```text
success: true
message: leader reached target
```

- ground station 只發布 `/swarm/leader_goal`，不在 following 階段持續發布 `/vehicle_2/staging_setpoint` 或 `/vehicle_3/staging_setpoint` 當 follower 目標。
- followers 只在 `/vehicle_1/status` 顯示 leader 已進入 `vehicle_state: following` 後開始跟隨；leader 仍在 `staging` 時不提前追隊形目標。

## 6. 確認 followers 以預設 vee offsets 跟隨

```bash
ros2 topic echo --once /vehicle_1/status
ros2 topic echo --once /vehicle_2/status
ros2 topic echo --once /vehicle_3/status
```

預設 `vee` offset：

```text
follower_left:  body-frame (-3, +4, 0)
follower_right: body-frame (-3, -4, 0)
```

如果 leader yaw 約 `0.0`，且 leader 目標約 `(3, 2, -5)`，通過條件大約是：

```text
vehicle_1: x ~= 3, y ~= 2,  z ~= -5, yaw ~= 0
vehicle_2: x ~= 0, y ~= 6,  z ~= -5, yaw ~= 0
vehicle_3: x ~= 0, y ~= -2, z ~= -5, yaw ~= 0
```

允許少量 overshoot 或 controller 收斂誤差；第一版請用 `position_tolerance_m` 附近的誤差判斷。Gazebo 中應看到 leader 前進後，兩台 followers 保持在 leader 後方左右兩側，而不是一起飛到 leader 的 absolute goal。

## 7. 確認 follower 沒有吃 `/swarm/leader_goal`

```bash
ros2 topic echo --once /swarm/leader_goal
ros2 topic echo --once /vehicle_2/status
ros2 topic echo --once /vehicle_3/status
```

通過條件：

- `/swarm/leader_goal` 是 operator 給 leader 的 absolute world-frame 目標。
- `vehicle_2` 不應接近 leader absolute goal `(3, 2, -5)`，而應接近 leader 加上 `follower_left` offset。
- `vehicle_3` 不應接近 leader absolute goal `(3, 2, -5)`，而應接近 leader 加上 `follower_right` offset。

## 8. LandSwarm 收尾

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
- `TakeoffSwarm` 完成後三台到 world-frame staging。
- `MoveLeader` 只讓 operator 指定 leader absolute world-frame target。
- Followers 在 `vehicle_node` 內根據 `/vehicle_1/status` 和自己的 fixed slot 本地計算目標。
- `vehicle_2` 維持 `follower_left`，`vehicle_3` 維持 `follower_right`，左右不反。
- Followers 不把 `/swarm/leader_goal` 當成自己的目標。
- leader telemetry stale 時，followers 應 hover/hold，不追舊 leader state。
- `LandSwarm` 最後讓三台 landed。
