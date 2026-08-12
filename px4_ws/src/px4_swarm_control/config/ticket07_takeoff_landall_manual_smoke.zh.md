# Ticket 07：TakeoffSwarm / LandSwarm 手動 smoke 驗證

目標：第一版 SITL 不依賴 QGC，使用 ROS 2 action 讓三台 `gz_x500` 完成：

```text
TakeoffSwarm -> LandSwarm -> TakeoffSwarm -> LandSwarm
```

本文件已在 2026-08-12 驗證通過。驗證結果：同一組 Micro XRCE-DDS Agent、PX4 SITL、Gazebo、vehicle nodes、ground station 不重開，第一次與第二次 `TakeoffSwarm` 都只送一次 action 就能到 staging，兩次 `LandSwarm` 都會等三台 landed 後才回成功。

所有指令都假設你已經進入 container，workspace 在：

```bash
/home/ncrl/docker_ubuntu24
```

## 前置條件

- QGC 關閉；QGC 只能作為 optional monitoring，不是第一版控制入口。
- `MicroXRCEAgent udp4 -p 8888` 正在執行。
- Gazebo 中可以看到 `x500_1`、`x500_2`、`x500_3`。
- 三台 PX4 instance 已用 `/vehicle_1`、`/vehicle_2`、`/vehicle_3` namespace 接到同一個 Micro XRCE-DDS Agent。
- 專案已 build：

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 0. 清乾淨舊 runtime

如果你要做 clean-runtime smoke，先在 container 裡清掉上一輪可能殘留的 ROS control nodes、Micro XRCE-DDS Agent、PX4 SITL、Gazebo：

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

通過條件：最後一個 `pgrep` 不應再看到 `MicroXRCEAgent`、`px4 -i 1/2/3`、`gz sim`、`vehicle_node` 或 `ground_station_node`。如果還有殘留，使用 `kill <PID>` 指定清掉，再重跑最後一個 `pgrep`。

清乾淨後，再照 `live_px4_gz_bridge_smoke.zh.md` 重開：

```text
MicroXRCEAgent -> PX4 instance 1 -> PX4 instance 2 -> PX4 instance 3
```

## 1. 確認 bridge 健康

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

## 2. 確認沒有舊的 swarm ROS node

先查是否有殘留：

```bash
pgrep -af 'ground_station_node|vehicle_node'
```

如果看到舊的 `vehicle_node` 或 `ground_station_node`，請用 `kill <PID>` 清掉。這一步很重要，因為重複的 action server 或舊 vehicle node 會讓 action feedback/status 變得不可信。

清完後再次確認：

```bash
pgrep -af 'ground_station_node|vehicle_node'
```

通過條件：只剩下你自己的查詢指令，沒有舊的 swarm ROS node。

## 3. 啟動三個 vehicle node

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

## 4. 啟動 ground station

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
ros2 topic list | grep -E '/swarm|/vehicle_[123]/staging_setpoint|/vehicle_[123]/status'
```

通過條件：

- actions 包含 `/swarm/takeoff` 和 `/swarm/land`
- topics 包含 `/vehicle_1/status`、`/vehicle_2/status`、`/vehicle_3/status`
- topics 包含 `/vehicle_1/staging_setpoint`、`/vehicle_2/staging_setpoint`、`/vehicle_3/staging_setpoint`

## 5. 第一次 TakeoffSwarm

```bash
ros2 action send_goal /swarm/takeoff px4_swarm_interfaces/action/TakeoffSwarm \
  "{altitude_m: 5.0, timeout_sec: 120.0}" --feedback
```

通過條件：

- action goal accepted。
- feedback 一開始是 `current_state: taking_off`，且不會剛送出就 success。
- feedback 最後要到 `vehicles_staged: 3`、`progress: 1.0`。
- result 顯示：

```text
success: true
message: all vehicles reached staging positions
```

- vehicle node terminal 會印出：

```text
不依賴 QGC：先用 PX4 NAV_TAKEOFF 到安全高度，再 warm up Offboard 後切換 staging control。
```

## 6. 檢查 staging status

```bash
ros2 topic echo --once /vehicle_1/status
ros2 topic echo --once /vehicle_2/status
ros2 topic echo --once /vehicle_3/status
```

通過條件：

- 三台 `x/y/z/yaw` 都是有限數值，不是 `nan`。
- 三台 `armed: true`。
- 三台 `nav_state: offboard`。
- 三台 `vehicle_state: staging`。
- 位置接近：
  - `vehicle_1`: `x ~= 0.0`, `y ~= 0.0`, `z ~= -5.0`
  - `vehicle_2`: `x ~= -3.0`, `y ~= 4.0`, `z ~= -5.0`
  - `vehicle_3`: `x ~= -3.0`, `y ~= -4.0`, `z ~= -5.0`

本次驗證觀察到的第二輪 staging 範例：

```text
vehicle_1: x=0.001,  y=0.006,  z=-5.005, nav_state=offboard, vehicle_state=staging
vehicle_2: x=-2.990, y=3.984,  z=-5.004, nav_state=offboard, vehicle_state=staging
vehicle_3: x=-2.997, y=-3.998, z=-5.008, nav_state=offboard, vehicle_state=staging
```

## 7. 第一次 LandSwarm

```bash
ros2 action send_goal /swarm/land px4_swarm_interfaces/action/LandSwarm \
  "{timeout_sec: 120.0}" --feedback
```

通過條件：

- action goal accepted。
- feedback 一開始是 `current_state: landing`。
- feedback 最後要到 `vehicles_landed: 3`。
- result 顯示：

```text
success: true
message: all vehicles reported landed
```

- ground station terminal 會輸出：

```text
swarm mission landing -> done: all vehicles reported landed
```

## 8. 檢查 landed status

```bash
ros2 topic echo --once /vehicle_1/status
ros2 topic echo --once /vehicle_2/status
ros2 topic echo --once /vehicle_3/status
```

通過條件：

- 三台 `armed: false`。
- 三台 `vehicle_state: landed`。
- 三台高度回到地面附近，例如 `z` 約在 `-0.1` 到 `0.1` 之間。

注意：PX4 在落地後可能仍短暫顯示 `nav_state: offboard` 或 `offboard_available: true`。第一版 status 是否 landed 以 PX4 landed telemetry 加上 ROS 2 vehicle state 為準，不用只看 `nav_state`。

## 9. 不重開 runtime，第二次 TakeoffSwarm

不要重開 Micro XRCE-DDS Agent、PX4 SITL、Gazebo、vehicle nodes 或 ground station，直接送第二次 takeoff：

```bash
ros2 action send_goal /swarm/takeoff px4_swarm_interfaces/action/TakeoffSwarm \
  "{altitude_m: 5.0, timeout_sec: 120.0}" --feedback
```

通過條件：

- 不需要重送第二次相同 action。
- action 最後 `success: true`。
- feedback 最後 `vehicles_staged: 3`。
- 三台再次 `armed: true`、`nav_state: offboard`、`vehicle_state: staging`。
- 三台再次回到相同 staging positions。

這一步保護的問題是：上一輪 landing 後，ROS 2 不能保留 stale `landed` 或 stale staging state，否則第二次 takeoff 會出現「Gazebo 已飛、status 還顯示 landed」或「action 提早 success」。

## 10. 不重開 runtime，第二次 LandSwarm

```bash
ros2 action send_goal /swarm/land px4_swarm_interfaces/action/LandSwarm \
  "{timeout_sec: 120.0}" --feedback
```

通過條件：

- action 最後 `success: true`。
- feedback 最後 `vehicles_landed: 3`。
- 三台在 Gazebo 地面穩定停住。
- 三台 `/vehicle_N/status` 最後都是 `armed: false`、`vehicle_state: landed`。

本次驗證觀察到的最終 landed 範例：

```text
vehicle_1: z=0.025,  armed=false, vehicle_state=landed
vehicle_2: z=-0.062, armed=false, vehicle_state=landed
vehicle_3: z=0.001,  armed=false, vehicle_state=landed
```

## 整體通過條件

完整 smoke 必須在同一組 runtime 中完成：

```text
TakeoffSwarm -> LandSwarm -> TakeoffSwarm -> LandSwarm
```

而且符合：

- 兩次 `TakeoffSwarm` 都只送一次 action。
- 兩次 `TakeoffSwarm` 都等三台真的抵達 staging 後才回 `success: true`。
- 兩次 `LandSwarm` 都等三台真的 landed 後才回 `success: true`。
- Gazebo 畫面、PX4 telemetry、`/vehicle_*/status` 在每個 milestone 都一致。

## 本次修正重點

- ground station 在 `TakeoffSwarm` 等待期間會重送 staging setpoint 與 mission command，避免 vehicle node 先收到 takeoff、後收到 staging target 時錯過目標。
- vehicle node 會先讓 PX4 用 `NAV_TAKEOFF` 起飛到安全高度，再 warm up Offboard heartbeat 與 trajectory setpoint，最後才切 Offboard。
- vehicle node 只有在 PX4 真的 `nav_state: offboard`、且高度已達標、且不是地面 landed telemetry 時，才承認 Offboard staging 成功。
- `LandSwarm` 後 vehicle node 會進入 `landing`，停止發布空中 staging setpoint，並等 PX4 landed telemetry 後才進 `landed`。
- `LandSwarm` 會清掉上一輪 staging latch，避免下一輪 `TAKEOFF` 比新 staging setpoint 早到時吃到舊目標。

## 常見失敗判讀

- 看到 duplicate action server warning：通常是舊的 `ground_station_node` 沒清掉，回到第 2 步清 runtime。
- `/vehicle_N/status` 是 `nan`：通常是該 PX4 instance 沒有真正 publisher，先重跑 bridge smoke check。
- Gazebo 飛了但 status 還是 `landed`：代表 landed/staging state 有殘留或 Offboard 接受條件太早，需要保留 logs 後用 `$diagnosing-bugs`。
- 未開 QGC 起飛失敗：不要先開 QGC 當 workaround，請保留 `VehicleCommandAck`、三台 `/vehicle_N/status`、PX4 terminal、Micro XRCE-DDS Agent log。
