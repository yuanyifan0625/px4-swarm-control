# 最終版：ROS 2 層實機部署與操作指南

本文件只涵蓋 ROS 2 層。假設每台 PX4 已透過 uXRCE-DDS 正確產生 `/MAV1`、`/MAV2`、`/MAV3` topics；不由本專案啟動 PX4、DDS Agent、QGC 或任何底層飛控服務。

## 架構圖文字版

```text
Ground station
  real_ground_station.launch.py
  operator_console
  /swarm actions

Pi-MAV1
  real_mav1_vehicle.launch.py
  /MAV1/vehicle_node
  /MAV1/fmu/in/* and /MAV1/fmu/out/*

Pi-MAV2
  real_mav2_vehicle.launch.py
  /MAV2/vehicle_node
  /MAV2/fmu/in/* and /MAV2/fmu/out/*

Pi-MAV3
  real_mav3_vehicle.launch.py
  /MAV3/vehicle_node
  /MAV3/fmu/in/* and /MAV3/fmu/out/*
```

## 最低前提

- 所有機器使用相同 `ROS_DOMAIN_ID`。
- DDS discovery/data 在同一網路可互通。
- `/MAV1`、`/MAV2`、`/MAV3` 是唯一支援 namespace；不支援 `/vehicle_1`、`/vehicle_2`、`/vehicle_3`。
- `px4_msgs` 必須和實際 PX4 bridge topic suffix 相容。
- 正式操作只在地面站跑一個 active `operator_console`。
- 小場地隊形完成 tolerance 預設 `0.10 m`；實機若無法穩定收斂，先改 `0.12 m`，最後 fallback `0.15 m`。
- 可用 launch-time override 調 `formation_position_tolerance_m`、`settle_position_tolerance_m`、`settle_stable_duration_s`，不需要改 code。

調整：`ROS_DOMAIN_ID` 要在每台機器設同一值，例如 `export ROS_DOMAIN_ID=13`。

## 1. Clone / build / source

每台機器：

```bash
git clone <repo-url> docker_ubuntu24
cd docker_ubuntu24/px4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_swarm_interfaces px4_swarm_control
source install/setup.bash
```

驗收條件：兩個 package build 成功。調整：若 ROS 版本不是 Humble，改 source 對應 ROS distro；實機仍需確認 `px4_msgs` 相容。

## 2. 實機 PX4 topic 檢查

任一 ROS 2 機器上：

```bash
ros2 topic info -v /MAV1/fmu/out/vehicle_local_position_v1
ros2 topic info -v /MAV1/fmu/out/vehicle_status_v4
ros2 topic info -v /MAV1/fmu/out/vehicle_command_ack_v1
ros2 topic info -v /MAV1/fmu/out/vehicle_land_detected
ros2 topic info -v /MAV1/fmu/out/failsafe_flags

ros2 topic info -v /MAV2/fmu/out/vehicle_local_position_v1
ros2 topic info -v /MAV2/fmu/out/vehicle_status_v4
ros2 topic info -v /MAV2/fmu/out/vehicle_command_ack_v1
ros2 topic info -v /MAV2/fmu/out/vehicle_land_detected
ros2 topic info -v /MAV2/fmu/out/failsafe_flags

ros2 topic info -v /MAV3/fmu/out/vehicle_local_position_v1
ros2 topic info -v /MAV3/fmu/out/vehicle_status_v4
ros2 topic info -v /MAV3/fmu/out/vehicle_command_ack_v1
ros2 topic info -v /MAV3/fmu/out/vehicle_land_detected
ros2 topic info -v /MAV3/fmu/out/failsafe_flags
```

驗收條件：每個 out topic 都有 PX4 publisher。調整：若 topic suffix 不同，先停下來確認 PX4/`px4_msgs` contract，不要用 ROS remap 假裝對齊。

## 3. 啟動三台 vehicle nodes

Pi-MAV1：

```bash
cd docker_ubuntu24/px4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch px4_swarm_control real_mav1_vehicle.launch.py
```

Pi-MAV2：

```bash
cd docker_ubuntu24/px4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch px4_swarm_control real_mav2_vehicle.launch.py
```

Pi-MAV3：

```bash
cd docker_ubuntu24/px4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch px4_swarm_control real_mav3_vehicle.launch.py
```

驗收條件：各機只啟動自己的 `vehicle_node`。調整：角色、namespace、target-system 固定在 `three_vehicle_nodes.yaml`；不要互換 MAV 編號。

## 4. 啟動 ground station

地面站：

```bash
cd docker_ubuntu24/px4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch px4_swarm_control real_ground_station.launch.py
```

明確測 `0.10 m` 時：

```bash
ros2 launch px4_swarm_control real_ground_station.launch.py formation_position_tolerance_m:=0.10
```

驗收條件：只啟動 `/swarm/ground_station_node`。調整：若 `0.10 m` 太嚴格，可用 `ros2 launch px4_swarm_control real_ground_station.launch.py formation_position_tolerance_m:=0.12`，最後 fallback `formation_position_tolerance_m:=0.15`；若找不到三台 status，先檢查第 3 步和 DDS discovery。

## 5. 檢查 swarm graph

地面站：

```bash
ros2 topic echo --once /MAV1/status
ros2 topic echo --once /MAV2/status
ros2 topic echo --once /MAV3/status
ros2 action list | grep /swarm
```

驗收條件：三個 status topic 都有 fresh telemetry，且 `/swarm/arm`、`/swarm/takeoff`、`/swarm/move_leader`、`/swarm/change_formation`、`/swarm/pause`、`/swarm/land` 存在。調整：若 status 是 stale，先不要飛，檢查 PX4 topic publisher 與網路。

## 6. 啟動 operator console

地面站：

```bash
cd docker_ubuntu24/px4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run px4_swarm_control operator_console
```

需要調整 settle 時：

```bash
ros2 launch px4_swarm_control operator_console.launch.py settle_position_tolerance_m:=0.10 settle_stable_duration_s:=1.5
```

驗收條件：出現 `swarm>` prompt。調整：正式操作只保留一個 console；`0.10 m` 太嚴格時改 `settle_position_tolerance_m:=0.12`，最後 fallback `settle_position_tolerance_m:=0.15`；不要多台機器同時下 command。

## 7. 實機 command 語意

- `0`：arm-only，不起飛，不切 Offboard。
- `1`：takeoff 到 `1.5 m`，三台進入小場地 staging。
- `2` / `x`：leader world x 正/反方向各移動 `1.0 m`。
- `3` / `y`：leader world y 正/反方向各移動 `1.0 m`。
- `4`：leader 上升一個 altitude step。
- `z`：leader 下降一個 altitude step。
- `5` / `c`：leader yaw `+30 deg` / `-30 deg`。
- `6`：切換 `vee`，三台形成邊長 `0.8 m` 正三角形。
- `7`：切換 `line_abreast`，leader 到左右 follower 各 `0.8 m`。
- `8`：全隊 land。
- `9`：完整 demo macro。

驗收條件：所有 movement/formation command 都只經 `/swarm` action，followers 由各自 vehicle node 追隊形。調整：高度、步距、yaw、console settle tolerance/stable duration 改 `operator_console.yaml` 或 `operator_console.launch.py`；隊形幾何和 ground-station formation tolerance 改 `three_vehicle_nodes.yaml` 或 ground-station launch override。

## 8. 實機 smoke test checklist

建議先無槳或安全固定完成 ROS 2 層檢查，再進入實飛：

```text
s
0
1
settle
2
settle
5
settle
7
settle
6
settle
8
```

完整 demo：

```text
9
```

驗收條件：`0` 完成 arm-only；`1` 完成 staging；`2/5` 完成 leader movement/yaw；`7/6` 完成隊形切換；`8` 後三台 `vehicle_state: landed` 且 `armed: false`；`9` 回報 `OK: demo macro completed`。調整：飛場不足時先只驗證 `0/1/8`，再逐步加入 `2/5/7/6/9`。

## 9. 常見 debug 指令

```bash
ros2 node list
ros2 topic list | grep -E '/MAV[123]|/swarm'
ros2 topic echo --once /MAV1/status
ros2 topic echo --once /MAV2/status
ros2 topic echo --once /MAV3/status
ros2 action list | grep /swarm
ros2 topic info -v /MAV1/fmu/in/vehicle_command
ros2 topic info -v /MAV2/fmu/in/vehicle_command
ros2 topic info -v /MAV3/fmu/in/vehicle_command
```

驗收條件：節點、status、actions、PX4 in/out topics 都存在。調整：若 `/fmu/in/vehicle_command` 只有 subscriber 或 publisher 數不合理，先確認 vehicle node 是否啟動和 DDS namespace 是否正確。
