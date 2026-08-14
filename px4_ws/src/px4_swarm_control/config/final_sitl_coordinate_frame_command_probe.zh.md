# SITL 座標軸 Command Probe 手冊

## 前置條件

假設你已經在 container 裡，且目前路徑是：

```bash
cd /home/ncrl/docker_ubuntu24
```

需要已經啟動：

- Gazebo SITL，包含 MAV1/MAV2/MAV3。
- Micro-XRCE-DDS Agent。
- PX4 `/MAV1`、`/MAV2`、`/MAV3` topics。
- `swarm_nodes.launch.py`。
- `operator_console` 已停止。

驗收條件：`ros2 node list` 可看到三個 vehicle node 和 `/swarm/ground_station_node`，但沒有 `/operator_console`。

## 進入 ROS 2 workspace

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

驗收條件：`ros2 action list` 可看到 `/swarm/move_leader`。

## 確認 topic

```bash
ros2 topic echo /MAV1/fmu/out/vehicle_local_position_v1 --once
ros2 topic echo /MAV1/status --once
```

驗收條件：兩個 topic 都有 `x/y/z`，且數值接近。

## 執行 commanded probe

```bash
ros2 run px4_swarm_control coordinate_frame_probe --ros-args \
  -p mode:=commanded \
  -p px4_namespace:=/MAV1 \
  -p axis_step_m:=0.30 \
  -p up_step_m:=0.20
```

參數用途：

- `mode:=commanded`：由工具透過 `/swarm/move_leader` 發送小位移。
- `px4_namespace:=/MAV1`：讀取 MAV1 的 PX4 raw topic 和 swarm status。
- `axis_step_m:=0.30`：測 PX4 `+X`、`+Y` 的水平小位移。
- `up_step_m:=0.20`：測 PX4 `-Z`，也就是上升。

驗收條件：

- 每段 action 顯示 `leader reached target`。
- 工具每段完成後回到 baseline pose。
- `PX4 raw delta` 和 `status delta` 同方向且接近。
- SITL/Gazebo 預期可觀察到：
  - PX4 `+X` 約等於 Gazebo `+Y`。
  - PX4 `+Y` 約等於 Gazebo `+X`。
  - PX4 `-Z` 約等於 Gazebo `+Z`。

## WARNING / ERROR

- `WARNING: Gazebo pose unavailable`：Gazebo pose 沒讀到；仍可用 PX4 raw/status delta 判斷 DDS/ROS2 frame。
- `ERROR: status does not match PX4 local position`：`/MAV1/status` 和 raw PX4 topic 不一致，先不要做 field-frame console 測試。
- `/swarm/move_leader unavailable`：ground station 沒啟動或 ROS_DOMAIN_ID/network 不一致。

## 已知 frame contract

ROS 2 透過 Micro-XRCE-DDS 看到的是 PX4 local NED frame，不是 Gazebo GUI 座標：

```text
PX4 +X -> Gazebo +Y
PX4 +Y -> Gazebo +X
PX4 -Z -> Gazebo +Z
```

實機不得假設 Gazebo mapping 等於飛場 mapping。
