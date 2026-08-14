# SITL Field-Frame Console Command 手冊

## 前置條件

假設你已經在 container 裡，且目前路徑是：

```bash
cd /home/ncrl/docker_ubuntu24
```

需要已經啟動：

- Gazebo SITL，包含 MAV1/MAV2/MAV3。
- Micro-XRCE-DDS Agent。
- PX4 `/MAV1`、`/MAV2`、`/MAV3` topics。
- `ros2 launch px4_swarm_control swarm_nodes.launch.py`。
- `operator_console` 已停止。
- Ticket 15 commanded probe 已確認 Gazebo visual profile。

驗收條件：`ros2 node list` 可看到 `/swarm/ground_station_node` 和三個 vehicle node，且沒有 `/operator_console`。

## 進入 ROS 2 workspace

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

驗收條件：`ros2 action list` 可看到 `/swarm/move_leader`、`/swarm/change_formation`、`/swarm/land`。

## 啟動 field-frame console

```bash
ros2 run px4_swarm_control field_frame_console --ros-args \
  -p field_x_axis:=px4_y \
  -p field_x_sign:=positive \
  -p field_y_axis:=px4_x \
  -p field_y_sign:=positive \
  -p field_up_axis:=px4_z \
  -p field_up_sign:=negative
```

參數用途：

- `field_x_axis/sign`：field `+X` 要轉成哪個 PX4 local NED 軸與方向。
- `field_y_axis/sign`：field `+Y` 要轉成哪個 PX4 local NED 軸與方向。
- `field_up_axis/sign`：field 上升要轉成哪個 PX4 local NED 軸與方向。
- 上面預設是 Gazebo visual profile：field `+X -> PX4 +Y`、field `+Y -> PX4 +X`、field up `-> PX4 -Z`。

驗收條件：console 顯示 `field-frame operator console`，並提醒 `operator_console` 是 raw PX4 local NED 入口。

## 指令表

```text
s / status      顯示三台 MAV status
p / pause       暫停 swarm action flow
r / resume      恢復 swarm action flow
q / quit        離開 console
h / help        顯示 help
0               arm only
1               takeoff，成功後 capture home pose
2 / x           field +X / -X
3 / y           field +Y / -Y
4 / z           field up / down
5 / c           yaw + / yaw -
6               vee
7               line_abreast
settle          等待目前隊形連續穩定
home            回到 captured home pose
home_yaw        回到 captured home yaw
8               land
9 demo          跑 demo，movement 走 field-frame mapping
```

驗收條件：`h` 顯示的 mapping 與啟動參數一致；若要 raw PX4 local NED，改用 `operator_console`。

## 小步長方向驗證

可先用較小步長：

```bash
ros2 run px4_swarm_control field_frame_console --ros-args \
  -p move_step_x_m:=0.30 \
  -p move_step_y_m:=0.30 \
  -p altitude_step_m:=0.20
```

依序輸入：

```text
1
2
x
3
y
4
z
8
```

驗收條件：

- `1`：三台機體進入起飛/編隊流程。
- `2`：Gazebo 畫面往 field `+X` 移動。
- `x`：Gazebo 畫面往 field `-X` 移動。
- `3`：Gazebo 畫面往 field `+Y` 移動。
- `y`：Gazebo 畫面往 field `-Y` 移動。
- `4`：高度上升。
- `z`：高度下降。
- `8`：降落完成。

## 編隊與 9 demo

手動編隊流程：

```text
1
6
settle
7
settle
6
settle
8
```

`9 demo` 會沿用既有 demo 流程，但其中 movement 會走 field-frame mapping：

```text
9
```

驗收條件：`settle` 需要連續穩定滿目前 `settle_stable_duration_s`，且 `vee`、`line_abreast` 完成後不應出現 follower 大幅偏離。

## Debug 指令

```bash
ros2 topic echo /MAV1/status --once
ros2 topic echo /MAV2/status --once
ros2 topic echo /MAV3/status --once
ros2 topic echo /MAV1/fmu/out/vehicle_local_position_v1 --once
ros2 action list | sort
ros2 node list | sort
```

驗收條件：`/MAV1/status` 的 `x/y/z` 與 `/MAV1/fmu/out/vehicle_local_position_v1` 同方向且即時；若不同，先回到 coordinate frame probe。
