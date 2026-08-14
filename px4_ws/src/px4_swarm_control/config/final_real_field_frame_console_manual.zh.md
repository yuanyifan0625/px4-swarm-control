# 實機 Field-Frame Console Manual 手冊

## 前置條件

假設 PX4、Micro-XRCE-DDS Agent、網路與 ROS_DOMAIN_ID 已完成設定，且 ROS 2 可看到 canonical topics：

```bash
ros2 topic list | grep '^/MAV[123]/fmu/out'
```

必要前置：

- `/MAV1`、`/MAV2`、`/MAV3` 必須存在。
- 不支援 `/vehicle_1`、`/vehicle_2`、`/vehicle_3`。
- `coordinate_frame_probe` manual mode 已確認 raw PX4 local NED 與 `/MAV1/status` 即時一致。
- 已確認飛場 field `+X/+Y/up` 各自對應哪個 PX4 local NED 軸與方向。
- 不要假設 SITL 的 Gazebo visual profile 等於實機飛場座標。

驗收條件：`/MAV1/fmu/out/vehicle_local_position_v1`、`/MAV2/fmu/out/vehicle_status_v4`、`/MAV3/fmu/out/vehicle_command_ack_v1` 都可 echo。

## 啟動 swarm 節點

地面站：

```bash
ros2 launch px4_swarm_control real_ground_station.launch.py \
  formation_position_tolerance_m:=0.10
```

三台樹莓派分別啟動：

```bash
ros2 launch px4_swarm_control real_mav1_vehicle.launch.py
ros2 launch px4_swarm_control real_mav2_vehicle.launch.py
ros2 launch px4_swarm_control real_mav3_vehicle.launch.py
```

驗收條件：地面站可看到 `/swarm/*` actions，且三台機體各自發布 `/MAV1/status`、`/MAV2/status`、`/MAV3/status`。

## 啟動 field-frame console

依 coordinate probe 結果覆蓋 mapping。範例：若飛場 field `+X` 對應 PX4 `-Y`，field `+Y` 對應 PX4 `+X`，上升對應 PX4 `-Z`：

```bash
ros2 run px4_swarm_control field_frame_console --ros-args \
  -p field_x_axis:=px4_y \
  -p field_x_sign:=negative \
  -p field_y_axis:=px4_x \
  -p field_y_sign:=positive \
  -p field_up_axis:=px4_z \
  -p field_up_sign:=negative \
  -p move_step_x_m:=0.30 \
  -p move_step_y_m:=0.30 \
  -p altitude_step_m:=0.20 \
  -p settle_position_tolerance_m:=0.10
```

參數用途：

- `field_x_axis` / `field_x_sign`：field 前後方向轉成 PX4 哪個軸。
- `field_y_axis` / `field_y_sign`：field 左右方向轉成 PX4 哪個軸。
- `field_up_axis` / `field_up_sign`：field 上下方向轉成 PX4 哪個軸。
- 預設 Gazebo visual profile 只用於讓 SITL 畫面直覺：field `+X -> PX4 +Y`、field `+Y -> PX4 +X`、field up `-> PX4 -Z`。
- `move_step_*`、`altitude_step_m`：實機初次測試建議先小步長。
- `settle_position_tolerance_m`：若 0.10 m 偶發不穩，先試 0.12 m，最後才放寬到 0.15 m。

驗收條件：help text 顯示目前 mapping；若要 raw PX4 local NED 指令，改用 `operator_console`。

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

驗收條件：每個 movement 指令都只讓 leader 朝預期 field 方向移動；follower 不應往 leader 反方向或錯軸追蹤。

## 實機 smoke test

建議先不跑 `9 demo`，完成小步長後再跑：

```text
0
1
2
x
3
y
4
z
6
settle
7
settle
8
```

全部通過後再輸入：

```text
9
```

驗收條件：起飛高度、前後左右、上下、yaw、vee、line_abreast、land 都符合飛場 field frame 預期，且 `settle` 連續穩定完成。

## Debug 指令

```bash
ros2 topic echo /MAV1/fmu/out/vehicle_local_position_v1 --once
ros2 topic echo /MAV1/status --once
ros2 topic echo /MAV2/status --once
ros2 topic echo /MAV3/status --once
ros2 action list | sort
ros2 node list | sort
```

若 `/MAV1/status` 與 `/MAV1/fmu/out/vehicle_local_position_v1` 不一致，不要使用 field-frame console 飛行；先回到 `coordinate_frame_probe`。若方向一致但 field 方向反了，只調 `field_x_sign`、`field_y_sign` 或 `field_up_sign`，不要修改 PX4-Autopilot 或 `px4_msgs`。
