# Ticket 15/16 Hack Notes

這份筆記不是正式使用手冊。用途是下次回來時快速接手 ticket 15、16 的續工、debug 與速度 sweep。

## 1. 目前已落地的東西

### ticket 15

- 新增 `coordinate_frame_probe`
- 目的：先確認 PX4 raw local NED、`/MAV*/status`、Gazebo world frame 之間的對應
- SITL 已確認：
  - PX4 `+X -> Gazebo +Y`
  - PX4 `+Y -> Gazebo +X`
  - PX4 `-Z -> Gazebo +Z`
- `manual` mode 不送 action，適合實機 preflight
- `commanded` mode 走 `/swarm/move_leader`，適合 SITL

主要檔案：

- `px4_ws/src/px4_swarm_control/px4_swarm_control/coordinate_frame_probe.py`
- `px4_ws/src/px4_swarm_control/config/final_sitl_coordinate_frame_command_probe.zh.md`
- `px4_ws/src/px4_swarm_control/config/final_real_coordinate_frame_manual_probe.zh.md`

### ticket 16

- 新增 `field_frame_console`
- 目的：把 human field frame 指令轉成既有 `/swarm` actions
- 預設 mapping 是 Gazebo visual profile：
  - field `+X -> PX4 +Y`
  - field `+Y -> PX4 +X`
  - field up `-> PX4 -Z`
- `operator_console` 仍保留 raw PX4 local NED world-frame jog
- `field_frame_console` 現在已支援：
  - `s/p/r`
  - `0/1/2/x/3/y/4/z/5/c/6/7/8/9`
  - `settle`
  - `home`
  - `home_yaw`

主要檔案：

- `px4_ws/src/px4_swarm_control/px4_swarm_control/field_frame_console.py`
- `px4_ws/src/px4_swarm_control/config/final_sitl_field_frame_console_command.zh.md`
- `px4_ws/src/px4_swarm_control/config/final_real_field_frame_console_manual.zh.md`

## 2. 先看哪些檔

如果目標是座標系：

- `coordinate_frame_probe.py`
- `field_frame_console.py`
- `CONTEXT.md`

如果目標是 demo / settle / followers 跟不上：

- `operator_console.py`
- `ground_station_node.py`
- `config/operator_console.yaml`
- `config/three_vehicle_nodes.yaml`

如果目標是速度 / 加速度：

- `px4_speed_profile.py`
- `config/px4_speed_profiles/slow_demo.yaml`
- `config/px4_speed_profiles/real_cautious.yaml`
- `.scratch/px4-swarm-control/issues/11d-add-px4-speed-profile-check-and-apply-workflow.md`

## 3. 已知風險

- `field_frame_console 9 demo` 不是固定失敗；它目前是 state-dependent。
- 已驗證在乾淨 `landed` 起始狀態下，`--command 9` 可以完成。
- 曾經出現的 timeout 是 `takeoff staging timed out`，來源在 `ground_station_node.py`，不是 field-frame mapping 本身。
- 這通常代表 demo 開始時 PX4 / vehicle status 還沒回到乾淨可起飛狀態。
- `field_frame_console` 的 `9` 第一個步驟仍然是 `TakeoffSwarm`，所以如果上一輪降落後還殘留 `auto_land`、`offboard_control_signal_lost`、stale telemetry，這一步就可能 timeout。
- PX4 速度參數不應寫進 `three_vehicle_nodes.yaml`。那裡是 ROS node identity / geometry，不是 flight-controller tuning。

## 4. 快速驗證指令

### build / source

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_swarm_interfaces px4_swarm_control
source install/setup.bash
```

### field frame probe

```bash
ros2 run px4_swarm_control coordinate_frame_probe --ros-args \
  -p mode:=commanded \
  -p px4_namespace:=/MAV1
```

### field frame console

```bash
ros2 run px4_swarm_control field_frame_console --ros-args \
  -p field_x_axis:=px4_y \
  -p field_x_sign:=positive \
  -p field_y_axis:=px4_x \
  -p field_y_sign:=positive \
  -p field_up_axis:=px4_z \
  -p field_up_sign:=negative
```

### 先清乾淨再跑 demo

```bash
ros2 run px4_swarm_control field_frame_console --command 8
ros2 topic echo /MAV1/status --once
ros2 topic echo /MAV2/status --once
ros2 topic echo /MAV3/status --once
ros2 run px4_swarm_control field_frame_console --command 9 --ros-args \
  -p field_x_axis:=px4_y \
  -p field_x_sign:=positive \
  -p field_y_axis:=px4_x \
  -p field_y_sign:=positive \
  -p field_up_axis:=px4_z \
  -p field_up_sign:=negative
```

成功前提：

- 三台都 `vehicle_state: landed`
- `armed: false`
- `pre_flight_checks_pass: true`

### 出問題時第一時間抓的狀態

```bash
ros2 topic echo /MAV1/status --once
ros2 topic echo /MAV2/status --once
ros2 topic echo /MAV3/status --once
ros2 node list | sort
ros2 action list | sort
```

## 5. 速度 / 加速度 sweep 怎麼做

### 先講結論

- 不要一次只改 `MPC_XY_VEL_MAX`
- 至少要一起看：
  - `MPC_XY_VEL_MAX`
  - `MPC_ACC_HOR`
  - `MPC_JERK_AUTO`
  - `MPC_YAWRAUTO_MAX`
  - `MPC_YAWRAUTO_ACC`
- 如果目標是「更準地到點」，通常要同時降低速度、水平加速度、jerk
- 只降速度、不降加速度，常見結果是看起來還是太衝

### 建議測法

固定測試流程，不要每組 profile 用不同指令：

```text
8
確認三台 landed
1
2
settle
x
settle
3
settle
y
settle
5
settle
7
settle
6
settle
8
```

理由：

- 這比直接跑 `9` 更容易看出是哪一段精度差
- 每段後面都有 `settle`
- 可以分離「leader 到點誤差」和「follower 收斂誤差」

### 每組 profile 要記的結果

- leader 最終位置誤差
- follower 是否能進 `settle`
- `settle` 花多久
- 是否出現 overshoot / 震盪
- `takeoff`、`move_leader`、`change_formation` 是否 timeout

### 先測哪些 profile

目前 baseline：

- `slow_demo.yaml`
  - `MPC_XY_VEL_MAX: 2.0`
  - `MPC_ACC_HOR: 2.0`
  - `MPC_JERK_AUTO: 1.0`
  - `MPC_YAWRAUTO_MAX: 25`
  - `MPC_YAWRAUTO_ACC: 10`

建議 sweep 順序：

1. baseline：`2.0 / 2.0 / 1.0 / 25 / 10`
2. 中速保守：`1.5 / 1.5 / 0.8 / 20 / 8`
3. 慢速保守：`1.2 / 1.2 / 0.6 / 18 / 6`
4. 若還太衝，再試：`1.0 / 1.0 / 0.5 / 15 / 5`

不建議一開始就測比 `1.0 m/s` 更低，因為很可能只是在拖長動作時間，而不是明顯改善精度。

## 6. 要不要改 demo 邏輯

### 目前判斷

先不要。

理由：

- `demo_commands` 現在在每個關鍵 movement / formation 後面都已經有 `settle`
- `settle` 本身已經要求 followers 連續穩定 `1.5 s`
- 所以「leader 還沒等 follower 就急著做下一步」這件事，對 `2`、`5`、`7`、`6`、`home_yaw`、`home` 其實已經有 gate

### 真正要先看的不是 demo 宏，而是 timeout

如果把 PX4 調慢，第一個要加的通常不是 demo step，而是 timeout：

- `operator_console.yaml`
  - `default_timeout_sec`
  - `settle_timeout_sec`
- 必要時也看 `ground_station_node` 的 takeoff / move leader timeout 來源

### 什麼情況才需要動 demo_commands

只有在下面其中一種情況成立時，才值得考慮改 demo sequence：

- 明明 `settle` 沒 timeout，但下一步視覺效果還是太趕
- `takeoff` 完成後想額外觀察一段時間，再做第一個 `2`
- 想在 `1` 後面顯式插一個 `settle` 當展示節奏，而不是功能修正

也就是說：

- 若是功能正確性問題，先改 profile 與 timeout
- 若是展示節奏問題，才改 `demo_commands`

## 7. 下次若要繼續做什麼

### 如果目標是 debug `9 demo` timeout

先做：

1. `8`
2. 抓三台 `/MAV*/status`
3. 再跑一次 `--command 9`
4. 若失敗，記錄失敗當下三台 status

重點要看：

- `nav_state`
- `vehicle_state`
- `pre_flight_checks_pass`
- `offboard_control_signal_lost`
- `last_telemetry_age_sec`

### 如果目標是找「哪個速度最準」

先做：

1. 建 3 到 4 組 speed profiles
2. 固定同一套命令序列
3. 每組記錄 leader / follower 誤差與 settle 時間
4. 先找「最慢但不至於拖太久」的甜蜜點

### 如果目標是實機前準備

先做：

1. `coordinate_frame_probe manual`
2. 確認 field frame mapping
3. 再決定 `field_frame_console` 參數
4. 最後才碰 speed profile
