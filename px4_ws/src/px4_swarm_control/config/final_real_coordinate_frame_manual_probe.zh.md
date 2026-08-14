# 實機座標軸 Manual Probe 手冊

## 前置條件

假設 PX4 已透過 Micro-XRCE-DDS Agent 轉出 canonical topics：

```text
/MAV1
/MAV2
/MAV3
```

本流程是 preflight diagnostic。第一版 manual probe 不送 `/swarm/move_leader`，也不送任何會移動飛機的 action。

驗收條件：執行前確認飛機安全、螺旋槳處於安全狀態，且定位來源在 disarmed/手持時會更新。若手持時定位不更新，manual probe 無法作為有效判定。

## 啟動環境

```bash
cd <repo>/px4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

實機依目前假設使用 Humble；若在本專案 container/SITL 重跑同流程，改 source `/opt/ros/jazzy/setup.bash`。

驗收條件：`ros2 topic list` 可看到 `/MAV1/fmu/out/vehicle_local_position_v1` 和 `/MAV1/status`。

## 先確認 raw PX4 topic 與 status

```bash
ros2 topic echo /MAV1/fmu/out/vehicle_local_position_v1 --once
ros2 topic echo /MAV1/status --once
```

驗收條件：

- raw PX4 topic 有 `x/y/z`。
- `/MAV1/status` 有 `x/y/z`。
- 兩者數值接近且持續更新。

## 執行 manual probe

```bash
ros2 run px4_swarm_control coordinate_frame_probe --ros-args \
  -p mode:=manual \
  -p px4_namespace:=/MAV1 \
  -p dominant_delta_m:=0.30 \
  -p cross_axis_delta_m:=0.10 \
  -p stable_duration_s:=1.0 \
  -p timeout_s:=15.0
```

參數用途：

- `mode:=manual`：只觀察座標，不送 movement action。
- `px4_namespace:=/MAV1`：檢查 MAV1。
- `dominant_delta_m:=0.30`：該段主要軸至少要變化 0.30 m。
- `cross_axis_delta_m:=0.10`：其他軸變化應小於 0.10 m。
- `stable_duration_s:=1.0`：條件連續成立 1.0 秒才算 PASS。
- `timeout_s:=15.0`：每段最多等待 15 秒。

## 手動動作

工具會依序提示：

```text
Move vehicle along field +X...
Move vehicle along field +Y...
Move vehicle along field up...
```

每段都要等工具印出 PASS、WARNING、ERROR 或 timeout 後，再做下一段。

驗收條件：

- PASS：指定 field 方向對應到預期 PX4 軸，且 cross-axis 小。
- WARNING：偵測到反方向、其他軸 dominant、或 cross-axis 過大；工具會記錄並進下一段。
- ERROR：沒有足夠位移、topic 不更新、或 `/MAV1/status` 與 raw PX4 topic 不一致。

## 三台機器

第一次實機部署建議分別跑：

```bash
ros2 run px4_swarm_control coordinate_frame_probe --ros-args -p mode:=manual -p px4_namespace:=/MAV1
ros2 run px4_swarm_control coordinate_frame_probe --ros-args -p mode:=manual -p px4_namespace:=/MAV2
ros2 run px4_swarm_control coordinate_frame_probe --ros-args -p mode:=manual -p px4_namespace:=/MAV3
```

驗收條件：三台的 raw PX4 local position 與各自 `/MAV*/status` 都一致。

## 安全注意事項

- 實機 ROS 2 看到的是 PX4 local NED frame，不是 Gazebo world frame。
- 不要假設 Gazebo mapping 等於飛場 mapping。
- manual probe 只負責確認座標軸；不會自動修改 YAML 或 field-frame console 參數。
- 若任何一台 status 與 raw PX4 topic 不一致，先停止後續 swarm 操作。
