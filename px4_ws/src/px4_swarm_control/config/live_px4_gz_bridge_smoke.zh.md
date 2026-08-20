# Ticket 05b：真實 PX4 Gz 三機 bridge 手動驗證

目標是確認「真的有三台 PX4/Gazebo 飛機」和「ROS 2 真的收到 PX4 publisher」，不是只看到三個 `vehicle_node` subscriber 造成 topic 名稱出現。

所有指令都假設你已經進入 container，且目前 workspace 在：

```bash
/home/ncrl/docker_ubuntu24
```

## Terminal 1：啟動 Micro XRCE-DDS Agent

```bash
cd /home/ncrl/docker_ubuntu24
MicroXRCEAgent udp4 -p 8888 | tee /tmp/microxrceagent_8888.log
```

通過條件：Agent 持續執行，等三台 PX4 啟動後應該會看到三個 client session，log 也會被存到 `/tmp/microxrceagent_8888.log`。

## Terminal 2：先 build PX4 SITL

```bash
cd /home/ncrl/docker_ubuntu24/PX4-Autopilot
make px4_sitl
```

通過條件：產生 `build/px4_sitl_default/bin/px4`。

## Terminal 3：啟動 MAV1，並讓它開 Gazebo

```bash
cd /home/ncrl/docker_ubuntu24/PX4-Autopilot
PX4_UXRCE_DDS_NS=MAV1 \
PX4_SYS_AUTOSTART=4001 \
PX4_SIM_MODEL=gz_x500 \
./build/px4_sitl_default/bin/px4 -i 1
```

通過條件：Gazebo 開啟，並出現 `x500_1`。

## Terminal 4：啟動 MAV2，加入同一個 Gazebo world

```bash
cd /home/ncrl/docker_ubuntu24/PX4-Autopilot
PX4_UXRCE_DDS_NS=MAV2 \
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="0,2,0" \
PX4_SIM_MODEL=gz_x500 \
./build/px4_sitl_default/bin/px4 -i 2
```

通過條件：Gazebo 裡新增 `x500_2`，位置和第一台有水平距離。

## Terminal 5：啟動 MAV3，加入同一個 Gazebo world

```bash
cd /home/ncrl/docker_ubuntu24/PX4-Autopilot
PX4_GZ_NO_FOLLOW=1 \
PX4_UXRCE_DDS_NS=MAV3 \
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="0,-2,0" \
PX4_SIM_MODEL=gz_x500 \
./build/px4_sitl_default/bin/px4 -i 3
```

通過條件：Gazebo 裡新增 `x500_3`，位置和其他兩台有水平距離。

三個 PX4 terminal 各自出現 `pxh>` 後，都輸入：

```text
param set NAV_DLL_ACT 0
```

PX4 v1.17 的 x500 profile 預設要求 GCS data link；本流程只允許 ROS 2
作為控制入口，因此以這個 SITL-only 參數解除 GCS arming requirement。三台都應回報
`Ready for takeoff!`，否則不得繼續。

## Terminal 6：確認 Gazebo 真的有三台

```bash
gz topic -l | grep -E '/model/x500_[123]'
gz topic -e -t /world/default/pose/info -n 1
```

通過條件：輸出中可以看到 `x500_1`、`x500_2`、`x500_3` 相關 topic，pose output 也要顯示三台的水平位置不是擠在一起。

## Terminal 6：確認 ROS 2 真的有 PX4 publisher

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic info -v /MAV1/fmu/out/vehicle_local_position_v1
ros2 topic info -v /MAV2/fmu/out/vehicle_local_position_v1
ros2 topic info -v /MAV3/fmu/out/vehicle_local_position_v1
ros2 topic info -v /MAV1/fmu/out/vehicle_status_v1
ros2 topic info -v /MAV2/fmu/out/vehicle_status_v1
ros2 topic info -v /MAV3/fmu/out/vehicle_status_v1
ros2 topic info -v /MAV1/fmu/out/vehicle_command_ack
ros2 topic info -v /MAV2/fmu/out/vehicle_command_ack
ros2 topic info -v /MAV3/fmu/out/vehicle_command_ack
```

通過條件：每個 topic 都要看到 `Publisher count: 1`。如果只有 `Subscription count`，代表只是 ROS 2 node 訂閱造成 topic 出現，不代表 PX4 有送資料。

## Terminal 6：使用 package smoke checker

完成 `colcon build --packages-select px4_swarm_control` 後，可以用：

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run px4_swarm_control check_live_px4_gz_bridge
```

通過條件：它回報三個 Gazebo model、三台所有必要的 PX4 v1.17 telemetry
publishers、正確 message type 與 bare-DDS endpoint identity 都存在，且 exit code 為
`0`。

如果 Terminal 1 有用 `tee` 存 Agent log，請用這個版本一起驗證三個 PX4 client session：

```bash
ros2 run px4_swarm_control check_live_px4_gz_bridge \
  --agent-log /tmp/microxrceagent_8888.log
```

## 本 ticket 固定的 convention

- 專案 ROS 2 namespace 使用 `/MAV1`、`/MAV2`、`/MAV3`。
- PX4 用 `PX4_UXRCE_DDS_NS=MAVN` 對齊這個 namespace convention。
- PX4 v1.17 topic suffix 由 message version 決定：`vehicle_local_position_v1`、`vehicle_status_v1` 有 `_v1`，version 0 的 `vehicle_command_ack` 沒有 suffix。
- 使用 `-i 1`、`-i 2`、`-i 3` 啟動 PX4 時，第一版 `px4_target_system` 對應為 `2`、`3`、`4`。
