# PX4 swarm control final manual

此手冊是本 package 唯一安裝的操作文件。所有 swarm command 都經由
`operator_console` 送到 ROS 2 actions；不要以 QGC 作為控制入口。

## Build

在 container 內執行：

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_swarm_interfaces px4_swarm_control
source install/setup.bash
```

## 固定三機 SITL baseline

先啟動 Agent：

```bash
MicroXRCEAgent udp4 -p 8888
```

在 `/home/ncrl/docker_ubuntu24/PX4-Autopilot` 依 MAV1、MAV2、MAV3 順序啟動。
`-d` 避免非互動 shell 反覆輸出 `pxh>`；若需設定 PX4 參數，使用獨立 PX4
shell 執行 `param set NAV_DLL_ACT 0`。

```bash
GZ_IP=127.0.0.1 PX4_GZ_NO_FOLLOW=1 PX4_UXRCE_DDS_NS=MAV1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -d -i 0
GZ_IP=127.0.0.1 PX4_GZ_NO_FOLLOW=1 PX4_UXRCE_DDS_NS=MAV2 PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='-1,1,0' PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -d -i 1
GZ_IP=127.0.0.1 PX4_GZ_NO_FOLLOW=1 PX4_UXRCE_DDS_NS=MAV3 PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='-1,-1,0' PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -d -i 2
```

接著啟動 nodes：

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch px4_swarm_control swarm_nodes.launch.py
```

此 launch 使用 `gazebo_enu_common_world`：上層的 `(x, y, z, yaw)` 是
Gazebo `(East, North, Up)`，`yaw=0` 朝 East、正值朝 North。只有
`Px4VehicleInterface` 會把它轉為各 MAV 自己的 PX4 raw local 座標；MAV2/
MAV3 的 spawn origin 已在 YAML 設定，不能把三台 raw `(0,0,0)` 當作同一個
物理點。

另一 terminal 執行（此 launch 載入固定的 0.5 m 起飛設定）：

```bash
export ROS_DOMAIN_ID=42
ros2 launch px4_swarm_control operator_console.launch.py
```

## Takeoff 驗證

確認 `/MAV1`、`/MAV2`、`/MAV3` 的 local-position 與 status topics 都持續更新。
每台 PX4 使用自己的 local-NED origin；不能將三台 raw `x/y/z` 視為 shared
world coordinates。輸入 console 的 `1` 前，MAV1 必須提供 fresh staging anchor。

起飛時每台先發布 local 垂直 target，再進入 Offboard、ARM，通過 0.1 m height
gate 後才切換完整 staging target。驗收時保留 bounded snapshots：

```bash
ros2 topic echo --once /MAV1/status
ros2 topic echo --once /MAV2/status
ros2 topic echo --once /MAV3/status
ros2 topic echo --once /MAV1/fmu/out/vehicle_local_position_v1
```

確認 `/MAV*/fmu/in/offboard_control_mode` 與
`/MAV*/fmu/in/trajectory_setpoint` 持續出現；`vehicle_command` 只能是 Offboard
mode 與 ARM，絕不可出現 command 22。若 takeoff timeout，GroundStation 會送
PAUSE；RESUME 不會重啟逾時流程。land complete 後，下一輪 takeoff 必須等待 fresh
staging anchor。

## SITL ENU 軸向人工驗收

起飛並確認三台都在穩定 VEE 後，以 `operator_console` 逐項操作 `2`、`x`、
`3`、`y`、`4`、`z`（每次等 setpoint 穩定）。在 Gazebo 觀察 MAV1，並以另一
terminal 對照：

```bash
ros2 topic echo --once /MAV1/status
ros2 topic echo --once /MAV1/fmu/in/trajectory_setpoint
ros2 topic echo --once /MAV1/fmu/out/vehicle_local_position_v1
```

- `2` / `x`：Gazebo 分別 East / West；raw trajectory `y` 分別增加 / 減少。
- `3` / `y`：Gazebo 分別 North / South；raw trajectory `x` 分別增加 / 減少。
- `4` / `z`：Gazebo 分別 Up / Down；raw trajectory `z` 分別減少 / 增加。
- `+` / `-`：比較 status yaw 與 raw `heading`，符合
  `wrap(pi/2 - heading + 0.102)`；MAV1、MAV2、MAV3 都各做一次。

再切換 `5`（VEE）與 `6`（LINE_ABREAST），在 leader yaw `0`、`+pi/2`、
`pi`、`-pi/2` 觀察 follower 仍在物理左／右；安全 fallback 出現時也必須往
遠離 peer 的方向移動。這些是 SITL 驗收，不可將 direct PX4 command 當作正常
操作入口。

## Cleanup

每輪 runtime 結束後，停止這次啟動的 swarm nodes、PX4、Agent 與 Gazebo，再確認：

```bash
pgrep -af '[M]icroXRCEAgent|[b]uild/px4_sitl_default/bin/px4|[g]z sim|[g]zserver' || true
pgrep -af '[v]ehicle_node|[g]round_station_node|[o]perator_console' || true
```
