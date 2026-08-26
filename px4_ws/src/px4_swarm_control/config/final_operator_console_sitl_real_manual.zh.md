# 最終版：operator_console SITL／實機部署與可重複驗證手冊

本手冊是唯一的 swarm 手動控制流程。所有 takeoff、leader movement、formation、pause／resume 與 land 指令只由 `operator_console` 送進既有 `/swarm` actions。以下 SITL 指令假設操作者已在 container 內，起始目錄為 `/home/ncrl/docker_ubuntu24`。

固定控制 frame 為 PX4 local NED；operator 輸入的 field `+X/+Y/up` 只在 `operator_console` 入口轉換。重新 takeoff 時，ground station 必須先收到 fresh MAV1 position/yaw，並以該 pose 作為 VEE staging anchor；缺少或 stale 時 command `1` 會被拒絕。

## 1. Build 與 PX4 speed profile

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_swarm_interfaces px4_swarm_control --symlink-install
source install/setup.bash
ros2 run px4_swarm_control px4_speed_profile check --profile slow_demo
```

在三個 PX4 shell 分別執行 `check` 輸出的 `param show`，記錄 current/desired/match。需要修改時，先由 operator 明確確認後產生命令：

```bash
ros2 run px4_swarm_control px4_speed_profile apply --profile slow_demo --yes
```

將輸出的 `param set` 貼到對應 PX4 shell。目標為 `MPC_XY_VEL_MAX=0.3 m/s`、`MPC_Z_VEL_MAX_UP=0.3 m/s`、`MPC_YAWRAUTO_MAX=30 deg/s`；ROS 2 不另做一般速度限幅。

## 2. SITL：啟動 DDS 與三台 PX4/Gazebo

Terminal DDS：

```bash
cd /home/ncrl/docker_ubuntu24
MicroXRCEAgent udp4 -p 8888 2>&1 | tee /tmp/microxrceagent_8888.log
```

Terminal MAV1（先啟動；確認 `pxh>`、estimator 正常且沒有 sensor timeout 後，才啟動 MAV2）：

```bash
cd /home/ncrl/docker_ubuntu24/PX4-Autopilot
PX4_GZ_NO_FOLLOW=1 PX4_UXRCE_DDS_NS=MAV1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 0
```

Terminal MAV2（確認 `pxh>`、estimator 正常且沒有 sensor timeout 後，才啟動 MAV3）：

```bash
cd /home/ncrl/docker_ubuntu24/PX4-Autopilot
PX4_GZ_NO_FOLLOW=1 PX4_UXRCE_DDS_NS=MAV2 PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='-1,-1,0' PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1
```

Terminal MAV3：

```bash
cd /home/ncrl/docker_ubuntu24/PX4-Autopilot
PX4_GZ_NO_FOLLOW=1 PX4_UXRCE_DDS_NS=MAV3 PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='-1,1,0' PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 2
```

必須依 MAV1、MAV2、MAV3 的順序逐台啟動與確認健康，不可同時啟動 MAV2/MAV3。模型生成時可能短暫出現一次 sensor timeout；若 `Ready for takeoff!` 後仍重複出現 `Accel fail: TIMEOUT`、estimator failure 或 preflight failure，停止流程並保存 log，不可進入 takeoff。沒有 GUI 需求時，可在 MAV1 命令最前方加入 `HEADLESS=1` 降低渲染負載。三個 PX4 terminal 都健康且出現 `pxh>` 後，各執行：

```text
param set NAV_DLL_ACT 0
```

三台都必須回報可起飛，且 DDS log 必須有三個 session。整段驗證期間不要重啟 Gazebo、PX4 或 DDS Agent。

## 3. SITL：啟動 ROS 2 nodes 與唯一 console

Terminal swarm nodes：

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch px4_swarm_control swarm_nodes.launch.py 2>&1 | tee /tmp/swarm_nodes_ticket22.log
```

Terminal operator：

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run px4_swarm_control operator_console
```

檢查 graph 與 raw NED/status：

```bash
ros2 action list | grep /swarm
ros2 topic echo --once /MAV1/fmu/out/vehicle_local_position_v1
ros2 topic echo --once /MAV1/status
ros2 topic echo --once /MAV2/status
ros2 topic echo --once /MAV3/status
```

## 4. 第一輪 operator-console demo

目標流程：`takeoff -> VEE staging -> leader movement -> formation change -> safety hold -> resume -> land`。

在 `swarm>` 依序輸入：

```text
s
1
settle
2
settle
7
settle
6
settle
p
```

`1` 必須使用當下 MAV1 pose 建立 VEE staging，而不是回到固定 `(0,0)`。`2` 是 field `+X` leader movement；`7` 切 line-abreast，`6` 回 VEE。

### SITL-only safety hold 注入

僅在螺旋槳模擬環境執行。保持 `p` 暫停後，使用 Gazebo GUI 的 Move 工具將一台 follower 移到距另一台機體水平小於 `0.7 m`；Gazebo 操作只注入外部擾動，不是 swarm 控制入口。回到 `operator_console` 輸入：

```text
r
s
```

距離低於 `0.7 m` 連續 `1.0 s` 後，受影響 follower 應顯示 safety hold/holding 並凍結最後安全 NED target。再用 Gazebo GUI 將 follower 移回大於等於 `0.7 m`，保持安全距離連續 `1.0 s`，確認自動 resume 為 following。實機不得刻意製造此情境；實機只檢查既有測試與 telemetry fail-closed log。

最後在 `operator_console` 輸入：

```text
settle
8
s
```

三台最後必須為 `vehicle_state: landed` 且 `armed: false`。

## 5. 不重啟 runtime 的第二輪

不重啟 Gazebo、PX4、DDS Agent 或 swarm nodes，也不要關閉 `operator_console`。等待三台持續發布 fresh landed status 後，直接輸入：

```text
1
settle
3
settle
7
settle
6
settle
8
s
```

第二次 `1` 必須以 land 後新的 MAV1 position/yaw 建立 staging targets，且可以再次完成 takeoff 與 land。也可在 landed 後直接輸入其他合法 operator command；不應要求重啟任何 runtime process。

## 6. 驗證記錄

每次 demo 保存：

- PX4 speed profile 的 current/desired/match output。
- `/MAV1/fmu/out/vehicle_local_position_v1` raw NED 與三台 `/MAV*/status`。
- 每次 `settle`、safety hold 進入／解除、`r` resume 的 console/status output。
- 最終 `vehicle_state: landed`、`armed: false`。
- `/tmp/microxrceagent_8888.log`、`/tmp/swarm_nodes_ticket22.log` 與三台 PX4 terminal logs。

若任何 telemetry stale、position/yaw 非 finite、speed profile mismatch 或 action timeout，停止 demo，先保存 log；不要用固定 staging origin 繞過拒絕。

## 7. 實機部署

實機各 ROS 2 主機 source 對應 distro（目前預期 Humble）與本 workspace `install/setup.bash`，並使用相同 `ROS_DOMAIN_ID`。三台樹莓派分別啟動：

```bash
ros2 launch px4_swarm_control real_mav1_vehicle.launch.py
ros2 launch px4_swarm_control real_mav2_vehicle.launch.py
ros2 launch px4_swarm_control real_mav3_vehicle.launch.py
```

地面站啟動：

```bash
ros2 launch px4_swarm_control real_ground_station.launch.py
ros2 run px4_swarm_control operator_console
```

實機先用 `ros2 topic echo --once /MAV1/fmu/out/vehicle_local_position_v1` 與 `/MAV1/status` 確認 NED position/yaw 一致，再依序做 `s -> 1 -> settle -> 2 -> settle -> 7 -> settle -> 6 -> settle -> 8 -> s`。正式飛行只使用 `operator_console`；安全 hold 不以刻意靠近飛機的方式測試。
