# 最終版：SITL 模擬端 ROS 2 手動驗證

假設你已經進入 container，且目前在 `/home/ncrl/docker_ubuntu24`。本流程只把 PX4/Gazebo/DDS 當外部前置，`swarm_nodes.launch.py` 只啟動 ROS 2 swarm nodes。

## Operation profile

- command `1`：takeoff/staging 高度 `1.5 m`
- command `2` / `3`：leader x/y 單軸移動 `1.0 m`
- command `x` / `y`：leader x/y 單軸反方向移動 `1.0 m`
- command `5`：leader yaw `30 deg`
- command `c`：leader yaw `-30 deg`
- `vee`：邊長 `0.8 m` 正三角形，`vee_lateral=0.4`、`vee_trail=0.6928`
- `line_abreast`：leader 到左右 follower 各 `0.8 m`
- formation/settle 完成 tolerance：預設 `0.10 m`，不穩時改 `0.12 m`，最後 fallback `0.15 m`
- settle stable duration：`1.5 s`

調整位置：

- vehicle/ground-station geometry：`px4_ws/src/px4_swarm_control/config/three_vehicle_nodes.yaml`
- console 指令步距與 settle tolerance：`px4_ws/src/px4_swarm_control/config/operator_console.yaml`

## 0. 清乾淨 runtime

```bash
pkill -x MicroXRCEAgent || true
pkill -f '[b]uild/px4_sitl_default/bin/px4' || true
pkill -f '[g]z sim' || true
pkill -f '[v]ehicle_node' || true
pkill -f '[g]round_station_node' || true
pkill -f '[s]warm_nodes.launch.py' || true
pkill -f '[o]perator_console' || true

pgrep -af '[M]icroXRCEAgent|[b]uild/px4_sitl_default/bin/px4|[g]z sim|[v]ehicle_node|[g]round_station_node|[s]warm_nodes.launch.py|[o]perator_console' || true
```

驗收條件：最後一行沒有列出殘留 runtime process。調整：若有殘留，先手動停止該 process 再繼續。

## 1. Build ROS 2 packages

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_swarm_interfaces px4_swarm_control
source install/setup.bash
```

驗收條件：兩個 package build 成功。調整：若改過 interface 或 config，重新執行本步驟。

## 2. 啟動 Micro XRCE-DDS Agent

Terminal DDS：

```bash
cd /home/ncrl/docker_ubuntu24
MicroXRCEAgent udp4 -p 8888 2>&1 | tee /tmp/microxrceagent_8888.log
```

驗收條件：Agent 持續等待 PX4 clients。調整：若 port 被占用，先回第 0 步清 runtime。

## 3. 啟動三台 PX4/Gazebo

Terminal MAV1：

```bash
cd /home/ncrl/docker_ubuntu24/PX4-Autopilot
PX4_GZ_NO_FOLLOW=1 PX4_UXRCE_DDS_NS=MAV1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1
```

Terminal MAV2：

```bash
cd /home/ncrl/docker_ubuntu24/PX4-Autopilot
PX4_GZ_NO_FOLLOW=1 PX4_UXRCE_DDS_NS=MAV2 PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='0,2,0' PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 2
```

Terminal MAV3：

```bash
cd /home/ncrl/docker_ubuntu24/PX4-Autopilot
PX4_GZ_NO_FOLLOW=1 PX4_UXRCE_DDS_NS=MAV3 PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='0,-2,0' PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 3
```

驗收條件：Gazebo 看到 `x500_1`、`x500_2`、`x500_3`。調整：namespace 必須是 `MAV1/MAV2/MAV3`，不要改成 `/vehicle_N`。

## 4. 檢查 PX4 bridge topics

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run px4_swarm_control check_live_px4_gz_bridge --agent-log /tmp/microxrceagent_8888.log
```

驗收條件：三台 Gazebo model、三組 ROS 2 PX4 publishers、三個 Agent sessions 都 OK。調整：若缺 topic，回第 0 步並確認 `PX4_UXRCE_DDS_NS=MAV1/MAV2/MAV3`。

## 5. 啟動 ROS 2 swarm nodes

Terminal launch：

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch px4_swarm_control swarm_nodes.launch.py
```

明確測 `0.10 m` 時：

```bash
ros2 launch px4_swarm_control swarm_nodes.launch.py formation_position_tolerance_m:=0.10
```

驗收條件：launch 只啟動 `/MAV1`、`/MAV2`、`/MAV3` 的 `vehicle_node` 和 `/swarm` 的 `ground_station_node`。調整：若 `0.10 m` 太嚴格，可用 `ros2 launch px4_swarm_control swarm_nodes.launch.py formation_position_tolerance_m:=0.12`，最後 fallback `formation_position_tolerance_m:=0.15`；若一開始 telemetry stale，先停 launch，回第 4 步確認 bridge。

## 6. 檢查 swarm topics/actions

Terminal check：

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo --once /MAV1/status
ros2 topic echo --once /MAV2/status
ros2 topic echo --once /MAV3/status
ros2 action list | grep /swarm
```

驗收條件：三個 status topic 都有輸出，且 `/swarm/arm`、`/swarm/takeoff`、`/swarm/move_leader`、`/swarm/change_formation`、`/swarm/pause`、`/swarm/land` 存在。調整：若 action 缺失，檢查第 5 步 launch terminal。

## 7. 手動 console 指令 smoke

Terminal console：

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run px4_swarm_control operator_console
```

若要用 launch 調整 settle：

```bash
ros2 launch px4_swarm_control operator_console.launch.py settle_position_tolerance_m:=0.10 settle_stable_duration_s:=1.5
```

直接隊形切換 smoke，建議依序輸入：

```text
s
0
1
settle
6
settle
7
settle
6
settle
8
```

驗收條件：

- `0` arm-only 經 `/swarm/arm` 完成；若 PX4 preflight auto-disarm，屬正常現象。
- `1` 起飛到約 `1.5 m` 並完成 staging。
- `1 -> 6` 不需要先手動 jog；`vee` 在 `0.10 m` tolerance 和 `1.5 s` stable duration 後 settle。
- `7` 形成 `line_abreast`，`6` 回到 `vee`，每次都能 settle。
- `8` land 成功。

world-frame jog smoke 可另外輸入：

```text
1
2
x
settle
3
y
settle
4
z
settle
5
c
settle
7
settle
6
settle
8
```

驗收條件：`2/x/3/y/4/z/5/c` 分別完成 leader world-frame 正反向小步移動、升降或 yaw。

調整：步距與 yaw 改 `operator_console.yaml`；隊形距離和 ground-station formation tolerance 改 `three_vehicle_nodes.yaml`；console settle tolerance 和 stable duration 改 `operator_console.yaml`，或用 `operator_console.launch.py` 的 `settle_position_tolerance_m:=0.12` / `0.15` fallback。

## 8. 完整 demo macro

建議在第 7 步 landed 後再跑：

```text
9
```

驗收條件：console 回報 `OK: demo macro completed`，流程包含 takeoff、leader x move、yaw、`line_abreast`、`vee`、home yaw、home、land。調整：macro sequence 在 `operator_console.yaml` 的 `demo_commands`。

## 9. 最終 landed 檢查

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo --once /MAV1/status
ros2 topic echo --once /MAV2/status
ros2 topic echo --once /MAV3/status
```

驗收條件：三台最後都顯示 `vehicle_state: landed` 且 `armed: false`。調整：若 landed 後下一輪 arm 被拒絕，等待 land-complete recovery 或檢查 `pre_flight_checks_pass`、`offboard_control_signal_lost`。
