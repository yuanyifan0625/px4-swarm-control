# Ticket 12：完整 SITL smoke workflow

目的：用同一套可重複流程驗證第一版三機 swarm。`swarm_nodes.launch.py` 只啟動 ROS 2 control nodes；PX4 SITL、Gazebo、Micro XRCE-DDS Agent、operator console、slow_demo speed profile 都由 operator 在不同終端機明確啟動。

## 0. 清乾淨 runtime

在 outer workspace 開一個終端機執行：

```bash
docker compose exec ros2_jazzy bash -lc "pkill -x MicroXRCEAgent || true"
docker compose exec ros2_jazzy bash -lc "pkill -f '[b]uild/px4_sitl_default/bin/px4' || true"
docker compose exec ros2_jazzy bash -lc "pkill -f '[g]z sim' || true"
docker compose exec ros2_jazzy bash -lc "pkill -f '[v]ehicle_node' || true"
docker compose exec ros2_jazzy bash -lc "pkill -f '[g]round_station_node' || true"
docker compose exec ros2_jazzy bash -lc "pkill -f '[s]warm_nodes.launch.py' || true"
docker compose exec ros2_jazzy bash -lc "pkill -f '[o]perator_console' || true"
```

確認沒有殘留：

```bash
docker compose exec ros2_jazzy bash -lc "pgrep -af '[M]icroXRCEAgent|[b]uild/px4_sitl_default/bin/px4|[g]z sim|[v]ehicle_node|[g]round_station_node|[s]warm_nodes.launch.py|[o]perator_console' || true"
```

通過條件：沒有列出仍在運行的 DDS、PX4 SITL、Gazebo 或 swarm ROS nodes。

## 1. 建置 ROS 2 packages

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24/px4_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select px4_swarm_interfaces px4_swarm_control"
```

通過條件：`px4_swarm_interfaces` 和 `px4_swarm_control` build 成功。

## 2. 啟動 Micro XRCE-DDS Agent

Terminal DDS：

```bash
docker compose exec ros2_jazzy bash -lc "MicroXRCEAgent udp4 -p 8888 | tee /tmp/microxrceagent_8888.log"
```

通過條件：terminal 顯示 Agent 已在 UDP 8888 等待 PX4 clients。

## 3. 啟動三台 PX4/Gazebo gz_x500

Terminal PX4-1：

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24/PX4-Autopilot && PX4_GZ_NO_FOLLOW=1 PX4_UXRCE_DDS_NS=MAV1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1"
```

Terminal PX4-2：

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24/PX4-Autopilot && PX4_GZ_NO_FOLLOW=1 PX4_UXRCE_DDS_NS=MAV2 PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='0,2,0' PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 2"
```

Terminal PX4-3：

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24/PX4-Autopilot && PX4_GZ_NO_FOLLOW=1 PX4_UXRCE_DDS_NS=MAV3 PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='0,-2,0' PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 3"
```

通過條件：Gazebo 看到 `x500_1`、`x500_2`、`x500_3` 三台飛機，且初始水平位置分開。

## 4. 檢查 bridge 與 ROS 2 PX4 publishers

Terminal check：

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24/px4_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run px4_swarm_control check_live_px4_gz_bridge --agent-log /tmp/microxrceagent_8888.log"
```

通過條件：

- `Gazebo models OK: x500_1, x500_2, x500_3`
- `Gazebo model pose separation OK`
- `ROS 2 PX4 publishers OK for all vehicle telemetry topics`
- Agent log 沒有缺少三個 established sessions。

## 5. slow_demo speed profile：先 check

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24/px4_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run px4_swarm_control px4_speed_profile print-check-commands --profile slow_demo"
```

把輸出的 `param show ...` 分別貼到三個 PX4 terminal。通過條件：三台都能顯示 `MPC_XY_VEL_MAX`、`MPC_Z_VEL_MAX_UP`、`MPC_Z_VEL_MAX_DN`、`MPC_ACC_HOR`、`MPC_JERK_AUTO`、`MPC_YAWRAUTO_MAX`、`MPC_YAWRAUTO_ACC`。

## 6. slow_demo speed profile：明確 apply

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24/px4_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run px4_swarm_control px4_speed_profile apply --profile slow_demo --yes"
```

通過條件：terminal 先出現中文警告，再列出每台 vehicle 的 `param set ...` 和 `param save`。把對應區塊貼到對應 PX4 terminal；這一步是明確套用，不會由 launch 自動做。

## 7. slow_demo speed profile：re-check

重跑第 5 步的 `print-check-commands --profile slow_demo`，再把 `param show ...` 貼到三個 PX4 terminal。

通過條件：

- 三台 `MPC_XY_VEL_MAX` 都是 `2.0`
- 三台 `MPC_Z_VEL_MAX_UP` 都是 `1.0`
- 三台 `MPC_Z_VEL_MAX_DN` 都是 `0.8`
- 三台 `MPC_ACC_HOR` 都是 `2.0`
- 三台 `MPC_JERK_AUTO` 都是 `1.0`
- 三台 `MPC_YAWRAUTO_MAX` 都是 `25`
- 三台 `MPC_YAWRAUTO_ACC` 都是 `10`

## 8. 啟動 swarm ROS nodes launch

Terminal launch：

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24/px4_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch px4_swarm_control swarm_nodes.launch.py"
```

通過條件：

- `/MAV1/status`、`/MAV2/status`、`/MAV3/status` 逐步出現。
- `/swarm/takeoff`、`/swarm/move_leader`、`/swarm/change_formation`、`/swarm/pause`、`/swarm/land` actions 存在。
- launch terminal 沒有啟動 PX4 SITL、Gazebo、Micro XRCE-DDS Agent、QGC、`operator_console` 或 `px4_speed_profile`。

可用下列指令確認：

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24/px4_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 action list | grep /swarm && ros2 topic list | grep -E '/MAV[123]/status|/swarm/(leader_goal|formation_mode)'"
```

## 9. 開 operator console 執行 demo macro

Terminal console：

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24/px4_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run px4_swarm_control operator_console --command 9 --ros-args --params-file /home/ncrl/docker_ubuntu24/px4_ws/src/px4_swarm_control/config/operator_console.yaml"
```

通過條件：

- console 最後輸出 `OK: demo macro completed`。
- ground station 曾回報或 action message 顯示 `all vehicles reached staging positions`。
- Gazebo 看到三台起飛到 staging、leader 移動、yaw 轉向、`vee` / `line_abreast` 切換、回 home、最後降落。
- followers 的移動來自各自 `vehicle_node` 根據 `/MAV1/status` 和 `/swarm/formation_mode` 計算，不是 operator 或 ground station 持續發布 follower absolute target。

## 10. 確認最後 landed 狀態

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24/px4_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 topic echo --once /MAV1/status && ros2 topic echo --once /MAV2/status && ros2 topic echo --once /MAV3/status"
```

通過條件：

- `/MAV1/status`、`/MAV2/status`、`/MAV3/status` 都顯示 `vehicle_state: landed`
- 三台都顯示 `armed: false`
- Gazebo 中三台飛機都已在地面停止。

## 11. 清乾淨 runtime

Demo 結束後回到 outer workspace 執行：

```bash
docker compose exec ros2_jazzy bash -lc "pkill -x MicroXRCEAgent || true"
docker compose exec ros2_jazzy bash -lc "pkill -f '[b]uild/px4_sitl_default/bin/px4' || true"
docker compose exec ros2_jazzy bash -lc "pkill -f '[g]z sim' || true"
docker compose exec ros2_jazzy bash -lc "pkill -f '[v]ehicle_node' || true"
docker compose exec ros2_jazzy bash -lc "pkill -f '[g]round_station_node' || true"
docker compose exec ros2_jazzy bash -lc "pkill -f '[s]warm_nodes.launch.py' || true"
docker compose exec ros2_jazzy bash -lc "pkill -f '[o]perator_console' || true"
docker compose exec ros2_jazzy bash -lc "pgrep -af '[M]icroXRCEAgent|[b]uild/px4_sitl_default/bin/px4|[g]z sim|[v]ehicle_node|[g]round_station_node|[s]warm_nodes.launch.py|[o]perator_console' || true"
```

通過條件：最後一行沒有列出殘留 runtime process。這代表下次 demo 可以從 clean runtime 重新開始。
