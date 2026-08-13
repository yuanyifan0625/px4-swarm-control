# Ticket 11d PX4 speed profile 手動驗證

目的：在不修改 PX4-Autopilot、`px4_msgs`、`vehicle_node`、`follower_controller` 的情況下，先檢查三台 PX4 的速度/加速度/yaw 參數，再由 operator 明確套用 `slow_demo`。

## 重要觀念

- `check` 只產生或比對目前值，不會修改 PX4。
- `apply --yes` 只產生明確的 PX4 shell `param set` / `param save` 指令；operator 需要貼到每台 PX4 terminal。
- 請在起飛前套用 speed profile，不要在飛行中臨時改飛控參數。
- QGC 只作為 optional monitoring，不是第一版控制入口。

## 0. 清乾淨 runtime

在 outer workspace 執行：

```bash
docker compose exec ros2_jazzy bash -lc "pkill -x MicroXRCEAgent || true"
docker compose exec ros2_jazzy bash -lc "pkill -f '[b]uild/px4_sitl_default/bin/px4' || true"
docker compose exec ros2_jazzy bash -lc "pkill -f '[g]z sim' || true"
docker compose exec ros2_jazzy bash -lc "pkill -f '[v]ehicle_node' || true"
docker compose exec ros2_jazzy bash -lc "pkill -f '[g]round_station_node' || true"
docker compose exec ros2_jazzy bash -lc "pkill -f '[o]perator_console' || true"
```

確認沒有殘留：

```bash
docker compose exec ros2_jazzy bash -lc "pgrep -af 'MicroXRCEAgent|build/px4_sitl_default/bin/px4|gz sim|ros2 run px4_swarm_control' || true"
```

## 1. 建置 ROS 2 package

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24/px4_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select px4_swarm_control"
```

## 2. 啟動 bridge 與三台 PX4/Gazebo

Terminal DDS：

```bash
docker compose exec ros2_jazzy bash -lc "MicroXRCEAgent udp4 -p 8888 | tee /tmp/microxrceagent_8888.log"
```

Terminal PX4-1：

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24/PX4-Autopilot && PX4_GZ_NO_FOLLOW=1 PX4_UXRCE_DDS_NS=vehicle_1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1"
```

Terminal PX4-2：

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24/PX4-Autopilot && PX4_GZ_NO_FOLLOW=1 PX4_UXRCE_DDS_NS=vehicle_2 PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='0,2,0' PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 2"
```

Terminal PX4-3：

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24/PX4-Autopilot && PX4_GZ_NO_FOLLOW=1 PX4_UXRCE_DDS_NS=vehicle_3 PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='0,-2,0' PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 3"
```

## 3. 產生 check 指令

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24/px4_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run px4_swarm_control px4_speed_profile print-check-commands --profile slow_demo"
```

把輸出的 `param show ...` 分別貼到三個 PX4 terminal。通過條件：三台都能列出 `MPC_XY_VEL_MAX`、`MPC_Z_VEL_MAX_UP`、`MPC_Z_VEL_MAX_DN`、`MPC_ACC_HOR`、`MPC_JERK_AUTO`、`MPC_YAWRAUTO_MAX`、`MPC_YAWRAUTO_ACC`。

## 4. 明確套用 slow_demo

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24/px4_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run px4_swarm_control px4_speed_profile apply --profile slow_demo --yes"
```

確認終端機有中文警告，然後把每台 vehicle 區塊的 `param set ...` 與 `param save` 貼到對應 PX4 terminal。

## 5. 再次檢查

重跑第 3 步並再次貼 `param show ...`。通過條件：

- 三台 `MPC_XY_VEL_MAX` 都是 `2.0`
- 三台 `MPC_Z_VEL_MAX_UP` 都是 `1.0`
- 三台 `MPC_Z_VEL_MAX_DN` 都是 `0.8`
- 三台 `MPC_ACC_HOR` 都是 `2.0`
- 三台 `MPC_JERK_AUTO` 都是 `1.0`
- 三台 `MPC_YAWRAUTO_MAX` 都是 `25`
- 三台 `MPC_YAWRAUTO_ACC` 都是 `10`

## 6. 跑 operator console demo 觀察資訊流

Terminal vehicle/ground station 請依 ticket 11b 文件啟動三個 `vehicle_node` 和 `ground_station_node`。然後執行：

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24/px4_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run px4_swarm_control operator_console --command 9 --ros-args --params-file /home/ncrl/docker_ubuntu24/px4_ws/src/px4_swarm_control/config/operator_console.yaml"
```

通過條件：

- Gazebo 看到三台飛機起飛、leader 移動、yaw 轉向、vee/line_abreast 切換、回到 home、最後降落。
- `/vehicle_1/status`、`/vehicle_2/status`、`/vehicle_3/status` 最後都顯示 `vehicle_state: landed`、`armed: false`。
- `operator_console` demo 沒有直接發布 follower absolute target；followers 仍只靠 leader status 與 formation mode 計算 setpoint。
