# 實機座標監看與逐步驗證

## 固定契約

- 場地座標：+X=North/forward、+Y=West/left、+Z=Up。
- 實測 PX4 local：x=East、y=South、z=Down。
- yaw：0=North、+pi/2=West、pi=South、-pi/2=East。
- MAV1、MAV2、MAV3 共用同一個 origin 與上述軸向。

`operator_console` 是唯一人工控制入口。它只在 console seam 將場地位移轉一次：
field +X -> PX4 -y、field +Y -> PX4 -x、field up -> PX4 -z。

## 必看話題

所有 ROS 2 指令必須在 Docker container 內執行。先在 host 開啟一個 shell：

```bash
docker compose exec ros2_jazzy bash -lc "cd /home/ncrl/docker_ubuntu24/px4_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && bash"
```

以下命令都在該 container shell 內執行。每台飛機都開啟下列監看；`MAV1` 的
leader-goal 只需監看一次：

```bash
ros2 topic echo /MAV1/status
ros2 topic echo /MAV2/status
ros2 topic echo /MAV3/status
ros2 topic echo /MAV1/fmu/out/vehicle_local_position_v1
ros2 topic echo /MAV2/fmu/out/vehicle_local_position_v1
ros2 topic echo /MAV3/fmu/out/vehicle_local_position_v1
ros2 topic echo /MAV1/fmu/in/trajectory_setpoint
ros2 topic echo /MAV2/fmu/in/trajectory_setpoint
ros2 topic echo /MAV3/fmu/in/trajectory_setpoint
ros2 topic echo /MAV1/fmu/in/vehicle_command
ros2 topic echo /MAV2/fmu/in/vehicle_command
ros2 topic echo /MAV3/fmu/in/vehicle_command
ros2 topic echo /MAV1/fmu/out/vehicle_command_ack
ros2 topic echo /MAV2/fmu/out/vehicle_command_ack
ros2 topic echo /MAV3/fmu/out/vehicle_command_ack
ros2 topic echo /MAV1/fmu/out/failsafe_flags
ros2 topic echo /MAV2/fmu/out/failsafe_flags
ros2 topic echo /MAV3/fmu/out/failsafe_flags
ros2 topic echo /swarm/leader_goal
ros2 topic echo /swarm/formation_mode
```

實飛時以 OptiTrack 同步觀察實際 North/West/Up 位移；每一步完成、位置穩定後才進下一步。

## 驗證順序

1. 確認三台 `/status` 與 `vehicle_local_position_v1` 皆持續更新且座標可互相比較；確認沒有 failsafe、telemetry stale 或意外的 command ack rejection。
2. 確認三台都在低風險測試區域、人工 safety pilot 已就位且可隨時 land。`0` 與 `1` 是全 swarm 指令；用 `operator_console` 執行後，確認三台的 `vehicle_command` 與 ack 正確，起飛目標維持 0.5 m。
3. 保持 MAV1 yaw=0，依序執行 `2`、`x`、`3`、`y`、`4`、`z`。每一步比對：console 意圖、`/swarm/leader_goal`、MAV1 trajectory setpoint、MAV1 local telemetry、OptiTrack 實測。預期為 +X/-X 改變 y 的 -/+ 向；+Y/-Y 改變 x 的 -/+ 向；up/down 改變 z 的 -/+ 向。
4. 以 `5` 或 `c` 將 MAV1 分別置於 0、+pi/2、pi、-pi/2；每個 cardinal yaw 都重做小步的 `2` 與 `3`，確認場地固定軸不隨 yaw 旋轉。
5. 三機保持安全間距後，執行 `6` 驗證 VEE，接著 `7` 驗證 LINE_ABREAST。每次確認 MAV2 是 leader 左側、MAV3 是 leader 右側，並將 telemetry 與 OptiTrack 同時記錄。
6. 在安全高度、充足間距與人工 safety pilot 監看下，驗證 formation switching、`p` pause、`r` resume 及 collision-safety fallback。pause/resume 後不應自動續跑舊 leader goal；fallback 應維持或往當前 slot 的 body-left/right 安全方向移動。
7. 執行 `8` landing；確認每台收到 land command、PX4 回報 landed，且降落期間不再由 ROS 2 trajectory setpoint 搶控制。

任一步的 raw PX4 telemetry、`/MAVx/status`、trajectory setpoint 或 OptiTrack 結果不符合契約時，立即 pause/land，停止後續編隊與 fallback 測試並保留話題記錄。
