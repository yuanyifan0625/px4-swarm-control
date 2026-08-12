# Ticket 11b：Operator 短指令 Console 手動 smoke 驗證

目標：第一版 SITL 不依賴 QGC，透過 `operator_console` 用短指令呼叫既有 `/swarm/*` actions，減少手動輸入長 action 指令。Console 只能控制 ground station action surface，不得替 followers 計算或發布 absolute target。

完整任務卡：

```text
clean runtime -> start bridge/PX4/vehicle nodes/ground station -> operator_console
-> 1 takeoff -> 2 leader +x -> 5 yaw +45deg -> 7 line_abreast -> 6 vee
-> p pause -> 2 move rejected while paused -> r resume -> 2 fresh move -> 8 land
```

所有指令都假設你已經進入 container，workspace 在：

```bash
/home/ncrl/docker_ubuntu24
```

## 0. 清乾淨舊 runtime

```bash
pgrep -af '[M]icroXRCEAgent|[g]round_station_node|[v]ehicle_node|[o]perator_console|[p]x4 -i|[g]z sim|[g]zserver|[g]zclient'
pkill -TERM -x MicroXRCEAgent || true
pkill -TERM -x px4 || true
pkill -TERM -x gz || true
pkill -TERM -x gzserver || true
pkill -TERM -x gzclient || true
pgrep -f 'px4_swarm_control/lib/px4_swarm_control/[v]ehicle_node' | xargs -r kill
pgrep -f 'px4_swarm_control/lib/px4_swarm_control/[g]round_station_node' | xargs -r kill
pgrep -f 'px4_swarm_control/lib/px4_swarm_control/[o]perator_console' | xargs -r kill
pgrep -f '[g]z sim' | xargs -r kill
sleep 2
pgrep -af '[M]icroXRCEAgent|[g]round_station_node|[v]ehicle_node|[o]perator_console|[p]x4 -i|[g]z sim|[g]zserver|[g]zclient' || true
```

通過條件：最後一個 `pgrep` 不應看到舊的 `MicroXRCEAgent`、`px4 -i 1/2/3`、`gz sim`、`vehicle_node`、`ground_station_node` 或 `operator_console`。

## 1. 啟動 bridge、PX4 Gz、vehicle nodes、ground station

照 `live_px4_gz_bridge_smoke.zh.md` 啟動：

```text
MicroXRCEAgent -> PX4 instance 1 -> PX4 instance 2 -> PX4 instance 3
```

照 `ticket09_follower_following_manual_smoke.zh.md` 啟動：

```text
vehicle_1 node -> vehicle_2 node -> vehicle_3 node -> ground_station_node
```

確認 bridge：

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run px4_swarm_control check_live_px4_gz_bridge --agent-log /tmp/microxrceagent_8888.log
```

通過條件：

- Gazebo 可看到 `x500_1`、`x500_2`、`x500_3`。
- `ROS 2 PX4 publishers OK for all vehicle telemetry topics`。
- `Micro XRCE-DDS Agent sessions OK`。

## 2. 啟動 operator console

開一個新的終端機：

```bash
cd /home/ncrl/docker_ubuntu24/px4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run px4_swarm_control operator_console --ros-args \
  --params-file src/px4_swarm_control/config/operator_console.yaml
```

Console 會顯示短指令說明：

```text
s=status, p=pause, r=resume, q=quit, h=help
1=takeoff, 2=leader x+1m, 3=leader y+1m, 4=leader up 1m, 5=yaw+45deg
6=vee, 7=line_abreast, 8=land, 9=demo macro
```

注意：PX4 local position 使用 NED 座標，所以 `4` 的「上升 1m」實際會讓 MoveLeader goal 的 `z` 減少 `1.0`。

## 3. 使用 console 起飛

在 console 輸入：

```text
1
```

通過條件：

- Console 輸出 `OK: all vehicles reached staging positions`。
- Gazebo 中三台起飛並到分離 staging。
- `/vehicle_1/status` 約在 `(0, 0, -5)`。
- `/vehicle_2/status` 約在 `(-3, 4, -5)`。
- `/vehicle_3/status` 約在 `(-3, -4, -5)`。

## 4. 使用 console 移動 leader 與旋轉 yaw

在 console 依序輸入：

```text
2
5
```

通過條件：

- `2` 會讀 `/vehicle_1/status`，把目前 leader 位置加上 world-frame `x + 1m`，再送既有 `MoveLeader` absolute goal。
- `5` 會讀 `/vehicle_1/status`，把目前 leader yaw 加上 `45deg`，再送既有 `MoveLeader` absolute yaw goal。
- Console 輸出 `OK: leader reached target`。
- Gazebo 中只有 leader 直接吃 operator 的 MoveLeader 目標；followers 仍透過 `/vehicle_1/status` 和 formation mode 在各自 `vehicle_node` 本地跟隨。

## 5. 使用 console 切換隊形

在 console 依序輸入：

```text
7
6
```

通過條件：

- `7` 輸出 `OK: formation established`，followers 移到 `line_abreast`。
- `6` 輸出 `OK: formation established`，followers 回到 `vee`。
- Ground station 只發布 `/swarm/formation_mode`。
- 不應有 console 或 ground station 持續發布 `/vehicle_2/staging_setpoint`、`/vehicle_3/staging_setpoint` 作為 formation movement 目標。

## 6. Pause 後確認 move 被拒絕

在 console 依序輸入：

```text
p
2
```

通過條件：

- `p` 輸出 pause 成功。
- `2` 輸出失敗，訊息應包含 paused 或 blocked。
- Gazebo 中三台 hold 目前位置附近，不應繼續追新的 leader movement。

## 7. Resume 後 fresh move

在 console 依序輸入：

```text
r
2
```

通過條件：

- `r` 輸出 resume 成功。
- resume 後不會自動續跑 pause 前被拒絕或舊的 move。
- 第二個 `2` 是 fresh command，應輸出 `OK: leader reached target`。

## 8. 使用 console 降落

在 console 輸入：

```text
8
```

通過條件：

- Console 輸出 `OK: all vehicles reported landed`。
- Gazebo 中三台降落。
- 三台 `/vehicle_N/status` 最後都是 `vehicle_state: landed`、`armed: false`。

## 9. Optional：one-shot command 模式

如果只想測單一短指令，也可以不用進互動式 prompt：

```bash
ros2 run px4_swarm_control operator_console -- --command s
ros2 run px4_swarm_control operator_console -- --command 1
```

通過條件：每次只執行一個 console command 後退出；這只是手動 action 的短指令包裝，不會改變既有 action API。

## 10. Optional：demo macro

乾淨 runtime 且三台 vehicle/ground station 都啟動後，可直接在 console 輸入：

```text
9
```

預設 macro：

```text
1 -> 2 -> 5 -> 7 -> 6 -> home -> 8
```

通過條件：

- 任一步失敗時 macro 會停止，不會硬跑後續命令。
- 若全部成功，console 輸出 `OK: demo macro completed`。
- `home` 會回到 takeoff 完成後讀到的 leader staging 位置。

## 整體通過條件

- QGC 全程不作為控制入口。
- Console 可以從 container 內的 `px4_ws` 啟動。
- Console 只呼叫既有 `/swarm/takeoff`、`/swarm/move_leader`、`/swarm/change_formation`、`/swarm/pause`、`/swarm/land`。
- Console 不發送 direct follower targets，也不繞過 `ground_station_node`。
- `2/3/4/5` 都先讀 leader status，再轉成 absolute `MoveLeader` goal。
- Paused 狀態允許 `s/r/8`，阻擋 `2/3/4/5/6/7/9`。
- 既有手動 `ros2 action send_goal` workflow 仍可用。
