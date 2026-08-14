# 13 - 統一小場地 operation profile 並新增實機分散式 ROS 節點 launch

**要建立什麼：** 將第一版 swarm 的 SITL 與實機操作參數統一成同一套小場地 operation profile，並新增實機分散式 ROS 2 launch 入口，讓使用者 git clone 後可以用簡單 launch 在地面站與三台 Raspberry Pi 分別啟動四個核心節點，同時保留既有單機 `swarm_nodes.launch.py` smoke path。

**被誰阻擋：** 12 - 新增 swarm launch 與 SITL smoke acceptance workflow。

**狀態：** ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、`ros2 interface`、ROS 2 package tests、ROS 2 launch commands 都要在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。PX4/Gazebo/Micro XRCE-DDS Agent/runtime commands 也必須在 container 裡執行。QGC 可以選擇性用來監控或人工安全觀察，但不是第一版控制入口。

## 背景

Ticket 12 已經提供單機啟動四個 ROS swarm nodes 的 `swarm_nodes.launch.py`，並完成完整 SITL smoke workflow。下一步要讓同一套專案 clone 到實機環境後，也能以清楚且短的命令啟動：

- 地面站電腦：`ground_station_node`
- Pi-MAV1：`/MAV1/vehicle_node`
- Pi-MAV2：`/MAV2/vehicle_node`
- Pi-MAV3：`/MAV3/vehicle_node`

這一版不維護「SITL 大場地一套、實機小場地一套」兩份 profile。SITL 與實機都使用同一套 first-version operation profile；差異只在啟動拓樸：SITL 可以單機跑 `swarm_nodes.launch.py`，實機則每台機器跑自己的 real launch。

目前隊形幾何使用單一 `lateral_spacing_m` / `trail_spacing_m`。這無法同時表達兩個小場地目標：

- `vee`：三機構成邊長 0.8 m 的正三角形。
- `line_abreast`：leader 到左右 follower 各 0.8 m。

因此本 ticket 需要小幅調整 formation geometry 參數模型，但不能改變 action/topic contract、vehicle identity、slot identity 或 `/MAV1`、`/MAV2`、`/MAV3` canonical namespace。

## 範圍

- 將第一版 operation profile 改成小場地值：
  - 起飛/staging 高度：1.5 m。
  - operator console 的 x/y 單軸移動：1.0 m。
  - operator console 的 yaw step：30 deg。
  - `vee` 為邊長 0.8 m 的正三角形。
  - `line_abreast` 的 leader-to-follower 橫向距離為 0.8 m。
- 調整 formation geometry 參數模型，讓 `vee` 與 `line_abreast` 可以使用不同的 spacing value。
- 讓 ground station 的 staging、formation validation、completion tolerance 使用可配置的 operation profile，而不是繼續只吃程式內的 4 m / 3 m default。
- 讓 vehicle-local follower setpoint 計算、ground-station mission validation、operator-console settle 判斷使用一致的 operation profile。
- 修改既有 configuration；不要新增另一套 real-only YAML profile。
- 新增四個實機分散式 launch 入口：
  - MAV1 vehicle node launch。
  - MAV2 vehicle node launch。
  - MAV3 vehicle node launch。
  - Ground station launch。
- 保留既有 `swarm_nodes.launch.py` 作為單機/SITL 一次啟動四個核心 ROS swarm nodes 的入口。
- 保留 `operator_console` 為獨立手動操作工具；不由 swarm launch 或 real launch 自動啟動。
- 新增或更新文件，說明同一套 operation profile 在 SITL 與實機上的兩種啟動拓樸與 smoke 驗證流程。

## 非目標

- 不新增第二套 real-only YAML profile。
- 不新增新的 ROS package。
- 不從任何 launch 啟動 PX4 SITL、Gazebo、Micro XRCE-DDS Agent、QGC、`operator_console` 或 speed-profile workflow。
- 不修改 PX4-Autopilot。
- 不修改 `px4_msgs`。
- 不支援 `/vehicle_1`、`/vehicle_2`、`/vehicle_3` 或任何 legacy namespace fallback。
- 不改變 `/MAV1`、`/MAV2`、`/MAV3` canonical contract。
- 不改變 `operator_console` 只呼叫 `/swarm` actions 的邊界。
- 不讓 ground station 變成 continuous follower absolute target publisher。
- 不新增 `column`、dynamic slots、leader reassignment、任意自訂隊形或完整路徑規劃。

## 實作前設計決策

- 第一版 operation profile 是 SITL 與實機共用的唯一 profile，不再把 simulation profile 與 real profile 拆成兩份。
- 實機 launch 是部署拓樸入口，不是另一套 control architecture。
- `swarm_nodes.launch.py` 保留為單機/SITL wrapper；四個 real launch 保留為分散式部署入口。
- `operator_console` 維持手動啟動：
  - `ros2 run px4_swarm_control operator_console`
- command `9` demo macro 保留既有完整流程：
  - takeoff
  - leader x move
  - settle
  - leader yaw
  - settle
  - line_abreast
  - settle
  - vee
  - settle
  - home_yaw
  - settle
  - home
  - settle
  - land
- `vee` 正三角形計算：
  - follower-left 到 follower-right 距離為 0.8 m，所以 `vee_lateral_spacing_m = 0.4`。
  - leader 到任一 follower 距離為 0.8 m，所以 `vee_trail_spacing_m = sqrt(0.8^2 - 0.4^2) ~= 0.6928`。
- `line_abreast` 不沿用 `vee_lateral_spacing_m`，而是使用自己的 `line_abreast_lateral_spacing_m = 0.8`。
- MAV1 仍是 leader，MAV2 仍是 follower-left，MAV3 仍是 follower-right。

## 驗收條件

- [ ] SITL 與實機共用同一套 first-version operation profile，不新增第二套 real-only YAML profile。
- [ ] Operation profile 將 command `1` 的 takeoff/staging 高度設為 1.5 m。
- [ ] Operation profile 將 command `2` 與 command `3` 的 leader x/y 單軸移動設為 1.0 m。
- [ ] Operation profile 將 command `5` 的 leader yaw step 設為 30 deg。
- [ ] `vee` formation 使用邊長 0.8 m 正三角形幾何，對應 `vee_lateral_spacing_m ~= 0.4` 與 `vee_trail_spacing_m ~= 0.6928`。
- [ ] `line_abreast` formation 使用 leader-to-follower 0.8 m 橫向距離，不被 `vee_lateral_spacing_m` 限制成 0.4 m。
- [ ] Ground station 的 staging targets、formation completion validation 與 mission-level checks 使用同一套小場地 geometry。
- [ ] Vehicle nodes 的 follower-local setpoint derivation 使用同一套小場地 geometry。
- [ ] Operator console 的 settle 檢查使用同一套小場地 geometry。
- [ ] 現有 `swarm_nodes.launch.py` 仍只啟動 `/MAV1`、`/MAV2`、`/MAV3` 三個 `vehicle_node` instances 和一個 `/swarm` `ground_station_node`。
- [ ] 新增的 MAV1 實機 launch 只啟動 `/MAV1/vehicle_node`，並使用 MAV1 的 leader identity、namespace、slot、PX4 target-system。
- [ ] 新增的 MAV2 實機 launch 只啟動 `/MAV2/vehicle_node`，並使用 MAV2 的 follower-left identity、namespace、slot、PX4 target-system。
- [ ] 新增的 MAV3 實機 launch 只啟動 `/MAV3/vehicle_node`，並使用 MAV3 的 follower-right identity、namespace、slot、PX4 target-system。
- [ ] 新增的 ground-station 實機 launch 只啟動 `/swarm/ground_station_node`。
- [ ] 任一 swarm launch 或 real launch 都不啟動 PX4/Gazebo/Micro XRCE-DDS Agent/QGC/operator console/speed-profile workflow。
- [ ] `operator_console` 仍可用 `ros2 run px4_swarm_control operator_console` 手動啟動，且只呼叫 `/swarm` actions。
- [ ] Command `9` 保留完整 demo macro，包含 `line_abreast` 與回到 `vee`。
- [ ] 文件說明 git clone/build/source 後，在地面站與三台 Raspberry Pi 各自應該跑哪一個 launch。
- [ ] 文件說明 `/MAV1`、`/MAV2`、`/MAV3` PX4 uXRCE-DDS topics 必須已經存在，且不支援 `/vehicle_1`、`/vehicle_2`、`/vehicle_3`。

## 手動驗證

### Path A：既有單機 launch path

- [ ] 先由外部啟動 PX4/Gazebo/Micro XRCE-DDS Agent，並確認 `/MAV1`、`/MAV2`、`/MAV3` PX4 bridge topics 存在。
- [ ] 手動啟動 `ros2 launch px4_swarm_control swarm_nodes.launch.py`。
- [ ] 另一個 terminal 手動啟動 `ros2 run px4_swarm_control operator_console`。
- [ ] 確認 `/MAV1/status`、`/MAV2/status`、`/MAV3/status` 出現且 telemetry fresh。
- [ ] 確認 `/swarm/arm`、`/swarm/takeoff`、`/swarm/move_leader`、`/swarm/change_formation`、`/swarm/pause`、`/swarm/land` actions 存在。
- [ ] 在 console 手動驗證 `0` arm-only 可以完成預期結果。
- [ ] 在 console 手動驗證 `1` takeoff 到 1.5 m 小場地 staging。
- [ ] 在 console 手動驗證 `2`、`3`、`4`、`5` 會完成預期 leader movement/yaw。
- [ ] 在 console 手動驗證 `6` 切換到 0.8 m 正三角形 `vee`。
- [ ] 在 console 手動驗證 `7` 切換到 0.8 m `line_abreast`。
- [ ] 在 console 手動驗證 `settle` 只在 followers 真的進入目標隊形 tolerance 後成功。
- [ ] 在 console 手動驗證 `p` pause 與 `r` resume 符合既有 pause/resume 語意。
- [ ] 在 console 手動驗證 `8` land 後三台都回報 landed 且 disarmed。
- [ ] 在 console 手動驗證 `9` 完整 demo macro 可以完成並回報成功。

### Path B：新增分散式 launch path

- [ ] 先由外部啟動 PX4/Gazebo/Micro XRCE-DDS Agent，並確認 `/MAV1`、`/MAV2`、`/MAV3` PX4 bridge topics 存在。
- [ ] 在三個 terminal 分別啟動 MAV1、MAV2、MAV3 實機 vehicle launch；SITL 中可以先在同一台機器用三個 terminal 模擬三台 Pi。
- [ ] 在另一個 terminal 啟動 ground-station 實機 launch。
- [ ] 再用 `ros2 run px4_swarm_control operator_console` 手動啟動 console。
- [ ] 確認 `/MAV1/status`、`/MAV2/status`、`/MAV3/status` 出現且 telemetry fresh。
- [ ] 確認 `/swarm/arm`、`/swarm/takeoff`、`/swarm/move_leader`、`/swarm/change_formation`、`/swarm/pause`、`/swarm/land` actions 存在。
- [ ] 重複 Path A 的 console command 驗證，確認分散式 launch path 與單機 launch path 行為一致。

## 測試方式

- 在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行 ROS 2 package tests。
- 新增或更新 formation geometry unit tests，覆蓋 `vee` 正三角形與 `line_abreast` 獨立 spacing。
- 新增或更新 vehicle-node tests，確認 follower setpoint derivation 使用小場地 geometry。
- 新增或更新 ground-station tests，確認 staging targets、formation validation 與 action completion 使用小場地 geometry。
- 新增或更新 operator-console tests，確認 takeoff altitude、movement step、yaw step、settle geometry 與 demo macro 使用小場地 operation profile。
- 新增或更新 launch tests，確認既有 single-machine launch 與新增 real launch 都只包含預期 ROS swarm nodes。
- 手動 SITL 驗證時，所有 ROS 2 build/test/launch/action/topic commands 都要在 container 裡執行。

## Blocking edges

- Blocked by 12 - 新增 swarm launch 與 SITL smoke acceptance workflow。
- Continues from 10 - 實作 `ChangeFormation` 在 `vee` 和 `line_abreast` 之間切換。
- Continues from 11b - 新增可參數化 operator 短指令 console。
- Continues from 11c - Add demo settle gate to operator console。
- Continues from 12a - 將第一版 vehicle namespaces 改成 `/MAV1`、`/MAV2`、`/MAV3`。
