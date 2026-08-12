# 11b - 新增可參數化 operator 短指令 console

**要建立什麼：** 新增第一版 terminal operator console，把短數字指令對應到既有 swarm actions，讓手動 SITL 測試更快，但不改 action APIs，也不破壞 distributed follower-control 架構。

**被誰阻擋：** 11 - Add minimal pause and failsafe behavior。

**狀態：** ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、`ros2 interface`、ROS 2 package tests、ROS 2 launch commands 都要在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。QGC 可以選擇性用來監控或人工安全觀察，但不是第一版的控制入口。

## 背景

手動 `ros2 action send_goal ...` 指令很長。它適合 debug 單一 action，但不適合反覆 SITL 操作或 demo。Console 的目的只是減少打字，同時保留既有架構。

Console 是 operator convenience layer。它必須呼叫 operator 原本就能呼叫的 `TakeoffSwarm`、`MoveLeader`、`ChangeFormation`、`PauseSwarm`、`LandSwarm` actions。它不能發布 follower setpoints、不能繞過 ground station、不能把 followers 變成 operator 直接控制的目標。

## 範圍

- 新增 terminal console 或 CLI executable 給手動 SITL 操作使用。
- 將短指令對應到既有 swarm actions：
  - `1`：使用設定檔中的預設高度和 timeout 執行 takeoff。
  - `2`：讓 leader 依設定的 `+x` step 移動。
  - `3`：讓 leader 依設定的 `+y` step 移動。
  - `4`：讓 leader 依設定的 altitude step 移動。
  - `5`：讓 leader yaw 依設定角度旋轉，預設 `45 deg`。
  - `6`：切換 formation 到 `vee`。
  - `7`：切換 formation 到 `line_abreast`。
  - `8`：全隊降落。
  - `9`：執行設定好的 demo macro：起飛、移動、yaw change、formation change、回到初始 leader 位置、降落。
- Relative movement commands 要先讀 leader status，再轉成既有 `MoveLeader` 使用的 absolute goal。
- Step size、takeoff altitude、yaw step、tolerances、timeouts、demo macro sequence 要放 ROS 2 parameters 或 YAML config。
- Console 是 optional。既有手動 action commands 必須繼續可用。
- 新增 command mapping、parameter usage、relative-to-absolute leader movement conversion、paused-state behavior、demo macro sequencing 測試。
- 新增中文手動驗證文件。

## 非目標

- 不修改既有 action message definitions。
- 不新增 direct follower absolute target commands。
- 不為 formation movement 發布 `/vehicle_2/staging_setpoint` 或 `/vehicle_3/staging_setpoint`。
- 不繞過 `ground_station_node`。
- 不取代手動 `ros2 action send_goal` commands。
- 不實作完整 GUI。
- 不在本 ticket 實作任意 trajectory planning 或未來 autonomy algorithms。

## 實作備註

- Console 應該只是 operator-facing actions 的薄 client，保護 ground-station action boundary，也避免未來 algorithm work 依賴數字鍵。
- Relative movement 只是 console convenience。內部要讀最新 leader status，然後送既有 absolute world-frame `MoveLeader` action。
- Demo macro 應該 declarative 或 parameterized，讓未來 formation settings 和 algorithm experiments 可以調整流程，不需要改 core control logic。
- 如果 swarm paused，console 要遵守 ticket 11：允許 status/resume/land，阻擋 movement、formation change、macro progression。
- 在 relative-to-absolute conversion 和 macro safety gate 附近加入一行中文註解，說明它保護什麼架構邊界或 operator-safety rule。

## 驗收條件

- [ ] Console 可以從 container 裡的 ROS 2 workspace 啟動。
- [ ] `1` 會用設定的 default altitude 和 timeout 發送 `TakeoffSwarm`。
- [ ] `2`、`3`、`4` 會讀 leader status，把 relative movement 轉成 absolute `MoveLeader` goal，且只移動 leader。
- [ ] `5` 會讀 leader yaw，把設定的 yaw delta 轉成 absolute `MoveLeader` yaw goal，且只改 leader yaw target。
- [ ] `6` 發送 `ChangeFormation(vee)`。
- [ ] `7` 發送 `ChangeFormation(line_abreast)`。
- [ ] `8` 發送 `LandSwarm`。
- [ ] `9` 會完整執行 configured demo macro，且第一個 action 失敗就停止後續 macro。
- [ ] Console defaults 透過 ROS 2 parameters 或 YAML 設定，不寫死在 control logic。
- [ ] Console 不發送 direct follower targets，也不繞過 `ground_station_node`。
- [ ] 既有手動 `ros2 action send_goal` workflow 仍可用。
- [ ] Paused 狀態下，console 允許 status/resume/land，阻擋 movement、formation change、demo macro progression。
- [ ] Tests 覆蓋 command mapping、relative leader movement conversion、yaw delta conversion、configurable defaults、paused-state blocked commands、macro failure stop。
- [ ] 手動 SITL 驗證不使用 QGC 作為控制入口，透過 console 完成：`takeoff -> leader relative move -> yaw delta -> line_abreast -> vee -> pause -> paused 時 move 被拒絕 -> resume -> fresh move -> land`，全程不重啟 runtime。

## 測試方式

- Unit-test command parsing 與 mapping，不需要 live ROS 2 actions。
- 使用 fake leader status unit-test relative movement conversion。
- 使用 fake leader status unit-test yaw delta conversion。
- Unit-test console config 控制 altitude、step size、yaw step、tolerances、timeouts、macro sequence。
- Unit-test paused-state command gating。
- 使用 fake action clients 驗證 console 只呼叫既有 swarm action surface。
- ROS 2 package commands 要從 `px4_ws` workspace 執行，不是從外層 Docker workspace。
- Ticket 11 pause/resume behavior 可用後，在 SITL 中手動驗證 console。

## Blocking edges

- Blocked by 11 - Add minimal pause and failsafe behavior。
- 如果 ticket 12 的 final smoke workflow 要包含 operator console，本 ticket 可作為 12 的前置參考。
