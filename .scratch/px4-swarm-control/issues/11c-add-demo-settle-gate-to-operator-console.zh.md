# 11c - 在 operator console demo 加入 settle 等待關卡

**要建立什麼：** 改善 operator console 的 demo macro，讓它在送出下一個 demo action 前，先等 followers 進入目前隊形並穩定一小段時間。目標是讓 SITL demo 更容易觀察，但不改整體 swarm-control 架構。

**被誰阻擋：** 11b - Add configurable operator short-command console。

**狀態：** ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、`ros2 interface`、ROS 2 package tests、ROS 2 launch commands 都要在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。QGC 可以選擇性用來監控或人工安全觀察，但不是第一版的控制入口。

## 背景

Ticket 11b 已經新增短指令 operator console 與可設定的 demo macro。實際手動 SITL demo 時，macro 可能看起來太快或有點晃，原因是每一步雖然會等自己的 action result，但 `MoveLeader` 成功只代表 leader 到目標，不代表 followers 已經追到 leader body-frame offset 並穩定。當下一個 yaw 或 formation action 太快開始時，followers 的目標又被改變，視覺上就會像一直在追趕。

這張 ticket 只改善 demo 節奏。不要修改核心 vehicle control logic、follower controller、PX4-Autopilot 或既有 action APIs。

## 範圍

- 新增 demo 專用的 `settle` macro step，讓它可以放在 `demo_commands` 設定序列裡。
- `settle` 的定義是 observation gate：operator console 只觀察最新 leader/follower status，等 followers 在目前 formation tolerance 內連續穩定一段設定時間，預設約 `1.0 s`。
- 讓 demo sequence 可設定，例如：`takeoff -> leader move -> settle -> yaw -> settle -> line_abreast -> settle -> vee -> settle -> home -> settle -> land`。
- 讓 `settle` 知道 console demo 目前使用的 formation mode。可以追蹤上一個成功的 `vee`/`line_abreast` command，或觀察 formation mode topic，但不能發布 formation target。
- 視需要新增可設定參數，例如 settle stable duration、settle timeout、position tolerance、yaw tolerance、lateral spacing、trail spacing。
- 加入 TDD 測試，覆蓋 settle 成功、timeout/failure、paused-state 行為，以及 demo macro 在 settle 失敗時停止。
- 更新 operator console 中文手動 SITL 驗證文件。

## 非目標

- 不修改 `vehicle_node`。
- 不修改 `follower_controller`。
- 不修改 PX4-Autopilot 或 PX4 內部控制邏輯。
- 不新增 follower controller ROS nodes。
- 不從 operator console 或 ground station 在 demo settle 階段發布 direct follower absolute targets。
- 不修改 `TakeoffSwarm`、`MoveLeader`、`ChangeFormation`、`PauseSwarm`、`LandSwarm` action definitions。
- 不在本 ticket 實作完整 trajectory planning 或 velocity limiting。
- 不在本 ticket 調 PX4 controller parameters。

## 實作備註

- `settle` 只是為了讓人工 demo 看起來更穩，不是新的 mission command，也不應該變成一般手動 action 的必要前置步驟。
- Console 可以為了「觀察是否到位」在本地計算 follower 應在的位置，使用第一版固定 formation geometry 與目前 formation mode。這個計算只能用於 readiness check，不能變成 follower command path。
- 如果 swarm paused，`settle` 與 demo macro progression 要延續 ticket 11/11b 行為，清楚停止，不要繼續往下送指令。
- 在 settle gate 附近加入一行中文註解，說明這段是保護 demo 不會在 followers 尚未收斂時送下一個 action。

## 驗收條件

- [ ] Configured demo macro 可以接受 `settle` step。
- [ ] `settle` 會等到兩台 followers 都在 configured position/yaw tolerance 內，且連續穩定達到 configured stable duration 後才成功。
- [ ] `settle` 只觀察 `/vehicle_1/status`、`/vehicle_2/status`、`/vehicle_3/status`，以及 readiness check 所需的目前 formation mode state。
- [ ] `settle` 不發布 `/vehicle_2/staging_setpoint`、`/vehicle_3/staging_setpoint`，也不發布任何 direct follower absolute target。
- [ ] Console demo macro 在 settle timeout 或 telemetry stale 時會停止，並回報清楚的失敗訊息。
- [ ] Paused-state 行為維持不變：允許 status/resume/land，阻擋或停止 movement、formation change、settle progression、demo macro progression。
- [ ] 既有 `1` 到 `8` 指令維持原本行為。
- [ ] 既有手動 `ros2 action send_goal` workflows 維持可用。
- [ ] Tests 覆蓋 settle 成功、settle timeout、stale status、paused-state blocking、含 `settle` 的 macro sequencing。
- [ ] 手動 SITL 驗證不使用 QGC 作為控制入口，透過 console command `9` 完成 `takeoff -> leader move -> settle -> yaw -> settle -> line_abreast -> settle -> vee -> settle -> home -> settle -> land`，並在 Gazebo 中確認 followers 會明顯穩定後才進下一步。

## 測試方式

- 使用 fake leader/follower statuses 和 fixed formation geometry unit-test settle readiness calculation。
- Unit-test `settle` 必須符合連續 stable window，不是單次 status sample 符合就成功。
- Unit-test stale telemetry 和 timeout failure。
- Unit-test demo macro sequencing，確認 `settle` 會插在 actions 中間，且失敗時停止後續 macro。
- Unit-test fake gateway 沒有紀錄任何 follower target publication 或 bypass command。
- ROS 2 package tests 要在 container 裡從 `px4_ws` workspace 執行。
- Package tests 通過後，在三機 SITL 中手動驗證更新後的 console demo。

## Blocking edges

- Blocked by 11b - Add configurable operator short-command console。
- 如果 ticket 12 的 final smoke workflow 要包含 operator console demo，本 ticket 可作為 12 的前置參考。
