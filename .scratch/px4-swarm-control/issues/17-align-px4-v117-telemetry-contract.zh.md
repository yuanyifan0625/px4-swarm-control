# 17: 對齊 PX4 1.17 telemetry contract

**What to build:** 在不修改上層 swarm interface 的前提下，讓每台 vehicle 的 PX4 adapter 正確使用實驗室 PC、飛控與樹莓派共同採用的 PX4 1.17 telemetry contract。使用 `px4_msgs` v1.17 時，vehicle status conversion 不得因缺少新版欄位而中斷，所有 input/output topic 名稱也必須由實際 message version 一致地決定。

**Blocked by:** None (can start immediately).

**Status:** ready-for-agent

## Fixed compatibility contract

- PX4 firmware baseline 是 `v1.17.0`，commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。
- ROS application 使用 `px4_msgs` `v1.17.0`，commit `86d8239e962f6939e05c3737784f60c02fa884db`。
- 這兩個 revisions 中，本專案使用的八個 PX4 message definitions 必須相符：`FailsafeFlags`、`OffboardControlMode`、`TrajectorySetpoint`、`VehicleCommand`、`VehicleCommandAck`、`VehicleLandDetected`、`VehicleLocalPosition`、`VehicleStatus`。
- `VehicleLocalPosition` 與 `VehicleStatus` 是 message version 1；其餘上述 messages 是 version 0。
- 本 contract 不需要 ROS translation node。
- PX4 main commit `c890d9db0a300795594fd5ba6c045be9ebd71c09` 不在本 ticket 的支援範圍。

## Behaviour and terminology

- Raw `px4_msgs` shape、message-version suffix 與 compatibility fallback 必須留在 PX4 adapter seam，不能散入 vehicle node、ground station 或 operator console。
- 保留既有上層欄位 `offboard_available`，不得要求上層 callers 配合改名或判斷 PX4 message 版本。
- **Offboard-setpoint acceptance** 表示 PX4 明確回報「目前 flight mode 接受 Offboard trajectory setpoints」；它不是能否切入 Offboard mode，也不是 pre-flight checks 的結果。
- PX4 1.17 `VehicleStatus` 不提供 `accepts_offboard_setpoints`。缺少此欄位時必須保守映射成 `offboard_available = false`，不得從 `nav_state` 猜測。
- 如果 message shape 明確提供 `accepts_offboard_setpoints`，adapter 應保留並轉換其實際值；這是防禦性相容，不代表本 ticket 正式支援新版 PX4 contract。
- Topic version suffix 應由 message class 的 `MESSAGE_VERSION` 決定：version 0 不加 suffix，非零版本使用 `_v<version>`。不得再用帶有錯誤 PX4 版本名稱的固定 tuple 表達混合版本 topics。
- Bridge configuration 必須提供一個具名的 `PX4_V117` compatibility profile，集中表達正式 PX4/`px4_msgs` revisions、使用中的 message types，以及 compatibility smoke check 的預期 contract。Profile 不得重新硬編碼一份 `_v1` topic tuple；實際 suffix 仍由各 message class 的 `MESSAGE_VERSION` 推導。

## Expected ROS topic contract

- `VehicleLocalPosition` 訂閱 `/MAVN/fmu/out/vehicle_local_position_v1`。
- `VehicleStatus` 訂閱 `/MAVN/fmu/out/vehicle_status_v1`。
- `VehicleCommandAck` 訂閱 `/MAVN/fmu/out/vehicle_command_ack`，不得加 `_v1`。
- `VehicleLandDetected` 訂閱 `/MAVN/fmu/out/vehicle_land_detected`。
- `FailsafeFlags` 訂閱 `/MAVN/fmu/out/failsafe_flags`。
- Version 0 的 Offboard control mode、trajectory setpoint 與 vehicle command input topics 維持無版本 suffix。
- `/MAVN/status` 繼續使用專案自訂 swarm status message；不得把它和 raw `/MAVN/fmu/out/vehicle_status_v1` 混為同一個 interface。

## Acceptance criteria

- [ ] `px4_msgs` source 固定在 `v1.17.0` / `86d8239e962f6939e05c3737784f60c02fa884db`，且實作紀錄包含實際 commit SHA。
- [ ] 切換 nested repository revision 前先檢查未提交修改；若 worktree 不乾淨就停止，不得強制 checkout 或覆蓋使用者變更。
- [ ] Compatibility check 證明 `VehicleStatus.MESSAGE_VERSION == 1`、存在 `pre_flight_checks_pass`，且不存在 `accepts_offboard_setpoints`。
- [ ] Compatibility check 證明八個使用中的 PX4 message definitions 在固定的 PX4 與 `px4_msgs` revisions 間相符。
- [ ] PX4 1.17-shaped `VehicleStatus` 可以完成 internal vehicle-state conversion，不拋出 `AttributeError`，並得到 `offboard_available = false`。
- [ ] 明確帶有 `accepts_offboard_setpoints = true` 的 message-shaped test input 仍會轉換成 `offboard_available = true`。
- [ ] 缺少欄位時不使用 `nav_state == offboard` 推導 Offboard-setpoint acceptance。
- [ ] `PX4_V117` compatibility profile 是版本 revisions、使用中 message types 與 compatibility smoke 預期值的單一來源，且 callers 不需要自行了解 raw PX4 message shape。
- [ ] `PX4_V117` compatibility profile 不硬編碼 version suffix；version 0 與 version 1 topic names 仍由 message class metadata 推導。
- [ ] Topic-name tests 覆蓋 version 0 與 version 1，並驗證 expected ROS topic contract 的所有 PX4 publishers/subscribers。
- [ ] `VehicleCommandAck` subscription 使用無 suffix topic；local position 與 vehicle status subscriptions 使用 `_v1`。
- [ ] Vehicle node、ground station、operator console 與自訂 swarm `VehicleStatus` interface 不因本 ticket 改變。
- [ ] Compatibility smoke check 能清楚回報實際 message versions、必要欄位與預期 topic names；不相容時必須以 non-zero status 明確失敗。
- [ ] 與 PX4 topic mapping 有關的常數、註解、smoke checks 與操作文件不再錯誤宣稱使用 v1.18 contract。
- [ ] ROS workspace 的舊 `build`、`install`、`log` 衍生內容已清除後重新建置，避免舊 overlay 造成假性通過。
- [ ] 測試輸出能證明載入的是本次 source/build，而不是清除前的 installed Python package。
- [ ] `px4_swarm_interfaces` 與 `px4_swarm_control` 的 focused tests、package tests、build 和 test-result 全部通過。

## Non-goals

- 不修改 PX4-Autopilot cooperative-control implementation。
- 不修改上層 ROS actions、topics 或 operator command semantics。
- 不用 QGC 作為控制入口。
- 不宣稱支援 PX4 main/current `VehicleStatus` v4 contract。
- 不在正常 vehicle runtime 內寫死 Git commit 並拒絕其他 revisions；正式操作前由 compatibility check 驗證部署 contract。

## Comments

- 2026-08-20 implementation：active `PX4-Autopilot` 已固定於
  `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`，active `px4_msgs` 已固定於
  `86d8239e962f6939e05c3737784f60c02fa884db`；兩個 nested worktrees 均乾淨。
- 已清除 ROS workspace 舊 `build/install/log` 後重新建置
  `px4_msgs`、`px4_swarm_interfaces`、`px4_swarm_control`。Compatibility CLI
  載入 fresh install 的 `px4_msgs`，回報兩個 active HEAD、正確 v0/v1 topic、
  `VehicleStatus` 欄位 shape，以及固定 revisions 間 8/8 definitions matched，exit 0。
- Ticket 17 focused tests：34 passed；changed production/test files 的
  `ament_flake8` 與 production files 的 `ament_pep257` 均通過。
- 完整 `px4_swarm_control` baseline 為 164 passed、16 failed；失敗皆位於本
  ticket 未修改的 operation-profile expectations（例如現行 spacing `0.8` 對舊測試
  `0.4`、現行 settle tolerance `0.02` 對舊測試 `0.10`），沒有 Ticket 17 adapter、
  compatibility 或 topic contract 測試失敗。本 ticket 未擴大範圍修改上層參數。
