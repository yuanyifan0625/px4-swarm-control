# 23 — 以 local-NED Offboard position control 取代 NAV_TAKEOFF

**What to build:** 讓 SITL 與實機共用同一套 ROS 2 起飛流程；即使 PX4 只有有效的 local NED position、沒有 global altitude reference，三台機體仍能先垂直起飛，再移動到各自的 VEE staging target。起飛不再發布 `VEHICLE_CMD_NAV_TAKEOFF` 或換算 AMSL，而是先 warm up Offboard position setpoint、確認 PX4 已進入 Offboard、再 ARM 並追蹤 local-NED target。本 ticket 是 Ticket 22 完整 operator-console demo 的起飛前置修正，但不擴張成 shared-origin alignment 或完整飛行安全系統。

**Blocked by:** None — can start immediately.

**Status:** ready-for-agent

- [ ] 使用 TDD 實作並記錄 red/green/refactor：PX4 boundary 將 raw local-position telemetry 封裝成單一 `local_position_ready` 契約；契約只要求 telemetry fresh、`xy_valid`、`z_valid`、有限的 `x/y/z/heading` 與 `dead_reckoning == false`，不依賴 global reference、velocity validity、`heading_good_for_control`、distance-to-bottom、位置誤差或 estimator speed-limit 欄位。
- [ ] ARM 成功前的每個 control tick 都重新確認 `local_position_ready`；ARM 成功後不新增 local-validity 監控、自動 land 或自動 disarm，飛行中的緊急處置仍由 PX4、RC kill 與操作者負責。
- [ ] 完全移除起飛路徑中的 `VEHICLE_CMD_NAV_TAKEOFF`、AMSL、`z_global` 與 `ref_alt` 依賴；PX4 command 22 不再由 swarm controller 發布。`VEHICLE_CMD_NAV_LAND` 與獨立 `ArmSwarm` 行為保持不變。
- [ ] VehicleNode 使用單一私有 takeoff phase model，至少能清楚區分 idle、等待 local position、Offboard warmup、等待 Offboard、等待 ARM、垂直上升與移動到 staging；phase 切換輸出可除錯 log，對外既有 vehicle-level state vocabulary 不擴張。
- [ ] 收到 fresh staging target 後，每台機體以自己的當前 local `x/y/heading` 和 staging `z` 建立垂直起飛 target；先以控制迴圈頻率發布 `OffboardControlMode` 與該 target 至少 `1.0 s`，再每 `1.0 s` 重試 Offboard mode request，只有 telemetry 回報 `nav_state == offboard` 後才開始 ARM retry，只有 telemetry 回報 `armed == true` 後才承認 ARM 成功。Command ACK 只供記錄與除錯，不直接驅動 phase transition。
- [ ] 垂直上升期間維持各機自己的起始 `x/y/heading`；當 local NED z 到達 `target_z + 0.1 m` 或更高時，才切換到完整 staging `x/y/z/yaw`。起飛高度容許誤差預設改為 `0.1 m`；共用的 Offboard／ARM retry 參數命名為 `takeoff_command_retry_s`，預設維持 `1.0 s`。
- [ ] Takeoff action timeout 時，GroundStation 對三台機體發布 PAUSE 並停止本輪 takeoff：VehicleNode 清除 takeoff phase，不再重試 Offboard／ARM；有 fresh position 時以當下 `x/y/z/heading` 建立 hold target，否則 fallback 到最後已發布的安全 setpoint；尚未 ARM 的機體保持 disarmed，已升空的機體保持 Offboard hold，RESUME 不得偷偷恢復逾時的起飛流程。
- [ ] Land 完成後清除本輪 takeoff phase 與 staging latch；不重啟 nodes 的下一輪 takeoff 仍必須等待 fresh staging anchor，且可重新走完相同 local-NED Offboard sequence。
- [ ] 自動測試涵蓋 local-position readiness 的有效／無效組合、沒有 command 22、每個 phase 與 `1.0 s` retry、Offboard-before-ARM、垂直 target 建立、`0.1 m` 高度 gate、完整 staging target 切換、timeout PAUSE、RESUME 不重啟舊流程，以及 land 後第二輪 takeoff；focused tests、完整 package tests、build 與 test-result 必須通過並留下結果。
- [ ] 三機 SITL 使用固定啟動基線驗證：三台都設 `GZ_IP=127.0.0.1`；MAV1 使用 PX4 instance 0；MAV2 使用 instance 1、`PX4_GZ_STANDALONE=1`、Gazebo pose `-1,1,0`；MAV3 使用 instance 2、`PX4_GZ_STANDALONE=1`、Gazebo pose `-1,-1,0`。這組基線取代缺少 `GZ_IP`、會造成 accelerator timeout 的舊啟動方式。
- [ ] SITL 證據顯示三台都持續收到 Offboard heartbeat 與 local trajectory setpoint、vehicle command 只出現必要的 Offboard mode／ARM 而沒有 command 22、先垂直離地再開始水平 staging，並進入 `nav_state=offboard`、`armed=true`。Ticket 23 不要求實機驗收；相同 swarm nodes、launch 與控制參數必須能直接部署到實機，不建立 simulation-only 或 real-only 控制分支。
- [ ] SITL 驗證明確承認每台 PX4 local origin 是各自 EKF 啟動位置：每次指令都記錄各機 local position、trajectory target 與 Gazebo 視覺位移，檢查自身 local frame 中的移動是否合理；不得假設三台 raw local position 共用原點，也不在本 ticket 實作 per-vehicle origin alignment。若發現視覺 formation offset，另開 shared-origin ticket。
- [ ] 保護 commit `5494897` 的座標基線：不得修改左右 slot 符號、formation geometry、field-to-NED mapping、collision-safety 座標方向，或交換 MAV2／MAV3 的 slot、namespace、PX4 instance；相關既有 regression tests 必須保持通過。若實作確實需要座標改動，先停止並清楚列出原因、frame 與預期符號，不得直接修改。
- [ ] 不覆蓋操作者目前尚未 commit 的 `takeoff_altitude_m: 0.5`，不修改 formation following、collision safety policy、speed profile、PX4-Autopilot 原始碼，也不承接 Ticket 22 的舊手冊刪除／整併與完整 movement／formation／safety-hold demo。
- [ ] 預期影響限制在 internal vehicle state model、PX4 topic boundary、per-vehicle takeoff orchestration、ground-station timeout handling、其自動測試、local-NED Offboard takeoff glossary 與一份精簡 ADR；ADR 記錄為何 local-only 實機不能使用 AMSL NAV_TAKEOFF、為何 SITL／實機統一路徑，以及 landing 為何仍保留 PX4 NAV_LAND。若需要超出此範圍，先回報並取得確認。
