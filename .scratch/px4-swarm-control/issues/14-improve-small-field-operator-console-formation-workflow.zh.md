# 14 - 改善小場地 operator console 隊形操作流程

**要建立什麼：** 讓第一版小場地 swarm 操作可以用更完整但仍簡短的 console 指令完成 world-frame 正反向 jog、直接在 takeoff/staging 後切換隊形、並以更嚴格且可調整的 settle 條件驗證 `vee` 與 `line_abreast`。完成後，使用者不需要先手動移動 leader 才能切隊形，也可以在 SITL 與實機 launch 時調整 formation/settle tolerance。

**被誰阻擋：** 13 - 統一小場地 operation profile 並新增實機分散式 ROS 節點 launch。

Status: ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、`ros2 interface`、ROS 2 package tests、ROS 2 launch commands 都要在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。PX4/Gazebo/Micro XRCE-DDS Agent/runtime commands 也必須在 container 裡執行。QGC 可以選擇性用來監控或人工安全觀察，但不是第一版控制入口。

## 背景

Ticket 13 已經把 SITL 與實機共用的小場地 operation profile 建好，並把 `vee` 幾何調成邊長 0.8 m 的正三角形。後續實測發現，`vee` 實際位置若仍在較寬 tolerance 內，ROS 2 層可能太早回報隊形完成。已在 debug 修正中將 formation/settle position tolerance 收斂到 0.15 m，並補上 loose VEE regression tests。

本 ticket 要繼續改善實機操作性，但不改變底層架構邊界：

- `operator_console` 仍只呼叫 `/swarm` actions。
- `ground_station_node` 仍做 mission-level validation。
- `vehicle_node` 仍是 vehicle-local PX4 boundary。
- 不修改 follower gating。
- 不修改 ground station 目前「`ChangeFormation` 需要 mission state 已在 `FOLLOWING`」的狀態機語意。
- 不做 body-frame forward/back/left/right；本 ticket 只處理 world-frame jog。

## 範圍

- 新增 world-frame 負方向 jog 指令，同時保留既有數字指令語意：
  - `2`：leader world x + step。
  - `x`：leader world x - step。
  - `3`：leader world y + step。
  - `y`：leader world y - step。
  - `4`：leader up，NED z -= altitude step。
  - `z`：leader down，NED z += altitude step。
  - `5`：leader yaw + step。
  - `c`：leader yaw - step。
- 讓手動 `6` / `7` 可以在 `1` takeoff/staging 後直接執行，不需要使用者先按 `2` 或其他 move command。
- 在 `6` / `7` 送出 `ChangeFormation` 前，由 `operator_console` 先用目前 leader pose 發一個 no-op `MoveLeader`，利用既有 action flow 進入 `FOLLOWING`。
- no-op `MoveLeader` 的目標必須是目前 leader status 的 `x/y/z/yaw`，不能造成刻意位移。
- no-op `MoveLeader` 失敗時，不繼續送 `ChangeFormation`，並回報明確錯誤。
- Command `9` demo macro 維持既有流程，但因為共用 `6` / `7`，也要受益於 no-op formation preflight。
- 將 `settle_stable_duration_s` 預設從 1.0 s 改成 1.5 s；手動 `settle` 和 command `9` 裡的 `settle` 共用同一條規則。
- 測試 tolerance ladder：
  - 先以 0.10 m 做 SITL manual smoke。
  - 若 0.10 m 穩定通過，將 `formation_position_tolerance_m` 與 `settle_position_tolerance_m` default 設為 0.10 m。
  - 若 0.10 m 偶發不穩，改測 0.12 m；若 0.12 m 穩定，default 設為 0.12 m。
  - 若 0.12 m 仍不穩，最後退回 0.15 m default。
- 無論最後 default 是 0.10 m、0.12 m 或 0.15 m，都要保留 launch-time override，方便 SITL 與實機飛場調整。
- 新增或更新 launch 入口，讓使用者可以不改 YAML 直接調整：
  - ground station 的 `formation_position_tolerance_m`。
  - operator console 的 `settle_position_tolerance_m`。
  - operator console 的 `settle_stable_duration_s`。
- `swarm_nodes.launch.py` 和實機 ground-station launch 都不得啟動 PX4/Gazebo/DDS/QGC/speed-profile workflow。
- operator console 可以新增獨立 launch 入口，但不得被 swarm launch 或 real ground-station launch 自動啟動。

## 非目標

- 不做 body-frame forward/back/left/right jog。
- 不重排既有數字指令 `0` 到 `9` 的語意。
- 不新增新的 ROS action/message/service。
- 不讓 `operator_console` 直接發布 vehicle-local topics。
- 不改 `ground_station_node` 的 action surface。
- 不改 follower 對 leader status 的 freshness/gating 條件。
- 不讓 ground station 變成 continuous follower absolute target publisher。
- 不修改 PX4-Autopilot。
- 不修改 `px4_msgs`。
- 不支援 `/vehicle_1`、`/vehicle_2`、`/vehicle_3` namespace。

## 實作前設計決策

- 負方向 jog 是 world-frame jog，不是 body-frame jog。
- no-op formation preflight 放在 `operator_console`，不是放在 `ground_station_node` 或 `vehicle_node`。
- `6` / `7` 每次送 `ChangeFormation` 前都先做 no-op `MoveLeader`；不依賴解析 ground station rejection message 後再 retry。
- 如果 no-op `MoveLeader` 無法完成，`6` / `7` 直接失敗，不送隊形切換。
- 手動 `settle` 與 demo macro 內的 `settle` 共用 `settle_stable_duration_s`。
- tolerance default 由 SITL manual smoke 結果決定，優先順序是 0.10 m、0.12 m、0.15 m。
- 若 SITL 在 0.10 m 或 0.12 m timeout 或偶發不穩，不為了硬過更小 tolerance 去改 controller、PX4、follower gating 或 ground station 狀態機。

## 驗收條件

- [ ] 既有指令 `0` 到 `9` 保持原本語意。
- [ ] `x` 會將 leader 目標轉成目前 world x 減去 configured x step。
- [ ] `y` 會將 leader 目標轉成目前 world y 減去 configured y step。
- [ ] `z` 會將 leader 目標轉成目前 NED z 加上 configured altitude step，語意是下降。
- [ ] `c` 會將 leader yaw 目標轉成目前 yaw 減去 configured yaw step，並沿用既有 yaw normalize 行為。
- [ ] console help text 與最終版手動驗證文件列出新增的 `x/y/z/c` 指令。
- [ ] 手動 `1 -> 6 -> settle` 可以完成，不需要使用者先按 `2`。
- [ ] 手動 `1 -> 7 -> settle` 可以完成，不需要使用者先按 `2`。
- [ ] `6` / `7` 前會用目前 leader pose 發 no-op `MoveLeader`，成功後才送 `ChangeFormation`。
- [ ] no-op `MoveLeader` 失敗時，不送 `ChangeFormation`，並回報包含 preflight move 失敗原因的訊息。
- [ ] `operator_console` 不直接發布 PX4 topics 或 vehicle setpoint topics，仍只呼叫 `/swarm` actions。
- [ ] `ground_station_node` 的 `ChangeFormation` state requirement 不因本 ticket 被放寬。
- [ ] follower 對 leader status 的 freshness/gating 條件不因本 ticket 被放寬。
- [ ] `settle_stable_duration_s` default 與 YAML 改為 1.5 s。
- [ ] 手動 `settle` 必須在 formation 進入 tolerance 後連續穩定 1.5 s 才成功。
- [ ] Command `9` 裡的每個 `settle` 也必須連續穩定 1.5 s 才繼續下一步。
- [ ] SITL manual smoke 先測 `formation_position_tolerance_m=0.10` 與 `settle_position_tolerance_m=0.10`。
- [ ] 若 0.10 m SITL 穩定通過，最終 default 設為 0.10 m，並把 0.12 m / 0.15 m 寫成 fallback override。
- [ ] 若 0.10 m SITL 偶發不穩，改測 0.12 m；若 0.12 m 通過，最終 default 設為 0.12 m，並把 0.15 m 寫成 fallback override。
- [ ] 若 0.12 m SITL 仍不穩，最終 default 保留或退回 0.15 m，並在文件記錄原因。
- [ ] `swarm_nodes.launch.py` 支援 launch-time override ground-station formation tolerance。
- [ ] 實機 ground-station launch 支援 launch-time override ground-station formation tolerance。
- [ ] 新增或更新 operator-console launch，支援 launch-time override settle tolerance 與 settle stable duration。
- [ ] swarm launch、real vehicle launch、real ground-station launch 都不自動啟動 operator console。
- [ ] 文件保留 `ros2 run px4_swarm_control operator_console` 作為最短 console 啟動方式，並補充 launch 方式供調參使用。

## SITL manual smoke 驗收

實作階段必須在 container 裡跑手動 SITL smoke。測試 tolerance 時依序嘗試 0.10 m、0.12 m、0.15 m，並在最後回報採用哪個 default。

### Path A：直接隊形切換

- [ ] 先由外部啟動 PX4/Gazebo/Micro XRCE-DDS Agent，並確認 `/MAV1`、`/MAV2`、`/MAV3` PX4 bridge topics 存在。
- [ ] 啟動 ROS swarm nodes。
- [ ] 啟動 operator console。
- [ ] 執行：
  - `1`
  - `6`
  - `settle`
  - `7`
  - `settle`
  - `6`
  - `settle`
  - `8`
- [ ] 驗收：`1 -> 6` 不需要額外手動 jog；`vee`、`line_abreast`、回到 `vee` 都能 settle；最後 land 成功。

### Path B：demo macro

- [ ] 重新準備乾淨 SITL runtime。
- [ ] 啟動 ROS swarm nodes。
- [ ] 啟動 operator console。
- [ ] 執行 `9`。
- [ ] 驗收：demo macro 完成並回報成功；流程中的每個 settle 都滿足 1.5 s stable duration。

### tolerance 不穩的判定

任一 tolerance 值出現以下狀況，即視為該 tolerance 在 SITL 不穩：

- [ ] `settle` timeout。
- [ ] `ChangeFormation` timeout。
- [ ] 必須額外手動 jog 才能讓 `1 -> 6` 成功。
- [ ] `1 -> 6` 仍然失敗。
- [ ] status 顯示隊形明顯未收斂，但 console 回 OK。

## 測試方式

- [ ] 使用 TDD：先加入會失敗的 tests，再實作。
- [ ] 新增或更新 operator-console command dispatcher tests，覆蓋 `x/y/z/c`。
- [ ] 新增或更新 operator-console tests，確認 `6/7` 會先做 no-op `MoveLeader` 再做 `ChangeFormation`。
- [ ] 新增或更新 operator-console tests，確認 no-op `MoveLeader` 失敗時不送 `ChangeFormation`。
- [ ] 新增或更新 settle gate tests，確認必須連續穩定 1.5 s。
- [ ] 新增或更新 launch tests，確認 tolerance / settle launch overrides 被傳給正確 node。
- [ ] 更新 package scaffold/docs tests，確認最終手動驗證文件仍存在且包含新增指令與 tolerance override 說明。
- [ ] 在 container 裡執行 package build、pytest、`colcon test`，並回報結果。

## Blocking edges

- Blocked by 13 - 統一小場地 operation profile 並新增實機分散式 ROS 節點 launch。
- Continues from 11b - 新增可參數化 operator 短指令 console。
- Continues from 11c - Add demo settle gate to operator console。
- Continues from 10 - 實作 `ChangeFormation` 在 `vee` 和 `line_abreast` 之間切換。

## Implementation notes

- SITL `0.10 m` tolerance smoke 已在 container 內通過。
- Path A 使用 `swarm_nodes.launch.py formation_position_tolerance_m:=0.10`，console 使用 `settle_position_tolerance_m:=0.10` 與 `settle_stable_duration_s:=1.5`，流程 `1 -> 6 -> settle -> 7 -> settle -> 6 -> settle -> 8` 成功，不需要額外 jog。
- Path B 在同一套 `0.10 m` / `1.5 s` 設定下執行 command `9`，回報 `OK: demo macro completed`。
- 因 `0.10 m` 通過 Path A 與 Path B，本 ticket 採用 `0.10 m` 作為 default，`0.12 m` 與 `0.15 m` 保留為 fallback override。
