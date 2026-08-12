# 10 - 實作 `ChangeFormation` 在 `vee` 和 `line_abreast` 之間切換

**要建立什麼：** 讓 operator 可以在 `vee` 與 `line_abreast` 兩種 formation mode 之間切換，followers 會在 body-frame slots 之間過渡，ground station 會回報 formation completion。

**被誰阻擋：** 09 - Implement follower fixed-slot following from leader state。

**狀態：** ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、`ros2 interface`、ROS 2 package tests、ROS 2 launch commands 都要在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。QGC 可以選擇性用來監控或人工安全觀察，但不是第一版的控制入口。

## 背景

第一版 formation changes 應保持簡單：只支援 `vee` 和 `line_abreast`，只使用 fixed slots，只做 body-frame slot transitions。Ground station 選擇並 broadcast formation mode，而 followers 自己計算 setpoints。

Ticket 07c 的前車之鑑是：mission completion 必須綁定 fresh vehicle status 和真實 tolerance checks，不能只因為 command 已發布就成功。因此 `ChangeFormation` 只有在 vehicles 真的抵達新 mode 的 slots 後才能回成功。

這張 ticket 必須延續 ticket 09 的 distributed follower-control 邊界。Formation change 只改變 mode，進而改變 follower slot offsets；它不能讓 ground station 變成 continuous absolute follower target publisher。

## 範圍

- 實作 `ChangeFormation` action behavior。
- Ground station broadcast target formation mode。
- Follower nodes 消費 formation mode，並切換自己的 body-frame target offsets。
- 根據 vehicle status 和 tolerances 偵測 formation 是否 established。
- Ground station 記錄 `formation established`。
- 為 formation change 加上 action feedback/result。
- 新增 formation mode switching 和 completion detection 的測試。
- 新增手動驗證卡，證明完整 SITL 任務可以完成：clean runtime、takeoff 到 staging、follow leader、change formation、確認新 slots、最後 land all。

## 非目標

- 不實作 `column`。
- 不實作 custom arbitrary offsets。
- 不實作 dynamic slot assignment。
- 不實作 follower-follower coordination。
- 不讓 ground station 計算或持續發布 absolute follower target positions。
- 不在 formation change 期間改變 vehicle identity 或交換 follower slots。

## 實作備註

- Formation mode changes 應該改變 desired slots，而不是改變 vehicle identity。
- Ground station 只 broadcast target formation mode；每個 follower 自己根據 leader state 把自己的 slot offset 轉成 world-frame setpoint。
- 套用 ticket 07c 經驗：formation completion 必須使用 fresh vehicle status 和 tolerance checks，不能用 mode command 已發布來判定完成。
- 在 formation completion logic 附近加入一行中文註解，說明它會防止 operator 在 vehicles 還沒到新 slots 前就誤以為 mode 已切換完成。

## 實作前設計決策

- `vee` body-frame offsets：
  - `follower_left`: `(-trail_spacing_m, +lateral_spacing_m, 0)`
  - `follower_right`: `(-trail_spacing_m, -lateral_spacing_m, 0)`
- `line_abreast` body-frame offsets：
  - `follower_left`: `(0, +lateral_spacing_m, 0)`
  - `follower_right`: `(0, -lateral_spacing_m, 0)`
- 兩種 formation mode 中，follower yaw 都跟 leader yaw。
- `ChangeFormation` 不移動 leader。Leader 繼續保持或追蹤目前的 leader goal。
- 第一版只允許 swarm mission state 是 `following` 時執行 `ChangeFormation`，否則清楚拒絕。
- 這張 ticket 不修改 `ChangeFormation.action` interface。Formation completion 使用 ground station config 固定值：
  - `formation_position_tolerance_m = 0.3`
  - `formation_yaw_tolerance_rad = 0.2`
- `ChangeFormation` pending 期間，ground station 可以重送 `/swarm/formation_mode`，避免 single-shot topic 脆弱。
- `ChangeFormation` pending 期間，ground station 不得發布 `/vehicle_2/staging_setpoint` 或 `/vehicle_3/staging_setpoint` 作為 follower formation target。
- Formation completion 必須滿足：
  - leader status fresh、armed、Offboard，且 `vehicle_state == following`
  - `vehicle_2` 維持 `follower_left`
  - `vehicle_3` 維持 `follower_right`
  - follower statuses fresh、armed、Offboard
  - followers 位於由 leader state、target formation mode、fixed slot 推導出的 target 附近，且進入 position/yaw tolerance
- Feedback `progress` 使用 follower 到位比例：
  - `0.0`: 沒有 follower 到新 slot
  - `0.5`: 一台 follower 到新 slot
  - `1.0`: 兩台 followers 都到新 slots
- 這張 ticket 不新增自訂軌跡插值。Followers 直接切換推導出的 setpoint，讓 PX4 Offboard position control 處理移動。

## 驗收條件

- [ ] `ChangeFormation` 接受 `vee` 和 `line_abreast`。
- [ ] 不支援的 formation modes 會被清楚拒絕。
- [ ] Swarm 不在 `following` 時，`ChangeFormation` 會清楚拒絕。
- [ ] Ground station broadcast target formation mode。
- [ ] Ground station 等待 completion 期間只重送 `/swarm/formation_mode`。
- [ ] Followers 根據 target mode 計算新的 body-frame slot setpoints。
- [ ] Ground station 在 formation change 期間不會計算或持續發布 absolute follower target positions。
- [ ] Ground station 在 formation change 期間不會發布 `/vehicle_2/staging_setpoint` 或 `/vehicle_3/staging_setpoint` 作為 follower formation targets。
- [ ] Follower slot identity 維持固定：`vehicle_2` 是 follower-left，`vehicle_3` 是 follower-right。
- [ ] Completion 使用 `formation_position_tolerance_m = 0.3` 和 `formation_yaw_tolerance_rad = 0.2`。
- [ ] Completion 要求 fresh leader/follower statuses，且 followers 必須到達由 leader state、target formation mode、fixed slots 推導出的 targets tolerance 內。
- [ ] Feedback `progress` 用 `0.0`、`0.5`、`1.0` 表示 follower 到位比例。
- [ ] 當所有 vehicles 都符合 formation tolerance 後，ground station 記錄 `formation established`。
- [ ] `ChangeFormation` action 只有在 fresh vehicle status 顯示 formation 到達 tolerance 後才成功。
- [ ] 測試覆蓋兩種 formation modes、invalid mode rejection、distributed follower setpoint derivation、completion behavior。
- [ ] 手動 SITL 驗證完成一張完整任務卡：`TakeoffSwarm -> MoveLeader/following -> verify vee -> ChangeFormation line_abreast -> verify line_abreast -> ChangeFormation vee -> verify vee -> LandSwarm`，且 followers 到達新 formation 後才 success。

## 測試方式

- Unit-test `vee` 和 `line_abreast` 的 slot offsets。
- Unit-test formation mode changes 只更新 follower-local offsets，不產生 ground-station absolute follower targets。
- Unit-test formation completion 會拒絕 stale vehicle status。
- Unit-test `ChangeFormation` 只在 `following` 階段接受。
- Unit-test invalid mode rejection。
- Unit-test stale leader status 和 stale follower status 不能 completion。
- Unit-test followers 未進入 `0.3m` position tolerance 或 `0.2rad` yaw tolerance 時不 success，到 tolerance 內才 success。
- Unit-test feedback progress 在零台、一台、兩台 followers 到位時分別是 `0.0`、`0.5`、`1.0`。
- ROS 2 package commands 要從 `px4_ws` workspace 執行，不是從外層 Docker workspace。
- 使用 fake follower status 測試 action feedback/result 和 completion detection。
- 手動 SITL 驗證要先從 clean runtime 開始，使用 ticket 07 manual smoke 文件中的 cleanup commands。
- 在 SITL 中手動驗證 followers 會在 `vee` 與 `line_abreast` 之間移動，而 leader 繼續定義 heading，最後 `LandSwarm` 成功。

## Blocking edges

- Blocked by 09 - Implement follower fixed-slot following from leader state。
- Blocks 12 - Add swarm launch and SITL smoke acceptance workflow。
