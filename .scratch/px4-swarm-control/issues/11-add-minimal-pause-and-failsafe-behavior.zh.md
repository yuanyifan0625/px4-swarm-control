# 11 - 新增最小版 pause 與 failsafe behavior

**要建立什麼：** 新增第一版 safety behavior：不重啟 runtime 的 pause、land-all recovery、vehicle telemetry timeout hover/hold，以及 leader timeout 時 followers hover。

**被誰阻擋：** 09 - Implement follower fixed-slot following from leader state。

**狀態：** ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、`ros2 interface`、ROS 2 package tests、ROS 2 launch commands 都要在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。QGC 可以選擇性用來監控或人工安全觀察，但不是第一版的控制入口。

## 背景

Spec 有意限制第一版 failsafe behavior。系統應避免 stale commands，並給 operator 一個簡單的介入路徑，但不實作複雜的 autonomous recovery。

Ticket 07c 已經證明 mission state 必須可以在不重啟 DDS、PX4、Gazebo、vehicle nodes、ground station 的情況下重複運作。Pause/failsafe 也要遵守同一個原則：正常 pause 是 runtime state transition，不是重啟整套模擬環境的理由。

## 範圍

- 實作 `PauseSwarm` behavior，讓 vehicles 在不重啟 Micro XRCE-DDS Agent、PX4 SITL、Gazebo、vehicle nodes、ground station 的情況下 hold safe setpoints。
- 實作 `PauseSwarm(pause=false)` 的 resume 語意：resume 後回到安全 holding state，不自動繼續 pause 前的舊 movement 或 formation command。
- 確保 `LandSwarm` 仍可作為 operator recovery。
- 在 paused 或 failsafe state 中，只要 PX4 telemetry 和 command path 還可用，就仍允許 `LandSwarm`。
- 偵測 vehicle telemetry timeout，並讓受影響的 vehicle hover 或保持最後安全 setpoint。
- 偵測 leader telemetry/status timeout，並讓 followers hover。
- 在 vehicle status 和 ground-station mission state 中反映 pause/failsafe states。
- 限制 paused state 的 operator 行為：允許 status observation、resume、land-all；拒絕新的 leader movement、formation change、demo/macro progression。
- 新增 timeout 和 pause/land behavior 的測試。

## 非目標

- 不實作 automatic leader reassignment。
- 不實作 dynamic slot reassignment。
- 不實作複雜 autonomous recovery。
- 不實作超出 fixed spacing/staging assumptions 之外的 collision avoidance。
- 不在本 ticket 新增短數字 operator console；那屬於 ticket 11b。
- 不讓 pause 重啟或 respawn DDS、PX4、Gazebo、vehicle nodes、ground station。
- 不在 resume 後自動繼續 stale pre-pause mission。

## 實作備註

- Timeout guards 會防止 stale leader 或 vehicle data 產生不安全 setpoints。
- Pause 是 operator 主動要求 hold；failsafe 是系統偵測 stale data 後的 hold。兩者可以共享 hover/hold 行為，但 status/log 要保留不同原因。
- Resume 應該清除 pause command，並讓 vehicles 回到安全 holding state。Operator 之後可以再送新的 `MoveLeader`、`ChangeFormation` 或 `LandSwarm`。
- Paused 狀態下若繼續舊 leader goal 或 formation change，容易讓 operator 誤判系統正在停住；因此 movement 和 formation actions 要等 resume 成功後才允許。
- 在 timeout thresholds 和 hover/hold fallbacks 附近加入一行中文註解，說明它防止什麼 stale-data risk。

## 驗收條件

- [ ] `PauseSwarm` 讓 vehicles hold safe setpoints，而不是繼續 mission progression。
- [ ] `PauseSwarm` 不需要重啟 Micro XRCE-DDS Agent、PX4 SITL、Gazebo、vehicle nodes 或 ground station。
- [ ] `PauseSwarm(pause=false)` resume 後進入安全 holding state，不自動繼續 stale pre-pause movement 或 formation action。
- [ ] Paused 狀態允許 status observation、resume、`LandSwarm`。
- [ ] Paused 狀態清楚拒絕 `MoveLeader`、`ChangeFormation` 和 demo/macro progression。
- [ ] `LandSwarm` 在 paused/failsafe states 中可行時仍保持可用。
- [ ] Vehicle telemetry timeout 會讓該 vehicle hover 或 hold 最後安全 setpoint。
- [ ] Leader telemetry/status timeout 會讓 followers hover。
- [ ] Ground station 和 vehicle statuses 會暴露 pause/failsafe state。
- [ ] 測試覆蓋 timeout、pause、land-all paths。
- [ ] 手動 SITL 驗證不使用 QGC 作為控制入口，完成：`TakeoffSwarm -> MoveLeader/following -> PauseSwarm -> verify hold -> paused 時拒絕 MoveLeader/ChangeFormation -> resume to holding -> 發送新的 MoveLeader 或 ChangeFormation -> PauseSwarm -> LandSwarm`，全程不重啟 runtime。

## 測試方式

- 使用 controlled timestamps unit-test timeout state transitions。
- ROS 2 package commands 要從 `px4_ws` workspace 執行，不是從外層 Docker workspace。
- 使用 fake telemetry/status streams 驗證 followers 在 leader timeout 時會停止 following。
- Unit-test paused-state action gating：movement 和 formation 被拒絕，status/resume/land 保持可用。
- Unit-test resume semantics，確保舊的 pre-pause goals 不會自動繼續。
- 在 follower following 可用後，於 SITL 中手動測試 pause、resume、paused 時拒絕命令、resume 後新命令、land-all。

## Blocking edges

- Blocked by 09 - Implement follower fixed-slot following from leader state。
- Blocks 12 - Add swarm launch and SITL smoke acceptance workflow。
