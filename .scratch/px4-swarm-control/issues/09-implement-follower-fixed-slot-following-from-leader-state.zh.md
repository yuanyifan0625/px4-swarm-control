# 09 - 根據 leader state 實作 follower fixed-slot following

**要建立什麼：** 讓 `vehicle_2` 和 `vehicle_3` 訂閱 leader position、yaw、velocity、status，並由此推導出各自 fixed-slot 的 position+yaw setpoints 來跟隨 leader。

**被誰阻擋：** 08 - Implement leader movement by absolute world position plus yaw。

**狀態：** ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、`ros2 interface`、ROS 2 package tests、ROS 2 launch commands 都要在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。QGC 可以選擇性用來監控或人工安全觀察，但不是第一版的控制入口。

## 背景

此架構是 logically distributed：每個 follower 會根據 leader state 和自己設定好的 slot 計算自己的 setpoint。Ground station 不計算連續的 follower setpoints。

Ticket 07c 的前車之鑑是：stale status、stale mission state、一次性的 topic publication 可能讓任務看起來完成，但飛機其實還沒到正確狀態。因此 follower following 必須使用 fresh leader state，leader timeout 時不能追 stale leader command，並且要能在 takeoff/land cycles 後重複執行。

Ticket 08 故意只移動 leader。這張 ticket 才是 followers 第一次可以因 leader movement 而移動；但 followers 只能訂閱 leader state 和 formation mode，然後在本地計算自己的 fixed-slot setpoint。

## 範圍

- 發布或暴露 leader state，讓 followers 可以訂閱 position、yaw、velocity、status。
- 發布或暴露 formation mode，讓 followers 可以在本地選擇目前 slot offset。
- 使用 leader body-frame offsets 實作 fixed slots 的 follower setpoint derivation。
- 以 `vee` formation 作為預設 following mode。
- 確保 `vehicle_2` 維持 follower-left slot 1，`vehicle_3` 維持 follower-right slot 2。
- 每個 follower 只透過自己的 `Px4VehicleInterface` 輸出自己的 position+yaw setpoint。
- 確保 followers 永遠不把 `/swarm/leader_goal` 或 ground station 給的 absolute leader target 當成自己的目標。
- 新增 follower setpoint derivation 測試；若沒有延後到 ticket 11，也測 leader timeout hover behavior。
- 新增手動驗證卡，證明完整 SITL 任務可以完成：clean runtime、takeoff 到 staging、移動 leader、followers 維持 `vee` slots、最後 land all。

## 非目標

- 不實作 `ChangeFormation`。
- 不實作 follower-follower coordination。
- 不實作 dynamic slot assignment。
- 不實作 automatic leader reassignment。
- 不讓 ground station 在 following 階段計算或持續發布 absolute follower target positions。
- 不把 operator leader goals 當成 follower direct goals。

## 實作備註

- Body-frame offsets 會在 leader yaw 改變時保護 formation orientation。
- Followers 會把自己的 body-frame slot offset 用 current leader yaw 旋轉成 world-frame offset，再加到 current leader world position。
- 套用 ticket 07c 經驗：follower outputs 必須根據 fresh leader status；leader timeout 或 telemetry stale 時要 safe hover/hold，而不是追舊資料。
- Takeoff/staging 是唯一例外，ground station 可以為 collision safety 發布 per-vehicle absolute staging positions。
- 在 follower setpoint derivation 附近加入一行中文註解，說明為什麼 offset 要用 leader yaw 旋轉。

## 驗收條件

- [ ] Followers 訂閱 leader position、yaw、velocity、status。
- [ ] Followers 訂閱 current formation mode，並用它選擇自己的 local fixed-slot offset。
- [ ] `vehicle_2` 根據 leader state 計算 follower-left slot setpoint。
- [ ] `vehicle_3` 根據 leader state 計算 follower-right slot setpoint。
- [ ] Followers 只發布自己的 position+yaw setpoints。
- [ ] Ground station 不計算連續的 follower setpoints。
- [ ] Ground station 在 following 階段不會持續發布 absolute follower target positions。
- [ ] Followers 不會把 `/swarm/leader_goal` 當成自己的目標。
- [ ] 當 leader movement 啟用時，followers 會追蹤固定的 `vee` slots。
- [ ] 測試驗證兩個 follower slots 在多個 leader yaw values 下的 setpoint derivation。
- [ ] 手動 SITL 驗證完成一張完整任務卡：`TakeoffSwarm -> MoveLeader with follower following -> LandSwarm`，且 followers 維持 default `vee` offsets。

## 測試方式

- Unit-test 從 internal models 推導 follower setpoint。
- Unit-test followers 會忽略 absolute leader goals，並改用 leader state 加 formation mode。
- Unit-test leader stale/timeout behavior，確保 followers hold 或 hover，而不是追 stale leader state。
- ROS 2 package commands 要從 `px4_ws` workspace 執行，不是從外層 Docker workspace。
- 使用 fake leader state streams 驗證 follower outputs。
- 手動 SITL 驗證要先從 clean runtime 開始，使用 ticket 07 manual smoke 文件中的 cleanup commands。
- 在 SITL 中手動驗證 followers 會跟隨 leader movement，同時維持預設 `vee` offsets，最後 `LandSwarm` 成功。

## Blocking edges

- Blocked by 08 - Implement leader movement by absolute world position plus yaw。
- Blocks 10 - Implement `ChangeFormation` between `vee` and `line_abreast`。
- Blocks 11 - Add minimal pause and failsafe behavior。
