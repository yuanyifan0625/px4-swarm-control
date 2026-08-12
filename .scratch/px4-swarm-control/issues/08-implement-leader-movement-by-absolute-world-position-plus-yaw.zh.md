# 08 - 實作 leader 以 absolute world position 加 yaw 移動

**要建立什麼：** 讓 operator 可以透過 `MoveLeader` 將 leader 移動到 absolute world-frame position 和 yaw，同時 followers 保持 staged 或 holding。

**被誰阻擋：** 07 - Deliver synchronized takeoff to staging and land-all milestone；07b - Fix takeoff-to-offboard staging sequencing；07c - Fix repeatable TakeoffSwarm/LandSwarm mission cycles and stale landed/staging state。

**狀態：** ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、`ros2 interface`、ROS 2 package tests、ROS 2 launch commands 都要在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。QGC 可以選擇性用來監控或人工安全觀察，但不是第一版的控制入口。

## 背景

Leader movement 是 takeoff/staging/landing 成功之後的下一個 vertical slice。第一版使用 absolute world position 加 yaw，因為它比 relative commands 更容易在 Gazebo/RViz 裡檢查。QGC 可以選擇性用作監控和人工安全觀察工具，但不是控制入口。

Ticket 07c 的前車之鑑是：action success、stale status、topic timing 都要小心處理。`MoveLeader` 必須用 fresh leader status 判斷完成，不能靠舊 mission state 或「命令已發布」就回成功，而且要能在 clean runtime 或上一輪任務後重複執行。

這張 ticket 也要保護 distributed follower-control 邊界：`MoveLeader` 是 operator 對 leader 的命令。Followers 在這張 ticket 裡只能保持 staged 或 holding，不能把 `/swarm/leader_goal` 或 ground station 給的 absolute position 當成自己的目標。

## 範圍

- 在 ground station 實作 `MoveLeader` action behavior。
- 以 absolute world position 加 yaw 的形式發布 internal leader goal。
- 讓 leader vehicle node 消費 leader goal，並透過 `Px4VehicleInterface` command PX4。
- 讓 followers 維持 staged 或 holding；這張還不讓它們 follow。
- 確保 follower vehicle nodes 在這張 ticket 中不會把 leader goal 當成自己的 movement target。
- 根據 leader progress 提供 action feedback/result。
- 新增 leader goal publication、leader acceptance、completion conditions 的測試。
- 新增手動驗證卡，證明完整 SITL 任務可以完成：clean runtime、takeoff 到 staging、只移動 leader、確認 followers hold、最後 land all。

## 非目標

- 不實作 relative movement commands。
- 不實作 follower following。
- 不實作 formation change。
- 不重新分配 leader。
- 不讓 ground station 為 followers 計算或持續發布 absolute follower target positions。
- 不讓 followers 把 `/swarm/leader_goal` 當成自己的 movement target。

## 實作備註

- Leader goal 要保持 world-frame 且明確，避免隱藏的 relative movement semantics。
- 套用 ticket 07c 經驗：action completion 必須根據 fresh telemetry 和 tolerance，不能根據舊 cached status 或 command publication。
- Takeoff/staging 的 per-vehicle setpoints 是第一版唯一例外，因為它是為了 collision-safe staging。
- 在 goal validation 和 completion tolerance 附近加入一行中文註解，說明它防止什麼 operator ambiguity 或 oscillation。

## 驗收條件

- [ ] `MoveLeader` 接受 absolute target x/y/z/yaw。
- [ ] Ground station 透過 internal topic 發布 leader goal。
- [ ] `/vehicle_1` leader node 消費該 goal，並送出 position+yaw setpoints。
- [ ] action 回報 progress，並且只有在 fresh `/vehicle_1/status` 顯示 leader 到達 tolerance 時成功。
- [ ] Followers 不會在這張 ticket 裡開始 formation following。
- [ ] Followers 不會把 `/swarm/leader_goal` 當成自己的 movement target；leader 移動時 followers 維持 staged 或 holding。
- [ ] Ground station 在 `MoveLeader` 期間不會持續發布 absolute follower targets。
- [ ] 測試覆蓋 accepted/rejected leader goals 和 completion behavior。
- [ ] 手動 SITL 驗證完成一張完整任務卡：`TakeoffSwarm -> MoveLeader -> LandSwarm`，且 `MoveLeader` 期間只有 leader 移動。

## 測試方式

- Unit-test leader goal validation 和 completion tolerance。
- Unit-test follower roles 會忽略 leader goals，不會當成 follower movement targets。
- Unit-test `MoveLeader` completion 需要 fresh leader status，不能用 stale cached status 完成。
- ROS 2 package commands 要從 `px4_ws` workspace 執行，不是從外層 Docker workspace。
- 使用 fake vehicle status 測試 action feedback/result。
- 手動 SITL 驗證要先從 clean runtime 開始，使用 ticket 07 manual smoke 文件中的 cleanup commands。
- 在 SITL 中手動驗證：staging 後 leader 會移動，而 followers 維持 hold，最後 `LandSwarm` 成功。

## Blocking edges

- Blocked by 07 - Deliver synchronized takeoff to staging and land-all milestone。
- Blocked by 07b - Fix takeoff-to-offboard staging sequencing。
- Blocked by 07c - Fix repeatable TakeoffSwarm/LandSwarm mission cycles and stale landed/staging state。
- Blocks 09 - Implement follower fixed-slot following from leader state。
