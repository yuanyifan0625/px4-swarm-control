# 06 - 建立 `ground_station_node` action 介面與 swarm topics

**要建立什麼：** 建立 ground-station node，提供 operator actions、管理 mission-level state，並透過 topics 和 vehicle nodes 做內部溝通。

**被誰阻擋：** 01 - Scaffold ROS 2 packages and interfaces；02 - Add internal models, state enums, and formation geometry。

**狀態：** ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、`ros2 interface`、ROS 2 package tests、ROS 2 launch commands 都要在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。QGC 可以選擇性用來監控或人工安全觀察，但不是第一版的控制入口。

## 背景

Operator commands 應該透過 ROS 2 actions 進入系統。Ground station 到 vehicle nodes 的內部溝通應該使用 topics，這樣比較容易觀察，也比較簡單。Ground station 負責監督整個 swarm，但不計算連續的 follower setpoints。

## 範圍

- 實作 `ground_station_node`。
- 新增 `TakeoffSwarm`、`MoveLeader`、`ChangeFormation`、`PauseSwarm`、`LandSwarm` action servers。
- 透過 internal topics 發布 mission command、leader goal、formation mode，以及 failsafe/pause commands。
- 訂閱 vehicle status summaries。
- 維護 mission-level state。
- 針對長時間執行的 commands，提供合適的 action feedback/results。
- 記錄 mission-level progress，但避免每台 vehicle 的細節 log 洗版。
- 新增 tests，驗證 action acceptance/rejection、topic publication，以及 mission-state transitions。

## 非目標

- 不計算 follower continuous setpoints。
- 不實作真正的 takeoff completion logic。
- 不直接 command PX4。
- 不管理 PX4 SITL 或 Micro XRCE-DDS Agent。

## 實作備註

- Mission-level state 屬於這裡；vehicle-level state 屬於 vehicle nodes。
- 在 state-transition guards 附近加入一行中文註解，說明它防止哪一種 invalid mission transition。

## 驗收條件

- [ ] ground-station node 可以在 `/swarm` namespace 底下啟動。
- [ ] 五個 operator actions 都存在，並且回傳清楚的 feedback/results。
- [ ] Ground-station-to-vehicle outputs 使用 topics，而不是下游 actions。
- [ ] Vehicle status summaries 會被 consumed，並反映到 mission-level state。
- [ ] Ground station 只記錄 mission-level transitions。
- [ ] 測試覆蓋 action command handling 和 mission-state transitions。

## 測試方式

- 使用 ROS 2 action clients 或 tests 送出每個 action，並驗證 feedback/result behavior。
- ROS 2 package commands 要從 `px4_ws` workspace 執行，不是從外層 Docker workspace。
- 使用 fake vehicle status publishers 來觸發 mission-state transitions。
- 驗證 action 被呼叫時，預期的 internal topics 有被發布。

## Blocking edges

- Blocked by 01 - Scaffold ROS 2 packages and interfaces。
- Blocked by 02 - Add internal models, state enums, and formation geometry。
- Blocks 07 - Deliver synchronized takeoff to staging and land-all milestone。
