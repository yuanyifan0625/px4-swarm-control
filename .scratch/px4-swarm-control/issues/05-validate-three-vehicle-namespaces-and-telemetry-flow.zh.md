# 05 - 驗證三台 vehicle 的 namespace 與 telemetry 流程

**要建立什麼：** 啟動三個已設定好的 `vehicle_node` instance，分別對應 `/vehicle_1`、`/vehicle_2`、`/vehicle_3`，並驗證每個 instance 都能接收/發布正確的單機 telemetry/status，不會發生 namespace cross-talk。

**被誰阻擋：** 04 - Build single parameterized `vehicle_node` for one vehicle。

**狀態：** ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、`ros2 interface`、ROS 2 package tests、ROS 2 launch commands 都要在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。QGC 可以選擇性用來監控或人工安全觀察，但不是第一版的控制入口。

## 背景

第一個 infrastructure risk 是 multi-vehicle namespace 是否正確。在建立 swarm actions 之前，每個 `vehicle_node` instance 都必須乾淨地對應到自己的 PX4 vehicle namespace，並發布自己的 status summary。

## 範圍

- 設定三個 vehicle node instances：
  - `/vehicle_1`：leader
  - `/vehicle_2`：follower-left slot 1
  - `/vehicle_3`：follower-right slot 2
- 驗證每個 instance 都使用自己設定的 PX4 namespace。
- 驗證每個 status summary topic 都從自己的 telemetry stream 更新。
- 新增三個 node setup 的手動啟動文件或 debug commands。
- 新增 tests 或 smoke checks，用來抓出 namespace 對錯交換或 vehicle ID 混用的問題。

## 非目標

- 還不實作 ground-station actions。
- 還不實作 synchronized takeoff。
- 還不實作 follower following。
- 還不把 launch-all 作為正常工作流程。

## 實作備註

- namespace 和 role assignment 要在 configuration 裡明確寫清楚。
- 在任何 namespace-to-vehicle mapping guard 附近加入簡短中文註解，說明它是為了防止跨 vehicle 的 command/telemetry 混用。

## 驗收條件

- [ ] 三個 `vehicle_node` instances 可以同時執行，並且各自有不同的 role、vehicle ID、PX4 namespace、slot 參數。
- [ ] `/vehicle_1` 發布 leader status。
- [ ] `/vehicle_2` 發布 follower-left slot 1 status。
- [ ] `/vehicle_3` 發布 follower-right slot 2 status。
- [ ] 某一台 vehicle 的 telemetry/status 不會出現在另一台 vehicle 的 status topic 底下。
- [ ] 手動 debug steps 有指出 PX4 SITL 和 Micro XRCE-DDS Agent 是外部前置條件。
- [ ] 沒有修改 PX4-Autopilot 的 cooperative-control 行為。

## 測試方式

- 在 container 裡使用 ROS 2 topic inspection 確認三條 status streams。
- package 和 topic-inspection commands 要從 `px4_ws` workspace 執行，不是從外層 Docker workspace。
- 盡可能使用 fake interfaces 或 controlled telemetry 來偵測 namespace 是否交換。
- 在 SITL 執行時，驗證每個 status topic 都對應到預期的 vehicle instance。

## Blocking edges

- Blocked by 04 - Build single parameterized `vehicle_node` for one vehicle。
- Blocks 07 - Deliver synchronized takeoff to staging and land-all milestone。
