# 07 - 交付同步起飛到 staging 與 land-all 的 milestone

**要建立什麼：** 完成第一個 milestone：operator 可以觸發三台 vehicle 起飛到分開的 staging positions，看到 staging-complete progress message，並觸發 land-all。

**被誰阻擋：** 05 - Validate three-vehicle namespaces and telemetry flow；06 - Build `ground_station_node` action surface and swarm topics。

**狀態：** ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、`ros2 interface`、ROS 2 package tests、ROS 2 launch commands 都要在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。QGC 可以選擇性用來監控或人工安全觀察，但不是第一版的控制入口。

## 背景

第一個 milestone 會在實作 formation following 之前，驗證完整 infrastructure path：multi-vehicle SITL、ROS 2-PX4 bridge、三個 vehicle nodes、ground-station actions、staging geometry、status aggregation，以及安全的 land-all。

第一版 SITL milestone 不可以依賴 QGC 已開啟。QGC 仍然可以作為 optional monitoring / safety observation tool，但 `TakeoffSwarm` 驗收必須證明 ROS 2/PX4 path 在沒有 QGC 的情況下也能 arm 並起飛，不能讓 QGC 變成隱藏 runtime prerequisite。

## 範圍

- 透過 ground station 實作 whole-swarm takeoff coordination。
- 讓三台 vehicles 一起 arm/take off 到設定好的高度。
- 確保第一版 SITL `TakeoffSwarm` 不需要 QGC 開啟也可以起飛。
- 指令三台前往分開的 horizontal staging positions：
  - leader 在中央
  - follower-left 位於相對 leader initial yaw 的後左方
  - follower-right 位於相對 leader initial yaw 的後右方
- 偵測所有 vehicles 是否到達 staging positions。
- 由 ground station 記錄 `all vehicles reached staging positions`。
- 實作三台 vehicles 的 land-all action behavior。
- 為 takeoff 和 land-all 加上 action feedback/result。
- 新增 staging completion 和 land-all state transitions 的測試。

## 非目標

- 不實作 staging 之後的 leader movement。
- 不實作 follower following。
- 不實作 formation change。
- 不新增 dynamic slot assignment。
- 不從 launch 管理 PX4 SITL 或 Micro XRCE-DDS Agent。
- 不把 QGC 放進必要控制流程或必要啟動流程。

## 實作備註

- Staging 使用 world-frame positions 來保護起飛/降落時的分離距離。
- 在 staging completion checks 附近加入簡短中文註解，說明它防止什麼 collision/sequencing risk。
- 如果為了達成 no-QGC takeoff，需要修改 PX4 參數、ROS 2 Offboard sequencing、command `target_system`、或手動啟動流程，必須在 terminal 輸出一行清楚中文說明採用的修正方式。這行 log 要點出修正類別，例如 PX4 parameter/preflight behavior、Offboard heartbeat warmup、target-system mapping、或 startup sequencing。
- 如果實作時發現未開 QGC 無法 arm/takeoff，先用 tight feedback loop 診斷，再加 workaround。需要蒐集的證據包含 PX4 commander/preflight output、`VehicleCommandAck`、`VehicleStatus`、arming state、Offboard availability、Micro XRCE-DDS/PX4 namespace checks。

## 驗收條件

- [ ] `TakeoffSwarm` 會 arm/take off 三台 vehicles，並指令它們前往 staging positions。
- [ ] `TakeoffSwarm` 可以在 QGC 關閉的 SITL 中手動展示；QGC 不是 arm/takeoff 的必要 runtime dependency。
- [ ] Vehicles 使用相同目標高度，但不同 horizontal staging positions。
- [ ] `vehicle_2` 會 stage 到相對 leader initial yaw 的後左方。
- [ ] `vehicle_3` 會 stage 到相對 leader initial yaw 的後右方。
- [ ] 當三台都完成 staging 後，ground station 會記錄 `all vehicles reached staging positions`。
- [ ] `LandSwarm` 會指令三台 vehicles 降落。
- [ ] 第一個 milestone 可以在 Micro XRCE-DDS Agent 執行中的 SITL 裡手動展示。
- [ ] 如果實作修改 PX4 參數、ROS 2 Offboard sequencing、command `target_system`、或啟動流程來移除 QGC 依賴，terminal 會輸出一行簡短中文說明採用的修正方式。

## 測試方式

- Unit-test staging geometry 和 staging-complete tolerance checks。
- ROS 2 package commands 要從 `px4_ws` workspace 執行，不是從外層 Docker workspace。
- 使用 fake vehicle status streams 測試 ground-station takeoff 和 land-all action behavior。
- 當 PX4 SITL 和 Micro XRCE-DDS Agent 已經由外部啟動後，在 container 裡執行 manual SITL smoke test。
- 手動 SITL smoke test 要在 QGC 關閉的情況下執行，並記錄 no-QGC arm/takeoff 結果。
- 如果 no-QGC arm/takeoff 失敗，先使用 `$diagnosing-bugs` 捕捉並驗證 failure mode，再修改控制邏輯。

## Blocking edges

- Blocked by 05 - Validate three-vehicle namespaces and telemetry flow。
- Blocked by 06 - Build `ground_station_node` action surface and swarm topics。
- Blocks 08 - Implement leader movement by absolute world position plus yaw。
