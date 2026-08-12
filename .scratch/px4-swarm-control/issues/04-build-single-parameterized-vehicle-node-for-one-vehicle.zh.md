# 04 - 建立單一可參數化的 `vehicle_node` 來控制一台 vehicle

**要建立什麼：** 建立一個可重複使用的 ROS 2 `vehicle_node` executable。它可以透過參數代表 leader 或 follower vehicle，並且可以透過 `Px4VehicleInterface` 控制一台 PX4 vehicle。

**被誰阻擋：** 02 - Add internal models, state enums, and formation geometry；03 - Implement `Px4VehicleInterface` PX4 topic boundary。

**狀態：** ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、`ros2 interface`、ROS 2 package tests、ROS 2 launch commands 都要在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。QGC 可以選擇性用來監控或人工安全觀察，但不是第一版的控制入口。

## 背景

Runtime design 使用三個相同 `vehicle_node` 的 instance，而不是分開寫 leader executable 和 follower executable。每個 instance 透過 `role`、`vehicle_id`、`px4_namespace`、`slot` 來設定。這張 ticket 會先建立「單台 vehicle」的垂直控制路徑，之後才擴展到三台 vehicle。

## 範圍

- 實作一個可參數化的 `vehicle_node` executable。
- 接收 role、vehicle ID、PX4 namespace、slot、control loop rate、staging/hold setpoint defaults，以及需要時的 initial yaw handling 等參數。
- 使用 `Px4VehicleInterface` 發布 Offboard heartbeat 和 position+yaw setpoints。
- 發布 vehicle status summary topic。
- 如果 role 是 leader，接收 leader goal topic input。
- 如果 role 是 follower，先初始化 follower behavior structure，但還不實作完整 following。
- 維護 vehicle-level state，並記錄 local state transitions。
- 新增測試，驗證 parameter parsing、status publication shape，以及 basic state transitions。

## 非目標

- 不同時執行三台 vehicles。
- 不實作 synchronized takeoff。
- 不實作 follower following。
- 不實作 ground-station actions。
- 不管理 PX4 SITL 或 Micro XRCE-DDS Agent。

## 實作備註

- executable 要保持用 role 參數切換，而不是拆成不同的 leader/follower entrypoints。
- 在 state guard 防止不安全 command publication，或保護 Offboard timing assumptions 的地方，加入簡短中文註解。

## 驗收條件

- [ ] 同一個 executable 可以透過參數以 leader 身分啟動。
- [ ] 同一個 executable 可以透過參數以 follower 身分啟動。
- [ ] node 使用 `Px4VehicleInterface`，而不是把 raw PX4 topic logic 散落在 controller code 裡。
- [ ] node 會發布 status summary，內容包含 role、vehicle ID、pose/yaw/velocity 如果可用、armed/nav state、Offboard availability、telemetry age，以及 vehicle-level state。
- [ ] 當 PX4 bridge 前置條件已啟動時，node 可以對單台 vehicle hold 或 command 一個 position+yaw setpoint。
- [ ] Local state transitions 會被記錄，但不會洗版 terminal。
- [ ] 測試覆蓋 parameter validation 和 basic node behavior。

## 測試方式

- 執行 parameter handling 和 vehicle-level state transitions 的 unit tests。
- ROS 2 package commands 要從 `px4_ws` workspace 執行，不是從外層 Docker workspace。
- 如果實務上可行，使用 fake 或 simulated `Px4VehicleInterface` 執行 ROS 2 node-level tests。
- 在 PX4 SITL 和 Micro XRCE-DDS Agent 已經啟動後，可以選擇性在 container 裡做 manual single-vehicle SITL smoke check。

## Blocking edges

- Blocked by 02 - Add internal models, state enums, and formation geometry。
- Blocked by 03 - Implement `Px4VehicleInterface` PX4 topic boundary。
- Blocks 05 - Validate three-vehicle namespaces and telemetry flow。
