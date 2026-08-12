# 03 - 實作 `Px4VehicleInterface` PX4 topic 邊界

**要建立什麼：** 新增一個共用的 internal interface，用來封裝每台 vehicle 的 PX4 `px4_msgs` topics，同時讓其他程式碼只看到內部的 swarm-control models。

**被誰阻擋：** 01 - Scaffold ROS 2 packages and interfaces；02 - Add internal models, state enums, and formation geometry。

**狀態：** ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、`ros2 interface`、ROS 2 package tests、ROS 2 launch commands 都要在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。QGC 可以選擇性用來監控或人工安全觀察，但不是第一版的控制入口。

## 背景

Micro XRCE-DDS Agent 是外部的 ROS 2-PX4 bridge。這個 application 不應該取代它，也不應該把它再包成另一個 node。相反地，每個 `vehicle_node` 應該使用共用的 `Px4VehicleInterface` class/module，讓它負責管理 `px4_msgs` topic 細節、QoS、timestamps、Offboard heartbeat、commands，以及 telemetry conversion。

## 範圍

- 實作一個可重複使用的 `Px4VehicleInterface` class/module，之後會在 vehicle nodes 裡使用。
- 封裝 `px4_msgs` publishers/subscribers，包括 Offboard control mode、trajectory setpoint、vehicle command、local position/odometry/status、必要時的 command ack，以及 telemetry freshness。
- 將 PX4 telemetry 轉換成內部的 `VehicleState`。
- 將內部的 `PositionYawSetpoint` 轉換成 PX4 Offboard position+yaw setpoint publication。
- 提供 Offboard heartbeat、arm/disarm、takeoff、land，以及 safe hover/hold setpoint 的方法。
- 處理每台 vehicle 的 PX4 namespace configuration。
- 新增聚焦的測試，在不需要 live PX4 的情況下驗證 conversion 和 topic-boundary behavior。

## 非目標

- 不實作 leader/follower behavior。
- 不實作 mission-level actions。
- 不啟動或管理 Micro XRCE-DDS Agent。
- 不修改 PX4-Autopilot。

## 實作備註

- 將 raw `px4_msgs` imports 限制在 PX4 boundary 內。
- 在有風險的 conversion 和 command publication 附近加入簡短中文註解，說明為什麼需要這個 conversion、heartbeat 或 command guard。
- 註解不應該逐行重述程式碼。

## 驗收條件

- [ ] Controller-facing code 可以使用 internal models，而不需要 import raw `px4_msgs`。
- [ ] `Px4VehicleInterface` 會透過 PX4 topics 發布 position+yaw setpoints 和 Offboard heartbeat。
- [ ] `Px4VehicleInterface` 可以送出 arm、disarm、takeoff、land commands。
- [ ] Telemetry conversion 會填入 internal vehicle state，包括 position、yaw、velocity、armed/nav state、Offboard availability，以及 telemetry age。
- [ ] Namespace handling 支援三個分開的 PX4 vehicle topic roots。
- [ ] Unit tests 覆蓋 PX4-to-internal 和 internal-to-PX4 mapping。
- [ ] 關鍵 conversion/command paths 有有意義的一行中文註解。

## 測試方式

- 盡可能使用 fake ROS 2 publishers/subscribers 或 isolated conversion helpers 做 unit tests。
- ROS 2 package commands 要從 `px4_ws` workspace 執行，不是從外層 Docker workspace。
- 驗證產生出來的 PX4 messages 具有預期的 position、yaw、timestamps，以及 command fields。
- 驗證 stale telemetry detection，不需要 live PX4。
- SITL validation 留到後面的 tickets。

## Blocking edges

- Blocked by 01 - Scaffold ROS 2 packages and interfaces。
- Blocked by 02 - Add internal models, state enums, and formation geometry。
- Blocks 04 - Build single parameterized `vehicle_node` for one vehicle。
