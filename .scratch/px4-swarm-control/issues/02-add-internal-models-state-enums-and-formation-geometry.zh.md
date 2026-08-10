# 02 - 新增 internal models、state enums 與 formation geometry

**要建置什麼：** 新增內部 domain models 和 geometry helpers，讓 vehicle 與 ground-station logic 可以用 swarm-control 概念溝通，而不是直接使用 raw PX4 messages。

**被誰阻擋：** 01 - Scaffold ROS 2 packages and interfaces。

**狀態：** ready-for-agent

**Workspace note：** Project ROS 2 packages 位於 `px4_ws/src/`。請在 container 內的 `/home/ncrl/docker_ubuntu24/px4_ws` 執行 `colcon`、`ros2 interface`、ROS 2 package tests 和 ROS 2 launch commands。QGC 可選，用於 monitoring/manual safety observation，但不是第一版 control entrypoint。

## 背景

Spec 要求 `px4_msgs` 留在 `Px4VehicleInterface` 後面。Control logic 應使用內部概念，例如 `VehicleState`、`PositionYawSetpoint`、formation modes、vehicle roles、mission states 和 vehicle states。Formation geometry 必須保留 left/right slot 的意義，並區分 world-frame staging 與 leader body-frame following。

## 範圍

- 定義 vehicle state、position+yaw setpoints、command results、roles、slots、mission states 和 vehicle-level states 的 internal data models。
- 實作 `vee` 和 `line_abreast` 的 formation geometry。
- 根據 leader initial yaw 實作 world-frame staging position calculation。
- 根據目前 leader yaw 實作 leader body-frame offset transformation。
- 保持 `vehicle_2` 為 follower-left slot 1，`vehicle_3` 為 follower-right slot 2。
- 為 geometry 和 enum/model behavior 加 unit tests。

## 非目標

- 這張 ticket 還不 publish 或 subscribe ROS 2 topics。
- 不命令 PX4。
- 不實作 follower control loops。
- 不加入 `column` formation。
- 不實作 dynamic leader 或 slot reassignment。

## 實作備註

- 在 coordinate-frame 和 yaw transformations 附近加入一行中文註解，說明該 transformation 保護哪個 safety/consistency property。
- Geometry 優先使用簡單 pure functions，讓 tests 不需要 ROS 2 runtime。

## 驗收條件

- [ ] Internal models 能表示 role、vehicle ID、PX4 namespace、slot、vehicle state、mission state、command result 和 position+yaw setpoint。
- [ ] `vee` 和 `line_abreast` slots 能為 left/right followers 產生 deterministic offsets。
- [ ] Staging positions 使用由 leader initial yaw 推導出的 world-frame positions。
- [ ] Following positions 使用由目前 leader yaw 推導出的 leader body-frame offsets。
- [ ] Unit tests 驗證 left/right sign conventions、leader initial yaw staging，以及 leader current yaw body-frame following。
- [ ] Geometry/controller-facing model tests 不需要 raw `px4_msgs` types。

## 測試方式

- 執行 geometry unit tests，不需要 PX4、Gazebo 或 Micro XRCE-DDS Agent。
- 從 `px4_ws` workspace 執行 ROS 2 package commands，而不是外層 Docker workspace。
- 包含 zero yaw、positive yaw、negative yaw 和 left/right follower slots 的 cases。
- 包含一個 regression case，證明 follower-left 會根據 leader initial heading 開始在 leader 左側。

## Blocking edges

- 被 01 - Scaffold ROS 2 packages and interfaces 阻擋。
- 阻擋 03 - Implement `Px4VehicleInterface` PX4 topic boundary。
- 阻擋 04 - Build single parameterized `vehicle_node` for one vehicle。
- 阻擋 06 - Build `ground_station_node` action surface and swarm topics。
