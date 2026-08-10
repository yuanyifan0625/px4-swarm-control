# 01 - 建立 ROS 2 packages 與 interfaces 骨架

**要建置什麼：** 為 swarm-control 工作建立可 build 的 ROS 2 Jazzy 基礎：一個 Python control package、一個 interfaces package，以及 ground station 和 vehicle nodes 第一版需要的 action/message 合約。

**被誰阻擋：** 無，可以立刻開始。

**狀態：** ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/`。`colcon`、`ros2 interface`、ROS 2 package tests 和 ROS 2 launch commands 都要在 container 內的 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。QGC 只作為 optional monitoring/manual safety observation，不是第一版控制入口。

## 背景

第一版會在 SITL 中執行，包含一個 `ground_station_node` 和三個可參數化的 `vehicle_node` instances。Operator commands 應透過 ROS 2 actions 進入系統，而內部 node communication 使用 topics。Generated ROS 2 interfaces 應與 Python node logic 分開。

## 範圍

- 建立一個 Python/rclpy control package，用於 nodes、shared modules、launch、config 和 tests。
- 建立一個獨立 ROS 2 interfaces package，用於 actions 和 messages。
- 定義初始 operator actions：`TakeoffSwarm`、`MoveLeader`、`ChangeFormation`、`PauseSwarm` 和 `LandSwarm`。
- 定義初始 internal messages，用於 leader goal、formation mode、mission/failsafe command 和 vehicle status summary。
- 確保兩個 packages 都能在 Docker container 內 build。
- 加入 package-level test scaffolding，讓後續 tickets 可以加入 unit 和 integration tests。

## 非目標

- 這張 ticket 不實作 PX4 communication。
- 這張 ticket 不實作 vehicle control behavior。
- 不修改 PX4-Autopilot 或 upstream ROS-PX4 packages。
- 這張 ticket 不從這裡啟動 PX4 SITL 或 Micro XRCE-DDS Agent。

## 實作備註

- Interface names 應保持穩定且 role-independent；leadership 應以 state/parameters 表示，不要圍繞 `leader` hard-code topic names。
- 只有在 interface choice 不明顯時，才加入簡短中文註解。註解應說明合約為什麼存在，或它避免了什麼歧義。

## 驗收條件

- [ ] control package 能在 container 中成功 build。
- [ ] interfaces package 能在 container 中成功 build。
- [ ] 五個 operator actions 存在，且 fields 足以支援 takeoff、absolute leader movement plus yaw、formation change、pause 和 land-all。
- [ ] internal messages 存在，用於 leader goal、formation mode、mission/failsafe command 和 vehicle status summary。
- [ ] Package tests 可以透過 container workflow 呼叫。
- [ ] 沒有任何 PX4-Autopilot 檔案被修改。

## 測試方式

- 在 container 內執行 package build commands。
- ROS 2 package commands 要從 `px4_ws` workspace 執行，不要從外層 Docker workspace 執行。
- 執行初始 package test suite，即使它目前只驗證 scaffolding/imports。
- 在 container 內使用 ROS 2 tooling 檢查 generated interfaces，確認 actions/messages 可用。

## Blocking edges

- 阻擋 02 - Add internal models, state enums, and formation geometry。
- 阻擋 03 - Implement `Px4VehicleInterface` PX4 topic boundary。
- 阻擋 06 - Build `ground_station_node` action surface and swarm topics。
