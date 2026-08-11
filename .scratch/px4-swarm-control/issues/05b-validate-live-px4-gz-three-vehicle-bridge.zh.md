# 05b - 驗證真實 PX4 Gz 三機 bridge

**要建立什麼：** 證明第一版 namespace 設計可以對上真實 PX4 Gz SITL，而不只是三個 ROS 2 `vehicle_node` instance。開發者應該能啟動三個真正的 `gz_x500` PX4 SITL instance，在 Gazebo 看到三台飛機，確認三個 PX4 client 都接到同一個 Micro XRCE-DDS Agent，並確認 ROS 2 telemetry topics 真的有 PX4 publisher。

**被誰阻擋：** 05 - Validate three-vehicle namespaces and telemetry flow。

**狀態：** ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、`ros2 interface`、ROS 2 package tests、ROS 2 launch commands、PX4 SITL commands、Gazebo commands、Micro XRCE-DDS Agent commands 都要在 container 裡執行。QGC 可以選擇性用來監控或人工安全觀察，但不是第一版的控制入口。

## 背景

Ticket 05 驗證的是 ROS 2 這一側三個已設定好的 `vehicle_node` instance，但後續診斷發現 `make px4_sitl gz_x500` 只會啟動一台 PX4/Gazebo vehicle。三個 `vehicle_node` instance 也可能因為自己建立 subscriber，讓 ROS 2 topic 看起來存在，即使 PX4 其實沒有發布 telemetry。在實作 swarm takeoff 前，專案需要一個 live SITL 證明：三個 PX4 instance、三個 Gazebo model、Micro XRCE-DDS Agent、ROS 2 topic 邊界都使用一致的 namespace 和 topic name。

## 範圍

- 文件化一套手動啟動三台 PX4 Gz 的流程，而且不修改 PX4-Autopilot 原始碼。
- 啟動三個 PX4 SITL `gz_x500` instance，並且每個 instance 使用不同 PX4 instance ID 和分開的 spawn position。
- 使用 `PX4_UXRCE_DDS_NS`，讓 ROS 2 namespace 維持 `/vehicle_1`、`/vehicle_2`、`/vehicle_3`，而不是把專案改成 PX4 預設的 `/px4_1`、`/px4_2`、`/px4_3`。
- 確認一個 `MicroXRCEAgent udp4 -p 8888` process 可以接收三個 PX4 client。
- 確認 Gazebo 真的有三個 `x500` model，而不是一台 model 加上三個 ROS 2 node。
- 確認 ROS 2 topic inspection 顯示三台飛機都有真正的 PX4 publisher，不只是 `vehicle_node` 建立 subscriber 造成 topic 出現。
- 更新 PX4 topic boundary 的期待，對齊 PX4 v1.18 runtime output topic name，包含已觀察到的 `vehicle_local_position`、`vehicle_status`、`vehicle_command_ack` 版本尾綴。
- 新增 tests 或 smoke-check 文件，讓 ROS 2 程式如果訂閱未加版本尾綴的 PX4 output topic，而 PX4 實際發布有版本尾綴的 topic 時，能明確失敗。
- 在 takeoff/land 實作前，確認並記錄 multi-vehicle `VehicleCommand.target_system` 策略。

## 非目標

- 不實作 swarm takeoff、staging、landing、leader movement、follower following、formation change。
- 不把 ROS 2 launch 改成正常工作流程裡管理 PX4 SITL 或 Micro XRCE-DDS Agent。
- 不把 QGC 作為第一版控制入口。
- 不修改 PX4-Autopilot 的 cooperative-control 行為。
- 在手動流程驗證前，不把 multi-vehicle startup 隱藏到 combined launch file 裡。

## 實作備註

- Micro XRCE-DDS Agent 是外部 bridge，不是 ROS 2 package node。
- 優先保留 `/vehicle_1..3` 作為專案 namespace convention，並透過 PX4 DDS namespace 設定讓它成立。
- `px4_msgs` message type 繼續藏在 `Px4VehicleInterface` 邊界內；只有這個邊界需要知道具體 PX4 topic name。
- PX4 output topic 的版本尾綴要集中設定或可參數化，避免未來 PX4 message version 改變時散落多處修改。
- 在 namespace guard、versioned-topic mapping、command target-system mapping 附近加入簡短一行中文註解，說明它在防止哪一種跨機或 bridge mismatch 風險。

## 驗收條件

- [ ] 開發者可以手動啟動三個真正的 PX4 Gz `gz_x500` SITL instance，而且不修改 PX4-Autopilot 原始碼。
- [ ] Gazebo 可以看到或查到三個分開初始位置的 `x500` model。
- [ ] 一個 `MicroXRCEAgent udp4 -p 8888` process 會和三個 PX4 client 建立 session。
- [ ] ROS 2 topic inspection 對 `/vehicle_1`、`/vehicle_2`、`/vehicle_3` 的 PX4 telemetry namespace 檢查 local position 時，能看到 DDS bridge 提供的 `Publisher count: 1`。
- [ ] ROS 2 topic inspection 也確認 vehicle status 和 command acknowledgement 使用 PX4 v1.18 runtime topic name，並且有真正 PX4 publisher。
- [ ] 專案文件明確說明預期 namespace convention 是 `/vehicle_1..3` 還是 `/px4_1..3`，並說明 PX4 要如何設定才能符合。
- [ ] ROS 2 PX4 boundary 不再把「subscriber 造成 topic 出現」誤判成「PX4 telemetry 已經存在」。
- [ ] Ticket 07 能開始命令 takeoff/land 前，multi-vehicle `VehicleCommand.target_system` 行為已文件化或參數化。
- [ ] 驗收不需要 QGC，QGC 只保留為 optional observation tool。

## 測試方式

- 在 container 裡用手動啟動流程測試：一個 terminal 跑 Micro XRCE-DDS Agent，另外三個 terminal 分別跑 PX4 instance。
- 用 Gazebo topic/model inspection 確認三台真實模擬飛機存在。
- 用 ROS 2 verbose topic inspection 分辨真正 DDS bridge publisher 和本地 ROS 2 subscriber。
- 使用 `ros2 topic echo` 前要 source ROS 2 workspace，避免 `px4_msgs` type 找不到。
- 如果更新 PX4 topic-boundary defaults 或設定，完成後要從 `px4_ws` 跑 package tests。
- Smoke check 聚焦在 bridge 是否真實成立：三個 PX4 client、三個 Gazebo model、三個 ROS 2 telemetry publisher。

## Blocking edges

- Blocked by 05 - Validate three-vehicle namespaces and telemetry flow。
- Blocks 07 - Deliver synchronized takeoff to staging and land-all milestone。
- Blocks 後續所有假設三台真實 PX4 vehicle 已存在的 SITL smoke acceptance 工作。
