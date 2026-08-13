# 12 - 新增 swarm launch 與 SITL smoke acceptance workflow

**要建立什麼：** 在第一版 vehicle namespace 已經改成 `/MAV1`、`/MAV2`、`/MAV3` 之後，提供四個 swarm ROS 2 nodes 的正常 launch path，並記錄與手動驗證完整 SITL smoke workflow：從 clean runtime、slow-demo speed-profile check/apply/re-check、ROS launch、operator-console demo，到最後 landed 狀態與 runtime cleanup。

**被誰阻擋：** 12a - 將第一版 vehicle namespaces 改成 `/MAV1`、`/MAV2`、`/MAV3`。

**狀態：** ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、`ros2 interface`、ROS 2 package tests、ROS 2 launch commands 都要在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。PX4/Gazebo/runtime commands 也必須在 container 裡執行。QGC 可以選擇性用來監控或人工安全觀察，但不是第一版控制入口。

## 背景

手動啟動對前期 debug 很有用，但第一版完成後，需要一個可重複的 ROS 2 swarm nodes launch path。這一版中，PX4 SITL、Gazebo、Micro XRCE-DDS Agent 仍然是外部 prerequisites。Launch file 應該讓 ROS node topology 可重複，但不能把 simulator、bridge 或 operator-console 的錯誤藏起來。

Tickets 11b、11c、11d 新增了兩個 operator-side tools，它們必須出現在 smoke workflow 裡，但不應該成為 core ROS launch 的一部分：

- `operator_console` 是 existing `/swarm` action surface 的短指令 wrapper。
- `px4_speed_profile` 是 PX4 runtime parameter 的 check/apply workflow，例如 `slow_demo`。

這兩個工具都不應該被 swarm launch file 偷偷啟動或套用。Launch file 只啟動 control nodes；manual smoke 文件要說明 operator 何時、如何使用 console 和 speed-profile workflow。

## 範圍

- 新增 ROS 2 launch workflow，啟動三個 parameterized `vehicle_node` instances 和一個 `ground_station_node`。
- 使用 12a 完成後的 namespace contract：`/MAV1` 是 leader，`/MAV2` 是 follower-left，`/MAV3` 是 follower-right。
- 使用現有 vehicle-node YAML configuration 作為 role、vehicle ID、PX4 namespace、PX4 target system、slot、control rate、hold setpoint 等設定的 source of truth。
- Launch configuration 要保持明確，讓 role/slot/namespace mistakes 容易檢查。
- 文件化 SITL smoke verification 的外部 prerequisites：
  - clean runtime
  - `MicroXRCEAgent udp4 -p 8888`
  - 三台使用 `/MAV1`、`/MAV2`、`/MAV3` 的 PX4/Gazebo `gz_x500`
  - live bridge smoke check
- 將 `slow_demo` PX4 speed-profile workflow 文件化為 manual preflight step：
  - 先 check
  - 只有明確 `apply --yes` 才套用
  - apply 後 re-check
- 將 `operator_console` 文件化為另一個 terminal workflow，不由 swarm launch file 啟動。
- 新增完整但簡潔的中文 SITL smoke 文件，包含從 clean runtime 到 demo 成功與 cleanup 的所有指令。
- 實作後 agent 必須手動跑一次完整 SITL 驗證，並用最後實際可用的 command sequence 更新手動文件。

## 非目標

- 不從 swarm launch file 啟動 PX4 SITL。
- 不從 swarm launch file 啟動 Gazebo。
- 不從 swarm launch file 啟動 Micro XRCE-DDS Agent。
- 不從 swarm launch file 啟動 QGC。
- 不從 swarm launch file 啟動 `operator_console`。
- 不從 swarm launch file check 或 apply `px4_speed_profile`。
- 不在任何 normal launch 時偷偷套用 PX4 speed parameters。
- 本 ticket 不支援實機部署。
- 不新增 `column`、dynamic slots、automatic leader reassignment 或新的 swarm behaviors。
- 不修改 PX4-Autopilot、`px4_msgs`、`vehicle_node` control behavior、follower-control logic；除非 launch/test/doc 暴露出現有 launch path 的狹窄 bug。

## 實作備註

- 建議 launch file 名稱為 `swarm_nodes.launch.py`，因為它只啟動 ROS swarm nodes，不啟動 PX4/Gazebo/DDS。
- `operator_console` 保持為明確的 separate command，讓 terminal input 和 demo macro 執行都由 operator 控制。
- `slow_demo` speed profile 保持在 launch 外。它是 PX4 runtime parameter workflow，不是 ROS node configuration。
- 只有在 launch/config logic 保護 role-slot-namespace mismatch 或 unsafe defaults 時，加入簡短中文註解。
- 如果 manual smoke workflow 需要 cleanup commands，優先使用不會匹配並 kill 自己 shell wrapper 的寫法。

## 驗收條件

- [ ] 一個正常 launch path 會啟動一個 `/swarm` `ground_station_node`，以及 `/MAV1`、`/MAV2`、`/MAV3` 三個 `vehicle_node` instances。
- [ ] Launch/config 將 `/MAV1` 對應到 leader，`/MAV2` 對應到 follower-left slot，`/MAV3` 對應到 follower-right slot。
- [ ] Launch file 不會啟動 PX4 SITL、Gazebo、Micro XRCE-DDS Agent、QGC、`operator_console` 或 `px4_speed_profile`。
- [ ] 文件說明 PX4 SITL/Gazebo 和 Micro XRCE-DDS Agent 必須先由外部啟動。
- [ ] 文件包含 `slow_demo` preflight sequence：check、明確 `apply --yes`、re-check。
- [ ] 文件包含獨立的 `operator_console --command 9` demo command，並說明 `operator_console` 只是 existing `/swarm` actions 的 wrapper。
- [ ] 手動 SITL smoke verification 到達 staging，並 log 或回報 `all vehicles reached staging positions`。
- [ ] 手動 SITL smoke verification 會移動 leader，followers 依 leader state 和 formation mode 維持 fixed-slot following。
- [ ] 手動 SITL smoke verification 會在 `vee` 和 `line_abreast` 之間切換，並回報 formation completion。
- [ ] 手動 SITL smoke verification 最後 `operator_console` 回報 `OK: demo macro completed`。
- [ ] 手動 SITL smoke verification 顯示 `/MAV1/status`、`/MAV2/status`、`/MAV3/status` 最後都是 `vehicle_state: landed` 且 `armed: false`。
- [ ] 手動 SITL smoke verification 確認 demo 沒有建立 operator 或 ground station 對 followers 的 new continuous absolute target flow。
- [ ] 驗證後 runtime 是乾淨的；沒有殘留 Micro XRCE-DDS Agent、PX4 SITL、Gazebo、swarm ROS nodes 或 operator console processes。

## 測試方式

- 在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行 ROS 2 package tests。
- 實務可行時，新增 launch syntax/import tests。
- 實務可行時，新增 tests/checks 確認 launch description 只包含預期四個 swarm nodes，而且不包含 PX4/Gazebo/DDS/operator-console/speed-profile processes。
- 啟動 swarm nodes 前，對外部啟動的三機 SITL 執行 `check_live_px4_gz_bridge`。
- 需要慢速 demo 時，在 demo 前手動執行 `px4_speed_profile` check/apply/re-check。
- 啟動 launch file 後，在另一個 terminal 執行 `operator_console --command 9`。
- 驗證 final status topics、action/console output，以及 cleanup state。

## Blocking edges

- Blocked by 12a - 將第一版 vehicle namespaces 改成 `/MAV1`、`/MAV2`、`/MAV3`。
- Continues from 11b - Add configurable operator short-command console。
- Continues from 11c - Add demo settle gate to operator console。
- Continues from 11d - Add PX4 speed profile check and explicit apply workflow。
