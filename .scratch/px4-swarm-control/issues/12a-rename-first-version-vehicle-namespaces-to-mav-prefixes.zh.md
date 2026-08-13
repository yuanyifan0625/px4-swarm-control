# 12a - 將第一版 vehicle namespaces 改成 `/MAV1`、`/MAV2`、`/MAV3`

**要建立什麼：** 將第一版 ROS 2 vehicle namespace 和 topic prefix contract，從 `/vehicle_1`、`/vehicle_2`、`/vehicle_3` 改成 `/MAV1`、`/MAV2`、`/MAV3`，並同步更新 swarm package、configuration、tests、manual verification docs，同時保持原本 leader/follower roles 和 distributed follower-control behavior 不變。

**被誰阻擋：** None - can start immediately.

**狀態：** ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、ROS 2 package tests、ROS 2 launch commands、PX4/Gazebo/runtime commands 都必須依 `AGENTS.md` 在 container 裡執行。不要修改 PX4-Autopilot 或 `px4_msgs`。

## 背景

目前第一版 code 使用 `/vehicle_1`、`/vehicle_2`、`/vehicle_3` 作為 role-independent ROS 2 vehicle namespaces。使用者現在希望 vehicle namespace/topic prefix contract 改成 `/MAV1`、`/MAV2`、`/MAV3`，讓命名更接近之後 operator workflow 和 smoke verification 想看的 vehicle naming style。

這是 topic-contract rename，不是 control-behavior change。它會影響 bridge expectations、vehicle configuration、subscriptions、publishers、tests、manual docs。它應該先於 ticket 12 完成，讓 ticket 12 可以直接針對最後的 namespace contract 建立 launch file。

## 範圍

- 將第一版 ROS 2 vehicle namespaces/topic prefixes 改名：
  - `/vehicle_1` -> `/MAV1`
  - `/vehicle_2` -> `/MAV2`
  - `/vehicle_3` -> `/MAV3`
- 保持第一版 roles 和 slots：
  - `/MAV1` 是 leader
  - `/MAV2` 是 follower-left
  - `/MAV3` 是 follower-right
- 更新 vehicle configuration，讓每個 `vehicle_node` 使用新的 namespace，且仍然對應正確 PX4 instance/system ID。
- 更新 bridge expectations 和 live bridge smoke checks，讓三台 PX4 SITL instances 預期發布 `/MAV1/fmu/out/...`、`/MAV2/fmu/out/...`、`/MAV3/fmu/out/...`。
- 更新 ground-station status subscriptions、staging setpoint publishers，以及其他仍 hard-code 舊 `/vehicle_N` 名稱的 swarm topics。
- 更新 follower subscriptions，讓 followers 訂閱 leader status `/MAV1/status`。
- 更新 operator console status observation 和 formation-settle checks，讓它觀察 `/MAV1/status`、`/MAV2/status`、`/MAV3/status`。
- 更新 tests 和 manual documentation 中所有 `/vehicle_1`、`/vehicle_2`、`/vehicle_3` 相關內容。
- 保持 distributed follower-control architecture 不變：followers 仍然根據 leader state、formation mode、slot 自己推導 setpoint。

## 非目標

- 不修改 PX4-Autopilot。
- 不修改 `px4_msgs`。
- 本 ticket 不新增 launch file；ticket 12 負責 launch。
- 不修改 action definitions。
- 不修改 `operator_console` command meanings。
- 不修改 speed profile behavior，也不把 speed-profile parameters 放進 vehicle-node YAML。
- 不導入實機部署支援。
- 不新增 dynamic leader reassignment、dynamic slots、新 formations 或 path planning。

## 實作備註

- 本 ticket 完成後，`/MAV1`、`/MAV2`、`/MAV3` 是第一版 canonical vehicle namespace contract。
- 除非 tests 或 interface constraints 需要明確 compatibility decision，`vehicle_id` values 和 display strings 應與新的 namespace naming 保持一致。
- PX4 SITL startup commands 必須設定 `PX4_UXRCE_DDS_NS=MAV1`、`PX4_UXRCE_DDS_NS=MAV2`、`PX4_UXRCE_DDS_NS=MAV3`，讓 Micro XRCE-DDS expose 新的 ROS 2 topic prefixes。
- 只有在 hard-coded namespace mapping 保護 role/slot/vehicle mismatch 時，加入簡短中文註解。
- 這是 wide topic-contract rename；請避免混入 behavior changes，讓失敗可以歸因於命名變更。

## 驗收條件

- [ ] ROS 2 topics 使用 `/MAV1`、`/MAV2`、`/MAV3` 作為第一版 vehicle prefixes。
- [ ] `/MAV1/fmu/out/vehicle_local_position_v1`、`/MAV2/fmu/out/vehicle_local_position_v1`、`/MAV3/fmu/out/vehicle_local_position_v1` 是預期 live PX4 telemetry topics。
- [ ] `/MAV1/status`、`/MAV2/status`、`/MAV3/status` 是預期 swarm status topics。
- [ ] `/MAV1` 仍是 leader，`/MAV2` 仍是 follower-left，`/MAV3` 仍是 follower-right。
- [ ] Followers 訂閱 `/MAV1/status` 作為 leader state，且不把 `/swarm/leader_goal` 當成自己的目標。
- [ ] Ground station 在 following 或 formation change 階段，不會 continuous publish absolute follower targets。
- [ ] 既有 takeoff/staging、land、MoveLeader、follower following、formation change、pause/failsafe、operator console、speed-profile tests 都更新並在 `/MAV*` namespace contract 下通過。
- [ ] Live bridge smoke documentation 和 tooling 指示 operator 用 `PX4_UXRCE_DDS_NS=MAV1`、`MAV2`、`MAV3` 啟動 PX4 SITL。
- [ ] manual 或 smoke check 確認舊 `/vehicle_1`、`/vehicle_2`、`/vehicle_3` prefixes 不再是第一版主要預期 topics。

## 測試方式

- Rename 後執行 package scaffold、bridge config、vehicle node、ground station、operator console、follower controller、live bridge smoke helpers 的 unit tests。
- 在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行完整 `px4_swarm_control` package tests。
- 實務可行時，執行 CLI/topic-name checks，確認 generated 或 expected topics 使用 `/MAV1`、`/MAV2`、`/MAV3`。
- 用三台 PX4 SITL instances 和 Micro XRCE-DDS Agent 手動驗證：
  1. 在 UDP port 8888 啟動 Agent。
  2. 使用 `PX4_UXRCE_DDS_NS=MAV1`、`MAV2`、`MAV3` 啟動 PX4 instances。
  3. 執行 live bridge smoke check。
  4. 確認 ROS 2 telemetry publishers 和 status topics 存在於 `/MAV1`、`/MAV2`、`/MAV3`。

## Blocking edges

- None - can start immediately.
- Blocks 12 - Add swarm launch and SITL smoke acceptance workflow.
