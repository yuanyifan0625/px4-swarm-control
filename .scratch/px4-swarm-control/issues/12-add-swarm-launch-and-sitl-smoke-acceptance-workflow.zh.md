# 12 - 新增 swarm launch 與 SITL smoke acceptance workflow

**要建立什麼：** 提供四個 swarm nodes 的正常 ROS 2 launch path，並記錄/驗證第一版 SITL smoke workflow，流程從外部 prerequisites 到 takeoff、leader move、following、formation change、pause/failsafe、land。

**被誰阻擋：** 10 - Implement `ChangeFormation` between `vee` and `line_abreast`；11 - Add minimal pause and failsafe behavior。

**狀態：** ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、`ros2 interface`、ROS 2 package tests、ROS 2 launch commands 都要在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。QGC 可以選擇性用來監控或人工安全觀察，但不是第一版的控制入口。

## 背景

為了 debug，手動啟動應該先存在；但完成第一版後，需要一個可重複的 ROS 2 swarm nodes launch path。這一版中，PX4 SITL 和 Micro XRCE-DDS Agent 仍是外部 prerequisites。

## 範圍

- 新增一個 launch workflow，啟動三個 parameterized `vehicle_node` instances 和一個 `ground_station_node`。
- 提供 roles、vehicle IDs、PX4 namespaces、slots、control rates、staging positions、tolerances、formation defaults 的 configuration。
- 文件化或編碼 smoke workflow prerequisites：外部 PX4 SITL multi-vehicle environment 和 `MicroXRCEAgent udp4 -p 8888`。
- QGC 只保持 optional，用於 monitoring/manual safety observation；smoke workflow control path 不可要求使用 QGC。
- 驗證完整第一版 behavior：takeoff to staging、leader move、follower fixed-slot following、formation change、pause/failsafe path、land-all。
- 在實務可行時新增 smoke-test checklist 或 automation。

## 非目標

- 不從 swarm launch file 啟動 PX4 SITL。
- 不從 swarm launch file 啟動 Micro XRCE-DDS Agent。
- 不支援 real vehicles。
- 不新增 `column`、dynamic slots 或 automatic leader reassignment。

## 實作備註

- Launch configuration 要保持明確，讓 namespace/slot mistakes 容易檢查。
- 只有在 launch/config logic 會防止 role-slot mismatches 或 unsafe defaults 的地方，加入簡短中文註解。

## 驗收條件

- [ ] 一個正常 launch path 會啟動 `/swarm`、`/vehicle_1`、`/vehicle_2`、`/vehicle_3` ROS 2 nodes。
- [ ] Configuration 將 `/vehicle_1` 對應到 leader，`/vehicle_2` 對應到 follower-left slot 1，`/vehicle_3` 對應到 follower-right slot 2。
- [ ] Documentation/checklist 說明 PX4 SITL 和 Micro XRCE-DDS Agent 必須由外部先啟動。
- [ ] Smoke workflow 到達 staging，並記錄 `all vehicles reached staging positions`。
- [ ] Smoke workflow 會移動 leader，且 followers 維持 fixed-slot following。
- [ ] Smoke workflow 會在 `vee` 和 `line_abreast` 之間切換，並記錄 `formation established`。
- [ ] Smoke workflow 會演練 pause/failsafe behavior，最後以 land-all 結束。

## 測試方式

- 在 container 內執行 launch syntax/import tests。
- Package 和 launch commands 要從 `px4_ws` workspace 執行，不是從外層 Docker workspace。
- 實務可行時，為 launch configuration 執行 ROS 2 tests。
- 使用外部 PX4 SITL 和 Micro XRCE-DDS Agent 執行 manual SITL smoke test。
- 使用 workspace container command pattern 記錄可重現的精確 command sequence。

## Blocking edges

- Blocked by 10 - Implement `ChangeFormation` between `vee` and `line_abreast`。
- Blocked by 11 - Add minimal pause and failsafe behavior。
