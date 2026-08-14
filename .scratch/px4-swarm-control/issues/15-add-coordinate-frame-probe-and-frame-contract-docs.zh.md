# 15 - 新增座標軸 probe 與 frame contract 文件

**要建立什麼：** 新增一個 ROS 2 層座標檢查工具，用來確認 PX4 透過 Micro-XRCE-DDS 發出的 local NED 座標、`/MAV*/status` 轉出座標、以及 Gazebo world frame 或實機 field frame 的對應關係。完成後，使用者在 SITL 與實機測試前，可以先用明確流程確認目前操作指令使用的 PX4 座標軸，不再只靠 Gazebo 畫面或肉眼直覺判斷。

**被誰阻擋：** 14 - 改善小場地 operator console 隊形操作流程。

Status: ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、`ros2 interface`、ROS 2 package tests、ROS 2 launch commands 都要在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。PX4/Gazebo/Micro XRCE-DDS Agent/runtime commands 也必須在 container 裡執行。

## 背景

SITL live probe 已確認目前資料鏈大致為：

- `/MAV1/fmu/out/vehicle_local_position_v1` 是 PX4 local NED frame。
- `/MAV1/status` 與 PX4 raw local position 幾乎一致，沒有發現 status 層 x/y 交換。
- Gazebo world frame 與 PX4 local NED 不是同一套軸：
  - PX4 `+X` 約等於 Gazebo `+Y`。
  - PX4 `+Y` 約等於 Gazebo `+X`。
  - PX4 `-Z` 約等於 Gazebo `+Z`，也就是上升。

這不是既有 `operator_console` 的座標轉換 bug，而是操作入口與測試文件沒有明確暴露 frame contract。實機使用 Micro-XRCE-DDS 時，ROS 2 也會看到 PX4 local frame，不會自動變成飛場座標。

## 範圍

- 新增 `coordinate_frame_probe` executable。
- 支援指定 PX4 namespace，預設 `/MAV1`：
  - `px4_namespace:=/MAV1`
  - 後續可手動改成 `/MAV2`、`/MAV3`。
- Probe 必須同時訂閱：
  - `/<namespace>/fmu/out/vehicle_local_position_v1`
  - `/<namespace>/status`
- raw PX4 local position 是主判定來源。
- `/MAV*/status` 是輔助一致性檢查；驗證時必須先確認 status 和 raw PX4 local position 即時且近似一致。
- 支援 `commanded` mode，用於 SITL：
  - 透過 `/swarm/move_leader` 發送小位移。
  - 依序測 PX4 `+X`、PX4 `+Y`、PX4 `-Z`。
  - 每段完成後回到 baseline pose。
  - 比對 raw PX4 local position、`/MAV1/status`、`/swarm/leader_goal`、`/MAV1/fmu/in/trajectory_setpoint`。
  - 若 Gazebo pose topic 存在，額外比對 Gazebo world delta。
- 支援 `manual` mode，用於實機 preflight：
  - 不送任何 movement action。
  - 提示使用者手動移動飛機沿 field `+X`、field `+Y`、field up。
  - 每段必須等該軸通過後才進下一段。
  - 若偵測到反方向、其他軸 dominant、或 cross-axis 過大，印 `WARNING`，記錄該段結果，然後進下一段。
- 第一版只做查看與報告 `x/y/z` delta。
- 第一版不做 expected mapping 參數。
- 第一版不自動產生 config 檔。
- 第一版不自動修改 YAML。

## 硬性限制

- 不修改 `operator_console` 現有指令語意。
- 不修改 `ground_station_node` action surface 或 mission state machine。
- 不修改 `vehicle_node` 的 PX4 boundary 行為。
- 不修改 `px4_vehicle_interface.py` 的 PX4 topic mapping。
- 不修改 `px4_msgs`。
- 不修改 PX4-Autopilot。
- 不新增新的 swarm action/message/service。
- 不讓 probe 被 `swarm_nodes.launch.py`、real launch、或 operator console launch 自動啟動。
- Probe 是 preflight diagnostic，不是正式飛行 runtime node。

## 建議參數

- `px4_namespace`：預設 `/MAV1`。
- `mode`：`commanded` 或 `manual`。
- `axis_step_m`：commanded mode 小位移，預設 `0.30`。
- `up_step_m`：commanded mode 上升測試，預設 `0.20`。
- `dominant_delta_m`：manual mode 該軸通過門檻，預設 `0.30`。
- `cross_axis_delta_m`：manual mode 其他軸最大容許變化，預設 `0.10`。
- `stable_duration_s`：條件連續成立時間，預設 `1.0`。
- `timeout_s`：每段等待上限，預設 `15.0`。
- `status_position_tolerance_m`：`/MAV*/status` 與 raw PX4 local position 一致性容許值，預設 `0.05`。
- `gazebo_pose_topic`：預設 `/world/default/dynamic_pose/info`。
- `gazebo_model_name`：預設 `x500_1`。

## 輸出語意

Probe 輸出應清楚區分：

- PX4 raw local position delta。
- `/MAV*/status` delta。
- `/swarm/leader_goal` 或 trajectory setpoint 目標。
- Gazebo world delta，若可取得。
- PASS / WARNING / ERROR。

範例輸出：

```text
PX4 +X commanded:
  PX4 raw delta:     (+0.31, +0.00, -0.01)
  status delta:      (+0.32, -0.00, -0.01)
  Gazebo world delta:(+0.04, +0.29, +0.02)
  observed: PX4 +X appears as Gazebo +Y
  PASS
```

Manual mode 範例：

```text
Move vehicle along field +X...
  PX4 raw dominant axis: +Y
  status consistency: PASS
  WARNING: field +X does not appear as PX4 +X
```

## 文件

新增兩份精簡手動手冊：

- `px4_ws/src/px4_swarm_control/config/final_sitl_coordinate_frame_command_probe.zh.md`
- `px4_ws/src/px4_swarm_control/config/final_real_coordinate_frame_manual_probe.zh.md`

SITL command 手冊假設：

- 使用者已在 container 裡。
- 目前路徑是 `/home/ncrl/docker_ubuntu24`。
- Gazebo、三台 PX4、Micro-XRCE-DDS Agent、`swarm_nodes.launch.py` 已作為背景程式啟動。
- `operator_console` 已停止。

實機 manual 手冊假設：

- PX4 透過 Micro-XRCE-DDS Agent 已能轉出 canonical topics `/MAV1`、`/MAV2`、`/MAV3`。
- 實機測試前先以 disarmed/manual handling 方式做座標檢查。
- 文件要提醒：若定位來源在 disarmed/手持時不更新，manual probe 無法作為有效判定。

兩份手冊都必須包含：

- 參數目的與功能。
- 啟動指令。
- 每一步驗收條件。
- `WARNING` / `ERROR` 該如何解讀。
- SITL 已知 frame contract：PX4 local NED 與 Gazebo world frame 不同。
- 實機不得假設 Gazebo mapping 等於飛場 mapping。

## SITL 驗收前置條件

- [ ] Gazebo 中有 MAV1/MAV2/MAV3。
- [ ] Micro-XRCE-DDS Agent 已啟動。
- [ ] PX4 `/MAV1`、`/MAV2`、`/MAV3` topics 存在。
- [ ] `ros2 launch px4_swarm_control swarm_nodes.launch.py` 已啟動。
- [ ] `operator_console` 沒有啟動。

## 驗收條件

- [ ] `coordinate_frame_probe` 可以用 `ros2 run px4_swarm_control coordinate_frame_probe` 啟動。
- [ ] Probe 預設 namespace 為 `/MAV1`，且可以用 `px4_namespace` 參數切換。
- [ ] Probe 啟動時會先確認 raw PX4 local position topic 與 `/MAV*/status` 都有新資料。
- [ ] Probe 會檢查 `/MAV*/status` 與 raw PX4 local position 的 `x/y/z` 是否即時且近似一致。
- [ ] 若 status 與 raw PX4 local position 不一致，probe 回報 `ERROR`，不把 status 當作主判定。
- [ ] Commanded mode 會依序發送 PX4 `+X`、PX4 `+Y`、PX4 `-Z` 小位移。
- [ ] Commanded mode 每段完成後會回 baseline pose。
- [ ] Commanded mode 會輸出 PX4 raw delta、status delta、leader goal / trajectory setpoint、Gazebo delta。
- [ ] Commanded mode 在 SITL 中能重現並輸出已知 mapping：PX4 `+X` 約等於 Gazebo `+Y`，PX4 `+Y` 約等於 Gazebo `+X`，PX4 `-Z` 約等於 Gazebo `+Z`。
- [ ] Manual mode 不送 `/swarm/move_leader` 或任何會移動飛機的 action。
- [ ] Manual mode 會逐段提示 field `+X`、field `+Y`、field up。
- [ ] Manual mode 每段等該段 PASS、WARNING、ERROR 或 timeout 後才進下一段。
- [ ] Manual mode 若偵測到反方向、其他軸 dominant、或 cross-axis 過大，印 `WARNING`，並繼續下一段。
- [ ] Probe 不修改任何 YAML、launch、PX4 params 或 runtime state。
- [ ] SITL command 手冊存在且精簡列出前置條件、指令、參數目的、驗收條件。
- [ ] 實機 manual 手冊存在且精簡列出前置條件、指令、參數目的、安全注意事項、驗收條件。

## 測試方式

- [ ] 使用 TDD：先新增會失敗的 tests，再實作。
- [ ] 新增 unit tests 覆蓋 delta dominant-axis 判定。
- [ ] 新增 unit tests 覆蓋 raw PX4 local position 與 `/MAV*/status` 一致性判定。
- [ ] 新增 unit tests 覆蓋反方向、cross-axis 過大、timeout 的 WARNING/ERROR 行為。
- [ ] 新增 launch/package scaffold 或 executable tests，確認 `coordinate_frame_probe` 被安裝。
- [ ] 在 container 裡執行 package build、pytest、`colcon test`，並回報結果。
- [ ] 在已啟動 SITL/Gazebo/ROS swarm 背景下執行 commanded mode，並回報 mapping 結果。

## Blocking edges

- Blocked by 14 - 改善小場地 operator console 隊形操作流程。
- Enables 16 - 新增 field-frame operator console。

## Implementation notes

- `coordinate_frame_probe` 已用 TDD 實作為獨立 executable，不會被既有 swarm launch 或 operator console launch 自動啟動。
- Focused tests：`test_coordinate_frame_probe.py` 與 package scaffold tests 通過。
- Full package pytest：`162 passed`。
- `colcon build --packages-select px4_swarm_interfaces px4_swarm_control` 通過。
- `colcon test --packages-select px4_swarm_interfaces px4_swarm_control` 通過；`px4_swarm_control` 回報 `162 tests, 0 errors, 0 failures`。
- SITL commanded probe 已在既有 Gazebo/PX4/ROS swarm runtime 上執行成功；`/MAV1/status` 與 raw PX4 local position 一致性每段皆 PASS。
- SITL observed mapping：
  - PX4 `+X` appears as Gazebo `+Y`。
  - PX4 `+Y` appears as Gazebo `+X`。
  - PX4 `-Z` appears as Gazebo `+Z`。
