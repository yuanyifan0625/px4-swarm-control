# 16 - 新增 field-frame operator console

**要建立什麼：** 新增一個 top-level `field_frame_console`，讓使用者可以用 human field frame 指令操作 swarm，console 內部再轉成既有 `/swarm` actions。此入口用於解決 Gazebo visual frame、實機 field frame 與 PX4 local NED frame 不一致造成的操作混淆，同時保留既有 `operator_console` 的 raw PX4 local NED 語意。

**被誰阻擋：** 15 - 新增座標軸 probe 與 frame contract 文件。

Status: ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、`ros2 interface`、ROS 2 package tests、ROS 2 launch commands 都要在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。PX4/Gazebo/Micro XRCE-DDS Agent/runtime commands 也必須在 container 裡執行。

## 背景

既有 `operator_console` 的 `2/x/3/y/4/z` 是 PX4 local NED world-frame jog，不是 Gazebo GUI 座標，也不是實機飛場座標。SITL probe 已確認：

- PX4 `+X` 約等於 Gazebo `+Y`。
- PX4 `+Y` 約等於 Gazebo `+X`。
- PX4 `-Z` 約等於 Gazebo `+Z`。

為了避免修改既有 console 造成模擬與實機語意混淆，本 ticket 新增另一個明確入口：`field_frame_console`。它只在最上層把 field-frame movement 轉成既有 `/swarm` action goal，不修改第二層 action、mission validation、vehicle-local PX4 boundary 或 PX4 topic contract。

## 範圍

- 新增 `field_frame_console` executable。
- `field_frame_console` 可以取代以下入口使用：

```bash
ros2 run px4_swarm_control operator_console
```

- 但不得刪除或改變 `operator_console`。
- `field_frame_console` 仍只呼叫既有 `/swarm` actions。
- `field_frame_console` 不直接發布 PX4 topics。
- movement 指令使用 field frame，再轉成 PX4 local NED goal。
- 非 movement 指令沿用既有 operator console action flow。
- 第一版要支援 `9 demo`，且 demo 裡的 movement 必須經過 field-frame mapping。

## 預設 mapping

第一版預設使用 Gazebo visual profile，讓 Gazebo 畫面移動方向符合人類 field-frame 直覺：

- field `+X` -> PX4 `+Y`
- field `-X` -> PX4 `-Y`
- field `+Y` -> PX4 `+X`
- field `-Y` -> PX4 `-X`
- field up -> PX4 `-Z`
- field down -> PX4 `+Z`

Help text 與文件必須明確寫：

- `field_frame_console` default mapping 是 SITL/Gazebo visual profile。
- `operator_console` 才是 raw PX4 local NED console。
- 實機不得假設 Gazebo visual profile 等於飛場 field frame。
- 實機使用前必須先跑 ticket 15 的 coordinate frame probe。

## 可調參數

支援 launch/runtime 參數來調整實機 field frame mapping：

- `field_x_axis`：`px4_x`、`px4_y`、`px4_z`。
- `field_x_sign`：`positive` 或 `negative`。
- `field_y_axis`：`px4_x`、`px4_y`、`px4_z`。
- `field_y_sign`：`positive` 或 `negative`。
- `field_up_axis`：`px4_x`、`px4_y`、`px4_z`。
- `field_up_sign`：`positive` 或 `negative`。

第一版預設值：

```text
field_x_axis: px4_y
field_x_sign: positive
field_y_axis: px4_x
field_y_sign: positive
field_up_axis: px4_z
field_up_sign: negative
```

實機若 coordinate probe 顯示 field `+X` 對應 PX4 `-Y`，則使用者用參數覆蓋，不改 code。

## 指令語意

`field_frame_console` 第一版必須支援：

- `s` / `status`：status。
- `p` / `pause`：pause。
- `r` / `resume`：resume。
- `q`：quit。
- `h` / `help`：help。
- `0`：ArmSwarm without takeoff。
- `1`：TakeoffSwarm。
- `2`：leader field `+X` step。
- `x`：leader field `-X` step。
- `3`：leader field `+Y` step。
- `y`：leader field `-Y` step。
- `4`：leader field up step。
- `z`：leader field down step。
- `5`：leader yaw + step。
- `c`：leader yaw - step。
- `6`：ChangeFormation `vee`。
- `7`：ChangeFormation `line_abreast`。
- `8`：LandSwarm。
- `9`：demo macro，movement 經 field-frame mapping。
- `settle`：wait until current formation settles。
- `home`：move leader back to captured home position through field-frame/absolute pose handling。
- `home_yaw`：restore captured home yaw。

## 硬性限制

- 不修改既有 `operator_console` 的 command 語意。
- 不把 Gazebo visual mapping 偷偷放進 `operator_console`。
- 不修改 `/swarm` action definitions。
- 不新增新的 ROS action/message/service。
- 不修改 `ground_station_node` action surface 或 mission state machine。
- 不修改 `vehicle_node` 的 follower geometry、freshness gating 或 PX4 boundary。
- 不修改 `px4_vehicle_interface.py` 的 PX4 raw topic mapping。
- 不修改 PX4-Autopilot。
- 不修改 `px4_msgs`。
- 不讓 `swarm_nodes.launch.py`、real vehicle launch、real ground-station launch 自動啟動 `field_frame_console`。
- `field_frame_console` 是 top-level adapter；第二層 `/swarm` action contract 必須維持不變。

## 文件

新增兩份精簡手動手冊：

- `px4_ws/src/px4_swarm_control/config/final_sitl_field_frame_console_command.zh.md`
- `px4_ws/src/px4_swarm_control/config/final_real_field_frame_console_manual.zh.md`

SITL command 手冊假設：

- 使用者已在 container 裡。
- 目前路徑是 `/home/ncrl/docker_ubuntu24`。
- Gazebo、三台 PX4、Micro-XRCE-DDS Agent、`swarm_nodes.launch.py` 已作為背景程式啟動。
- Ticket 15 的 commanded probe 已確認 Gazebo visual profile。

實機 manual 手冊假設：

- PX4 透過 Micro-XRCE-DDS Agent 已能轉出 canonical topics `/MAV1`、`/MAV2`、`/MAV3`。
- Ticket 15 的 manual probe 已完成。
- 使用者依實機 field frame mapping 用參數啟動 `field_frame_console`。

兩份手冊都必須包含：

- 啟動指令。
- mapping 參數目的與功能。
- 預設 Gazebo visual profile 的意義。
- 實機覆蓋參數範例。
- 指令表。
- `9 demo` 的 field-frame 語意。
- 每一步驗收條件。

## 驗收條件

- [ ] `field_frame_console` 可以用 `ros2 run px4_swarm_control field_frame_console` 啟動。
- [ ] `field_frame_console` help text 明確說明它是 field-frame adapter。
- [ ] Help text 明確說明預設 mapping 是 Gazebo visual profile。
- [ ] Help text 明確提醒 raw PX4 local NED 請使用 `operator_console`。
- [ ] `operator_console` 既有指令語意不因本 ticket 改變。
- [ ] `2` 會依 field `+X` mapping 轉成對應 PX4 axis/sign 的 absolute leader goal。
- [ ] `x` 會依 field `-X` mapping 轉成對應 PX4 axis/sign 的 absolute leader goal。
- [ ] `3` 會依 field `+Y` mapping 轉成對應 PX4 axis/sign 的 absolute leader goal。
- [ ] `y` 會依 field `-Y` mapping 轉成對應 PX4 axis/sign 的 absolute leader goal。
- [ ] `4` 會依 field up mapping 轉成對應 PX4 axis/sign 的 absolute leader goal。
- [ ] `z` 會依 field down mapping 轉成對應 PX4 axis/sign 的 absolute leader goal。
- [ ] `5` 與 `c` 保持 yaw +/- step 語意。
- [ ] `0`、`1`、`6`、`7`、`8`、`settle` 沿用既有 `/swarm` action flow。
- [ ] `9 demo` 可以執行，且 demo 內 movement 走 field-frame mapping。
- [ ] `9 demo` 的 settle 行為沿用既有 settle tolerance 與 stable duration 設定。
- [ ] 支援 mapping 參數覆蓋，不需要改 YAML 或 code。
- [ ] 無效 mapping 參數會明確失敗，不能靜默使用錯誤 mapping。
- [ ] `field_frame_console` 不直接發布 PX4 topics 或 vehicle-local setpoints。
- [ ] SITL command 手冊存在且精簡列出前置條件、指令、mapping 參數、驗收條件。
- [ ] 實機 manual 手冊存在且精簡列出前置條件、實機 mapping 覆蓋方式、安全注意事項、驗收條件。

## SITL 驗收

- [ ] 先完成 ticket 15 commanded probe，確認 Gazebo visual profile。
- [ ] 啟動 Gazebo、三台 PX4、Micro-XRCE-DDS Agent、`swarm_nodes.launch.py`。
- [ ] 不啟動 `operator_console`。
- [ ] 啟動 `field_frame_console`。
- [ ] 用小步長執行 `2/x/3/y/4/z`，確認 Gazebo 畫面方向符合 field frame 直覺。
- [ ] 執行 `1 -> 6 -> settle -> 7 -> settle -> 6 -> settle -> 8`，確認和既有 action flow 相容。
- [ ] 執行 `9`，確認 demo macro 完成且 movement 符合 field-frame mapping。

## 測試方式

- [ ] 使用 TDD：先新增會失敗的 tests，再實作。
- [ ] 新增 mapping unit tests，覆蓋 field axis/sign 到 PX4 `x/y/z` delta。
- [ ] 新增 command dispatcher tests，覆蓋 `2/x/3/y/4/z` 轉換後的 absolute leader goal。
- [ ] 新增 invalid mapping parameter tests。
- [ ] 新增 demo macro tests，確認 demo movement 走 field-frame mapping。
- [ ] 新增 tests 確認既有 `operator_console` raw PX4 NED tests 不被改壞。
- [ ] 新增 package scaffold 或 executable tests，確認 `field_frame_console` 被安裝。
- [ ] 在 container 裡執行 package build、pytest、`colcon test`，並回報結果。
- [ ] 在已啟動 SITL/Gazebo/ROS swarm 背景下執行 manual command smoke 和 demo smoke。

## Blocking edges

- Blocked by 15 - 新增座標軸 probe 與 frame contract 文件。
- Continues from 14 - 改善小場地 operator console 隊形操作流程。

## Implementation notes

- `field_frame_console` 已用 TDD 實作為獨立 executable，不會被既有 swarm launch、real launch 或 operator console launch 自動啟動。
- 新增 field-frame mapping model，預設 Gazebo visual profile：
  - field `+X -> PX4 +Y`
  - field `+Y -> PX4 +X`
  - field up `-> PX4 -Z`
- `2/x/3/y/4/z` 會先把 field-frame delta 轉成 PX4 local NED absolute leader goal，再呼叫既有 `/swarm/move_leader`。
- `0/1/6/7/8/settle/home/home_yaw` 沿用既有 operator console `/swarm` action flow。
- `9 demo` 沿用既有 demo command sequence；其中 movement 指令會經過 field-frame mapping。
- 可用 ROS 2 parameters 覆蓋實機 mapping：`field_x_axis/sign`、`field_y_axis/sign`、`field_up_axis/sign`。
- `operator_console` 既有 raw PX4 local NED 指令語意未修改。
- Focused tests：`test_field_frame_console.py`、`test_coordinate_frame_probe.py` 與 package scaffold tests 通過，`28 passed`。
- Full package pytest：`173 passed`。
- `colcon build --packages-select px4_swarm_interfaces px4_swarm_control` 通過。
- `colcon test --packages-select px4_swarm_interfaces px4_swarm_control` 通過；package-specific result：
  - `px4_swarm_control`: `173 tests, 0 errors, 0 failures`
  - `px4_swarm_interfaces`: `0 tests, 0 errors, 0 failures`
- Workspace-wide `colcon test-result --verbose` 仍會掃到既有 `px4_ros_com` lint failures；該 package 不屬於本 ticket 範圍。
- SITL manual field-frame smoke 已在既有 Gazebo/PX4/ROS swarm runtime 上通過：
  - `2/x/3/y/4/z` 使用 `move_step_x_m:=0.20`、`move_step_y_m:=0.20`、`altitude_step_m:=0.15`
  - 六個 movement 都回傳 `OK: leader reached target`
- SITL `9 demo` 在當時非乾淨 runtime 狀態下未完成：先前 swarm 已在 airborne/following，demo 重新 takeoff 後停在 `takeoff staging timed out`。隨後已送 `8` land，三台機體回到 `landed/disarmed`；MAV2/MAV3 暫時停在 `auto_land` 且 `pre_flight_checks_pass: false`，因此沒有硬跑第二次完整 demo。
- Code review 後補齊：
  - help text 明確要求實機先跑 `coordinate_frame_probe`，且不得假設 Gazebo visual profile 等於實機 field frame。
  - SITL/real field-frame 手冊補完整指令表。
  - `s/status`、`p/pause`、`r/resume`、`home`、`home_yaw` dispatcher tests。
  - `home/home_yaw` 作為一般 field console 指令支援；`1` 成功後 capture home pose。
  - ticket 15 probe regression：commanded snapshot 失敗仍會嘗試 return home；manual sample 每次檢查 raw/status 一致性。
