# 07c - 修正可重複 TakeoffSwarm/LandSwarm 任務循環與 stale landed/staging 狀態

**要建立什麼：** 讓第一版 SITL swarm 任務可以從 operator action interface 穩定重複執行：DDS、三台 PX4 Gz 飛機、vehicle nodes、ground station 啟動一次後，一次 `TakeoffSwarm` command 必須讓三台飛機到 staging，一次 `LandSwarm` command 必須讓三台飛機降落，而且不用重開 runtime processes 就能再次執行同樣的 takeoff/land 循環。

**被誰阻擋：** 07b - Fix takeoff-to-offboard staging sequencing。

**狀態：** ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、ROS 2 package tests、ROS 2 launch commands、PX4 SITL commands、Gazebo commands、Micro XRCE-DDS Agent commands 都要在 container 裡執行。QGC 可以選擇性用來監控或人工安全觀察，但不是第一版的控制入口。

## 背景

Ticket 07b 修正了 takeoff 到 Offboard staging 的預期順序，但 clean runtime 的 live diagnosis 顯示，從 operator 角度來看，任務還不能穩定重複，也還沒有讓 ROS 2 狀態和模擬畫面完全一致。

診斷到的現象是：

- 三台 PX4 Gz vehicles、Micro XRCE-DDS Agent、PX4 publishers、ROS 2 vehicle status topics 都存在，而且 namespace 正確。
- 從 clean runtime 第一次送 `TakeoffSwarm` 時，PX4 確實會 arm 並爬升到約 `z=-4`，但 ROS 2 vehicle status 仍回報 `vehicle_state: landed`。
- 第二次送同一個 `TakeoffSwarm` 後，三台才會進入 Offboard 並抵達預期 staging positions。
- `LandSwarm` 後，不重開 runtime 再送一次 `TakeoffSwarm`，ROS 2 狀態仍可能卡在 `landed`，導致 staging mission 沒有完成。
- PX4 `vehicle_land_detected` telemetry 顯示飛機離地後並不是 landed，所以 stale `landed` 狀態來自 ROS 2 mission/state 邊界，不是 Gazebo、DDS、PX4 publisher discovery、namespace mapping 或 QGC 的問題。

可能根因是：

- vehicle-level `landed` 狀態被當成「飛機仍在地上」的證據，所以舊的 internal state 會在 PX4 telemetry 已改變後繼續強化自己。
- ground station completion logic 可能吃到上一個 mission 或 startup 時留下來的 cached `landed` statuses，導致新的 takeoff mission 還沒收到 fresh staging status 就被判定 `done`。
- `TakeoffSwarm` 目前比較像「命令已接受/已發布」就 success，不是「三台已真的抵達 staging」才 success。
- staging setpoints 太依賴 single-shot delivery，讓 startup order 和 topic timing 比較脆弱。

這張 ticket 要在進入 leader movement 和 follower formation 前，先補上 command accepted 與實際 mission complete 之間的缺口。

## 範圍

- 不修改 PX4-Autopilot。
- 不把 QGC 放進必要控制流程。
- 保留 `TakeoffSwarm` 和 `LandSwarm` 作為 operator-facing action commands。
- 修正 vehicle-level landed-state handling，讓 fresh PX4 landed telemetry、arming state、高度、active mission phase 共同決定 vehicle 是否可以回報 `landed`。
- 新 takeoff 或 land command 開始時，要清除或版本化 stale vehicle mission state。
- 防止舊的 internal `landed` 狀態覆蓋新的 airborne PX4 telemetry。
- 防止 ground station 使用 cached landed statuses 把新的 `TakeoffSwarm` mission 提早判定完成。
- ground station 只針對目前 takeoff mission generation 判斷 `all vehicles reached staging positions`。
- `TakeoffSwarm` action completion 要代表 staging milestone 已完成；如果 timeout 前沒完成 staging，要回傳 timeout/failure。
- `LandSwarm` action completion 要代表三台都已針對目前 land mission 確認 landed；如果 timeout 前沒完成降落，要回傳 timeout/failure。
- 改善 staging setpoint delivery，讓每台 vehicle 在 takeoff-to-Offboard sequence 前和過程中都能拿到目前 staging target，而不是依賴一次 publish。
- 保持第一版 staging geometry 不變：world-frame staging、leader 在中心、`vehicle_2` 後左、`vehicle_3` 後右。
- 在重要 state/epoch/telemetry checks 附近加入簡短一行中文註解，說明它保護的是哪一種 stale-state 或 sequencing risk。
- 更新中文 manual smoke document，讓它驗證 repeated takeoff-land-takeoff-land cycle，而不是只驗證單次 takeoff 和 land。

## 非目標

- 不修改 PX4-Autopilot source 或 PX4 internal controllers。
- 不要求 QGC 參與 arming、takeoff、Offboard transition、staging、landing 或 repeated cycles。
- 不更改 `/vehicle_1`、`/vehicle_2`、`/vehicle_3` namespace convention。
- 不更改 PX4 target-system mapping，除非 focused test 證明 mapping 本身直接錯誤。
- 不實作 staging 之後的 leader movement。
- 不實作 follower following。
- 不實作 formation changes。
- 不新增 collision avoidance、自訂 takeoff controller 或自訂 landing controller。
- 不新增完整 PX4/Gazebo/DDS launch orchestration，除非它是讓 repeatability tests 可執行的必要條件。

## 驗收條件

- [ ] 從 clean runtime 送一次 `TakeoffSwarm` action，`altitude_m: 5.0` 時三台 vehicles 會 arm、爬升、進入 Offboard staging control，並抵達預期 staging positions，不需要送第二次 action。
- [ ] 任一 vehicle 只要 armed 且 airborne，或 PX4 landed telemetry 顯示 `landed=false`，該 vehicle 的 ROS 2 status 就不可以回報 `vehicle_state: landed`。
- [ ] ground station 不會使用 startup 或上一個 landing 留下的 cached `landed` statuses，把 takeoff mission 直接轉成 `done`。
- [ ] ground station 只會在三台 current-mission statuses 都有真實 telemetry、已 armed、Offboard accepted 或 active、且進入 staging tolerance 後，才輸出 `all vehicles reached staging positions`。
- [ ] `TakeoffSwarm` action result 只有在三台都針對目前 takeoff mission 抵達 staging 後才 success。
- [ ] 如果 timeout 前沒有抵達 staging，`TakeoffSwarm` action result 會 failure 或 timeout。
- [ ] `LandSwarm` action result 只有在三台都針對目前 land mission 回報 confirmed landed state 後才 success。
- [ ] 如果 timeout 前三台沒有全部 landed，`LandSwarm` action result 會 failure 或 timeout。
- [ ] 成功 `LandSwarm` 後，不重開 Micro XRCE-DDS Agent、PX4 SITL instances、Gazebo、vehicle nodes 或 ground station，再送一次 `TakeoffSwarm` 仍會成功。
- [ ] live SITL repeated cycle `TakeoffSwarm -> LandSwarm -> TakeoffSwarm -> LandSwarm` 在每個 milestone 都讓 Gazebo motion、PX4 telemetry、`/vehicle_*/status` 保持一致。
- [ ] staging target delivery 對正常 startup/topic timing 具有容錯性：vehicle 不會因為先收到 mission command、後收到 setpoint message 而錯過目前 staging target。
- [ ] 中文 manual smoke document 明確列出第一次 takeoff、第一次 landing、不重開 runtime 的第二次 takeoff、第二次 landing 的通過條件。

## 測試方式

- 新增 vehicle-node core 層級的 TDD 測試來覆蓋 stale landed bug：
  - 先前回報 `landed` 的 vehicle，在 fresh PX4 telemetry 顯示 airborne 後，必須停止回報 `landed`；
  - active takeoff mission 不可以立刻被舊的 internal `landed` 狀態覆蓋；
  - confirmed landing 後，新的 takeoff command 必須清掉 stale landed condition，並再次走完 takeoff staging。
- 新增 ground-station mission generation 或等價 freshness handling 的 TDD 測試：
  - startup 時 cached landed statuses 不會完成新的 takeoff mission；
  - 上一次 land mission 留下的 cached landed statuses 不會完成新的 takeoff mission；
  - staging completion 必須使用 fresh current-mission vehicle statuses；
  - landing completion 必須使用 fresh current-mission landed statuses。
- 新增 action-level tests 證明：
  - `TakeoffSwarm` 不會在 staging 完成前回傳 success；
  - staging completion 沒發生時，`TakeoffSwarm` 會 timeout；
  - `LandSwarm` 不會在三台 landed 前回傳 success；
  - landing completion 沒發生時，`LandSwarm` 會 timeout。
- 新增 staging setpoint robustness 測試，確保每台 vehicle 在 takeoff-to-Offboard sequence 依賴 staging target 前，能取得目前 staging target。
- ROS 2 build 和 package tests 要在 container 內從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。
- 使用 Micro XRCE-DDS Agent 和三個外部 PX4 Gz instances 做手動 SITL 驗證：
  - `check_live_px4_gz_bridge --agent-log ...` 通過；
  - 送一次 `TakeoffSwarm`，確認三台都抵達 staging；
  - 送一次 `LandSwarm`，確認三台都 landed；
  - 不重開 runtime，再送一次 `TakeoffSwarm`，確認三台再次抵達 staging；
  - 再送一次 `LandSwarm`，確認三台再次 landed；
  - 確認 simulator 和 PX4 telemetry 顯示飛機 airborne 時，`/vehicle_*/status` 不會顯示 `landed`。

## Blocking edges

- Blocked by 07b - Fix takeoff-to-offboard staging sequencing。
- Blocks 08 - Implement leader movement by absolute world position plus yaw。
- Blocks 09 - Implement follower fixed-slot following from leader state。
- Blocks 10 - Implement ChangeFormation between vee and line-abreast。
- Blocks 12 - Add swarm launch and SITL smoke acceptance workflow。
