# PX4 Swarm Control 建置規格

狀態：ready-for-agent

## 問題陳述

使用者想要用 PX4、ROS 2 Jazzy 和 Gazebo SITL 建置第一版三機協同控制系統。系統應該能讓三台 `gz_x500` vehicle 一起起飛，指定其中一台作為 leader，讓兩台 follower 跟隨 leader 並切換編隊，最後讓整個 swarm 一起降落。

使用者希望這項工作發生在外層 Docker workspace 和 ROS 2 workspace 中，而不是透過修改 PX4 內部實作完成。對協同控制工作而言，PX4-Autopilot 應保持為 upstream dependency。ROS 2 應透過 Micro XRCE-DDS 上的 `px4_msgs` topic 命令 PX4，使用高階 Offboard position/yaw setpoint，加上 takeoff 與 land command。PX4 應繼續負責低階穩定、姿態控制、馬達控制和飛行安全行為。

使用者也希望架構容易 debug 並容易擴充。第一版只支援 SITL，但 ROS 2 interface 和 namespace 慣例應避免只適用於模擬的假設，讓設計未來可以往實機方向移動。

## 解決方案

為三台 PX4 SITL `gz_x500` vehicle 建置一個 ROS 2 Jazzy swarm-control 系統。runtime 設計有四個主要 ROS 2 nodes：

- 一個 `ground_station_node`
- 三個同一個可參數化 `vehicle_node` executable 的 instance

`ground_station_node` 提供 operator-facing action interface、監控 swarm state、發送 leader goal、廣播 formation mode、協調 takeoff 和 landing，並處理 pause/failsafe command。它不計算 followers 的連續 formation setpoint。

每個 `vehicle_node` 代表一台 aircraft，並透過 role、vehicle ID、PX4 namespace 和 formation slot 參數化。leader instance 接收 leader goal。每個 follower instance 訂閱 leader state 和 formation mode，然後根據固定 slot 計算自己的 position/yaw setpoint。followers 不命令彼此，也不直接接收 operator goal。

Micro XRCE-DDS Agent 視為外部 ROS 2-PX4 bridge。ROS 2 package 應透過 `px4_msgs` topic 與 PX4 溝通。共用的內部 `Px4VehicleInterface` class/module 應封裝 `px4_msgs` publishers/subscribers、QoS、timestamp、namespace handling、Offboard heartbeat、takeoff、land、arm/disarm，以及 position+yaw setpoint publication。該 interface 之外的控制邏輯應使用內部概念，例如 `VehicleState`、`PositionYawSetpoint` 和 `VehicleCommandResult`。

第一個 milestone 應在 formation following 之前驗證完整基礎：三台 vehicles spawn、telemetry 到達 ROS 2、ground-station action 觸發同步 takeoff 到分離的 staging positions、terminal output 顯示所有 vehicles 都到達 staging positions，並且 ground-station action 能讓三台 vehicles 安全降落。

## User Stories

1. 作為 developer，我希望第一版限制在 PX4 SITL、Gazebo 和 ROS 2 Jazzy，讓我能在沒有實飛風險的情況下建置與 debug 系統。
2. 作為 developer，我希望 ROS 2 interfaces 避免 SITL-only 假設，讓未來仍可能整合實機。
3. 作為 developer，我希望 PX4-Autopilot 被視為 upstream dependency，讓協同控制變更不 fork PX4 控制邏輯。
4. 作為 developer，我希望避免為 swarm-control 行為修改 PX4-Autopilot，讓所有協同邏輯保留在我的 ROS 2 workspace。
5. 作為 developer，我希望只有在明確必要時才允許 PX4 bug fix，讓真實 upstream/environment bug 仍可被審慎處理。
6. 作為 developer，我希望 Micro XRCE-DDS Agent 被視為 ROS 2-PX4 bridge，讓 ROS 2 nodes 能透過 `px4_msgs` topics 與 PX4 溝通。
7. 作為 operator，我希望有 ground-station action interface，讓我能發出高階 commands，而不用直接 publish 低階 PX4 topics。
8. 作為 operator，我希望能觸發 whole-swarm takeoff，讓三台 vehicles 一起進入任務。
9. 作為 operator，我希望能觸發 whole-swarm landing，讓三台 vehicles 能一起回收。
10. 作為 operator，我希望能以 absolute world position plus yaw 發送 leader goal，讓我能在 Gazebo/RViz 中可預期地移動 formation target，並可選擇用 QGC 觀察。
11. 作為 operator，我希望能切換 formation mode，讓 followers 能在 leader 周圍重新排列。
12. 作為 operator，我希望能 pause swarm，讓我能在 debugging 或 unsafe behavior 時停止流程推進。
13. 作為 operator，我希望有 failsafe/land-all command，讓我能在測試中從壞狀態恢復。
14. 作為 operator，我希望 major mission transitions 有 terminal progress messages，讓我不用檢查每個 topic 就能理解系統正在做什麼。
15. 作為 operator，我希望 ground station 在所有 vehicles 到達 staging positions 時回報，讓我知道 takeoff staging 已完成。
16. 作為 operator，我希望 ground station 在 formation established 時回報，讓我知道系統已進入 formation behavior。
17. 作為 developer，我希望每台 vehicle 都由一個可參數化的 `vehicle_node` 表示，讓 leader 和 follower behavior 共用同一種 implementation shape。
18. 作為 developer，我希望有 `role`、`vehicle_id`、`px4_namespace` 和 `slot` parameters，讓一個 executable 能表示三台 vehicles。
19. 作為 developer，我希望 `MAV1` 是 leader namespace，讓第一版有穩定且容易檢查的 leader assignment。
20. 作為 developer，我希望 `MAV2` 是 follower-left slot 1，讓 slot naming 符合 vehicle 初始幾何位置。
21. 作為 developer，我希望 `MAV3` 是 follower-right slot 2，讓 left/right slot assignment 不含糊。
22. 作為 developer，我希望 follower-left 和 follower-right staging positions 相對於 leader initial yaw 定義，讓 "left" 和 "right" 保持幾何意義。
23. 作為 developer，我希望 staging 使用 world-frame positions，讓 takeoff 和 landing 能降低碰撞風險。
24. 作為 developer，我希望 formation following 使用 leader body-frame offsets，讓 cruise/follow behavior 中 formation 能隨 leader heading 旋轉。
25. 作為 developer，我希望第一版 formation changes 只在 body-frame slots 之間切換，讓 formation behavior 保持足夠簡單以利 debug。
26. 作為 developer，我希望支援 `vee` formation，讓 followers 能位於 leader 後方左右 slot。
27. 作為 developer，我希望支援 `line_abreast` formation，讓 followers 能位於 leader 左右兩側。
28. 作為 developer，我希望延後 `column`，讓第一版控制不要從對延遲敏感的 formation 開始。
29. 作為 developer，我希望 followers 訂閱 leader position、yaw、velocity 和 status，讓每個 follower 能根據 leader state 計算自己的 setpoint。
30. 作為 developer，我希望第一版 followers 避免 follower-follower coordination，讓固定 spacing 和 staging 先處理碰撞風險。
31. 作為 developer，我希望第一版使用 fixed follower slots，讓 dynamic slot assignment 不讓 state machine 複雜化。
32. 作為 developer，我希望 operator 只命令 leader goal、formation mode、takeoff、land、pause 和 failsafe，讓 followers 保持 derived-control participants。
33. 作為 developer，我希望每個 follower 只輸出自己的 position/yaw setpoint，讓 nodes 不命令其他 vehicles。
34. 作為 developer，我希望 `Px4VehicleInterface` 封裝 `px4_msgs`，讓 formation logic 不直接依賴 PX4 message shapes。
35. 作為 developer，我希望 control logic 使用像 `VehicleState` 這類 internal models，讓未來 bridge 或 namespace 變更能被局部化。
36. 作為 developer，我希望有像 `PositionYawSetpoint` 這類 internal models，讓 controller code 直接表達控制意圖。
37. 作為 developer，我希望重要轉換和 critical functions 有一行式註解，讓之後檢查與 debug 更容易。
38. 作為 developer，我希望在 `px4_msgs` conversion、coordinate conversion、yaw conversion、Offboard command publishing、takeoff 和 landing 周圍有 comments，讓高風險轉換可見。
39. 作為 developer，我希望 Offboard control 限制在高階 position plus yaw，讓 PX4 繼續負責 attitude 和 motor control。
40. 作為 developer，我希望 action APIs 只存在於 operator-to-ground-station 邊界，讓 long-running commands 能 expose progress 和 result。
41. 作為 developer，我希望 ground-station-to-vehicle communication 使用 topics，讓內部 ROS 2 graph 保持可觀察且簡單。
42. 作為 developer，我希望有 vehicle status topics，讓 ground station 和 operator 能監控 vehicle role、position、yaw、velocity、arming state、navigation state、Offboard availability、telemetry age 和 local control state。
43. 作為 developer，我希望 ground station 有 mission-level state machine，讓 multi-vehicle sequencing 明確。
44. 作為 developer，我希望每個 vehicle node 維護自己的 vehicle-level state，讓 vehicle-specific transitions 和 timeouts 隔離。
45. 作為 developer，我希望第一版 mission states 包含 idle、arming、taking off、staging、forming、following、reconfiguring、paused、landing、done、failsafe 和 error，讓主要行為可檢查。
46. 作為 developer，我希望三台 vehicles 一起 arm 和 take off，讓第一版驗證同步 swarm startup。
47. 作為 developer，我希望每台 vehicle 使用不同 horizontal staging position，讓 vehicles 一開始不會太靠近。
48. 作為 developer，我希望 leader 在 staging 中置中，followers 位於 behind-left 和 behind-right，讓 staging 符合預設 `vee` 直覺。
49. 作為 developer，我希望第一版 failsafe 在 vehicle telemetry timeout 時 hover，讓暫時資料遺失不導致失控運動。
50. 作為 developer，我希望 followers 在 leader telemetry/status timeout 時 hover，讓 leader loss 不產生 stale follow commands。
51. 作為 operator，我希望 pause 保持 safe setpoints，讓我能停止 motion 而不是立即 landing。
52. 作為 operator，我希望 land-all 可作為 action 使用，讓我能從 ground station 回收整個 swarm。
53. 作為 developer，我希望第一版使用 Python/rclpy，讓 architecture debugging 期間 iteration speed 和 logging 更好。
54. 作為 developer，我希望有一個主要 ROS 2 package 放 control nodes、launch、config 和 shared modules，讓第一版容易瀏覽。
55. 作為 developer，我希望有一個獨立 interfaces package 放 actions 和 messages，讓 generated ROS 2 interfaces 不與 Python node logic 混在一起。
56. 作為 developer，我希望 control loops 初始為 20 Hz，讓 Offboard setpoints 和 follower control 對 SITL debugging 來說足夠頻繁。
57. 作為 developer，我希望 ground-station state loops 為 5-10 Hz，讓 mission state monitoring 有反應但 logs 不會過多。
58. 作為 developer，我希望先支援 manual node startup，讓早期 debugging 能隔離 PX4、Micro XRCE-DDS、vehicle nodes 和 ground station。
59. 作為 developer，我希望 manual startup 可行後再提供 swarm launch file，讓正常操作能一起啟動所有 ROS 2 swarm nodes。
60. 作為 developer，我希望第一版 externally start Micro XRCE-DDS Agent 和 PX4 SITL，讓 launch complexity 不隱藏 bridge 或 simulator failures。
61. 作為 developer，我希望第一個 milestone 在 follow behavior 前證明 three-vehicle staging 和 landing，讓最困難的 infrastructure risks 先被處理。
62. 作為 developer，我希望後續 milestones 逐步加入 leader move、follower following、formation change 和 failsafe，讓失敗能歸因到單一 layer。

## 實作決策

- 第一版範圍只包含 PX4 SITL、Gazebo 和 ROS 2 Jazzy。架構應在實務可行時避免 simulation-only naming 和 assumptions，但實機部署不屬於本 build spec。
- 不得為 cooperative-control behavior 修改 PX4-Autopilot。任何 PX4-Autopilot edit 都不在正常路徑內，且應限制為明確識別的 environment、compatibility、build 或 upstream bug fixes。
- ROS 2 system 透過高階 Offboard position plus yaw setpoints，加上 takeoff 和 land commands 來命令 PX4。它不發送 attitude、thrust、actuator 或 motor commands。
- Micro XRCE-DDS Agent 是外部 ROS 2-PX4 bridge。開發與測試需要 agent 在 UDP port 8888 上執行，才應期待有 `px4_msgs` traffic。
- 在新的 Python/rclpy control package 和獨立 ROS 2 interfaces package 中建置新的 ROS 2 功能。既有 upstream packages 保持為 dependencies。
- 新的專案 ROS 2 packages 放在 ROS 2 workspace `px4_ws/src/`，不要直接放在外層 Docker workspace。
- Runtime topology 有四個主要 ROS 2 nodes：一個 ground-station node 和三個 parameterized vehicle node instances。
- vehicle node executable 由三台 vehicles 共用，並透過 parameters 切換 behavior：role、vehicle ID、PX4 namespace 和 formation slot。
- Vehicle namespace convention 是穩定且 role-independent 的：`/MAV1` 是第一版 leader，`/MAV2` 是 follower-left slot 1，`/MAV3` 是 follower-right slot 2，而 `/swarm` 是 swarm-level namespace。
- 不要使用 `leader` 作為 vehicle namespace。Leadership 是 role parameter，讓未來 leader reassignment 不需要重新命名 topics。
- ground-station node 擁有 operator-facing action interface、mission-level state、swarm monitoring、leader goal publication、formation-mode publication、whole-swarm takeoff/land、pause 和 failsafe commands。
- ground-station node 不計算 continuous follower setpoints。Follower setpoint generation 分散在每個 follower vehicle node 中。
- Operator-to-ground-station commands 是 ROS 2 actions：`TakeoffSwarm`、`MoveLeader`、`ChangeFormation`、`PauseSwarm` 和 `LandSwarm`。
- Ground-station-to-vehicle communication 使用 ROS 2 topics 來傳遞 leader goal、formation mode、mission/failsafe command 和 status aggregation。
- 每個 vehicle node publish status summary topic。status model 應包含 role、vehicle ID、position、yaw、velocity、armed state、navigation state、Offboard availability、telemetry age 和 vehicle-level state。
- 每個 follower 訂閱 leader position、yaw、velocity 和 status，然後根據固定 slot 和目前 formation mode 計算自己的 position plus yaw setpoint。
- Followers 不直接接收 operator movement goals。使用者 movement intent 只透過 leader state 和 formation mode 到達 followers。
- 第一版 followers 不命令彼此，也不做 follower-follower coordination。
- 第一版 follower slots 是固定的。Dynamic slot assignment 延後。
- `MAV2` 必須以 follower-left slot 1 開始，且它的初始 staging position 必須在 leader 左側。
- `MAV3` 必須以 follower-right slot 2 開始，且它的初始 staging position 必須在 leader 右側。
- Left/right staging directions 相對於 leader initial yaw/heading 定義。Staging points 是由該 initial heading 推導出的 world-frame positions。
- Takeoff、landing 和 staging 使用 world-frame positions 來降低碰撞風險。
- Cruise/follow formation 使用 leader body-frame offsets，讓 slots 隨目前 leader yaw/heading 旋轉。
- 第一版 formation changes 只在 body-frame slots 之間 transition。
- 第一版 formation modes 是 `vee` 和 `line_abreast`。`column` 延後。
- Whole-swarm takeoff 會 arm 並讓三台 vehicles 一起起飛到相同高度，同時維持分離的 horizontal staging positions。
- ground-station node 記錄 mission-level progress lines，包括所有 vehicles 到達 staging positions，以及 formation established。
- Vehicle nodes 只記錄自己的 state transitions 和相關 local events，以避免 terminal output 太吵。
- ground station 擁有 mission states，包括 idle、arming、taking off、staging、forming、following、reconfiguring、paused、landing、done、failsafe 和 error。
- 每個 vehicle node 擁有 vehicle-level state，用於 arming、Offboard availability、local setpoint tracking、telemetry freshness、pause/hover、landing 和 error handling。
- 第一版 `MoveLeader` 使用 absolute world position plus yaw。Relative movement commands 延後。
- 第一版 failsafe behavior 刻意保持最小：vehicle telemetry timeout 會讓該 vehicle hover 或保持最後 safe setpoint；leader telemetry/status timeout 會讓 followers hover；operator 可以觸發 pause 或 land-all。
- 第一版不實作 automatic leader reassignment、dynamic slot reassignment 或複雜 autonomous recovery。
- `Px4VehicleInterface` 是每個 vehicle node 使用的內部共用 class/module。它不是 ROS 2 node，也不取代 Micro XRCE-DDS Agent。
- `Px4VehicleInterface` 擁有 `px4_msgs` publishers/subscribers、QoS choices、namespace handling、timestamps、Offboard heartbeat、arm/disarm、takeoff、land、position+yaw setpoint publication、telemetry subscriptions 和相關 command result tracking。
- `Px4VehicleInterface` 之外的 control logic 應使用 internal models，例如 `VehicleState`、`PositionYawSetpoint` 和 `VehicleCommandResult`，而不是 raw `px4_msgs`。
- 重要 transformations 和 critical functions 應有短的一行式 comments，特別是 PX4 message conversion、coordinate-frame conversion、yaw conversion、Offboard heartbeat、takeoff command 和 land command。
- Vehicle control 和 follower formation loops 初始為 20 Hz。Ground-station mission monitoring 初始為 5-10 Hz。
- 在 combined launch file 之前應先支援 manual startup。manual startup 可行後，再提供 launch file 一起啟動三個 vehicle nodes 和 ground station。
- PX4 SITL 和 Micro XRCE-DDS Agent 在第一版維持為 externally started prerequisites，而不是由 swarm launch file 管理。
- ROS 2、PX4、build、test 和 runtime 的開發與驗證 commands 必須依 agent instructions 的 workspace command pattern 在 Docker container 內執行。
- `colcon build`、`colcon test`、`colcon test-result`、`ros2 interface show` 和 ROS 2 launch 等 ROS 2 package commands 應在 container 內的 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。
- QGC 在開發期間只作為 optional monitoring 和 manual safety observation tool。它不是第一版 swarm takeoff、leader movement、formation change、pause、failsafe 或 land-all 的控制入口。

## 測試決策

- 最高價值的 test seam 是 operator-facing action interface，並透過 vehicle status topics、ground-station progress logs 和 SITL vehicle behavior 觀察。Tests 應驗證 externally visible outcomes，而不是 internal callback structure。
- 第一個 milestone acceptance test：在三台 PX4 SITL `gz_x500` vehicles 和 Micro XRCE-DDS Agent 執行中時，呼叫 swarm takeoff action 會讓三台 vehicles arm、take off 到 configured altitude、到達分離的 staging positions、emit ground-station progress line for staging completion，並在呼叫 land-all action 時安全降落。
- 第一個 milestone 不應 assert formation following。它應證明 multi-vehicle namespace handling、bridge traffic、Offboard command publication、vehicle status aggregation、action handling、staging geometry 和 land-all behavior。
- 為 geometry transformations 加 unit tests：leader initial yaw 到 world-frame staging positions；leader current yaw 到 body-frame formation offsets；follower-left 和 follower-right slot sign conventions。
- 為 ground-station level 的 mission state transitions 加 unit tests：takeoff request、all vehicles staged、formation established、pause、land、timeout、failsafe 和 error transitions。
- 使用 internal models 為 follower setpoint derivation 加 unit tests：給定 leader state、formation mode 和 fixed slot，follower 計算出 expected position plus yaw setpoint。
- 在 PX4 topic 邊界為 `Px4VehicleInterface` 的 internal model conversion 加 unit tests，並將其與 controller logic 隔離。這些 tests 應驗證 mapping behavior，而不需要 live PX4。
- 在可行時，加入 integration-style ROS 2 tests，使用 fake 或 simulated vehicle interfaces instantiate ground station 和 vehicle nodes。這些 tests 應驗證 action feedback/results、status publication 和 mission-command topics。
- 只有在 manual workflow 穩定後才加入 SITL smoke tests。這些 tests 應 exercise 真實 Micro XRCE-DDS bridge 和 PX4 topics，但聚焦於 broad behavior，而不是逐筆 message 的 implementation details。
- Upstream ROS-PX4 bridge examples 和 tests 中已有 prior art：Python Offboard example 展示 position setpoint 和 command publication，而 bridge tests 展示如何透過 ROS 2/PX4 topics 檢查 data flow。這些應指引 behavior expectations，而不是直接成為被複製的 product architecture。
- 除非 private timer counters、exact method names 或 internal callback ordering 是 state machine 的 public seam，否則不要測這些 raw implementation details。
- Test logs 應要求 ground station 在 major transitions 輸出 mission-level progress messages，讓 operator experience 可被驗證。
- Tests 應包含 missing 或 stale telemetry 的 negative scenarios，尤其是 leader timeout 造成 followers hover。
- ROS 2 build 和 test commands 必須在 container 內執行。不要假設 host 上有 ROS 2 Jazzy、PX4 或 Gazebo tools。
- ROS 2 workspace 驗證必須從 container 內的 `px4_ws` 執行，避免在外層 workspace 產生無關的 `build/`、`install/` 或 `log/` artifacts。
- 自動化驗收不依賴 QGC 是否開啟。驗收以 ROS 2 actions、topics、logs 和 SITL behavior 為主；QGC 只作為 optional observation。

## 範圍外

- 實機部署。
- 修改 PX4-Autopilot 以實作 cooperative control。
- 低階 PX4 control changes，包括 attitude、thrust、actuator 或 motor control。
- 替換 Micro XRCE-DDS Agent，或把它包成 custom bridge。
- Dynamic leader reassignment。
- Dynamic follower slot assignment。
- Follower-follower coordination。
- 複雜 autonomous fault recovery。
- `column` formation。
- Relative leader movement commands。
- 在第一版 swarm launch file 中管理 PX4 SITL 和 Micro XRCE-DDS Agent。
- 使用 QGC 作為第一版 operator control entrypoint。
- 超出 operator actions 和 terminal/log feedback 的完整 production-grade UI。

## 其他備註

- 第一版 build 應優先考慮 inspectability 和 staged debugging，而不是聰明的 autonomy。
- 第一個成功 milestone 應被視為 infrastructure validation：三台 vehicles、namespaces、ROS 2-PX4 bridge、actions、status、staging geometry 和 land-all。
- 建議 milestone 順序如下：
  1. Multi-vehicle SITL、Micro XRCE-DDS Agent，以及 three-vehicle ROS 2 telemetry namespace validation。
  2. 一個 parameterized vehicle node 控制一台 PX4 vehicle 到 position+yaw hover。
  3. 三個 vehicle node instances 同時 arm、take off，並到達 staging positions。
  4. Ground-station actions for takeoff、land 和 pause。
  5. Leader movement，followers 暫時不跟隨。
  6. Followers 根據 leader state 計算 fixed-slot setpoints。
  7. 在 `vee` 和 `line_abreast` 之間 change formation。
  8. Minimal failsafe。
- 除非使用者明確批准 targeted fix，否則保持 `PX4-Autopilot` 作為 upstream dependency code。
- Terminal output 應有用但不要吵：mission-level progress 屬於 ground station；vehicle-local transitions 屬於每個 vehicle node。
