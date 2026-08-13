# 11d - 新增 PX4 速度 profile 檢查與明確套用流程

**要建立什麼：** 新增一個乾淨的 PX4 speed profile 流程，讓 operator 可以在 SITL demo 和未來實機部署前，檢查並明確套用比較慢、比較平滑的 PX4 飛控參數；不修改 PX4-Autopilot、`px4_msgs`、`vehicle_node` 或 follower-control 邏輯。

**被誰阻擋：** 11b - Add configurable operator short-command console。

**狀態：** ready-for-agent

**Workspace note：** 專案 ROS 2 packages 放在 `px4_ws/src/` 底下。`colcon`、`ros2 interface`、ROS 2 package tests、ROS 2 launch commands 都要在 container 裡，從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。PX4/Gazebo/runtime commands 也必須在 container 裡執行。QGC 可以選擇性用來監控或人工安全觀察，但不是第一版的控制入口。

## 背景

目前 swarm system 送給 PX4 的是高階 `position + yaw setpoint`。它沒有送 velocity setpoint，也沒有在 ROS 2 端實作速度限制器。因此飛機看起來飛多快、加速多快、動作多平滑、yaw 轉多快，主要由 PX4 runtime parameters 控制，例如 `MPC_XY_VEL_MAX`、`MPC_ACC_HOR`、`MPC_JERK_AUTO`、`MPC_YAWRAUTO_MAX`。

使用者希望 demo 時飛機動作更慢、更平滑，同時保持架構乾淨，並保留未來實機部署可能。速度參數不應該放進 `three_vehicle_nodes.yaml`，因為那個檔案是 ROS 2 vehicle node 的身份、namespace、loop rate、timeout 和 formation spacing 設定。PX4 速度與加速度參數應該放在獨立的 PX4 speed profile。

未來實機部署可能是三台 Raspberry Pi 加一台本機 ground-station computer，每台 Pi 代表一台飛機。同一套 speed profile 概念應該同時支援 SITL 和實機，但兩邊都應採用「檢查 + 明確套用」流程，不要在啟動時偷偷改飛控參數。

## 範圍

- 定義版本化的 PX4 speed profile 格式，用來描述飛控參數。
- 新增至少一個 SITL demo 用的 `slow_demo` profile。
- 新增一個未來實機初期用的保守 `real_cautious` profile 草案。
- Profile 需包含這類參數：
  - `MPC_XY_VEL_MAX`
  - `MPC_Z_VEL_MAX_UP`
  - `MPC_Z_VEL_MAX_DN`
  - `MPC_ACC_HOR`
  - `MPC_JERK_AUTO`
  - `MPC_YAWRAUTO_MAX`
  - `MPC_YAWRAUTO_ACC`
- 提供流程，能檢查三台飛機目前 PX4 參數是否符合指定 profile。
- 提供流程，只有在 operator 明確要求時才套用指定 profile。
- 套用前後要有清楚終端機輸出，包含目前值、目標值、每台飛機結果。
- 第一版先支援 SITL，但檔案格式和流程要能延伸到未來每台 Raspberry Pi 的實機部署。
- 在不需要 live PX4 的部分加入測試，例如 profile parsing、validation、diff generation、explicit apply gating。
- 新增中文手動文件，說明三機 SITL 如何檢查與套用 speed profile。

## 非目標

- 不修改 PX4-Autopilot 原始碼。
- 不修改 `px4_msgs`。
- 不修改 `vehicle_node`。
- 不修改 `follower_controller`。
- 不在本 ticket 實作 ROS 端 velocity limiter 或 trajectory planner。
- 不修改 action definitions。
- 不在 swarm 啟動時偷偷套用 PX4 參數。
- 除非未來手動流程明確標示安全，不在飛行中套用參數。
- 不把 QGC 當成控制入口。

## 實作備註

- `three_vehicle_nodes.yaml` 要維持只描述 ROS 2 node identity 和 runtime behavior，不要把 PX4 controller parameters 塞進去。
- PX4 speed profiles 要跟 ROS 2 node config 分開存放，例如放在獨立 config 子資料夾。
- 第一版 workflow 可以是小型 ROS 2 CLI、Python helper 或明確文件化的 script，但必須明確區分 `check` 和 `apply`。
- Speed profile 的 source of truth 要放在 repo。未來實機部署時，可以把同一份 profile 同步到三台 Raspberry Pi，或由 ground-station workflow 集中檢查。
- Apply workflow 要印出中文警告，說明這會修改 PX4 runtime parameters，應在飛行前執行。
- 在任何 command generation 邊界附近加入簡短中文註解，說明這是在套用 PX4 runtime parameters，不是在改 ROS setpoint 邏輯。

## 建議初始 profile 數值

先用這組作為保守 SITL demo 起點，之後再依手動驗證微調：

- `MPC_XY_VEL_MAX`: `2.0`
- `MPC_Z_VEL_MAX_UP`: `1.0`
- `MPC_Z_VEL_MAX_DN`: `0.8`
- `MPC_ACC_HOR`: `2.0`
- `MPC_JERK_AUTO`: `1.0`
- `MPC_YAWRAUTO_MAX`: `25`
- `MPC_YAWRAUTO_ACC`: `10`

如果 SITL 看起來還是太快，可以把水平速度降到約 `1.5 m/s`，yaw rate 降到約 `20 deg/s`，但要同步拉長 action timeout，避免因為飛得慢而 timeout。

## 驗收條件

- [ ] PX4 speed profiles 與 ROS 2 vehicle-node config 分開存放。
- [ ] 存在 `slow_demo` 和 `real_cautious` profiles，且文件說明用途。
- [ ] Workflow 可以檢查三台飛機目前 PX4 parameter values 是否符合指定 profile。
- [ ] Workflow 只有在 operator 明確確認或明確下 apply command 後才會套用指定 profile。
- [ ] 終端機輸出清楚顯示每台飛機每個參數的 current value、desired value、是否符合或是否已套用。
- [ ] 一般啟動 `vehicle_node`、`ground_station_node`、`operator_console` 時，不會偷偷修改 PX4 parameters。
- [ ] 既有 swarm control 行為維持不變：`position+yaw setpoint` 仍透過 `Px4VehicleInterface`，followers 仍是 distributed control，operator console 不直接控制 followers。
- [ ] Tests 覆蓋 profile parsing、supported parameter validation、profile diff output、missing/invalid profile handling、explicit apply gating。
- [ ] 手動 SITL 驗證顯示三台 PX4 instances 可以先 check，再明確套用 `slow_demo`，再透過 parameter output 確認，最後使用 `operator_console` demo 且不依賴 QGC 作為控制入口。

## 測試方式

- Unit-test 從 YAML 解析 profile。
- Unit-test required/supported PX4 parameter names validation。
- 使用 fake current PX4 parameter values unit-test diff/report generation。
- Unit-test apply mode 和 check mode 分開，避免 accidental apply。
- ROS 2 package tests 要在 container 裡從 `/home/ncrl/docker_ubuntu24/px4_ws` 執行。
- SITL 手動驗證：
  1. 啟動 Micro XRCE-DDS Agent 和三台 PX4/Gazebo vehicles。
  2. 對三台 PX4 instances check 指定 speed profile。
  3. 明確 apply 指定 profile。
  4. 再次 check，確認所有設定值符合 profile。
  5. 執行 `operator_console` demo，觀察飛機動作更慢、更平滑。

## Blocking edges

- Blocked by 11b - Add configurable operator short-command console。
- Complement 11c - Add demo settle gate to operator console。
- 應該 inform 12 - Add swarm launch and SITL smoke acceptance workflow，因為 launch/smoke workflow 若要 demo 慢速穩定飛行，應引用 speed-profile check/apply step。
