# 18: 以 PX4 1.17 驗證三機 Gazebo 操控

**What to build:** 在 container 內以和實驗室飛控、PC ROS application 及樹莓派相同的 PX4 1.17 contract，完成三台 Gazebo vehicle 的端到端手動操控驗證。驗證必須證明修正後的 PX4 adapter 能接收真實 bare-DDS telemetry，既有上層 operator flow 不需修改，三機可安全完成起飛、移動、隊形切換與降落。

**Blocked by:** 17: 對齊 PX4 1.17 telemetry contract。

**Status:** ready-for-human

## Fixed validation matrix

- PX4-Autopilot 使用 `v1.17.0` / `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。
- ROS application 使用 `px4_msgs` `v1.17.0` / `86d8239e962f6939e05c3737784f60c02fa884db`。
- 使用一個 Micro XRCE-DDS Agent、三個 PX4 SITL instances、三個分開的 Gazebo models，以及 `/MAV1`、`/MAV2`、`/MAV3` namespaces。
- 不啟動 ROS translation node。
- QGC 只可作為選擇性的安全觀察工具，不是操控或驗收入口。

## Clean-build preconditions

- 切換 PX4-Autopilot 與 `px4_msgs` revisions 前，必須確認兩個 nested repositories 都沒有未提交修改；若不乾淨就停止並回報，不得強制覆蓋。
- 清除既有 ROS workspace `build`、`install`、`log` 衍生內容，再重新 build application workspace。
- 清除先前由其他 PX4 revision 產生的 SITL build，再從固定的 PX4 v1.17.0 revision 重新 build。
- 驗證完成後 workspace 保持在上述兩個 v1.17.0 revisions，不切回 PX4 main 或新版 `px4_msgs`。
- 所有 PX4、Gazebo、ROS 2、build、test 與 runtime commands 都必須在 container 內執行。

## Runtime evidence

- 不得只用 `ros2 topic list` 判定 telemetry 存在；subscriber 自己建立的 topic 不算 PX4 runtime 證據。
- 每台 vehicle 的必要 telemetry topic 都必須用 verbose topic inspection 確認 `Publisher count >= 1`。
- PX4/Micro XRCE-DDS publisher endpoint 預期顯示為 bare DDS application，並使用與 `px4_msgs` 1.17 相符的 message type。
- 必須分開辨識 raw PX4 `/MAVN/fmu/out/vehicle_status_v1` 與專案自訂 `/MAVN/status`。
- 三個 `/MAVN/status` 必須持續發布；因 PX4 1.17 無法明確提供 Offboard-setpoint acceptance，`offboard_available = false` 是預期且保守的結果，不得視為控制失敗。

## Manual control flow

從乾淨、landed、disarmed 的三機狀態開始，只啟動一個 active operator console，依序執行：

1. `1`：三機 takeoff 並到達 staging。
2. `2`：leader 以小步長朝正方向移動。
3. `x`：leader 以相同步長返回。
4. `7`：切換為 line-abreast formation 並等待穩定。
5. `6`：切回 VEE formation 並等待穩定。
6. `8`：三機 land。

若任何階段進入 failsafe、telemetry stale、動作 timeout 或無法確認安全狀態，停止後續 movement，優先執行既有安全降落流程並保留 logs。

## Acceptance criteria

- [x] PX4-Autopilot runtime/build identity 是 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`，且 `px4_msgs` identity 是 `86d8239e962f6939e05c3737784f60c02fa884db`。
- [x] PX4 v1.17 SITL、ROS interfaces 與 swarm-control packages 都從清除後的衍生目錄乾淨 build 成功。
- [x] Compatibility smoke check 在啟動飛行流程前通過，確認 message versions、必要欄位與 topic contract。
- [x] Gazebo 中存在三個分開初始位置的 vehicle models。
- [x] 一個 Micro XRCE-DDS Agent 接受三個 PX4 client sessions。
- [x] MAV1、MAV2、MAV3 的 local position `_v1`、vehicle status `_v1`、無 suffix command ack、land-detected 與 failsafe-flags topics 都有真實 bare-DDS publishers。
- [x] 三個 vehicle nodes 與 ground-station node 啟動後，三個 `/MAVN/status` 持續更新且 telemetry 不 stale。
- [x] Runtime logs 全程沒有 `AttributeError`，特別不得出現缺少 `accepts_offboard_setpoints` 的錯誤。
- [x] `operator_console` 的 `1` 成功完成三機 takeoff/staging。
- [x] `2` 與 `x` 都成功完成 leader 小步移動，且 followers 維持各自 formation slots。
- [x] `7` 與 `6` 都成功完成 formation change 並達到既有 settle criteria。
- [x] `8` 成功完成三機降落。
- [x] 最終 MAV1、MAV2、MAV3 都回報 `vehicle_state: landed` 與 `armed: false`。
- [x] 若自動 tests 通過但任一 PX4 build、publisher endpoint、takeoff、movement、formation change、landing 或 no-`AttributeError` 條件失敗，本 ticket 保持未完成。
- [ ] Focused tests、package tests、build 與 test-result 在最終 v1.17 workspace 上再次通過。

## Evidence to record

完成後在本 ticket 的 implementation notes 或 comments 記錄：

- 兩個實際 Git commit SHAs。
- 八個使用中 PX4 messages 的 version/shape compatibility 摘要。
- 三台 PX4 必要 publisher endpoints 的檢查摘要。
- Compatibility smoke、focused tests、package tests、build 與 test-result 摘要。
- Operator command sequence 每一步的成功或失敗結果。
- Runtime logs 中 `AttributeError` 搜尋結果。
- 三機最終 landed/disarmed status。
- 暫存 runtime log 路徑；大型 logs 不提交進 Git。

## Non-goals

- 不以完整 demo macro 取代本 ticket 的聚焦 control flow。
- 不修改上層 swarm action definitions 或 operator command semantics。
- 不修改 PX4-Autopilot source implementation。
- 不使用 PX4 main/current 或新版 `px4_msgs` 執行本驗收。
- 不因自動 tests 通過而跳過三機 Gazebo runtime 驗證。

## Comments

### 2026-08-20 implementation and container validation

- 固定 revisions：PX4-Autopilot `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`；
  `px4_msgs` `86d8239e962f6939e05c3737784f60c02fa884db`。
- 清除舊 PX4 SITL `build` 與 ROS workspace `build/install/log` 後，`make px4_sitl`
  及三個 ROS packages 的 fresh build 成功；compatibility gate 為 8/8 definitions
  matched。
- live bridge gate 驗證三個分離的 `x500_1/2/3` models、三個 Agent sessions，以及
  MAV1/MAV2/MAV3 共 15 個必要 telemetry publishers；message types 全部符合
  `px4_msgs` 1.17，publisher identity 全部為 bare DDS application。
- PX4 v1.17 x500 的 `NAV_DLL_ACT=2` 會要求 GCS；每個 SITL prompt 明確執行
  `param set NAV_DLL_ACT 0` 後，三台皆回報 `Ready for takeoff!`。未啟動 QGC 或
  translation node。
- container 實測發現 `VEHICLE_CMD_NAV_TAKEOFF.param7` 在 PX4 v1.17 是 AMSL，故
  adapter 在 boundary 以 `VehicleLocalPosition.ref_alt + relative altitude` 轉換，
  上層相對高度 API/action semantics 不變；未指定的 MAVLink command params 改為
  `NaN`，避免 `0` 被誤認為有效座標。
- 單一 operator console 的固定序列全部成功：`1` staging OK；`2` leader target
  OK；`x` leader target OK；`7` line-abreast established；`6` VEE established；
  `8` all vehicles landed。
- 最終 MAV1/MAV2/MAV3 都是 `armed: false`、`vehicle_state: landed`，telemetry age
  分別約 0.006 s、0.002 s、0.023 s；`offboard_available: false` 符合 v1.17
  conservative contract。
- `/tmp/ticket18_run4_{agent,px4_mav1,px4_mav2,px4_mav3,bridge_check,swarm,operator}.log`
  搜尋 `AttributeError|Traceback|FAIL:` 無結果；大型 PX4 ULog 未提交。
- Ticket 18 focused boundary/compatibility/bridge tests 通過。完整 package suite 為
  169 passed / 16 failed；16 項均為本 ticket 開始前已存在的 operation-profile
  defaults 與測試期望差異（`0.8/0.8` vs `0.4/0.6928`、`0.02` vs `0.10`）。依本
  ticket non-goal 未改動上層幾何或 operator semantics，因此最終 package-test
  acceptance 暫不勾選，交由 human 決定是否另開 ticket 對齊 operation profile。
- Final review 補強兩個 fail-closed 條件：若 `z_global/ref_alt` 尚未有效就不發布
  takeoff command，等待下一個 control tick 重試；live bridge gate 必須在同一個
  endpoint block 同時看到 bare-DDS node 與 `Endpoint type: PUBLISHER`。對抗性測試
  已加入，Ticket 18 focused suite 為 27 passed。
