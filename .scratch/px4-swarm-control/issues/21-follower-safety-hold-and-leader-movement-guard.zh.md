# 21 — follower collision safety hold 與 leader movement guard

**What to build:** 當 follower 與其他機體的水平距離進入不安全區域時，follower 不再追逐新的 formation target，而是重發最後一個通過安全檢查的 NED target；當 leader 的新 movement goal 會造成 follower 不安全時，ground station/`operator_console` 拒絕該 goal。telemetry 不可信時系統 fail closed。

**Blocked by:** 19 — 統一 operator_console field frame 與 canonical NED。

**Status:** ready-for-agent

- [ ] safety distance 使用水平 XY 距離，minimum distance 為 `0.7 m`。
- [ ] 距離低於 `0.7 m` 連續 `1.0 s` 才進入 safety hold；恢復到 `0.7 m` 以上連續 `1.0 s` 才解除。
- [ ] safety hold 發布最後一個通過安全檢查的 NED target，不加入一般 ROS 速度限制器。
- [ ] 沒有 last-safe target 時，左／右 follower 依 leader body-left/body-right slot 方向建立受 PX4 速度上限約束的退避 target。
- [ ] leader movement guard 同時檢查 follower 實際 telemetry position 和理論 formation position；不安全 goal 必須拒絕並輸出原因。
- [ ] own/follower/leader telemetry stale 或不可信時，拒絕新的 movement 並凍結安全 target。
- [ ] safety hold、解除 debounce、last-safe target、slot fallback、stale telemetry 和 leader guard 都有 public-seam tests。
- [ ] 明確記錄檔案影響：新增 `collision_safety_gate.py`；修改 `vehicle_node.py`、`ground_station_node.py`、必要的 follower/status models 與 tests；不新增 ROS action/message/service。
