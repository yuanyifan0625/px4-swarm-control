# 19 — 統一 operator_console field frame 與 canonical NED

**What to build:** 使用者只透過 `operator_console` 輸入固定 field-frame 指令；系統在操作入口將 field `+X/+Y/up` 轉成固定的 PX4 local NED mapping，之後 ground station、formation、staging、follower 和 PX4 adapter 全部只使用 canonical NED。VEE 與 line-abreast 的新幾何也在這個 ticket 完成。

**Blocked by:** None — can start immediately.

**Status:** ready-for-agent

- [ ] 固定 mapping 為 field `+X -> NED +Y`、field `+Y -> NED +X`、field `+up -> NED -Z`，且不隨 leader yaw 重新定義。
- [ ] `operator_console` 是唯一 field-frame 手動入口，既有 `/swarm` action surface 不變。
- [ ] 所有下層 position/setpoint/formation/staging 計算使用 NED；PX4 interface 不承擔 field-frame 語意。
- [ ] VEE staging/following 使用 leader body-frame offset：`x=-1.0 m`、左右 `y=+/-1.0 m`，並依 leader yaw 旋轉。
- [ ] line-abreast 使用 `x=0`、左右 `y=+/-1.0 m`。
- [ ] 新增 frame transform tests，並保留目前 user-owned target-system、SITL pose 與部署參數修改。
- [ ] 明確記錄檔案影響：新增 `frame_transform.py`；修改 `operator_console.py`、`geometry.py`、`operation_profile.py`、相關 tests、`setup.py`/文件註冊；不修改 PX4-Autopilot、`px4_msgs` 或 `/swarm` interfaces。
