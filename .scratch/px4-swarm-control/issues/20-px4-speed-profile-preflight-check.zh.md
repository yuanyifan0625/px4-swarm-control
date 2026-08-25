# 20 — PX4 speed profile 與啟動前參數檢查

**What to build:** 在 SITL 或實機控制前，操作者可以選擇一份明確的 PX4 speed profile，取得可貼到各 PX4 shell 的 `param show`／`param set` 指令，並在 terminal 看到目前值與期望值的比對結果。ROS 2 不加入一般速度限制器；實際水平、上升和 yaw 速度上限由 PX4 位置控制器負責。

**Blocked by:** None — can start immediately.

**Status:** ready-for-agent

- [ ] profile 明確設定水平速度上限 `MPC_XY_VEL_MAX=0.3 m/s`、上升速度上限 `MPC_Z_VEL_MAX_UP=0.3 m/s`、yaw 速度上限 `MPC_YAWRAUTO_MAX=30 deg/s`；下降速度沿用已確認的 PX4 設定。
- [ ] `check` 模式輸出每台 MAV 的 current/desired/match 結果；無 live parameter client 時清楚輸出需要在 PX4 shell 執行的檢查命令。
- [ ] apply 必須保留明確 operator confirmation，不會由 ROS 2 隱式修改 PX4 參數。
- [ ] profile validation、diff report、missing/mismatch 和 explicit apply tests 通過。
- [ ] 明確記錄檔案影響：修改 `px4_speed_profile.py`、`config/px4_speed_profiles/slow_demo.yaml`、`config/px4_speed_profiles/real_cautious.yaml`、相關 tests 與操作文件；不加入 ROS setpoint rate limiter。
