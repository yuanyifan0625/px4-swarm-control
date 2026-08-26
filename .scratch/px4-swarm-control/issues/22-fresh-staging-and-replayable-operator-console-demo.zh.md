# 22 — fresh staging anchor 與可重複的 operator_console 手動驗證

**What to build:** 三台機體完成 `LandSwarm` 後，不重啟 Gazebo、PX4 或 Micro-XRCE-DDS Agent，操作者仍可使用同一個 `operator_console` 再次完成 takeoff、VEE staging、leader movement、formation change、safety hold、resume 和 land。重新 takeoff 時使用 fresh leader position/yaw 計算 staging anchor。

**Blocked by:** 19 — 統一 operator_console field frame 與 canonical NED；20 — PX4 speed profile 與啟動前參數檢查；21 — follower collision safety hold 與 leader movement guard。

**Status:** ready-for-agent

- [ ] `TakeoffSwarm` 開始時讀取 fresh leader position/yaw，使用該 staging anchor 計算 VEE staging；不存在或 stale 時拒絕 takeoff，不 fallback 到固定 `(0,0)`。
- [ ] 新手動流程只使用 `operator_console`，不使用 `field_frame_console` 或 `coordinate_frame_probe` 作為控制入口。
- [ ] container 內文件明確列出 MicroXRCE-DDS Agent、三台 PX4/Gazebo（MAV1 `-i 0`、MAV2 `-i 1`、MAV3 `-i 2`）、ROS 2 launch/node 啟動順序與必要的 `NAV_DLL_ACT` 設定。
- [ ] 手動驗證可完成：`takeoff -> VEE staging -> leader movement -> formation change -> safety hold -> resume -> land`。
- [ ] land 後不重啟 Gazebo、PX4、DDS 或 swarm nodes，即可再次輸入 `1` 或其他 operator command 完成第二輪驗證。
- [ ] 驗證記錄包含 PX4 speed profile output、raw NED/status、formation settle、safety hold、resume、最終 landed/disarmed 和 runtime logs。
- [ ] 驗證通過後，刪除舊的 coordinate-probe、field-frame-console、分散 real/SITL smoke 手冊並重建一份統一手冊；在驗證前不得刪除原手冊。
- [ ] 明確記錄檔案影響：修改 `ground_station_node.py`、staging tests、launch/config README、`setup.py`；刪除或停止安裝 `coordinate_frame_probe` 與 `field_frame_console`；重建 `config/final_operator_console_sitl_real_manual.zh.md`。
- [ ] container 內 focused tests、package tests、build、test-result 和指定的 operator-console 手動 demo 結果均有記錄；既有非本 ticket failure 必須分開列明。
