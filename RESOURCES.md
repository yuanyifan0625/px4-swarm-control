# PX4 訊息版本相容性資源

## Knowledge

- [MoRE: Unlocking Scalability in Reinforcement Learning for Quadruped Vision-Language-Action Models](https://arxiv.org/abs/2503.08007)
  本次課程的主要原始論文；定義 MoRE 的 LoRA experts、稀疏 router、MDP 結構與 Q-function training objective。

- [RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control](https://arxiv.org/abs/2307.15818)
  代表性的 VLA 原始論文，說明如何把視覺、語言與機器人 action 放進端到端模型。
- [OpenVLA: An Open-Source Vision-Language-Action Model](https://arxiv.org/abs/2406.09246)
  開放的 VLA 原始論文，可用來理解大型 vision-language backbone 與 robot demonstrations 的結合。
- [DAgger: A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning](https://arxiv.org/abs/1011.0686)
  模仿學習的經典原始論文，介紹如何處理模型自己造訪到的狀態分布與專家標註。

- [PX4 v1.17.0 `VehicleStatus` definition](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/versioned/VehicleStatus.msg)
  飛控端的權威 message shape。用於確認 `MESSAGE_VERSION = 1`、`pre_flight_checks_pass` 存在，以及 `accepts_offboard_setpoints` 不存在。
- [`px4_msgs` v1.17.0 `VehicleStatus` definition](https://github.com/PX4/px4_msgs/blob/86d8239e962f6939e05c3737784f60c02fa884db/msg/VehicleStatus.msg)
  ROS application 編譯時看到的權威 message shape。用於和 PX4 firmware revision 做逐欄位比對。
- [PX4 ROS 2 Message Translation Node documentation](https://docs.px4.io/main/en/ros2/px4_ros2_msg_translation_node.html)
  PX4 官方對 message version suffix 與跨版本 translation 的說明。用於區分相同版本直接通訊和不同版本需要 translation 的情境。
- [Ticket 17: PX4 1.17 telemetry contract](./.scratch/px4-swarm-control/issues/17-align-px4-v117-telemetry-contract.zh.md)
  本專案已定案的 adapter fallback、topic naming 與 automated acceptance contract。
- [Ticket 18: PX4 1.17 three-vehicle Gazebo validation](./.scratch/px4-swarm-control/issues/18-validate-px4-v117-three-vehicle-gazebo-control.zh.md)
  本專案端到端驗證流程與安全完成條件。

## Wisdom (Communities)

- [PX4 Discuss](https://discuss.px4.io/)
  PX4 官方社群；適合核對特定 firmware、uXRCE-DDS 與 ROS 2 deployment 組合的實務問題。
- [PX4 GitHub Discussions](https://github.com/PX4/PX4-Autopilot/discussions)
  適合攜帶精確 commits、topic endpoint 與最小重現案例詢問維護者。
