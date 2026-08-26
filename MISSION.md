# Mission: PX4 訊息版本相容性診斷

## Why
能快速定位 PX4 firmware、`px4_msgs` 與 ROS 2 application 之間的 contract 差異，並在不擴散版本知識到上層 swarm logic 的前提下安全修正實驗室 PC、Gazebo 與樹莓派部署。

## Success looks like
- 能由例外、message shape 與 publisher endpoint 判斷版本不相容的位置
- 能把相容處理放在 PX4 adapter seam，維持上層 interface 穩定
- 能用 unit、compatibility smoke 與 Gazebo runtime 三層證據驗證修正

## Constraints
- 說明必須精準、快速，優先使用目前 workspace 的真實程式與版本
- ROS 2、PX4、Gazebo、build 與 runtime 驗證都在 container 內進行
- 正式 contract 是 PX4 v1.17.0 與 `px4_msgs` v1.17.0

## Out of scope
- 深入 PX4 flight-control algorithms
- 修改上層 swarm actions 或 operator command semantics
- 正式支援 PX4 main/current 的新版 message contract
