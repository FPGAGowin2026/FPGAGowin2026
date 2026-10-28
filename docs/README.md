# 文档入口

当前实现为极坐标基础导航，不再把旧EKF、距离场CSM、栅格和A*当作活动架构。

- [当前架构与取舍](小车软硬件协同规划架构.md)：板内职责、算法、单位、任务协议、安装补偿及验收边界。
- [板级工程与下载](https://github.com/FPGAGowin2026/fpga_ws/blob/main/boards/BX72_138K_robot/README.md)：构建、SRAM入口、资源时序、真实板测和保留产物。
- [可随源码获取的当前验证摘要](https://github.com/FPGAGowin2026/fpga_ws/blob/main/boards/BX72_138K_robot/docs/validation_current.json)：明确区分通过、失败、未运行；原始输出仍在本机build目录。
- [Jetson软件与协议](https://github.com/FPGAGowin2026/ugv_ws/blob/main/README.md)、[网页说明](https://github.com/FPGAGowin2026/website/blob/main/README.md)。
- [红蓝白色卡](红蓝白.docx)：打印参考；不是精确色度标定。
- [室内异构集群方案](室内异构集群方案.md)、[学习与参考](学习与参考.md)：独立拓展/阅读资料，不表示本车已实现对应算法。

`candidate_05`已有可追溯位流、真实软核执行、姿态、彩色视频、零丢样稳定观察与任务链路。2026-10-28用户授权后完成外部Flash写入、校验与重新加载；RX需要先用probe训练再恢复网桥，自动启动ACK恢复失败及训练期超时均保留，训练后独立99发/99 ACK通过。当前部署记录见板工程 `docs/deployment_current.json`，启动步骤见板工程README。任务负载下31.227 ms控制间隔超过30 ms，整体导航未通过。`candidate_06`的调度改动没有PNR或板测通过结果。10 cm只是后续外部测量目标；当前不启用ARM、不进行真实电机动作，物理断电冷启动尚未测试。

板卡倒装默认IMU Rx(pi)，原始IMU遥测保留传感器轴；车头X、偏航外参、轮速尺度、保护包络和颜色质量仍须标定。
