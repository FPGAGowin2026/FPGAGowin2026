> 当前状态与候选身份见[项目状态](project_status_20261005.md)；板级配置和原始证据见[BX72 基线记录](../FPGA_ws/boards/BX72_138K_robot/docs/baseline_unification_20261005.md)。

# 项目文档索引

更新：2026-10-05。当前 BX72 已融合 WASD/UDP 三轴速度与 RX 相位训练，20 步离线回归、完整构建及内部 STA 通过；新候选 296674D6 未烧录。下一任务是候选实板并发/停车验收、测速尺度与落地验证。ACG720 保留独立工程，其 DDR 时序仍未闭合。

## 当前开发入口

| 文档 | 内容 |
| --- | --- |
| [项目当前状态](project_status_20261005.md) | 源码/候选身份、构建证据、整理边界和下一验收顺序 |
| [后续开发计划](后续开发计划.md) | 当前基线、三人分工、接口约定和下一共同里程碑 |
| [handoff](handoff.md) | 当前接续入口、进行中任务边界与历史迁移记录 |
| [系统总体方案](系统总体方案.md) | Jetson 协同架构、FPGA 实时反射与两板职责 |
| [赛题 FPGA 部分实施规划](赛题FPGA部分实施规划.md) | FPGA＋软核独立基础任务与扩展任务部署边界 |
| [数据契约](udp_protocol.md) | 已实现的 BX72 速度/遥测协议、后续扩展与历史差速格式 |
| [BX72 工程](../FPGA_ws/boards/BX72_138K_robot/README.md) | 当前主线板卡、配置、构建及证据入口 |
| [BX72 WASD 遥控](../FPGA_ws/boards/BX72_138K_robot/docs/pc_wasd_teleop.md) | 独立 PC/Jetson 网关、速度线格式、停车和恢复行为 |
| [BX72 双向 UDP](../FPGA_ws/boards/BX72_138K_robot/docs/bidirectional_udp.md) | C934 基本应答、视频共存与手动刷新邻居后的恢复验收 |
| [BX72 传感器接入](../ros_ws/src/robot/fpga_gateway/BX72_SENSORS.md) | BX72 v1 五类接收、物理核对及 ROS 只读接入 |
| [BX72 硬件接口](../FPGA_ws/boards/BX72_138K_robot/docs/hardware_interfaces.md) | 本板管脚、UART、电机控制与辨认记录边界 |
| [运动控制开发](motor_development.md) | 运动学、参考参数、标定与日志分析 |
| [两板 FPGA EKF](robot_fpga_ekf.md) | 已有七状态定点原型、时间处理和实车验证边界 |
| [ACG720 工程](../FPGA_ws/boards/ACG720_60K_robot/README.md) | 独立 TFT 集成、固件与未闭合整板时序 |
| [公共 RTL 与研究模块](research_modules.md) | 保留源码范围、独立实验边界及回归入口 |
| [验证历史](validation_history.md) | 两板阶段结果和证据使用范围 |

当前身份以[项目状态](project_status_20261005.md)和候选清单为准，任务顺序见[开发计划](后续开发计划.md)。远端实板记录与本地候选验收分别管理。

## 历史与参考资料

| 文档 | 阅读范围 |
| --- | --- |
| [EF20/BXT1 历史计划](bx72_ef20_status_and_plan_20261005.md) | 已退役候选及当时任务，不作为当前部署/运行入口 |
| [电机软件验证记录（2026-09-23）](电机软件验证记录_20260923.md) | 早期 ACG720 独立工程软件/离线结果，不是当前 BX72 完成度 |
| [GitHub 参考项目清单](GitHub参考项目清单.md) | 开源实现及借鉴边界，不作为实板证据 |
| [学习路线](嵌赛FPGA国测零基础详细学习路线.md) | 独立学习资料，不是项目排期 |

历史文档中的旧路径、频率和默认参数只对应其记录日期；当前运行命令以板级和 ROS 包 README 为准。

## 文档集中维护

原 FPGA_ws/docs 的运动控制、两板 EKF、研究模块和验证历史已整体融合至本目录；板级专属接线/操作/验收手册仍随 boards 下工程维护。共享正文仅维护本目录一份，FPGA_ws 不再保留共享文档副本。
