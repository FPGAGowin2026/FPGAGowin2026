# FPGAGowin2026

BX72 / GW5AT-138C 四轮麦轮机器人研究原型。FPGA 负责传感器接入、轮速闭环、近障保护、位姿融合及局部栅格规划；Jetson 负责 UDP/ROS 网桥、视觉任务、slam_toolbox 全局建图、全局 A* 与航点调度。网页用于展示、选点和提交目标。速度型 FPGA 的局部规划由 Jetson 兼容执行。

2026-11-04，局部导航 candidate 08 已写入 FPGA Flash 并经工具校验，用户随后进行了航点实地测试，反馈效果可以。版本、测试范围和待确认事项见[航点导航部署与现场记录](FPGA_ws/boards/BX72_138K_robot/docs/field_waypoint_20261104.md)。[跨端功能链路审查](docs/跨端功能链路审查_20261102.md)保留迁移过程中的历史方案。

| 目录 | 内容 |
| --- | --- |
| `FPGA_ws` | 板级 RTL、软核固件、约束和构建入口 |
| `ugv_ws` | Jetson 网桥、ROS 传感器、视觉、全局导航和局部兼容执行 |
| `uav_ws` | 无人机 VIO、PX4/MAVROS 控制和本机协同导航 |
| `swarm_ws` | 机器人间的协同消息、目标与中继任务入口 |
| `website` | 网页服务、地图展示与目标提交 |
| [docs](docs/README.md) | 审查和架构文档入口 |
| [manifests](manifests/README.md) | 各独立仓库的 repo 清单 |

上述五个工作区分别是独立 Git 仓库。`FPGA_ws`、`ugv_ws` 和 `website` 从各自的提交创建；`uav_ws` 和 `swarm_ws` 在创建后复制了原工作区现有的未提交源码，以便继续协同开发，原文件仍保留在原位置。

板上版本及验证范围见[部署记录](FPGA_ws/boards/BX72_138K_robot/docs/deployment_current.json)和[验证摘要](FPGA_ws/boards/BX72_138K_robot/docs/validation_current.json)。本轮尚未获得定量到点误差、动态绕障及冷启动的完整验收记录；历史版本的三航点结果不自动等同于本轮结果。
