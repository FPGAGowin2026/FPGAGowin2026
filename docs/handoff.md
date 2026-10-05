> 当前状态与候选身份见[项目状态](project_status_20261005.md)；板级配置和原始证据见[BX72 基线记录](../FPGA_ws/boards/BX72_138K_robot/docs/baseline_unification_20261005.md)。

# 项目交接与证据边界

更新：2026-10-05。当前主线为 BX72 / GW5AT-138C＋四轮麦轮＋Jetson。最新遥控/遥测源码已融合并通过离线构建，新候选 296674D6 待实板验收；两人职责和下一交付见[开发计划](后续开发计划.md)。

## 接续入口

| 内容 | 入口 |
| --- | --- |
| 当前配置、命令与通信基线 | [BX72 README](../FPGA_ws/boards/BX72_138K_robot/README.md) |
| 当前融合候选与传感器验收 | [候选验证记录](../FPGA_ws/boards/BX72_138K_robot/docs/baseline_unification_20261005.md)、[Jetson 接入](../ros_ws/src/robot/fpga_gateway/BX72_SENSORS.md) |
| 辨认配置、测速和速度环 | [本板接口](../FPGA_ws/boards/BX72_138K_robot/docs/hardware_interfaces.md)、[运动控制开发](motor_development.md) |
| 已有定点融合实现 | [两板 EKF](robot_fpga_ekf.md) |
| 保留的独立板卡工程 | [ACG720 README](../FPGA_ws/boards/ACG720_60K_robot/README.md) |

## 工作区与证据

父仓库保存文档和同步脚本；FPGA_ws、ros_ws 是独立 Git 检出，需分别同步。FPGA_ws 当前本地分支为 `codex/integrate-latest-20261005`，融合提交 `160dfca`、整理提交 `b97279e`，未推送。旧远端获取方式不能自动取得本地候选/证据。

C934 是已验收通信基线，不等于当前源码或新遥测候选。其摘要在[BX72 验收记录](../FPGA_ws/boards/BX72_138K_robot/docs/udp_acceptance_20261004.json)，本地位流/原始日志是否齐备按板级说明核对。仿真、时序、板测、物理标定和实车性能分别签收，不能相互替代。

BX72 生成产物使用板内 build/、impl/；ACG720 保留板内 artifacts/ 的历史集成证据，整板时序尚未闭合。两板的接线、IP、位流与实测结果分别管理。

## 当前候选与整理后的接续

先阅读[项目状态](project_status_20261005.md)确认位流 SHA256，再按[遥控手册](../FPGA_ws/boards/BX72_138K_robot/docs/pc_wasd_teleop.md)准备配套 PC/Jetson 脚本；当前候选未烧录。下一阶段先悬空并发和停车，再整圈计数、落地负载及 ROS 控车。

退役 EF20/BXT1 支线、旧候选和可重建缓存已移出工程；当前位流和清单引用的证据仍完整。清理明细可从 Git `b97279e` 追溯，工作区外旧归档路径当前不可作为已存在的恢复入口。

## 迁移追溯

原 gowingame 于 2026-09-20 并入 FPGA_ws，独立同步入口已停用。历史迁移核对了 51 个搬移文件、21 个原位 N10 源文件和 29 个仿真文件的哈希。

当时 Linux 端归档位置为 `/home/aa/.fpga-workspace-backups/merge-20260920-174755/`，含 before-merge.tar.gz、gowingame-retired/ 和 migration_manifest.json；这不是当前 Windows 工作区的可运行入口。早期差速工程的完成度百分比、已移除工程命令和独立辨识下载阻塞不再列入当前交接。
