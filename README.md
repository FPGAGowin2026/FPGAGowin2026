# FPGAGowin2026

Jetson Orin Nano、四轮麦轮与BX72/GW5AT-138C研究原型。当前小车只使用UDP通信，不实现全局地图。OV5647/ISP输出800×480 YUYV PT97，DDR3独占视频双缓冲；局部地图与规划工作集优先使用片上RAM。

| 位置 | 内容 |
| --- | --- |
| `../bx72-gowin-isp` | 唯一FPGA实施工作树：传感器、连续odom、滚动障碍图、轻量样条、统一运动入口与视频网络 |
| `FPGA_ws` | 同一仓库的旧检出，仅保留本地修改与参考，不另行实现 |
| [ugv_ws](ugv_ws/README.md) | 单UDP网桥、ROS扫描/里程计/TF、视觉和局部任务 |
| [website](website/README.md) | PC控制台、本地服务、UDP任务与状态 |
| [docs](docs/README.md) | 公共合同、来源清单、交接和本轮验证记录 |

uav_ws、swarm_ws与llm_ws继续作为独立工作区保留，不属于本轮小车重构。其他板卡工程功能不变。

本次交接按用户要求将未完成实施正常推送各仓库main，再删除整合分支；具体结果见docs/refactor_20261024/交接报告.md。main不表示代码已完成或通过。历史支线已归档后删除。完整原文件、未跟踪内容、ignored证据和Git历史归档位于`E:/project/FPGA_workspace/_refactor_backups/20261024_lunamax`。

本轮禁止综合、PNR、烧录、Flash和任何真实电机动作；代码全部完成并冻结后才统一离线验证。设计吞吐/资源预算不是实测结果。