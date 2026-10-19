# FPGAGowin2026

Jetson Orin Nano＋四轮麦轮＋FPGA 实时控制。主板为 BX72 / GW5AT-138C，连接 AT8236、编码器、IMU660RC 和 N10；UDP 版本另带 OV5647 CSI-2 与 DDR3 视频，OV5640 源码参考保留。

| 工作区 | 内容 |
| --- | --- |
| [FPGA_ws](https://github.com/FPGAGowin2026/fpga_ws#readme) | 板级 RTL、约束、构建与仿真；BX72 分蓝牙版和 UDP 版 |
| [ugv_ws](https://github.com/FPGAGowin2026/ugv_ws#readme) | 小车 ROS 2、FPGA 网关、导航与任务 |
| [uav_ws](https://github.com/FPGAGowin2026/uav_ws#readme) | 无人机工作区 |
| [swarm_ws](https://github.com/FPGAGowin2026/swarm_ws#readme) | 集群接口与协调，目前含占位模块 |
| [llm_ws](https://github.com/FPGAGowin2026/llm_ws#readme) | 独立语音实验 |
| [website](website/README.md) | 从 FPGA_ws 提取的独立 BX72 控制台网页（浅色设计、本机 Node 服务、离线模拟器与测试） |
| [docs](docs/README.md) | 当前状态、控制定位、Jetson 接入及集群方案 |

当前 FPGA 基线为 `main`（融合与迁移节点 `453f8dd`，精简节点 `3314eba`）：已合入 OV5647 CSI-2，并完成保持行为的代码精简，源码与工程清单净减少 812 行，UDP 活动文件由 74 项减至 72 项。蓝牙独立使用 DDR3 地图，UDP 视频与地图共用一套 DDR3 控制器。两版保留定位、打滑融合与独立避障，移除 GoRV32/APB 和板载调试串口。融合与精简的静态检查、RTL 仿真通过；当前输入尚未重新综合、PNR、烧录或实板验收。详见 [项目状态](docs/handoff.md) 与 [精简记录](FPGA_ws/boards/BX72_138K_robot/docs/code_simplification_20261019.md)。

网页部分已提取到独立工程 [website](website/README.md)：服务端、页面、适配器、前端测试与验收文档全部迁入并在该目录独立运行，界面按参考稿重做为浅色控制台。`FPGA_ws` 中的原网页文件、网页文档与旧截图已删除，历史文档归档在 [website/docs/archive](website/docs/archive/README.md)。

## 同步与开发

本目录和六个子工程独立版本管理。website 仓库为 [FPGAGowin2026/website](https://github.com/FPGAGowin2026/website)。`GoWin/` 为本机工具，不由 repo 同步。

```bash
./Scripts/sync_workspaces.sh FPGA_ws ugv_ws
```

已有 repo 客户端也可运行 `git pull --ff-only` 更新顶层，再运行 `repo sync` 更新工作区；两种同步入口不要同时运行。初始化、选择性获取及版本锁定见 [清单说明](manifests/README.md)，提交/推送范围见 [脚本说明](Scripts/README.md)。同步前检查各仓库未提交改动。
