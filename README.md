# FPGAGowin2026

Jetson Orin Nano＋四轮麦轮＋FPGA 实时控制。主板为 BX72 / GW5AT-138C，连接 AT8236、编码器、IMU660RC 和 N10；UDP 版本另带 OV5640 与 DDR3 视频。

| 工作区 | 内容 |
| --- | --- |
| [FPGA_ws](https://github.com/FPGAGowin2026/fpga_ws#readme) | 板级 RTL、约束、构建与仿真；BX72 分蓝牙版和 UDP 版 |
| [ugv_ws](https://github.com/FPGAGowin2026/ugv_ws#readme) | 小车 ROS 2、FPGA 网关、导航与任务 |
| [uav_ws](https://github.com/FPGAGowin2026/uav_ws#readme) | 无人机工作区 |
| [swarm_ws](https://github.com/FPGAGowin2026/swarm_ws#readme) | 集群接口与协调，目前含占位模块 |
| [llm_ws](https://github.com/FPGAGowin2026/llm_ws#readme) | 独立语音实验 |
| [docs](docs/README.md) | 当前状态、控制定位、Jetson 接入及集群方案 |

FPGA 各分支已合入 `fpga_ws/main`，工程源码基线为 `bbc5c95`，开发分支已清理。BX72 两版都保留雷达定位、打滑融合与独立避障；GoRV32/APB 和板载调试串口已移除。两版内部时序通过，当前版本尚未上板验收。详情见 [项目状态](docs/handoff.md) 与 [BX72 工程](FPGA_ws/boards/BX72_138K_robot/README.md)。

## 同步与开发

本目录和五个工作区独立版本管理。`GoWin/` 为本机工具，不由 repo 同步。

```bash
./Scripts/sync_workspaces.sh FPGA_ws ugv_ws
```

已有 repo 客户端也可运行 `git pull --ff-only` 更新顶层，再运行 `repo sync` 更新工作区；两种同步入口不要同时运行。初始化、选择性获取及版本锁定见 [清单说明](manifests/README.md)，提交/推送范围见 [脚本说明](Scripts/README.md)。同步前检查各仓库未提交改动。
