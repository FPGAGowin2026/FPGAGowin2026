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

本地 `fpga_ws/main` 已合入 `codex/bx72-ddr-mapping@911a158` 并通过 `dec08a5` 完成双版本雷达 SLAM，保留网络网页及在途改版。蓝牙独立使用 DDR3 地图，UDP 的视频双缓冲和地图共用一套 DDR3 控制器；两版共用雷达定位、打滑融合与独立避障。GoRV32/APB 和板载调试串口已移除。本轮软件回归及两版综合通过，PNR、厂商 DDR 颗粒仿真及硬件验收仍待完成，详见 [融合记录](docs/DDR建图融合记录.md)、[项目状态](docs/handoff.md) 与 [BX72 工程](FPGA_ws/boards/BX72_138K_robot/README.md)。远端 `main` 本轮未推送。

## 同步与开发

本目录和五个工作区独立版本管理。`GoWin/` 为本机工具，不由 repo 同步。

```bash
./Scripts/sync_workspaces.sh FPGA_ws ugv_ws
```

已有 repo 客户端也可运行 `git pull --ff-only` 更新顶层，再运行 `repo sync` 更新工作区；两种同步入口不要同时运行。初始化、选择性获取及版本锁定见 [清单说明](manifests/README.md)，提交/推送范围见 [脚本说明](Scripts/README.md)。同步前检查各仓库未提交改动。
