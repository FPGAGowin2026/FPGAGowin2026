# FPGAGowin2026

当前设计：**Jetson Orin Nano＋四轮麦轮＋FPGA 实时反射层**。电机驱动采用 AT8236，主要 IMU 选 IMU660RC，摄像头使用 OV5640，N10 全部实验保留。当前优先功能正确、可调试与开发速度，停车与恢复需单独验证；规划不等于已实现。

## 工作区

| 路径 | 职责 |
|---|---|
| [FPGA_ws](FPGA_ws/README.md) | 独立 `fpga_rtl` 仓库，统一维护 RTL、PicoRV32/总线/存储器、软核程序与仿真 |
| [ros_ws](ros_ws/README.md) | Jetson ROS 2 通信、导航、视觉和任务包 |
| [docs](docs/README.md) | 总体方案、交接与通信/控制说明 |
| `GoWin/` | 本机 Gowin EDA 与许可证；60K 工程位于 `FPGA_ws/` |
| [llm_ws](llm_ws/README.md) | 独立语音实验，不是主线依赖 |

当前交接基线、候选交付缺口和待验收项见 [handoff](docs/handoff.md)。

## 同步入口

```bash
./Scripts/sync_fpga_workspace.sh
./Scripts/sync_ros_packages.sh
./Scripts/sync_llm_workspace.sh
```

同步脚本仅负责克隆或快进更新独立仓库，不生成模型、固件或依赖。旧 `sync_gowingame.sh` 只返回停用提示，不再克隆旧仓库。发布脚本使用方法及 add-all 范围见 [Scripts](Scripts/README.md)。

## FPGA 当前目录

```text
FPGA_ws/
├── boards/       # BX72 与 ACG720 独立板级工程、约束及固件
├── rtl/          # 活动工程共享 FIFO、UDP 接收及电机回归
├── sim/          # 仿真与离线工具
└── Scripts/      # 构建和检查入口
```

板级工程、活动约束与验证入口见 [FPGA_ws](FPGA_ws/README.md)。本地生成位流是否齐备以 [handoff](docs/handoff.md#证据与交付缺口) 为准。

ROS 运行节点与工具已迁为 C++17；BX72 遥控使用 PC UDP，传感器 ROS 入口只读，`/cmd_vel` 适配仍待完成。部署与验证边界见 [Jetson 接入](docs/Jetson通信与接入.md)。
