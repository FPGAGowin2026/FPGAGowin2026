# FPGAGowin2026

当前设计：**Jetson Orin Nano＋四轮麦轮＋FPGA 实时反射层**。电机驱动采用 AT8236，主要 IMU 选 IMU660RC，保留 IMU963RA，摄像头使用 OV5640，N10 全部实验保留。当前优先功能正确、可调试与开发速度，安全基础暂不实现；规划不等于已实现。

## 工作区

| 路径 | 职责 |
|---|---|
| [FPGA_ws](FPGA_ws/README.md) | 独立 `fpga_rtl` 仓库，统一维护 RTL、PicoRV32/总线/存储器、软核程序与仿真 |
| [ros_ws](ros_ws/README.md) | Jetson ROS 2 通信、导航、视觉和任务包 |
| [docs](docs/README.md) | 总体方案、数据契约与历史验证边界 |
| `GoWin/` | 本机 Gowin EDA 与许可证；60K 工程位于 `FPGA_ws/` |
| [llm_ws](llm_ws/README.md) | 独立语音实验，不是主线依赖 |

原 gowingame 已退役，源码并入 FPGA_ws；完整旧仓库与未提交状态已本地归档，位置见 [handoff](docs/handoff.md)。迁移尚未提交或推送，远端仓库尚未同步此布局。

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
├── rtl/          # 通用 RTL 与 softcore 硬件
├── firmware/     # 软核程序入口，目前尚无应用源码
├── src/          # 保持原位的 N10/n10_rtt 实验
├── sim/          # 现有仿真与桩
└── README.md     # 目标依赖和验证边界
```

`build/` 按需生成并忽略。未创建空板级工程、虚构固件镜像或恢复此前删除的程序。当前缺完整 Gowin 工程和约束，旧 `run_all_sim.sh` 等入口已不存在。最新迁移没有重新验证仿真、综合、STA 或实车；历史结果见 handoff。

ROS 当前代码仍是旧差速协议，尚未适配四轮麦轮。其实际包名 `fpga_gateway` 对应方案中的 fpga_bridge 职责，不能因目录迁移宣称新系统联调完成。
