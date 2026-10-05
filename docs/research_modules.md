# 公共 RTL 与研究模块

更新：2026-10-05。本文目录和命令以 `FPGA_ws` 为根；共享文档统一维护于父仓库 `docs`。本页只列保留内容；当前板级接线、控制和传感器验收以各板 README 为准。

## 保留范围

| 路径 | 用途 |
| --- | --- |
| rtl/common | 当前 BX72 使用的异步帧 FIFO，以及公共 UART/帧发送 |
| rtl/ethernet | 当前 BX72 使用的 UDP 接收，以及网络研究与回归模块 |
| rtl/motor | 通用麦轮运动学、编码器、速度环及软件参数对照 |
| rtl/sensors/ov5640 | 独立 DVP 采集研究 |
| rtl/softcore、rtl/vendor | 保留 N10/RT-Thread 研究所需的 PicoRV32、总线、存储器、外设与供应商 IP |
| src/control/n10、src/sensors/n10 | 极坐标转换、占据/TSDF/NDT/CSM 研究 |
| src/protocol、src/system、src/vendor | N10 UART/USB、RT-Thread 桥接、Tang 顶层及 PLL |
| sim/fusion、sim/n10 | 板级 EKF 回归与 N10 测试 |
| sim/motor、sim/tools、sim/common | 通用电机、日志工具及 UART 回归 |

BX72 活动工程直接引用 `rtl/ethernet/eth_udp_rx_gmii.v` 和 `rtl/common/cdc/async_frame_fifo.v`。板级验证脚本还调用根目录融合脚本及 `sim/n10/tb_v4_csm_submap.sv`，这些文件不能随缓存删除。

## PicoRV32 与 N10/RT-Thread

N10 SoC 使用 rtl/softcore/soc/n10_rtt、src/system/n10_rtt 和 src/protocol/n10_rtt；头文件先搜索 n10_rtt，再搜索 pico_root。共享 CPU、总线、存储器和外设继续保留。

该历史研究目标的固件源和镜像此前已移除，因此不是当前可完整重建的板级交付；当前两板使用各自 GoRV32 工程。保留硬件依赖，避免仅删除 PicoRV32 后留下断裂的 N10 顶层。

## 回归入口

在 FPGA_ws 根目录执行，工具需已安装：

```bash
bash Scripts/run_motor_sim.sh
```

```powershell
python Scripts/verify_workspace.py
python -m unittest discover -s sim/tools -p test_motor_log_analyzer.py -v
pwsh -NoProfile -File Scripts/run_fusion_sim.ps1 -Board BX72_138K_robot
```

两板融合 HDL 使用板内 src/fusion/EKF；数学包先编译。视频通用脚本按板分流，融合/网络回归的生成目录可以重新创建。

## 已移除范围

2026-10-05 移除六个旧架构/JY62 例程、旧 IMU963RA 支线、根目录独立 AT8236 辨识/闭环包装及配套测试。当前板级电机调试和实现仍保留在 boards；根目录旧 build 缓存与 artifacts 校验产物不再作为交付入口。

独立网络研究仍保留；udp_loopback_probe.py 只针对支持回环的目标，不是当前 BX72 视频或运动工具。
