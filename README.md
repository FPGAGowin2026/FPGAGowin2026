# FPGAGowin2026

BX72 / GW5AT-138C 四轮麦轮机器人研究原型。当前采用GoRV32 FP 40 MHz、RTL雷达聚合、互补里程计、点线ICP和极坐标局部绕障；OV5647输出640×360彩色YUYV422，经Jetson网桥到网页。外部AI负责视觉和高层任务，基础控制在FPGA内执行。

当前板上版本为2026-10-28的`candidate_05`，已按用户授权写入外部Flash并完成校验、从Flash重新加载及无动作观察。重配置后ACK需先通过诊断probe进行RX相位训练，再启动在线网桥，步骤见板工程README；自动启动握手恢复问题仍未修复。固件、综合、PNR、真实软核执行和彩色视频已有证据，**完整导航验收未通过**：任务联调累计最长控制间隔31.227 ms，超过30 ms门槛。后续调度尝试`candidate_06`仅综合通过、PNR超时，未下载；活动源码和ROM保持已上板版本。电机保持锁定，真实闭环行驶、动态绕障、物理断电冷启动和约10 cm实际到点精度未验证。

| 本地目录 / 远端仓库 | 内容 |
| --- | --- |
| `FPGA_ws` / [fpga_ws](https://github.com/FPGAGowin2026/fpga_ws) | 活动GPRJ、RTL、固件、构建/下载入口与当前验证摘要 |
| `ugv_ws` / [ugv_ws](https://github.com/FPGAGowin2026/ugv_ws) | 唯一在线UDP网桥、ROS传感器/里程计/TF、视觉与任务接口 |
| `website` / [website](https://github.com/FPGAGowin2026/website) | 本机Node服务、实时彩色视频、遥测与局部任务工作台 |
| [docs](docs/README.md) | 当前架构、使用入口、标定与未完成项 |
| [manifests](manifests/README.md) | 六个独立工作区的repo清单；子仓库不嵌入本仓库 |

从[当前方案](docs/小车软硬件协同规划架构.md)和[板级说明](https://github.com/FPGAGowin2026/fpga_ws/blob/main/boards/BX72_138K_robot/README.md)进入。源码仓库提供当前验证摘要；位流、完整原始报告和板测包保留在本机`FPGA_ws/boards/BX72_138K_robot/build/polar_navigation_20261028/`，不把生成缓存提交到Git。

FPGA为192.168.0.2:5000，Jetson有线192.168.0.4/24、SSH `aa@192.168.1.103`。视频6102、ACK6103、遥测6104、PC网桥6200；`record`与在线网桥互斥占用接收端口。网页从`website`运行`node server.mjs --jetson 192.168.1.103 --port 6200`，访问`http://127.0.0.1:8765`，默认只读监看。

仅在所有代码集成、写入者停止后统一验证；Gowin串行，综合与PNR共用截止预算。SRAM无动作自检与Flash、ARM、真实动作分开授权。其他板卡及uav_ws、swarm_ws、llm_ws按各自工程维护，不继承BX72结论。
