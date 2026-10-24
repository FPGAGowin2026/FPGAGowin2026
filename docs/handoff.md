> 2026-10-20 状态更新：BX72 已收敛为纯 RTL 单 UDP、视频独占 DDR3；全局地图与蓝牙移除，CSM 局部定位子图保留。本文其余旧版本描述和构建数字均为历史记录，本轮结果见 [实施记录](../FPGA_ws/boards/BX72_138K_robot/docs/udp_only_implementation_20261020.md)。

# 项目状态与交接

更新：2026-10-19。当前 FPGA 分支为 `main`（已快进合入融合分支至 `453f8dd`，融合分支随后删除），融合提交 `e0f188a`，精简提交 `3314eba`。网页迁出到独立仓库 [FPGAGowin2026/website](https://github.com/FPGAGowin2026/website)，本地目录为 [website](../website/README.md)。

| 范围 | 当前状态 |
| --- | --- |
| BX72 公共核心 | 四轮闭环、编码器、IMU660RC、N10、CSM、七状态 EKF、打滑融合和独立避障；无 GoRV32/APB 或板载调试串口 |
| 蓝牙版 | UART 9600/8N1、v4 遥测、DDR3 雷达 SLAM 地图；无视频和以太网 |
| UDP 版 | OV5647 CSI-2 RAW10，经 Bayer/裁剪、白平衡与 Gamma 输出 800×480 RGB565；视频双缓冲与雷达地图共用唯一 DDR3 控制器；无蓝牙 |
| OV5640 | 原样参考源码和 ACG720 工程保留，不作为 BX72 UDP 活动摄像头 |
| 精简 | 源码与工程清单净减 812 行，UDP 活动文件 74→72；保持协议、流水和功能，不据行数宣称资源或时序收益 |
| website | 页面、Node 服务、适配器、模拟器、测试与网页文档独立维护；FPGA_ws 已移除旧副本 |
| Jetson/ROS | 当前 C++ 网关位于 ugv_ws/src/fpga_gateway；真实跨仓接入待验收 |
| ACG720 / 25K | ACG720 整板资源/时序未通过；25K 为固定脉宽舵机仿真，引脚和完整器件待核对 |

## 当前验证与边界

- OV5647 融合阶段：RTL 48 次执行、47 个不同测试台通过，Python 59/59；详细证据见 [融合记录（历史文档已整理）](../FPGA_ws/boards/BX72_138K_robot/docs/udp_only_implementation_20261020.md)。
- 精简阶段：Gamma 新旧四态对照 22532 次通过、相关 RTL 20 次执行通过、Python 59/59；工程与非加密 HDL 检查通过，见 [精简记录（历史文档已整理）](../FPGA_ws/boards/BX72_138K_robot/docs/udp_only_implementation_20261020.md)。厂商加密 IP 与黑盒检查边界保留。
- 网站迁移复核：本机 `npm test` 为 44 通过、0 失败、1 跳过。WSL C++ 网关验收因未提供程序参数跳过；现行启动、测试及历史资料恢复方式见 [网站说明](../website/README.md)。
- 当前 OV5647 融合与精简源码未重新综合、PNR、生成匹配验收位流或烧录。此前蓝牙 PNR 通过、UDP PNR 失败均属于历史输入，详见 [PNR 历史记录](PNR闭环验证记录.md)。
- 遥测 RTL 交叉解码已使用仓库内历史 Python 参考快照通过，不代表当前 C++/ROS 网关已集成验收。2 项 OpenCV/NumPy 桥接测试因依赖缺失跳过。
- 厂商 DDR PHY/颗粒仿真、外部 IO 时序、相机稳定性、共享带宽、真实运动与定位精度仍待验证。旧镜像板测不能继承。

两版均无地图上行或 ROS OccupancyGrid 发布；网站雷达点、扇区和轨迹不是 DDR 栅格地图。自动模式默认关闭，切换先停车，控制层租约 200 ms；UDP 新会话先零速解锁。ACK 只证明入队。

## 下一步

1. 对当前输入重新综合与 PNR，每轮 PNR 仍限制 1800 秒；保存源码、工具、选项、时序、资源和位流哈希，不复用不匹配的历史通过结果。
2. 完成真实 Jetson C++ 网关与网站联调，核验控制、遥测、视频、断线恢复及租约停车。
3. 固定车体完成通信和停车验收，再标定编码器、轮径、车体几何、IMU/N10 外参；低速验证定位、DDR 建图/回环与视频并发。
4. 补齐 ROS 里程计、完整 LaserScan、速度和任务接口，推进导航；当前统一网关不订阅 /cmd_vel。

接线见 [硬件接口](../FPGA_ws/boards/BX72_138K_robot/docs/video_jetson.md)，部署见 [Jetson 接入](Jetson通信与接入.md)，三人分工见 [集群方案](室内异构集群方案.md)。
