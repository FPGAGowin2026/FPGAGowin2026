# FPGA 机器人赛题 GitHub 参考项目清单

检索：2026-09-24。面向 ACG720-60K、OV5640、四轮编码器、IMU660RC、2D 雷达、EKF 与 FPGA 实时反射层。以下项目用于阅读设计和建立对照实验；器件、传感器协议与时钟约束均须按本车重做。

## 优先阅读

| 项目 | 借鉴位置 | 与本车的关系和边界 |
|---|---|---|
| [lawrie/fpga_roomba](https://github.com/lawrie/fpga_roomba) | `read_lidar.py`、`map.py` 和 `blackice_mx.py`：雷达 UART 接入、角度查表、占据栅格与硬件近距停车 | 最贴近“实时反射层”；采用 LD19、Roomba 和 Amaranth HDL，未在 FPGA 中实现完整 SLAM/EKF。可借鉴雷达数据一分为二：本地保护与地图。 |
| [alebal123bal/Gowin_FPGA_CV](https://github.com/alebal123bal/Gowin_FPGA_CV) | OV5640 DVP、RGB565 帧同步、DDR3 多帧缓存、显示和调试波形 | 同为 Gowin＋OV5640＋DDR3，适合对照帧缓存与时序；目标为 Tang Primer 20K，输出 HDMI/USB，不是本车 60K＋TFT 的现成工程。本地 `ch45_ov5640_ddr3_tft` 仍是 TFT 移植的直接基线。 |
| [LittleDang/2DLIW-SLAM](https://github.com/LittleDang/2DLIW-SLAM) | 2D 雷达＋IMU＋轮速的融合前端、外参和回环案例 | 传感器组合最接近本车。ROS 1、Ceres 和 OpenCV 依赖较重；适合作算法/数据流参考，不适合整体放入软核或 RTL。仓库作者也指出外参误差影响地图。 |
| [AndreaCensi/csm](https://github.com/AndreaCensi/csm) | `sm/` 中的 2D 扫描匹配实现与手册 | 用于离线验证扫描匹配能否从所选雷达数据产出可信的相对运动；C/GSL 依赖和迭代计算量需先测，再决定软核简化或硬件加速。相邻扫描匹配不等于全局定位。 |
| [simondlevy/TinyEKF](https://github.com/simondlevy/TinyEKF) | 小型 C/C++ EKF 和 Python 原型 | 建立四状态或六状态软件黄金模型、记录回放与 RTL 数值对照；实现使用浮点，不能直接综合成 Gowin 定点 RTL。 |
| [sfox14/pynq-ekf](https://github.com/sfox14/pynq-ekf) | `build/src/` 和 `ekf/`：EKF 的硬件/软件分工、状态维度与观测维度参数化 | 适合比较“矩阵计算放硬件、模型函数放软件”和全硬件两种分工。基于 Xilinx PYNQ/HLS/ARM、旧版 SDSoC，位流和构建流程不能用于 Gowin 60K。 |

## 按需要再看

| 项目 | 使用条件 | 借鉴点 |
|---|---|---|
| [cra-ros-pkg/robot_localization](https://github.com/cra-ros-pkg/robot_localization) | 设计多速率、延迟和协方差处理时 | EKF 输入语义、时间与协方差接口；作为软件行为对照，不移植整个 ROS 包到软核。 |
| [love-sabers/FPGA_ISP](https://github.com/love-sabers/FPGA_ISP) | OV5640 实时显示完成后，决定增加图像流水处理时 | Gowin 上相机→ISP→DDR→显示的模块划分。目标板为 138K，主要处理 RAW→RGB；本车 RGB565 彩色路标提取不需要先实现完整 ISP。 |
| [Slamtec/rplidar_sdk](https://github.com/Slamtec/rplidar_sdk) | 仅当雷达选用 Slamtec RPLIDAR 系列时 | 设备命令、健康状态、扫描模式与主机侧黄金数据；C++ SDK 本身不直接进入 FPGA RTL。换其他品牌应找对应厂商协议/SDK。 |

## 建议阅读与实现顺序

1. **先看本地基线与 `fpga_roomba`：** 明确雷达输入的时间/有效性、近距保护路径；本车已有四轮控制，不从其他机器人复制轮序或电机驱动。
2. **再看 `Gowin_FPGA_CV`：** 对照本地 OV5640→DDR3→TFT 参考工程，记录帧边界、跨时钟域和缓存策略。
3. **用 `TinyEKF`＋`2DLIW-SLAM` 建离线数据链：** 明确轮速、IMU、雷达扫描匹配输出的坐标、时间、协方差；用真实记录回放验证模型。
4. **用 `csm` 作雷达前端对照：** 测匹配成功率、耗时和走廊/空旷场景退化；若软核预算不足，再挑热点加速。
5. **最后参考 `pynq-ekf` 做硬件划分：** 先定点模型、后 RTL，再做 60K 整机资源和 STA。没有仓库提供可直接移植的四源 Gowin 60K EKF 整机。

注意 GitHub 工程的 README 演示、独立位流与资源数字只对应其各自硬件。若要直接复用源码，逐仓库核对许可；例如 `Gowin_FPGA_CV` 的 GitHub 元数据未显示许可证。

