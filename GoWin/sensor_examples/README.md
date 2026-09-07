# GW5A-LV25MG121 传感器独立例程集

本目录服务于《2026 FPGA 赛道选题指南》选题三“基于 FPGA 的具身智能机器人实时感知与自主控制系统设计”。它覆盖本次项目清单中适合接入 FPGA 的摄像头、IMU、编码器、测距、近接、环境、电源与定位模块；这里的“29 项”指冻结后的比赛相关工程集合，不代表穷举市场上的全部传感器。

每个目录都是可独立打开、仿真、综合、布局布线和生成位流的 Gowin 工程：

- Device：`GW5A-25B`
- Part number：`GW5A-LV25MG121NC1/I0`
- Device database：`gw5a25b-003`（Device B）
- 顶层：`sensor_example_top`
- 生产 RTL：Verilog-2001；自检 testbench：SystemVerilog
- 单工程顶层 I/O：不超过 MG121 的 69 个普通 I/O

独立例程没有板级 CST。其 `.fs` 是离线可实现性证据，不可直接下载到开发板；实板工程位于 `fpga_rtl/boards`，使用 Tang Primer 25K 的真实引脚。

## 1. 29 个工程

| # | 工程 | 传感器/模块 | 接口 | 已实现与验证的边界 |
|---:|---|---|---|---|
| 1 | `digital_ir_proximity` | 数字红外近接 | GPIO | 两级同步、独立置位/释放滤波、变化脉冲 |
| 2 | `gnss_nmea_uart` | 通用 NMEA GNSS | UART 9600 | `$...*HH` 校验、GGA/RMC 分类；经纬度数值换算留给 CPU |
| 3 | `gy39_uart` | GY-39 v3 | UART 9600 | `5A 5A TYPE LEN` 帧、校验和、照度/BME 数据解析 |
| 4 | `hc_sr04` | HC-SR04 | Trigger/Echo | 触发、回波计时、毫米换算、无回波超时 |
| 5 | `icm20602_spi` | ICM-20602 | SPI Mode 0 | WHO_AM_I、初始化、数据就绪、六轴突发读取、故障恢复 |
| 6 | `icm42688p_spi` | ICM-42688-P | SPI Mode 0 | Bank 0 初始化、45 ms 启动等待、六轴原子发布、无效样本拒绝 |
| 7 | `imu660ra_spi` | IMU660RA/BMI270 | SPI Mode 0 | 身份与原始六轴传输；明确输出 `configuration_required=1`，未伪造 BMI270 的 8192 字节配置镜像 |
| 8 | `imu660rx_spi` | IMU660RX（RA/RB 变体） | SPI | 变体探测、原始帧与错误恢复；BMI270 变体仍需外部加载配置镜像 |
| 9 | `imu963ra_spi` | IMU963RA | SPI Mode 0 | 加速度计/陀螺仪身份、初始化和六轴读取；磁力计 sensor-hub 仍需板级配置 |
| 10 | `ina219_i2c` | INA219 | 开漏 I²C | 配置读回、分流/总线电压、单位换算、NACK/拉伸超时 |
| 11 | `itr9606_pulse_speed` | ITR9606 槽型光电 | 数字脉冲 | 消抖、窗口脉冲数、最近周期测量 |
| 12 | `jy61p_uart` | JY61P | UART | `55 53` 姿态帧、校验、Roll/Pitch/Yaw 原始量 |
| 13 | `lis3mdl_spi` | LIS3MDL | SPI Mode 3 | WHO_AM_I、连续转换、数据就绪、XYZ 小端读取 |
| 14 | `maxsonar_uart` | MaxBotix MaxSonar | UART 9600 | `R`+十进制距离解析；单位由具体型号决定，故输出为 `range_raw` |
| 15 | `mechanical_bumper` | 左右碰撞开关 | GPIO | 去抖、事件脉冲、锁存与清除 |
| 16 | `menc15a_spi` | MENC15A | SPI Mode 1 | 命令/响应采集、断线全 1 拒绝、恢复路径 |
| 17 | `mpu6050_i2c` | MPU-6050 | 开漏 I²C | WHO_AM_I、显式六轴唤醒、默认 30 ms 稳定等待、7 项配置、14 字节加速度/温度/角速度原子读取 |
| 18 | `mt6816_spi` | MT6816 磁编码器 | SPI Mode 3 | 角度读取、偶校验、断线/坏帧拒绝 |
| 19 | `mt9v03x_dvp` | MT9V03X/MT9V034 模组 | 8-bit DVP + UART 配置 | 灰度像素、帧/行门控、坐标与 4 字节配置命令 |
| 20 | `multi_ir_array` | 8 路数字红外阵列 | GPIO×8 | 通道独立同步/去抖、状态与变化位图；active-high/low 复位均不伪触发 |
| 21 | `ov5640_dvp` | OV5640 | 8-bit DVP + SCCB | XCLK/上下电、ID、独立黄金表验证的 QVGA RGB565 初始化、SCCB Don’t-Care 第九位、PCLK 域复位同步、低字节优先组帧 |
| 22 | `ov7725_dvp` | OV7725 | 8-bit DVP + SCCB | RGB565 捕获与 SCCB 单寄存器写；完整上电表由调用者提供 |
| 23 | `quadrature_encoder` | AB/ABZ 类增量编码器 | A/B GPIO | 两级同步、四倍频、方向、非法两位跳变检测 |
| 24 | `scc8660_dvp` | SCC8660 模组 | 8-bit DVP + UART 配置 | RGB565 捕获、坐标、UART 配置命令发送 |
| 25 | `tcrt5000` | 带比较器 TCRT5000 模块 | 数字 GPIO | 饱和积分滤波、滞回、上升/下降事件 |
| 26 | `tf_luna_uart` | Benewake TF-Luna | UART 115200 | 9 字节距离帧、校验、连续帧与截断超时恢复 |
| 27 | `tsl1401_adc_frontend` | TSL1401/TSL1401CL | SI/CLK + 外部 ADC | 上电首轮清洗、128 个样本、强制第 129 时钟、20 us 电荷转移保护、ADC 超时后补钟复位 |
| 28 | `vl53l0x_i2c` | VL53L0X | 开漏 I²C | 型号、stop variable、SPAD、独立黄金表验证的完整调参、VHV/phase 校准、原始 RangeStatus 与连续测距 |
| 29 | `vl53l1x_i2c` | VL53L1X | 开漏 I²C | 软复位、Boot 轮询、ULD 配置、首次测量收尾、质量门控与连续测距 |

各项目固定结构：

```text
<project>/
├─ sensor_example.gprj
├─ src/
├─ sim/
├─ constraints/timing.sdc
├─ reports/{simulation.log,gowin.log}
└─ impl/{gwsynthesis,pnr}
```

部分功能完整核心端口较多。为满足 MG121 封装限制，生产顶层用 `selector + debug_data/status` 导出观测量；testbench 直接实例化完整 core，因此协议字段、像素、坐标和错误输出的验证没有被窄接口裁剪。

## 2. 一键验证

要求 PowerShell 7.3+：

```powershell
pwsh -File .\scripts\run_all_sim.ps1
pwsh -File .\scripts\run_all_gowin.ps1
pwsh -File .\scripts\audit_results.ps1
```

本机默认使用：

- Icarus Verilog：`C:\Users\32035\AppData\Local\Programs\IcarusVerilog\bin`
- Gowin Shell：`D:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe`

`audit_results.ps1` 对 29 项逐一检查：工程清单、精确器件三元组、唯一 `PASS:` 仿真终态、Gowin 综合/PnR/位流成功标志、报告和位流存在、I/O≤69、时钟约束非空，以及 setup/hold 违例为 0。最新资源明细见 [RESULTS.md](RESULTS.md)。

## 3. 验证结论的分层

必须严格区分以下三层：

1. **自检仿真通过**：证明测试平台覆盖的正常帧、坏帧、超时、重试和边界行为符合 RTL 预期。
2. **Gowin 离线实现通过**：证明该独立工程能在指定 25K 器件完成综合、布局布线、内部 STA 和位流生成。
3. **传感器实物通过**：必须有真实模块、正确供电/电平/上拉/线序，并在板上采到符合物理场景的数据。前两层不能替代第三层。

本目录保存前两层证据。Tang Primer 25K 的 UART 实板诊断与最终机器人系统工程单独存放，并在总报告中逐项标明“真实外设”“无设备故障路径”或“片上激励”是哪一种，绝不把未接传感器说成实物验证成功。

## 集成、运行时与审计风险清单（9 项）

1. **审计范围仅为 29 个一级传感器工程。** 本目录递归共有 114 个 `.gprj`，其中 29 个是一级目录中的 `sensor_example.gprj`，另外 85 个属于 `acg720_tutorial` 教程、规范化示例和 BIST 工程。本文及三个批处理脚本的“29/29 PASS”只适用于冻结的 29 个一级传感器工程，不能外推为递归 114 个 Gowin 工程全部通过。
2. **示例顶层不能直接多工程合并。** 29 个 `src/top.v` 都定义 `sensor_example_top`；系统集成时只能选取所需核心 RTL 和公共总线 RTL，或重命名/重写系统顶层，不能把多个示例 `top.v` 同时加入同一工程。
3. **UART 数据需要统一新鲜度门控。** GNSS、GY39、JY61P、MaxSonar 和 TF-Luna 等解析器主要验证帧格式与校验，系统仍需增加最后有效帧时间、失联超时、连续坏帧计数和 `stale/fault` 状态，陈旧数据不得继续驱动导航或安全决策。
4. **I²C 运行期错误状态需要统一。** INA219、VL53L0X、VL53L1X 等模块的部分运行期 NACK/拉伸超时路径会置位粘滞 `bus_error`，但未必同时撤销 `initialized/sensor_ok`。系统层必须以数据新鲜度和总线错误联合判定有效性，后续优化应增加连续错误阈值、自动重新初始化和明确的健康状态机。
5. **SPI 传感器需要运行期健康检查。** ICM42688P、LIS3MDL 等驱动主要在启动阶段检查身份；正式系统应增加周期性 WHO/config 回读、全零/全一及器件无效码拒绝、连续错误计数和自动重启，避免断线数据被当作有效样本。
6. **多颗 VL53 需要地址管理。** 现有 VL53L0X/VL53L1X 例程使用默认 `0x29` 地址，未实现 XSHUT 上电编排和重新寻址；多传感器系统必须使用独立 I²C 总线，或补充 XSHUT 时序及地址配置。
7. **公共 I²C 主机不是完整容错控制器。** 当前实现支持开漏、ACK 和 clock-stretch 超时，但不支持多主仲裁，也没有 SDA 卡死时的 9 脉冲总线恢复。机器人上电和热插拔场景应在板级总线管理器补充恢复与重新枚举策略。
8. **DVP 约束仍需最终 PCB 数据。** 独立工程约束了系统时钟和相机 PCLK，但没有依据最终模组、连接器和走线填写 `set_input_delay/set_output_delay`。完成 CDC、异步 FIFO 和板外时序约束后必须重新 STA 与上板验证。
9. **离线证据不等于实物证据，且现有审计未做内容哈希绑定。** `audit_results.ps1` 检查当前日志、报告、位流、器件和时序门禁，但尚未用源码/TB/约束哈希把输入与产物做不可混淆绑定；同时独立工程没有板级 CST、下载记录和真实传感器数据。因此 29/29 只能表述为仿真及离线实现通过，不能表述为传感器实物验证通过。

## 4. 上板前必须处理

1. 根据 Tang Primer 25K Core/Dock 原理图补 CST、Bank 电压、I/O 标准和板外输入/输出延迟；独立例程位流不可直接下载。
2. I²C/SCCB 必须使用与 FPGA I/O 电压一致的外部上拉。普通 I²C 检查 ACK；OV5640 SCCB 按规范把第九位视为 Don’t-Care，但仍检查总线空闲与 SCL 超时。
3. HC-SR04 常见 Echo 为 5 V，必须分压或电平转换；所有 SPI/UART/DVP 模块也要核对 VDDIO，供电电压不等于逻辑电平。
4. TSL1401 的 AO 是模拟电压，绝不能直接接普通 FPGA GPIO；必须使用外部 ADC/模拟前端。默认 27 MHz 参数强制第 129 个终止时钟与至少 540 周期（20 us）的电荷转移保护。
5. 摄像头 PCLK 是独立时钟域。OV5640 独立工程已按规格上限 96 MHz 约束该域并同步释放采集复位，但最终帧缓存、跨域 FIFO、显示链路，以及依据具体模组/走线实测值填写的 DVP `set_input_delay` 仍属于板级系统工程。
6. BMI270 系模块必须加载 8192 字节配置镜像；IMU963RA 的外置磁力计 sensor-hub 也需独立配置。例程显式报告 `configuration_required`，不会用假状态冒充初始化完成。
7. 距离偏差、IMU 零偏、磁力计硬铁/软铁、编码器每转计数、轮径/轮距、超声波温度补偿和安全阈值均需实车标定。

VL53L0X 的 `distance_valid` 是单周期“新结果就绪”脉冲：只保证该轮数据就绪、距离读取和中断清除成功，不代表目标质量或距离精度一定合格。`range_status_raw` 原样导出寄存器 `0x14`，上层若要形成可用于安全决策的质量状态，仍需按所采用的 ST API 版本完成状态映射与现场阈值验证。

## 5. OV5640 与 D435i 的边界

`ov5640_dvp` 已实现 XCLK、硬件上下电、SCCB ID/重试、最小 QVGA RGB565 表和 DVP 捕获。寄存器 `0x4300=0x6F` 产生低字节先的 RGB565，总线捕获会还原成标准 16 位像素；`0x4740=0x23` 与采集逻辑统一为 PCLK/HREF/VSYNC 高有效。它仍不包含异步帧 FIFO、DDR 帧缓存、HDMI 或图像算法。

Intel RealSense D435i 是 USB 3 深度相机，不是 DVP/SPI/I²C 传感器。普通 FPGA GPIO 无法直连读取；若用于比赛，推荐把它接 Jetson/RK3588/PC，由 RealSense SDK 输出降维后的深度/障碍物结果，再通过以太网、UART 或 SPI 送入 FPGA。让 FPGA 直接接 D435i 意味着另做 USB 3 Host、PHY、枚举/UVC/RealSense 流和大容量缓存子系统，不属于这些传感器寄存器例程。

## 6. 主要一手依据

- ams OSRAM《TSL1401CL DS000136》：129 时钟、20 us `tqt`、首轮不确定数据清洗。
- OmniVision《OV5640 Product Specification v2.03》：SCCB、`0x4300` RGB565、`0x4740` DVP 极性。
- ST/Pololu VL53L0X 与 ST VL53L1X ULD 参考流程。
- TDK ICM-42688-P、InvenSense MPU-6000/6050、ST LIS3MDL、TI INA219 原厂数据手册。
- 各模组协议手册及仓库中已有的经过核对的驱动资料；无法确定协议的“USART6 五字节激光”“IMU660RB 非公开差异”“DM1XA 未知编码器”没有被伪造为可用例程。

资源、工具版本与每项构建证据见 [RESULTS.md](RESULTS.md)。
