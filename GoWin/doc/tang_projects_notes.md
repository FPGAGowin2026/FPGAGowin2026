**高云(Gowin) Sipeed Tang 系列开发板汇总**

**与 Tang Mega 60K 开源项目整理**

*整理时间：2025年7月*

**目录**

*右键目录，选择"更新域"刷新页码。*

高云(Gowin) Sipeed Tang 系列开发板汇总与 Tang Mega 60K 开源项目整理 3

> 一、高云 × Sipeed Tang 系列开发板全型号 4
>
> 1.1 Tang Nano 系列（入门级 / 小尺寸） 5
>
> 1.2 Tang Primer 系列（中阶 / 扩展型） 6
>
> 1.3 Tang Mega 系列（高性能 / 高速接口） 7
>
> 1.4 Tang Console（复古游戏主机底座） 8
>
> 1.5 第三方二次开发板 9
>
> 二、Tang Mega 60K 开发板详解 10
>
> 2.1 核心板（SOM）参数 11
>
> 2.2 底板（NEO Dock）外设 12
>
> 2.3 开发环境要求 13
>
> 三、Tang Mega 60K 开源项目与例程汇总 14
>
> 3.1 Sipeed 官方例程（GitHub） 15
>
> 3.2 TangCore — 复古游戏 FPGA 核（强烈推荐） 16
>
> 3.3 MiSTle-Dev 系列 — 经典电脑/游戏机 FPGA 核 17
>
> 3.4 DDR3 帧缓冲 / 视频处理 18
>
> 3.5 第三方综合例程库 19
>
> 3.6 其他社区项目与适配方向 20
>
> 四、Tang Mega 60K 适合的项目方向推荐 21
>
> 4.1 复古游戏与娱乐 22
>
> 4.2 高速图像/视频处理 23
>
> 4.3 高速通信与协议转换 24
>
> 4.4 嵌入式系统与软核 25
>
> 4.5 音频处理 26
>
> 4.6 教育与竞赛 27
>
> 五、资源链接汇总 28
>
> 六、IDE 芯片选型速查 29

**高云(Gowin) Sipeed Tang 系列开发板汇总与 Tang Mega 60K 开源项目整理**

> 整理时间：2025年7月

**一、高云 × Sipeed Tang 系列开发板全型号**

Sipeed（矽速科技）与高云半导体（Gowin Semiconductor）合作推出的 **Tang 系列** FPGA 开发板，是目前国产 FPGA 生态中最活跃、社区资源最丰富的开发板家族之一。以下是目前在售/已发售的全部型号汇总。

**1.1 Tang Nano 系列（入门级 / 小尺寸）**

|                   |                       |        |             |                                                       |
|-------------------|-----------------------|--------|-------------|-------------------------------------------------------|
| 板卡              | FPGA 芯片             | 系列   | 逻辑单元    | 核心特点                                              |
| **Tang Nano 1K**  | GW1NZ-LV1QN48C6/I5    | GW1NZ  | 1,152 LUT4  | 最小尺寸，超低价入门，RGB 屏幕接口                    |
| **Tang Nano 4K**  | GW1NSR-LV4CQN48PC6/I5 | GW1NSR | 4,608 LUT4  | 集成 Cortex-M3 硬核 + 64Mbit PSRAM，HDMI + DVP 摄像头 |
| **Tang Nano 9K**  | GW1NR-LV9QN88PC6/I5   | GW1NR  | 8,640 LUT4  | HDMI + RGB + SPI 屏幕三接口，TF 卡槽，USB-UART        |
| **Tang Nano 20K** | GW2AR-LV18QN88C8/I7   | GW2AR  | 18,000 LUT4 | 更高逻辑容量，板载 SDRAM，适合复古游戏核              |

**1.2 Tang Primer 系列（中阶 / 扩展型）**

|                     |                      |       |             |                                          |
|---------------------|----------------------|-------|-------------|------------------------------------------|
| 板卡                | FPGA 芯片            | 系列  | 逻辑单元    | 核心特点                                 |
| **Tang Primer 20K** | GW2A-LV18PG256C8/I7  | GW2A  | 18,000 LUT4 | 更多 IO 引出，曾用于全国大学生 FPGA 大赛 |
| **Tang Primer 15K** | GW5AT-LV15MG132C1/I0 | GW5AT | 15,000 LUT4 | 22nm 新架构，MIPI D-PHY 硬核             |
| **Tang Primer 25K** | GW5A-LV25MG121NC1/I0 | GW5A  | 25,000 LUT4 | 支持 NESTang / SNESTang 游戏核           |

**1.3 Tang Mega 系列（高性能 / 高速接口）**

|                        |                          |        |                  |                                                                                  |
|------------------------|--------------------------|--------|------------------|----------------------------------------------------------------------------------|
| 板卡                   | FPGA 芯片                | 系列   | 逻辑单元         | 核心特点                                                                         |
| **Tang Mega 60K**      | GW5AT-LV60PG484AC1/I0    | GW5AT  | **59,904 LUT4**  | 4 路高速收发器 (8.8Gbps)，PCIe 2.0 硬核，DDR3 512MiB，MIPI D-PHY 收发器          |
| **Tang Mega 138K**     | GW5AST-LV138PG484AC1/I0  | GW5AST | **138,240 LUT4** | 8 路高速收发器 (12.5Gbps)，PCIe 3.0 硬核，RISC-V AE350 硬核 @800MHz，DDR3 512MiB |
| **Tang Mega 138K Pro** | GW5AST-LV138FPG676AC1/I0 | GW5AST | 138,240 LUT4     | 更大封装 FPG676，双 HDMI，SFP+ 万兆网口，M.2 接口                                |

**1.4 Tang Console（复古游戏主机底座）**

|                       |                    |                                                |
|-----------------------|--------------------|------------------------------------------------|
| 板卡                  | 核心 SOM           | 说明                                           |
| **Tang Console 60K**  | Tang Mega 60K SOM  | 紧凑复古游戏主机底板，支持 TangCore 全部游戏核 |
| **Tang Console 138K** | Tang Mega 138K SOM | 同上，性能更强                                 |

> **Tang Mega 60K 与 138K 共用同一套底板（TANG MEGA NEO Dock）**，外设完全相同。60K 的优势在于价格更低，且包含 **MIPI D-PHY 收发器**（138K 仅有 RX）。

**1.5 第三方二次开发板**

|                           |                        |                                                |
|---------------------------|------------------------|------------------------------------------------|
| 板卡                      | 来源                   | 说明                                           |
| GW020AB / GW020C / GW025A | BigPig-Bro（大猪蹄子） | 基于 Tang Primer 20K/25K 二次开发              |
| GW138\_060A               | BigPig-Bro             | 基于 Tang Mega 138K/60K 二次开发               |
| GW020AB                   | BigPig-Bro             | A 款原生 Sipeed 下载器，B 款独立 USB+标准 JTAG |

**二、Tang Mega 60K 开发板详解**

**2.1 核心板（SOM）参数**

|                      |                                          |
|----------------------|------------------------------------------|
| 项目                 | 参数                                     |
| FPGA 芯片            | GW5AT-LV60PG484AC1/I0                    |
| 逻辑单元 (LUT4)      | 59,904                                   |
| 寄存器 (FF)          | 59,904                                   |
| 分布式 SRAM (S-SRAM) | 468 Kbits                                |
| 块 SRAM (B-SRAM)     | 2,124 Kbits / 118 个                     |
| 乘法器 (18×18)       | 118 个                                   |
| PLL                  | 8 个                                     |
| 高速收发器 (SerDes)  | 4 个，270 Mbps \~ 8.8 Gbps               |
| PCIe 硬核            | 1 个，PCIe 2.0（当前仅支持 x1 模式）     |
| MIPI D-PHY           | 软核，RX 最高 1.6 Gbps，TX 最高 2.0 Gbps |
| DDR3                 | 4Gb (512 MiB) @ 1100 Mbps                |
| Flash                | 64 Mbits                                 |
| ADC                  | 2 通道                                   |
| IO Bank              | 11 个                                    |
| 封装尺寸             | 35 mm × 45 mm（BTB 连接器连接底板）      |

**2.2 底板（NEO Dock）外设**

|            |                                                     |
|------------|-----------------------------------------------------|
| 外设       | 说明                                                |
| LED        | 4 个电量指示灯 + 8 个 PMOD LED                      |
| 按键       | 3 个用户按键 + 1 个 Reconfig 按键                   |
| WS2812     | 1 个，与 aRGB 灯带接口共引脚                        |
| PCIe       | 4-lane 金手指（当前因 SerDes 通道互换，仅 x1 可用） |
| USB3.0     | 2 个 SuperSpeed @ 5Gbps（CH569 16bit HSPI）         |
| 千兆以太网 | 1 个 RJ45                                           |
| HDMI / DVI | 1 个，支持 TX 和 RX（互相占用）                     |
| PMOD       | 2 个标准 PMOD 接口                                  |
| DVP 摄像头 | 1 个，与 PMOD 和 40P 排针复用                       |
| RGB 屏幕   | 1 个，支持 RGB888                                   |
| SD 卡槽    | 1 个，1-bit SDIO/MMC 或 SPI 模式                    |
| 麦克风阵列 | 支持 Sipeed 6+1 麦克风阵列接口                      |
| 音频       | 3.5mm 耳机输出 + 2×3W 扬声器接口                    |
| ADC        | 2 个差分输入通道                                    |
| 电池       | 3.7V 锂电池充放电管理                               |
| 风扇       | 5V PWM 风扇接口，支持测速                           |
| 40P 排针   | 2 组 2×20P 排针，与 PMOD/DVP 复用                   |
| 电源       | 12V DC5521 或双 USB 供电                            |

**2.3 开发环境要求**

• **IDE**：Gowin 云源软件（教育版免费，商业版需 License）

• **最低版本**：教育版 ≥ V1.9.11.03，商业版 ≥ V1.9.10.03

• **芯片型号选择**：\`GW5AT-LV60PG484A\`，封装 \`PBG484A\`，Device Version: **B**

• **在线 License**：Sipeed 提供浮动 License 服务器

• IP: \`106.55.34.119\`，Port: \`10559\`

• 备选域名: \`gowinlic.sipeed.com\`

**三、Tang Mega 60K 开源项目与例程汇总**

**3.1 Sipeed 官方例程（GitHub）**

**仓库地址**：<https://github.com/sipeed/TangMega-60K-example>

|                     |                                            |          |
|---------------------|--------------------------------------------|----------|
| 例程                | 说明                                       | 难度     |
| **pmod\_led\_neo**  | 入门项目，控制 PMOD LED 和 WS2812 灯带     | ⭐ 入门   |
| **cam\_dvi**        | OV5640 MIPI/DVP 摄像头采集 → DVI/HDMI 显示 | ⭐⭐ 中级  |
| **ddr\_test**       | 板载 DDR3 内存读写测试                     | ⭐⭐ 中级  |
| **hdmi\_colorbar**  | HDMI 彩条输出测试                          | ⭐ 入门   |
| **key\_led**        | 按键控制 LED                               | ⭐ 入门   |
| **uart\_test**      | 串口通信测试                               | ⭐ 入门   |
| **eth\_test**       | 以太网相关测试                             | ⭐⭐⭐ 高级 |
| **pcie\_dma\_demo** | PCIe DMA 传输演示                          | ⭐⭐⭐ 高级 |
| **usb3\_demo**      | USB3.0 高速传输演示                        | ⭐⭐⭐ 高级 |
| **mipi\_rx\_tx**    | MIPI D-PHY 收发器测试                      | ⭐⭐⭐ 高级 |

> 注：官方仓库持续更新中，部分例程可能处于验证阶段。

**3.2 TangCore — 复古游戏 FPGA 核（强烈推荐）**

**核心作者**：nand2mario（社区大神） **主仓库**：<https://github.com/nand2mario/tangcore>

TangCore 是一套综合性的 FPGA 复古游戏发行版，将多个高质量游戏核心整合到统一菜单系统中，支持快速核心切换和 USB 存储。

|              |                                                 |            |
|--------------|-------------------------------------------------|------------|
| 游戏核心     | 说明                                            | 状态       |
| **NESTang**  | 任天堂 NES / FC 红白机，周期精确模拟            | ✅ 完全支持 |
| **SNESTang** | 超级任天堂 SNES / SFC，支持 LoROM/HiROM/ExHiROM | ✅ 完全支持 |
| **GBATang**  | 任天堂 GBA，支持全部 32MB ROM，开源 BIOS        | ✅ 完全支持 |
| **MDTang**   | 世嘉 Genesis / Mega Drive                       | ✅ 完全支持 |
| **SMSTang**  | 世嘉 Master System                              | ✅ 完全支持 |
| **486Tang**  | x86 486 模拟（实验性）                          | 🌟 新特性   |

**Tang Mega 60K 支持状态**：✅ **Fully supported（所有核心均可用）**

**相关独立仓库**：

• NESTang：<https://github.com/nand2mario/nestang>

• SNESTang：<https://github.com/nand2mario/snestang>

• GBATang：<https://github.com/nand2mario/gbatang>

• TangCore 固件：<https://github.com/nand2mario/firmware-bl616>

**特色功能**：

• 720p HDMI 输出

• 从 MicroSD / USB 加载 ROM

• RISC-V 软核菜单系统

• 支持 DS2 / SNES / USB 手柄

• 自动 BSRAM 存档备份与恢复

**3.3 MiSTle-Dev 系列 — 经典电脑/游戏机 FPGA 核**

MiSTle-Dev 团队将大量 MiST / MiSTer 经典核心移植到 Tang 系列开发板。

|                |                       |                                            |                         |
|----------------|-----------------------|--------------------------------------------|-------------------------|
| 项目           | 说明                  | 仓库                                       | 60K 支持                |
| **C64Nano**    | Commodore 64 电脑     | <https://github.com/MiSTle-Dev/C64Nano>    | ✅ HDMI/LCD, 双 DS2 手柄 |
| **VIC20Nano**  | Commodore VIC-20 电脑 | <https://github.com/MiSTle-Dev/VIC20Nano>  | ✅ HDMI/LCD, 双 DS2 手柄 |
| **NanoApple2** | Apple IIe 电脑        | <https://github.com/MiSTle-Dev/NanoApple2> | ✅ HDMI, USB 键盘/手柄   |
| **A2600Nano**  | Atari 2600 VCS 游戏机 | <https://github.com/MiSTle-Dev/A2600Nano>  | ✅ HDMI/LCD, 双 DS2 手柄 |

**通用特性**：

• HDMI 720p 视频输出 + 音频

• 支持 USB 键盘 / 鼠标 / 游戏手柄（需外接 µC 如 Sipeed M0S Dock）

• 支持 DualShock 2 手柄（通过 PMOD 接口）

• 从 SD 卡加载磁盘镜像 / ROM

• 部分核心支持 WiFi Modem（Console 60K / Mega 60K 底板）

**3.4 DDR3 帧缓冲 / 视频处理**

|                              |                                                |                                                        |
|------------------------------|------------------------------------------------|--------------------------------------------------------|
| 项目                         | 说明                                           | 仓库                                                   |
| **ddr3\_framebuffer\_gowin** | DDR3 高性能帧缓冲，支持多色深、动态缩放至 720p | <https://github.com/nand2mario/ddr3_framebuffer_gowin> |

• 已在 Tang Console 60K/138K 测试，**Tang Mega 60K 仅需极少修改即可运行**

• DDR3 运行频率 297 MHz，支持 640×480 帧缓冲 upscale 到 720p

• 资源消耗约 3,600 LUTs + 4,600 寄存器 + 16 BRAMs

**3.5 第三方综合例程库**

|                    |                        |                                   |                                       |
|--------------------|------------------------|-----------------------------------|---------------------------------------|
| 项目               | 作者                   | 说明                              | 仓库                                  |
| **Gowin 例程汇总** | BigPig-Bro（大猪蹄子） | 涵盖 Tang Nano/Primer/Mega 全系列 | <https://github.com/BigPig-Bro/Gowin> |

包含内容：

• Tang Nano 1K/4K/9K/20K 基础例程

• Tang Primer 20K/25K 例程

• Tang Mega 60K/138K 例程

• 大部分从 Cyclone IV 仓库移植适配

**3.6 其他社区项目与适配方向**

|                   |                             |                                                 |
|-------------------|-----------------------------|-------------------------------------------------|
| 方向              | 项目/资源                   | 说明                                            |
| **RISC-V 软核**   | Sipeed 官方 AE350 定制 Demo | 基于 Andes AE350 的 FPGA 集成 SoC               |
| **PCIe 加速**     | 自定义 PCIe 设备            | 利用 PCIe 2.0 硬核做高速数据采集/协议转换       |
| **MIPI 图像处理** | OV5640/IMX 系列             | 利用 MIPI D-PHY 收发器做高速相机采集            |
| **千兆网 UDP**    | udp\_rgmii\_send 例程       | 基于 RJ45 的千兆以太网 UDP 发送测试             |
| **10G 光口**      | SFP+ 万兆网                 | Tang Mega 138K Pro 支持，60K 受限于 SerDes 速率 |
| **USB2 软 PHY**   | usb2\_soft                  | 基于 IO 软 PHY 的 USB2 高速/全速设备            |
| **复古计算**      | 多平台 MiSTer 移植          | 社区持续将 MiSTer 核心移植到 Tang 系列          |

**四、Tang Mega 60K 适合的项目方向推荐**

基于其硬件特性，以下方向特别适合用 Tang Mega 60K 实现：

**4.1 复古游戏与娱乐**

• **最佳起点**：TangCore（NES/SNES/GBA/MD/SMS）

• **进阶选择**：MiSTle-Dev 系列（C64/VIC20/Apple II/Atari 2600）

• 所需配件：HDMI 显示器、USB 手柄/M0S Dock、SD 卡

**4.2 高速图像/视频处理**

• MIPI D-PHY 摄像头采集（OV5640 等）

• HDMI 输入/输出视频处理

• DDR3 帧缓冲 + 视频缩放/叠加

• 适合做人脸识别、边缘检测、图像增强等视觉算法硬件加速

**4.3 高速通信与协议转换**

• PCIe 2.0 x1 设备端开发（配合树莓派 5 等主机）

• USB3.0 5Gbps 高速数据传输

• 千兆以太网协议栈实现

• SerDes 自定义协议（8.8Gbps 内）

**4.4 嵌入式系统与软核**

• RISC-V 软核（VexRiscv、PicoRV32 等）运行 Linux

• 基于 AE350 硬核的嵌入式系统（138K 专属，60K 可用软核替代）

• 实时控制系统

**4.5 音频处理**

• 麦克风阵列信号处理（配合 6+1 麦克风阵列）

• 音频 DSP 算法（FFT、滤波、合成器）

• 立体声输出 + 耳机接口

**4.6 教育与竞赛**

• **2025 全国大学生 FPGA 创新设计竞赛（高云杯）** 指定用板之一

• 数字逻辑教学、计算机体系结构实验

• 信号处理、通信原理课程设计

**五、资源链接汇总**

|                                 |                                                                        |
|---------------------------------|------------------------------------------------------------------------|
| 资源                            | 链接                                                                   |
| **Tang Mega 60K 官方 Wiki**     | <https://wiki.sipeed.com/hardware/zh/tang/tang-mega-60k/mega-60k.html> |
| **Tang Mega 60K 官方例程**      | <https://github.com/sipeed/TangMega-60K-example>                       |
| **Tang Mega 138K 官方例程**     | <https://github.com/sipeed/TangMega-138K-example>                      |
| **Tang Mega 138K Pro 官方例程** | <https://github.com/sipeed/TangMega-138KPro-example>                   |
| **TangCore 复古游戏**           | <https://github.com/nand2mario/tangcore>                               |
| **MiSTle-Dev 项目集**           | <https://github.com/MiSTle-Dev>                                        |
| **BigPig-Bro 例程库**           | <https://github.com/BigPig-Bro/Gowin>                                  |
| **高云 IDE 下载**               | <http://www.gowinsemi.com.cn/faq.aspx>                                 |
| **Sipeed 论坛**                 | <https://maixhub.com/discussion>                                       |
| **Reddit 社区**                 | <https://reddit.com/r/GowinFPGA/>                                      |
| **QQ 交流群**                   | 834585530                                                              |
| **Gowin 大学计划**              | <https://www.gowinsemi.com.cn/university/>                             |

**六、IDE 芯片选型速查**

在 Gowin IDE 中选择器件时，Tang Mega 60K 必须按以下参数设置：

|                |                       |
|----------------|-----------------------|
| 参数           | 值                    |
| 芯片型号       | GW5AT-LV60PG484A      |
| 封装           | PBG484A               |
| Device Version | **B**                 |
| 完整名称       | GW5AT-LV60PG484AC1/I0 |

> ⚠️ 注意：2025 年 7 月后，138K 的芯片型号从 GW5AST-138B 变更为 GW5AST-138C，但 **60K 仍为 GW5AT-60B**，未变更。

\*本文档基于 Sipeed 官方 Wiki、GitHub 公开仓库及社区资源整理，持续更新中。\*
