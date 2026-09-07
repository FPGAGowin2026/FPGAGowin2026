# 选题三机器人系统（Tang Primer 25K）

本工程面向 2026 全国大学生嵌入式芯片与系统设计竞赛 FPGA 赛道选题三，目标器件固定为 `GW5A-LV25MG121NC1/I0`（`GW5A-25B`，Device B）。工程同时提供完整外设版和无传感器片上演示版。

## 工程配置

| Profile | 顶层 | 用途 | 产物 |
|---|---|---|---|
| Release | `choice3_robot_25k_top` | OV5640、JY61P、TF-Luna、4 路 IR、4 路 AB 编码器、四路 TB6612、DVI 显示 | `impl/pnr/choice3_robot_25k.fs` |
| Demo | `choice3_robot_25k_demo_top` | 仅板载 50 MHz、复位和 USB-UART；片上生成传感器/电机闭环场景 | `impl/pnr/choice3_robot_25k_demo.fs` |

Demo 不是绕过控制链的脚本：它把真实控制器产生的内部 PWM/方向反馈给可综合电机与 AB 编码器模型，并注入有效距离、慢行区、硬障碍和姿态数据。为防止未接电机时误动作，Demo 顶层对所有物理电机 `PWM/IN1/IN2/STBY` 无条件输出 0。

## 数据与控制链

```text
OV5640 DVP -> RGB565 捕获 -> 160x120 RGB444 双缓冲 -> 640x480 DVI
JY61P UART -> 校验/角度换算 -> R/P/Y 数字 OSD -----------^
TF-Luna + IR -> 新鲜度/质量门控 -> 距离融合 -> 慢行/停车/绕障
AB 编码器 -> 4x 解码 -> 速度估计 -> 四轮 PI -> TB6612 死区驱动
                                   ^
PicoRV32 UART/Wishbone -> 命令/状态/心跳看门狗
```

安全执行链保持在硬件中。PicoRV32 只下发武装、解除、路线开始、故障清除和手动动作请求，不能直接写 PWM 或 STBY。物理急停、传感器新鲜度、编码器堵转、CPU 心跳超时和方向切换死区均可独立关闭执行输出；健康恢复后不会自动重新武装。

## 主要功能

- OV5640 SCCB 身份/初始化、320x240 RGB565 DVP 捕获，隔点降采样后用双时钟双缓冲显示。
- 640x480 DVI 输出，画面叠加有符号十进制度数 `ROLL/PITCH/YAW` 和故障状态。
- JY61P 角度帧校验；TF-Luna 距离、强度、范围和超时质量门控；4 路红外去抖。
- 前进、后退、左右弧转、左右原地转向；四轮编码器速度闭环。
- 固定两航段路线、里程航点、点转向；障碍慢行、硬停、后退和绕行后恢复当前航段。
- PicoRV32 RV32IMC，8 KiB ITCM + 4 KiB DTCM，115200 8N1 UART 和开放 Wishbone 服务寄存器。
- `reserve_io[3:0]` 作为 4 根物理扩展脚保持高阻，软核侧保留开放 Wishbone 服务页。它们目前只是安全的接口预留，不是已经实现的外部 SPI/UART 协议；扩展模块必须经过命令仲裁、安全门控和超时保护，不能直连 PWM/STBY。
- 需要 D435i 时应由外部 USB 3 主机提取深度结果，再经 UART/SPI/以太网桥送入 FPGA，不可把 D435i 直接接普通 GPIO。

## UART 命令

| 命令 | 作用 |
|---|---|
| `P` / `I` | 连通性、系统 ID 和目标器件 |
| `S` / `V` | 完整状态、片上演示覆盖位 |
| `X` | 当前健康正常时清除历史故障 |
| `A` / `D` | 武装 / 解除武装 |
| `G` | 启动固定航点路线 |
| `0` | 手动停止 |
| `F` / `B` | 前进 / 后退 |
| `L` / `R` | 左 / 右弧转 |
| `Q` / `E` | 左 / 右原地转向 |
| `?` | 帮助 |

Wishbone 地址页：`0x2000_0000` 系统与心跳，`0x2000_1000` 状态快照，`0x2000_2000` 命令，`0x2000_F000` scratch/总线计数；当前 ABI 为 2（系统页返回 `0x0002_0000`，UART `I` 命令返回 `ABI=2`）。未定义或未对齐访问也会有限期 ACK 并返回 `0xDEADBEEF`，避免软核永久挂起。

## 构建和验证

```powershell
# RTL、自检 testbench、固件生成与 RV32I 启动解释执行
pwsh -File .\Scripts\run_sim.ps1

# 完整版和无外设版：综合、PnR、STA、位流及资源门禁
pwsh -File .\Scripts\run_gowin_profiles.ps1 -Profile All

# 只把安全 Demo 临时下载到 SRAM
pwsh -File .\Scripts\program_demo_sram.ps1 -CableIndex 4

# USB-UART 实板整机场景
pwsh -File .\Scripts\serial_demo_test.ps1 -Port COM5
```

当前验证结果（2026-08-29）：

| 项目 | 结果 |
|---|---|
| RTL/固件回归 | `sim` 目录 6/6 个 RTL testbench + 固件启动解释执行全部 PASS；脚本强制清单完整且每项唯一 PASS |
| 固件容量/ABI | 1644 B / 8 KiB ITCM，ABI 2，PASS |
| Release PnR | Logic 9645 / BSRAM 46 / DSP 16，setup/hold 0/0；`u_attitude_osd` 保留为 4 Register / 274 LUT |
| Demo PnR | Logic 8390 / BSRAM 8 / DSP 12，setup/hold 0/0 |
| Release SHA-256 | `00e625ce3c4801c9aece1d34efb6ad429e1b89c713058805f4d68e34a0fb3b4e` |
| Demo SHA-256 | `dd16de46e2a30d428ff2626e161a5e5248966bf833f7f25def39158d81a8a9b8` |
| SRAM 下载 | GW5A-25B ID `0x0001281B`，PASS |
| UART 实板终态 | `ABI=2`，`DEMO=0x000000FF`，`STATUS0=0xFF006F00`，`PHYSICAL_DRIVE=0` |

证据位于 `reports/simulation.log`、`reports/*.gowin.log`、`reports/board_program_demo.log` 和 `reports/board_uart_demo.log`。Programmer 原始日志记录了位流哈希、器件扫描 ID、User Code、Status Code 和 `Finished.`。

运行 `Scripts/write_evidence_manifest.ps1` 会生成 `reports/evidence_manifest.json`，用 SHA-256 绑定当前 RTL/TB/GPRJ/约束/脚本/固件 HEX 与保留的仿真、综合、PnR、位流、Programmer 和 UART 证据。

RTL 归属见 `src/INVENTORY.md`：GPRJ 的显式文件清单是生产配置；另有 4 个仅供 testbench 使用的 UART/guard 模块，以及 4 个未实例化的 HC-SR04、ICM42688P、VL53L0X/倾角参考模块，不能据其存在宣称已集成。

## 接线与边界

完整引脚见 `constraints/choice3_robot_25k.cst`。其中板载时钟 E2、复位 H11、USB-UART RX/TX 为 B3/C3；其余接口按 Core/Dock 的 PMOD/J3 规划。上板前必须再次对照所用 Dock 版本、原理图、Bank 电压与模组线序。

- I2C/SCCB 需要与 VCCIO 匹配的外部上拉；FPGA 逻辑电平统一按 3.3 V 设计。
- 传感器供电电压不等于 I/O 电压；5 V Echo/串口必须先做分压或电平转换。
- `MM_PER_TICK_Q16`、轮径/轮距、左右轮极性、PI 参数、慢行/停车阈值必须在真实底盘上标定。
- 当前实板 PASS 只验证无传感器片上激励、PicoRV32、UART 和 FPGA 下载链；Demo 顶层不导出 DVI、传感器或电机端口，不能用它证明 OV5640、JY61P、TF-Luna、DVI 或真实电机联调成功。
- Release SDC 约束了系统时钟、相机 PCLK 与异步时钟组，但没有最终模组/PCB 的 `set_input_delay`；当前 setup/hold 0/0 仅对已声明路径成立，最差 setup 余量约 +0.103 ns。必须按实际 OV5640 模组、连接器和走线补齐输入延时后重新 STA。
- 当前“航点导航”是固定的两航段里程 FSM（LEG1 → 原地转向 → LEG2），不是坐标定位、任意航点表或全局路径规划。
- Demo 因顶层主动隐藏物理视频/传感器接口，综合日志会显示这些无外部消费者的模块被裁剪；完整 Release 的 OSD 层级和资源已单独门禁，不存在 OSD 被扫除。
