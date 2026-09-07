# Tang Primer 25K 赛题三智能小车 FPGA 工程

本目录是面向全国大学生嵌入式芯片与系统设计竞赛 2026 FPGA 赛道选题三的可综合整机 RTL 工程，目标器件为 Tang Primer 25K Core + Dock 上的 `GW5A-LV25MG121NC1/I0` Device B。

工程已经完成 RTL 回归、顶层原语展开、Gowin 综合/布局布线/STA、SRAM 下载以及“外设未连接”条件下的 UART fail-safe 板级门禁。冻结位流 SHA-256 为：

```text
3c5a590f1bd14b4b8dca2a6910708e3286d3cbe2a88c9205ff4b50173978d957
```

详细证据见 [RESULTS.md](RESULTS.md)。当前证据不包含摄像头、DVI 显示器、IMU、测距器、红外、编码器、电机或真实路线的联合实测，也没有写入片上 Flash；不得把本工程表述为已经完成整车验收或功能安全认证。

## 1. 已实现功能

| 子系统 | 实现内容 | 关键边界 |
|---|---|---|
| 摄像头 | OV5640 SCCB 初始化、8-bit DVP QVGA RGB565 接收 | DVP 小端字节序；外部时序仍需实板测量 |
| 帧缓存 | 320×240 隔点降采样到 160×120；双缓冲 | RGB565 量化为 RGB444，两个 bank 共 460800 bit |
| 视频 | 160×120 以 4×最近邻显示为 640×480，约 59.9 Hz；TMDS/DVI 输出 | 只实现视频，无音频、EDID 或 HDCP |
| IMU | ICM-42688-P SPI 初始化和六轴样本；全 `0000`/全 `FFFF` 哨兵拒绝 | 仅首次身份检查，没有运行期 WHO_AM_I 重探测 |
| 姿态显示 | 加速度低通后生成倾角像素代理，在画面叠加绿色十字 | 不是角度，不是互补滤波，不提供 yaw |
| 测距 | VL53L0X I2C、HC-SR04、4 路数字 IR | VL53 原始状态没有转换成 ST PAL RangeStatus |
| 融合避障 | 双测距新鲜度、取较小距离、IR 独立阻挡；220 mm 停、420 mm 慢行 | 有效毫米距离要求 VL53 和超声均新鲜 |
| 里程/闭环 | 四路正交解码、独立轮速估计、四路 PI | 100 Hz；轮径、编码器 PPR、方向和 PI 均需实车标定 |
| 电机 | 两块 TB6612FNG、四路独立 20 kHz PWM/方向、100 us 换向死区、共用 STBY | 换向死区是短刹车，见安全说明 |
| 导航 | 两段直行加一次原地转向；遇障后退再绕行 | 最小演示 FSM，不是定位、SLAM 或通用航点导航 |
| 控制台 | UART 115200 8-N-1，武装、撤武装、路线、手动动作和状态 | 单字符协议，没有通信丢失动作看门狗 |
| 安全 | reset、active-low 急停、健康、新鲜度、故障锁存、显式重新武装 | 不是安全认证电路；必须增加板外硬件急停链路 |

## 2. 数据与控制架构

### 2.1 视频链

```text
OV5640 QVGA RGB565
  -> DVP 字节拼接
  -> 偶数行/偶数列抽取 160x120
  -> RGB565 高位量化 RGB444
  -> 2 x 19200 x 12 bit 双时钟双缓冲
  -> 640x480 4x 放大/故障条/姿态十字
  -> TMDS 编码 + OSER10 + ELVDS_OBUF
  -> Sipeed PMOD_DVI
```

RGB444 的转换为 `{RGB565[15:12], RGB565[10:7], RGB565[4:1]}`，显示时每个 4-bit 分量复制为 8-bit。它保留 4096 色，但会丢失 RGB565 的低位色彩精度。这一折中把双缓冲帧存储从 614400 bit 降为 460800 bit，并在 25K 器件上保留了可用 BSRAM 余量。

写域初始写 bank 1，显示域初始读 bank 0。完整且尺寸正确的输入帧只翻转 commit token；显示域只在输出帧边界切换 bank，再回传 ack token。跨域不传递 bank 旁带值。提交尚未确认时，新输入帧会整帧丢弃，避免覆盖正在等待显示的 bank；因此该设计以掉帧换取帧边界切换，不承诺不掉帧。RTL TB 已验证两帧交替、未确认时不覆写、短帧拒绝和首末像素，但尚未用真实相机/显示器证明无撕裂。

### 2.2 时钟

| 时钟 | 目标频率 | 用途 |
|---|---:|---|
| `system_clk` | 50.000 MHz | 传感器配置、UART、控制、安全 |
| `camera_pclk` | 最大约束 95.997 MHz | OV5640 DVP 捕获写域 |
| `pixel_clk` | 25.156 MHz | 640×480 像素域 |
| PLL `CLKOUT0` | 125.781 MHz | OSER10 5×串行时钟 |
| PLL `CLKOUT1` | 23.958 MHz | OV5640 XCLK |

`camera_pclk` 与 system/pixel 被声明为异步时钟组；双缓冲只通过 toggle/ack 同步器交换所有权。system 与 pixel 同源，仍由 STA 分析。当前 SDC 没有为 OV5640 的 `D[7:0]`、HREF、VSYNC 相对 PCLK 建立板级 `set_input_delay`；报告中的 camera PCLK Fmax 只证明 FPGA 内部路径，不能代替相机输出延迟、连接器/排线偏斜和 I/O setup/hold 校核。接入具体模组后必须按其数据手册和实测波形补齐 min/max input-delay 约束并重新 P&R。

### 2.3 传感器健康与新鲜度

50 MHz 域中的超时设置如下：摄像头 1.5 s、IMU 40 ms、VL53L0X 500 ms、HC-SR04 200 ms。融合器还以 100 Hz 控制 tick 对 VL53/超声样本保留最多 50 tick。仅“初始化成功”不足以保持健康，必须持续收到可接受样本。

VL53 距离只接受 30…4000 mm 且不为 `FFFF`；HC-SR04 只接受 20…4000 mm。VL53 的 `distance_valid` 仅代表结果就绪/读取成功，`VLRS` 只是芯片寄存器原始值，不是已经转换并验证的 ST PAL RangeStatus。ICM 样本守卫拒绝六轴全 0 和全 F，但无法识别所有总线卡死或物理异常。

### 2.4 电机与导航

四轮分为左侧 `[0:1]` 和右侧 `[2:3]`。轮速估计和 PI 都是四路独立实例，默认 `MM_PER_TICK_Q16=6554`，即约 0.1 mm/tick，只是起始占位值。PI 默认 `KP=8`、`KI=1`、`GAIN_SHIFT=4`。PWM 周期 2500 个 50 MHz 周期，即 20 kHz。

自动路线状态：`0 IDLE`、`1 LEG1`、`2 TURN`、`3 LEG2`、`4 BACK`、`5 AVOID_TURN`、`6 DONE`。默认里程阈值为 400/180/400 tick；后退和绕行分别为 20/35 个 100 Hz 样本，约 200/350 ms。这些数值没有经过实车标定。尤其是 BACK 没有专用后向测距，必须在受控空旷区域、低速和随时可断电条件下验证。

## 3. 安全语义

`HEALTH` 位定义：

| 位 | 含义 | 1 的条件 |
|---:|---|---|
| 7:5 | 保留 | 固定为 `111`，不代表三个额外传感器 |
| 4 | 编码器 | 四路已建立基线且没有锁存堵转 |
| 3 | HC-SR04 | 初始化条件成立且持续有新鲜有效样本 |
| 2 | VL53L0X | 初始化/总线状态正常且持续有新鲜有效样本 |
| 1 | ICM-42688-P | 初始化正常、姿态代理可用且持续有非哨兵样本 |
| 0 | OV5640 | 身份/总线正常且持续提交完整帧 |

`FAULT[4:0]` 与缺失的必需健康位逐位对应，`FAULT[7]` 表示急停；状态行显示的是当前故障与锁存故障的按位或。健康恢复不会自动重新武装，必须再次发送 `A`。`X` 只会在全部必需健康位正常且急停释放时清锁存故障。

顶层用 `reset_n AND estop_n AND motor_enable` 组合门控 PWM、IN1、IN2 和 STBY；急停不等待控制时钟。急停输入为 active-low，CST 使用下拉，未连接时默认禁止。建议在 TB6612 STBY 上增加板外下拉，并让真正的蘑菇头急停直接切断驱动器 STBY 或电机电源，不要只依赖 FPGA 逻辑。

换向死区期间每路 PWM/IN1/IN2 为 0，但共用 STBY 仍为高。按 TB6612 真值表，这属于 short-brake（短刹车），不是 coast（高阻滑行）；它避免逻辑方向瞬时反转，却可能产生机械冲击和再生电流。只有急停/复位/未武装使 STBY 为低时，驱动器才进入 standby 高阻。实车必须检查电源吸收能力、制动电流和热设计。

手动模式也受健康、急停和障碍门控。未武装时收到的动作不会缓存；健康故障撤武装后旧动作不会恢复。前进遇前障立即停，慢区降速；其他动作按侧向 IR 保守停止。但工程没有 UART 通信超时看门狗：已武装的手动动作会持续到收到 `0`/`D`、发生健康故障、触发急停或障碍门控。正式上车前应增加主机心跳/动作租约，或由外部 MCU/硬件看门狗负责超时停车。

## 4. 引脚规划

下表给出 FPGA 封装球位，不代表连接器物理针脚顺序。接线时必须再对照 Tang Primer 25K Core/Dock 和各模块原理图，禁止按表格行序直接插排线。全部用户 I/O 为 3.3 V LVCMOS。

### 4.1 板载与 PMOD0 DVI

| 功能 | 信号 | FPGA 引脚 |
|---|---|---|
| 50 MHz | `clk_50m_i` | E2 |
| 复位，高有效 | `reset_i` | H11 |
| UART RX/TX | `uart_rx_i` / `uart_tx_o` | B3 / C3 |
| TMDS data 0 P/N | `tmds_data[0]` | B11 / B10 |
| TMDS data 1 P/N | `tmds_data[1]` | D11 / D10 |
| TMDS data 2 P/N | `tmds_data[2]` | G11 / G10 |
| TMDS clock P/N | `tmds_clk` | C11 / C10 |

### 4.2 PMOD1/2 OV5640

| 信号 | FPGA 引脚 | 信号 | FPGA 引脚 |
|---|---|---|---|
| D0 | G5 | D1 | F5 |
| D2 | G8 | D3 | G7 |
| D4 | H7 | D5 | H8 |
| D6 | J5 | D7 | H5 |
| PCLK | L5 | HREF | K5 |
| VSYNC | K11 | XCLK | L11 |
| RESET_N | E11 | PWDN | E10 |
| SCCB SCL | A11 | SCCB SDA | A10 |

### 4.3 J3 底盘 I/O

| 设备 | 信号 | FPGA 引脚 |
|---|---|---|
| Motor 0 | PWM / IN1 / IN2 | K1 / K2 / L1 |
| Motor 1 | PWM / IN1 / IN2 | L2 / K4 / J4 |
| Motor 2 | PWM / IN1 / IN2 | G2 / G1 / L3 |
| Motor 3 | PWM / IN1 / IN2 | L4 / J2 / J1 |
| 两块 TB6612 共用 | STBY | C2 |
| Encoder 0 | A / B | B2 / F1 |
| Encoder 1 | A / B | F2 / A1 |
| Encoder 2 | A / B | E1 / D1 |
| Encoder 3 | A / B | E3 / H4 |
| VL53L0X | SCL / SDA | G4 / H2 |
| ICM-42688-P | CS_N / SCLK / MOSI / MISO | H1 / J7 / K7 / L8 |
| HC-SR04 | TRIG / ECHO | L7 / K10 |
| IR 0…3 | 左前 / 右前 / 左侧 / 右侧 | L10 / K9 / L9 / K8 |
| 急停 | `ESTOP_N` | J8 |
| 预留，高阻 | reserve 0…3 | F6 / F7 / J10 / J11 |

B2 是器件专用 I2C 复用脚。构建脚本必须保留 `set_option -use_i2c_as_gpio 1`，否则布局布线会拒绝 Encoder 0 A 相。任何器件版本、封装或引脚调整都要重新核对数据库和原理图。最终报告显示 bank 0/1/2/3/6 已满，不能仅凭总 I/O 余量判断还能继续加信号。

## 5. 电气连接要求

- HC-SR04 的 ECHO 常为 5 V，必须通过电阻分压或合适电平转换降到 3.3 V；禁止直接接 FPGA。
- VL53L0X I2C 和 OV5640 SCCB 必须有到 3.3 V 的外部上拉；确认模块板上是否已装，避免并联后阻值过低。
- 编码器输入必须是 3.3 V 兼容信号。本工程只有同步和非法相变检测，没有模拟施密特、差分接收或专门毛刺滤波；长线和电机噪声环境应增加硬件整形/屏蔽并做故障注入。
- 四路 IR 被假定为 active-low 数字量并做约 1 ms 去抖。不同模块的极性、阈值、视场、串扰和安装方向必须现场校准；IR 不是毫米测距，也不能覆盖后方盲区。
- 两块 TB6612 的逻辑电源使用 3.3 V 兼容电平，电机 VM 单独供电并与 FPGA 共地；按电机堵转电流选择驱动板、电源、保险和去耦。
- PMOD_DVI 只允许连接到与该差分引脚排列相符的 Sipeed 模块；连接显示器前先核对 P/N 和 5 V/3.3 V 电源针脚。

## 6. UART 协议

UART 为 115200 baud、8 data bits、no parity、1 stop bit。命令不需要回车：

| 命令 | 含义 | 典型响应 |
|---|---|---|
| `P` | 探活 | `PONG;CHOICE3=READY` |
| `S`、`?` | 状态 | `STATUS;ARM=0;HEALTH=F0;FAULT=9F;NAV=0;VLRS=00;DRV=0` |
| `A` | 请求武装 | `ACK;ARM`；条件不满足时另发 `ERR;ARM_REJECTED` |
| `D` | 撤武装 | `ACK;DISARM` |
| `G` | 启动固定路线 | `ACK;GO=ROUTE` |
| `X` | 请求清锁存故障 | `ACK;FAULT_CLEAR` |
| `F` / `B` | 前进 / 后退 | `ACK;MANUAL` |
| `L` / `R` | 左 / 右弧转 | `ACK;MANUAL` |
| `Q` / `E` | 左 / 右原地旋转 | `ACK;MANUAL` |
| `0` | 手动停止 | `ACK;MANUAL` |

`DRV` 取自最终 `motor_stby_o` 逻辑信号，比单独的 `ARM` 更接近功率门状态，但仍不是示波器对 C2 实际电压的独立测量。

## 7. 构建与验证

工程依赖本仓库中的 `fpga_rtl/src/motor`、`fpga_rtl/boards/tang_primer_25k_bringup` 与 `sensor_examples`，不能把本目录孤立复制后直接构建。

```powershell
cd C:\Users\32035\Desktop\GoWin\fpga_rtl\boards\tang_primer_25k_choice3_robot
pwsh -File .\Scripts\run_sim.ps1
pwsh -File .\Scripts\run_gowin.ps1
```

默认工具路径对应 Icarus Verilog 和 Gowin EDA V1.9.12.03，可通过脚本参数覆盖。`run_sim.ps1` 要求四个 TB 各出现且只出现一个 PASS，失败时返回非零；默认不生成 VCD。`run_gowin.ps1` 检查目标器件、Logic/BSRAM/DSP 门限和全局 setup/hold 0/0，只构建，不调用 programmer。

板级 UART 脚本专用于“传感器未连接且必须保持禁能”的安全基线，默认 COM5，不烧录：

```powershell
pwsh -File .\Scripts\board_uart_test.ps1 *>&1 |
  Tee-Object -FilePath .\reports\board_uart.log
```

它自动发送 `P/S/A/S/F/S`，断言三次状态均为 `ARM=0;DRV=0`，断言武装被拒绝，并在不满足时退出失败。连接齐全且希望真正武装时不要使用这组“必须拒绝武装”断言。

## 8. 上车前必须完成

1. 按真实轮径、减速比和编码器每轮计数重算 `MM_PER_TICK_Q16`，核对四轮 A/B 相和电机方向。
2. 架空车轮，从低占空比逐路验证 PWM、IN1/IN2、STBY、死区、电流和堵转检测，再落地整定 PI。
3. 用逻辑分析仪测 DVP PCLK 与数据/HREF/VSYNC，补齐 input-delay SDC 后重跑 STA。
4. 对 IMU 断线/短路、I2C 卡低、超声无回波、IR 卡死、编码器断线和急停毛刺做实物故障注入。
5. 增加 UART/上位机心跳看门狗和后向防撞措施，再测试自动路线。
6. 任何 RTL、约束、工具版本或温压条件变化后重新执行完整仿真和 P&R；当前 system 域只有约 0.074 ns 最差 setup 余量。

## 9. 许可与未包含内容

`choice3_hdmi_phy.v` 的 OSER10/ELVDS 连接方式派生自 Sipeed Tang Primer 25K 官方 SVO 示例，文件内保留 Clifford Wolf 的 ISC License 声明。其余新写 RTL 及仓库复用模块的许可应按各自上游文件处理。

Intel RealSense D435i 没有集成到本工程。D435i 的 USB 3.x 主机、枚举和深度数据链远超本工程的直接 GPIO/DVP 接口范围；若后续使用，应由具备 USB Host 的处理器接收并通过受控接口向 FPGA 提供预处理结果。
