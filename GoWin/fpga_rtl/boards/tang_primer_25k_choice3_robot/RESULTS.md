# 冻结版验证结果

本文只记录 `2026-08-27` 冻结版本的可复核证据。生产 RTL/约束在最终位流之后未再修改；随后只增加了文档和串口日志。

## 1. 结论与证据等级

| 等级 | 本工程定义 | 状态 |
|---|---|---|
| L1 | 源码完整、依赖可解析、顶层可展开 | PASS |
| L2 | 控制/UART/视频/传感器守卫 RTL 回归 | PASS |
| L3 | 指定器件综合、P&R、STA、位流生成 | PASS |
| L4 | 位流 SRAM 下载、FPGA 启动、UART 无外设 fail-safe | 部分 PASS |
| L5 | 摄像头、显示、全部传感器、四电机和导航联合实车验收 | 未执行 |

因此当前可准确表述为“可综合、时序收敛并完成基本板级安全冒烟测试”，不能表述为“整车功能已实测通过”。

## 2. RTL 仿真

执行：

```powershell
pwsh -File .\Scripts\run_sim.ps1
```

最终日志时间：`2026-08-27T04:58:08.5881926+08:00`。日志文件：[reports/simulation.log](reports/simulation.log)。四个唯一 PASS 标记为：

```text
PASS: tb_choice3_control_core
PASS: tb_choice3_uart_console
PASS: tb_choice3_video_pipeline
PASS: tb_choice3_sensor_guards
```

| TB | 已覆盖重点 |
|---|---|
| `tb_choice3_control_core` | 默认禁能、外设缺失拒绝武装、急停下拒绝武装、raw 急停异步切断、故障锁存、新鲜度过期撤武装且不自动恢复、旧手动命令清除、六种手动动作符号、前障碍禁止手动前进、固定路线/避障状态、四路不同编码器速度与 PI duty、速度正常值/正负饱和、换向死区输出互斥 |
| `tb_choice3_uart_console` | 上电状态 53 字节字段、P/A/D/G/X/F 命令、ARM reject、故障变化状态、CRLF 与 TX 帧格式 |
| `tb_choice3_video_pipeline` | DVP RGB565 小端拼接、RGB444 量化、首末地址、双 bank 交替、pending 未确认不覆写、短帧拒绝、VGA 全帧计数、同步 RAM 像素/DE/坐标逐拍对齐、TMDS 控制码和运行差分 |
| `tb_choice3_sensor_guards` | ICM 六轴全 0/全 F 拒绝、正常样本接受、事件 freshness 建立/老化/source-ready 撤销 |

控制 TB 注入后的四路速度为 `40,20,0,-21`，四路 duty 为 `156,167,178,190`，证明四路估计/控制实例没有被错误并成同一路。仿真属于数字模型验证，不包含传感器电气时序、机械负载或模拟噪声。

另用 Icarus Verilog 加 Gowin 官方 GW5A `prim_sim` 对最终顶层完成独立 elaboration，退出码为 0；出现的 13 条 warning 来自供应商 `prim_sim` 模型本身。该检查证明顶层端口/原语/模块依赖可展开，不等价于门级时序仿真。

## 3. Gowin 构建与位流

执行：

```powershell
pwsh -File .\Scripts\run_gowin.ps1
```

工具：Gowin EDA V1.9.12.03；最终 P&R/位流时间：`2026-08-27 04:57:38 +08:00`。

| 项目 | 结果 |
|---|---|
| Part | `GW5A-LV25MG121NC1/I0` |
| Device Version | B |
| 位流 | `impl/pnr/tang_primer_25k_choice3_robot.fs` |
| 大小 | 7,142,416 bytes |
| SHA-256 | `3c5a590f1bd14b4b8dca2a6910708e3286d3cbe2a88c9205ff4b50173978d957` |
| Setup violated endpoints | 0 |
| Hold violated endpoints | 0 |

### 3.1 资源

| 资源 | 使用 | 利用率 | 细分 |
|---|---:|---:|---|
| Logic | 6866 / 23040 | 30% | 5620 LUT、1246 ALU |
| Register | 3189 / 23280 | 14% | 3165 logic FF、24 I/O FF、0 latch |
| CLS | 4390 / 11520 | 39% |  |
| I/O Port | 66 / 86 | 77% | 62 I/O buffer |
| IOLOGIC | 6 / 80 | 8% | OSER10 ×3 |
| BSRAM | 39 / 56 | 70% | SDPB ×36、pROM ×3 |
| DSP | 9 / 28 | 33% | MULT27X36 ×4、MULTALU27X18 ×1 |
| PLLA | 1 / 6 | 17% |  |
| CLKDIV | 1 / 16 | 7% |  |

资源门禁为 Logic ≤18000、BSRAM ≤44、DSP ≤20，均通过。framebuffer 使用 36 个 SDPB；整个设计另有 3 个 pROM。bank 0/1/2/3/6 均为 100%，bank 7 为 13/16；即使总 I/O 仍有余量，也不能假定任意新引脚都可放置。

### 3.2 STA

| 时钟 | 约束 | 报告 Fmax | 逻辑级数 | TNS |
|---|---:|---:|---:|---:|
| system | 50.000 MHz | 50.186 MHz | 28 | 0.000 ns |
| camera PCLK | 95.997 MHz | 197.508 MHz | 7 | 0.000 ns |
| pixel | 25.156 MHz | 51.960 MHz | 26 | 0.000 ns |
| PLL CLKOUT0 / serial | 125.781 MHz | 1309.114 MHz | 1 | 0.000 ns |
| PLL CLKOUT1 / camera XCLK | 23.958 MHz | 无内部时序路径 | — | 0.000 ns |

system 域最差 setup slack 约 `+0.074 ns`，虽然为 PASS，但余量很小；任何 RTL、布局、器件速度等级、工具版本或约束变化都必须重新 P&R。serial 的 1309.114 MHz 只是一级内部路径的 STA 数字，不是对板级 TMDS 信号完整性、显示器兼容性或 OSER 极限的性能承诺。

SDC 把 camera PCLK 与 system/pixel 设为异步组，并显式建立 50 MHz 到 25.156 MHz 的 161/320 generated clock。最终 setup/hold 均为 0 个违规端点。综合阶段 `gowin.log` 仍有 3 条 CK3000，都是工具在应用最终 P&R SDC 前无法推导 pixel 自动时钟与 system/camera PCLK 的关系；最终 STA 已由显式 generated clock/async group 闭环。没有遗留 DI0004 或 EX3791。

必须强调：当前没有对 OV5640 DVP 数据/HREF/VSYNC 相对 PCLK 设置板级 min/max `set_input_delay`。camera 域 Fmax 不能证明外部采样窗口闭合，这属于 L5 前必须补齐的约束缺口，而不是可以忽略的 warning。

构建使用 `set_option -use_i2c_as_gpio 1`，使专用 I2C 复用脚 B2 可作为 Encoder 0 A 相。去掉该选项会导致布局失败。

## 4. SRAM 下载与 UART 板测

冻结位流通过 Gowin Programmer SRAM Program（op2）下载成功，未写 Flash：

```text
UserCode = 0x0000EA0A
Status   = 0x70026020
```

在外部传感器未连接、默认 fail-safe 条件下，对 COM5 执行自动脚本：

```powershell
pwsh -File .\Scripts\board_uart_test.ps1 *>&1 |
  Tee-Object -FilePath .\reports\board_uart.log
```

最终自动断言日志：[reports/board_uart.log](reports/board_uart.log)。实收内容：

```text
P <= PONG;CHOICE3=READY<CR><LF>
S <= STATUS;ARM=0;HEALTH=F0;FAULT=9F;NAV=0;VLRS=00;DRV=0<CR><LF>
A <= ACK;ARM<CR><LF>ERR;ARM_REJECTED<CR><LF>
S <= STATUS;ARM=0;HEALTH=F0;FAULT=9F;NAV=0;VLRS=00;DRV=0<CR><LF>
F <= ACK;MANUAL<CR><LF>
S <= STATUS;ARM=0;HEALTH=F0;FAULT=9F;NAV=0;VLRS=00;DRV=0<CR><LF>
PASS: choice3 UART 板级安全门禁通过（端口=COM5；脚本未烧录）
```

这证明 FPGA 启动、UART 收发、状态打包、外设缺失拒绝武装、旧手动命令不执行和最终逻辑 `DRV=0`。脚本本身不读取位流哈希，也不测量 C2/PWM/IN1/IN2 的实际电压；哈希与下载镜像的对应关系来自同一冻结构建和操作者的下载记录，物理功率引脚仍需示波器/逻辑分析仪复核。

`HEALTH=F0` 表示保留高三位和编码器基线位为 1，摄像头/IMU/VL53/超声为 0。`FAULT=9F` 包含 active-low 急停/缺失外设以及启动阶段锁存的编码器故障。发送 `A` 后明确返回拒绝且保持 ARM/DRV 为 0；发送 `F` 虽返回命令接收 ACK，但未武装状态会清除动作，所以仍保持禁能。

## 5. 已知风险与未验证项

1. **DVP 外部时序**：缺少板级 input-delay，必须按具体 OV5640 模组和排线补约束并测量。
2. **system 时序余量**：最差 slack 仅约 +0.074 ns，不适合在不重跑 P&R 的情况下继续修改。
3. **UART 无 watchdog**：已武装手动动作不会因串口静默自动停止；必须增加心跳/动作租约或板外看门狗。
4. **TB6612 死区语义**：方向翻转时 STBY 高而 PWM/IN1/IN2 低，是 short-brake，不是 coast；尚未测制动电流、母线回灌和机械冲击。
5. **编码器**：未做真实 PPR/轮径/相序标定；单端长线噪声、丢边、抖动和堵转阈值未实测。数字非法相变检测不能替代硬件整形。
6. **IR**：只验证了 active-low 数字模型和 1 ms 去抖；阈值、极性、日光干扰、串扰、视场和安装方向未实测，且没有后向覆盖。
7. **盲区后退**：自动避障含约 200 ms 后退，但没有后向毫米测距；不可无人值守运行。
8. **VL53 质量**：`VLRS` 未做 ST PAL RangeStatus 映射，质量门控不完整；需按实际 API/模组验证。
9. **IMU 诊断**：全 0/全 F 守卫只能拦截典型断线/短路哨兵，未做运行期 WHO_AM_I 重探测；倾角十字不是 roll/pitch/yaw 姿态解算。
10. **视频**：RGB565 降为 RGB444 有色彩损失；双缓冲 RTL 边界已验证，但真实 OV5640、PMOD_DVI 和显示器没有实测。
11. **电气安全**：HC-SR04 ECHO 必须降压，I2C/SCCB 必须外部上拉，TB6612 需独立电机电源/共地/保险/去耦，急停应形成板外硬件链路。
12. **I/O 可扩展性**：多个 bank 已满，B2 依赖专用脚释放选项；增加 D435i 或其他接口不能只看总 LUT/引脚余量。

未执行项目包括：摄像头出图、DVI 锁定、传感器身份/测量精度、四编码器实际速度、四电机 PI、堵转故障注入、急停引脚电压/关断时间测量、整车避障/路线、温压角 STA、EMI/电源完整性、长期运行和 Flash 烧录。Intel RealSense D435i 不在该 RTL 工程内。

## 6. 证据文件索引

- RTL 回归：[reports/simulation.log](reports/simulation.log)
- Gowin 控制台日志：[reports/gowin.log](reports/gowin.log)
- P&R 资源报告：`impl/pnr/tang_primer_25k_choice3_robot.rpt.txt`
- P&R 时序报告：`impl/pnr/tang_primer_25k_choice3_robot_tr_content.html`
- 最终位流：`impl/pnr/tang_primer_25k_choice3_robot.fs`
- 自动 UART 板测：[reports/board_uart.log](reports/board_uart.log)
- 可复跑脚本：`Scripts/run_sim.ps1`、`Scripts/run_gowin.ps1`、`Scripts/board_uart_test.ps1`
