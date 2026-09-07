# Tang Primer 25K 传感器片上功能 BIST

本工程在 `GW5A-LV25MG121NC1/I0`（Device B）内部生成可综合激励，验证 17 个非 I2C/SPI 传感器接口核心的正常路径和关键边界。它只占用板载 50 MHz 时钟、复位和 USB-UART，不需要外接传感器引脚。

## 结论边界

`FUNC_BIST=PASS` 证明的是：位流已在真实 FPGA 逻辑资源中执行这些 RTL 核、片上激励确实发生、核心输出了预期协议结果，并且 UART 调试链路可读。

它**不证明**下列外部环节可用：真实传感器器件、接线、电源、5 V/3.3 V 电平转换、模拟 ADC、光学与机械安装、真实 DVP 时序裕量、真实串口波特率误差或现场噪声。TSL1401 的 `AO` 模拟电压尤其不能直接接普通 FPGA GPIO，实际系统必须另配 ADC。

## 三层自检

- `ACT`：观察到片上激励线的真实活动，不允许未发送协议就 PASS。
- `OBS`：观察到被测核心的有效脉冲或边界事件。
- `PASS`：有效脉冲携带的数据与独立黄金值完全匹配。

只有 `ACT=OBS=PASS=0x1FFFF` 且 `DONE=1` 时，`B` 命令才返回 PASS。严格 testbench 还在 DUT 外独立统计上述事件，并检查每个新 PASS 位都晚于对应的活动和有效结果，避免常量假 PASS。

## 位图定义

| 位 | 模块 | 片上激励与关键判据 |
|---:|---|---|
| 0 | `digital_ir_proximity` | 先给单拍毛刺，再稳定低电平；仅稳定检测和 `changed` 通过 |
| 1 | `gnss_nmea_uart` | `$GPGGA,1,2,3*4A`；类型、长度 11、XOR 校验均匹配 |
| 2 | `gy39_uart` | 合法 `5A 5A 15 04 ... 36` 照度帧；结果 `0x00003039` |
| 3 | `hc_sr04` | 观察片上 trigger，延迟后回送 58 拍 echo；结果 58 us、10 mm |
| 4 | `itr9606_pulse_speed` | 两个有效槽脉冲；计数 2 且周期在独立范围内 |
| 5 | `jy61p_uart` | 合法 11 字节角度帧；滚转、俯仰、偏航、版本逐项匹配 |
| 6 | `maxsonar_uart` | ASCII `R1234\r`；解析值 1234 |
| 7 | `mechanical_bumper` | 单拍抖动后稳定左碰撞；去抖、事件和锁存均通过 |
| 8 | `mt9v03x_dvp` | 新 VSYNC 后送灰度 `3C`；检查像素有效与坐标 |
| 9 | `multi_ir_array` | 通道 5 单拍毛刺、通道 1 稳定有效；掩码 `02` |
| 10 | `ov5640_dvp` capture | 按 `4300h=6Fh` 低字节优先送 `00 F8`；输出 `F800` |
| 11 | `ov7725_dvp` capture | 高字节优先送 `F8 00`；输出 `F800` 且无半像素错误 |
| 12 | `quadrature_encoder` | 四个合法正向状态后再给 `00->11`；位置 +4 并报告非法跳变 |
| 13 | `scc8660_dvp` capture | 高字节优先送 `07 E0`；输出 `07E0` |
| 14 | `tcrt5000` | 稳定低有效输入；积分滤波到 3 并产生上升事件 |
| 15 | `tf_luna_uart` | 合法 9 字节帧；距离 1234、强度 567、温度原始值 2104 |
| 16 | `tsl1401_adc_frontend` | 每像素返回 `A00h+index`；128 样本及第 129 个终止 CLK |

HC-SR04 与传感器 UART 在 BIST 中使用加速参数，目的是缩短片上测试时间；功能状态机仍是原始 `sensor_examples` 核，未修改原源码。

## 固定引脚

| 信号 | 引脚 | 说明 |
|---|---|---|
| `clk_i` | E2 | 板载 50 MHz |
| `reset_i` | H11 | 高有效复位 |
| `uart_rx_i` | B3 | 板载 USB-UART RX |
| `uart_tx_o` | C3 | 板载 USB-UART TX |

## 构建与验证

在本目录执行：

```powershell
& .\Scripts\run_sim.ps1
& .\Scripts\run_gowin.ps1
```

仿真脚本要求唯一的 `17/17` PASS 行。Gowin 脚本要求精确器件/版本、0 个 setup/hold 违规，并确认 `.fs` 已生成。

2026-08-27 已将该位流通过 `USB Debugger A` 下载到实板 SRAM，并在 COM5 上完成 UART 验证。复现命令：

```powershell
& .\Scripts\serial_debug.ps1 -Port COM5 -BaudRate 115200
```

命令响应：

```text
P -> PONG;FUNC_BIST=READY
B -> FUNC_BIST=PASS;MODULES=17/17;MODE=ONCHIP_STIMULUS
D -> PASS=0x1FFFF;OBS=0x1FFFF;ACT=0x1FFFF;DONE=1
```

实测三条命令均返回上述精确响应；原始串口证据保存在 `reports/serial_debug.log`。本工程自身不包含自动烧录步骤，避免在多任务协作时覆盖板上正在验证的位流。

## 关键文件

- `src/sensor_functional_stimulus.v`：可综合片上黄金激励。
- `src/sensor_functional_bist_core.v`：17 核实例、三层位图和精确判据。
- `src/sensor_functional_bist_console.v`：115200 UART 命令控制台。
- `sim/tb_tang_primer_25k_sensor_functional_bist.sv`：独立严格自检。
- `RESULTS.md`：当前可复现证据与资源结果。
