# Tang Primer 25K 板级自检工程

本工程用于验证当前 `GW5A-LV25MG121NC1/I0` 测试板的最小开发闭环：

1. GowinSynthesis 与布局布线；
2. 生成 `.fs` 位流；
3. 通过板载 USB 调试器写入 FPGA SRAM；
4. 通过同一调试器的 UART 通道完成双向命令测试。

工程不驱动电机、传感器或外部总线，只使用四个经过官方原理图核对的信号：

| 信号 | 芯片引脚 | 用途 |
|---|---|---|
| `clk_i` | E2 | 核心板 50 MHz 有源晶振 |
| `reset_i` | H11 | Dock 按键 S1，高有效，内部下拉 |
| `uart_rx_i` | B3 | FPGA RX，连接调试器 TX |
| `uart_tx_o` | C3 | FPGA TX，连接调试器 RX |

串口参数为 `115200 8-N-1`。上电后发送 `GW5A25 BRINGUP READY`，之后每秒发送一次 `HB`。支持以下单字符命令：

| 命令 | 响应 |
|---|---|
| `P` / `p` | `PONG` |
| `I` / `i` | 芯片标识 |
| `S` / `s` | 时钟和波特率状态 |
| 其他字符 | 命令帮助 |

## 构建与仿真

在本目录运行 RTL 自检：

```powershell
& 'C:\Users\32035\AppData\Local\Programs\IcarusVerilog\bin\iverilog.exe' `
  -g2012 -Wall -s tb_tang_primer_25k_bringup `
  -o impl\tb_tang_primer_25k_bringup.vvp `
  src\bringup_uart_rx.v src\bringup_uart_tx.v src\bringup_console.v `
  src\tang_primer_25k_bringup_top.v sim\tb_tang_primer_25k_bringup.sv
& 'C:\Users\32035\AppData\Local\Programs\IcarusVerilog\bin\vvp.exe' `
  impl\tb_tang_primer_25k_bringup.vvp
```

执行 Gowin 综合与布局布线：

```powershell
& 'D:\Gowin\Gowin_V1.9.12.03_x64\IDE\bin\gw_sh.exe' Scripts\run_gowin.tcl
```

构建脚本会显式把 CPU/SSPI 双用途脚释放为普通 GPIO；E2 同时是 SSPI 复用脚和本板 50 MHz 时钟脚，缺少该选项时 P&R 会拒绝官方原理图中的时钟约束。

首次实板验证优先使用 SRAM 模式，不改写启动 Flash：

```powershell
& 'D:\Gowin\Gowin_V1.9.12.03_x64\Programmer\bin\programmer_cli.exe' `
  --device GW5A-25B --operation_index 2 `
  --fsFile impl\pnr\primer25k_uart_bringup.fs --cable-index 4
```

`--operation_index 2` 是 SRAM Program；断电后配置会消失，不会改写板载启动 Flash。这里的 `--cable-index 4` 是 Programmer CLI 对线缆**类型** `USB Debugger A` 的编号，不是“第 4 根物理线缆”。换用其他调试器类型时应先执行扫描并按其返回值调整；同一块 Dock 仍识别为 `USB Debugger A` 时保持 `4`。本机对 `SRAM Program and Verify`（操作 4）的写入阶段能够完成，但随后的 SRAM 回读在地址 0 报 `Verify failed`；操作 2 配合应用层双向串口自检可稳定通过，因此本工程把可运行协议响应作为首要实板验收证据。

烧录后运行双向串口自检：

```powershell
.\Scripts\serial_debug.ps1 -Port COM5
```

## 与选题三正式架构的关系

该工程只验证板级工具链和调试面，不替代 `fpga_rtl/src` 中的机器人实时控制逻辑。正式 ACG720 工程采用四层边界：

1. **板级适配层**：时钟、复位、引脚、DDR3、RGMII、摄像头和传感器物理接口；
2. **纯 RTL 实时层**：编码器、PWM、运动学、控制看门狗、急停、测距限速与局部避障；
3. **可选软核服务层**：只处理相机/传感器初始化、参数管理、低速诊断和升级，不进入电机安全闭环；
4. **边缘 AI 层**：Jetson/RK3588 承担 SLAM、视觉识别、语义导航与任务规划，通过有 CRC 和心跳的固定协议连接 FPGA。

这样既保留 FPGA 在实时性和安全性上的主体地位，又避免把不适合硬连线实现的设备配置与高级决策塞入大规模状态机。软核故障时，纯 RTL 层仍能独立执行急停、看门狗和安全停车。

## 从 25K 测试板迁移到 ACG720

当前测试工程只复用工具链与调试接口，不把 25K 的板级文件直接冒充 ACG720 顶层。迁移时保留 UART 控制台和协议语义，替换器件、时钟/复位、CST/SDC 与外设 wrapper：

| 边界 | 25K 自检阶段 | ACG720 正式阶段 |
|---|---|---|
| 器件 | `GW5A-LV25MG121NC1/I0` | 按实际购板核对 `GW5AT-60` 或 `GW5AT-138` 完整料号 |
| 实时安全 | UART 自检 | 纯 RTL 编码器、PI/PWM、限速、急停、看门狗、局部避障 |
| 低速管理 | 纯 RTL 工具链自检 | 正式采用已在 25K 验证的 PicoRV32 服务层，负责初始化、参数、路径任务和诊断 |
| 高级感知 | 不接入 | Jetson/RK3588 承担 SLAM、视觉与语义任务 |
| 通信 | USB-UART | RGMII/UDP 主链路，UART 保留为维护后门 |

正式主线优先复用仓库已有的 `robot_rtl_core`、网络协议和安全模块。软核不进入编码器、PWM、电机闭环、急停和硬件限速路径；这样即使软核卡死，确定性安全层仍可独立停车。

## 2026-08-27 实板验证记录

- Gowin Programmer 通过板载 `USB Debugger A` 扫描到器件 ID `0x0001281B`，识别为 `GW5A-25B`。
- Gowin V1.9.12.03 对完整料号 `GW5A-LV25MG121NC1/I0` 完成综合、布局布线、时序分析和位流生成。
- P&R 使用 308/23040 个逻辑单元、112/23280 个寄存器和 4/86 个 I/O；50 MHz 约束下实际 Fmax 为 215.409 MHz，setup/hold 违例端点均为 0。
- 位流 `impl/pnr/primer25k_uart_bringup.fs` 的 SHA-256 为 `409F442CD53F486D88A72A5E2A6B07D8038F7A37F8225E6C279D98565CDD1F90`。
- SRAM Program（操作 2）完成，Programmer 报告 User Code `0x00009946`、Status Code `0x70026020`。
- Windows `COM5` 在 `115200 8-N-1` 下收到启动横幅和每秒心跳；`P`、`I`、`S` 三条命令分别返回预期内容，双向串口自检退出码为 0。

以上证明本机 Gowin 工具链、JTAG SRAM 配置、50 MHz 时钟、板载 USB-UART 收发和最小 RTL 工程闭环正常。该结果不等同于 Flash 固化、ACG720 引脚验证、RGMII、传感器、电机或整机安全验证。
