# Tang Primer 25K GoRV32 + RT-Thread

本工程在 `GW5A-LV25MG121NC1/I0`（Device B）上实例化 Gowin GoRV32 1.0，并从板载 W25Q64 的物理偏移 `0x100000` XIP 运行 RT-Thread 4.1.1。最终实板已在 COM5 @ 115200 8-N-1 提供可交互的 FinSH/MSH shell。

## 已验证链路

```text
50 MHz 时钟
  -> GoRV32 RV32IMC + 64KB SRAM + MTIME + UART1 + QSPI XIP
  -> RT-Thread 4.1.1 静态内核与 100Hz tick
  -> FinSH/MSH（msh >）
  -> version/list + 20 轮 ps/hello 严格串口回归
```

2026-08-27 的最终状态：

- wrapper 仿真 PASS；
- Gowin 综合、布局布线和位流生成 PASS；
- 50MHz setup/hold 违规均为 0，实际 Fmax 50.604MHz；
- FPGA `.fs` 通过 operation 2 下载到 SRAM；
- raw RT-Thread `.bin` 通过 operation 56 写入并校验 W25Q64 物理 `0x100000`；
- `version`、`list` 和连续 20 轮 `ps`/`hello` 全部通过，未发现缺字、乱码或非法控制字符。

完整结果见 `RESULTS.md`，串口原始证据见 `reports/serial_shell.log`。

## GoRV32 配置与地址

| 项目 | 值 |
|---|---|
| CPU/ABI | GoRV32，RV32IMC，machine mode，ILP32 |
| IP SRAM | 64KB |
| UART | UART1 开启，UART2 关闭 |
| Flash Burn Address | `0x100000` |
| XIP 逻辑地址 | `0x80000000` |
| SRAM | `0x40000000..0x4000FFFF` |
| PLIC | `0xE4000000`，当前 BSP 未使用 |
| CLINT | `0xE6000000` |
| MTIMECMP / MTIME | `0xE6004000` / `0xE600BFF8` |
| UART1 | `0xF0200000` |
| 时钟 / tick | 50MHz / 100Hz |

磁盘上的最终生成配置必须同时包含：

```verilog
`define ONCHIP_RAM_SIZE 14
`define FLASH_BURN_ADDR 24'h100000
`define UART1_SUPPORT
```

IPC 还必须包含 `SRAM_Size=64KB`、`Enable_UART1=true` 和 `Enable_UART2=false`。固件构建脚本会主动核验这些条件。

## XIP 与两类下载文件

这里有两个不可混用的二进制：

| 文件 | 含义 | 已验证操作 |
|---|---|---|
| `impl/pnr/tang_primer_25k_rtthread.fs` | FPGA 配置位流，包含 GoRV32 SoC | Gowin Programmer operation 2，下载到 FPGA SRAM |
| `firmware/build/xip/rtthread_gorv32.bin` | CPU raw XIP 固件 payload | operation 56，写入 W25Q64 物理偏移 `0x100000` 并 verify |

ELF 中 `.text/.rodata` 的逻辑地址是 `0x80000000`，仅 `.data` 在启动时复制到 `0x40000000`，随后清零 `.bss`。`rtthread_gorv32.hex` 也使用逻辑地址 `0x80000000`，不能把它直接当作物理起点为 `0x100000` 的 Flash HEX。

本工程没有自动烧录脚本，避免覆盖板卡上的其它验证任务。

## 固定引脚

| 信号 | 引脚 | 说明 |
|---|---|---|
| `clk_i` | E2 | 板载 50MHz |
| `reset_i` | H11 | 高有效复位 |
| `uart_rx_i` | B3 | 板载 USB-UART RX |
| `uart_tx_o` | C3 | 板载 USB-UART TX |
| `flash_qspi_csn` | E6 | 板载 W25Q64 CSN |
| `flash_qspi_miso` | E5 | 板载 W25Q64 MISO |
| `flash_qspi_mosi` | D6 | 板载 W25Q64 MOSI |
| `flash_qspi_clk` | E7 | 板载 W25Q64 CLK |
| `flash_qspi_holdn` | E4 | 板载 W25Q64 HOLDN |
| `flash_qspi_wpn` | D5 | 板载 W25Q64 WPN |

工程将 MSPI 引脚作为普通 IO 交给 GoRV32 QSPI XIP 接口使用。

## 构建与验证

在仓库根目录运行：

```powershell
# wrapper 连接与 POR 仿真
& .\fpga_rtl\boards\tang_primer_25k_rtthread\scripts\run_sim.ps1

# Gowin 综合、timing-driven PnR、时序核验与位流生成
& .\fpga_rtl\boards\tang_primer_25k_rtthread\scripts\run_gowin.ps1

# RT-Thread XIP 固件
& .\fpga_rtl\boards\tang_primer_25k_rtthread\firmware\scripts\build_firmware.ps1

# 已正确下载 FPGA 与固件后，默认 20 轮严格串口回归
& .\fpga_rtl\boards\tang_primer_25k_rtthread\scripts\serial_shell_test.ps1 `
    -Port COM5 -BaudRate 115200 -TimeoutMs 8000
```

串口脚本可用 `-StressRounds 100` 增加压力轮数。它会逐轮检查完整 `ps` 表格、精确 `hello` 输出和所有非预期控制字符，不接受只出现提示符或部分文本的假通过。

## trap 入口的重要修复

RT-Thread 4.1.1 原始 E310 `interrupt_gcc.S` 在保存 x5 之前用 `t0/x5` 构造常量，100Hz tick 下会随机破坏被中断线程。修复前，短命令可以偶尔成功，但严格回归捕获到 `ps` 字段缺失及 `0x05` 控制字符。

`firmware/bsp/interrupt_gcc.S` 在任何 `t0` 临时使用前先保存 x5，并由构建脚本明确选用。trap 修复版固件完成写入后，20 轮严格回归全部通过。

## 关键文件

- `src/tang_primer_25k_rtthread_top.v`：POR、GoRV32、UART1 与板载 QSPI 顶层连接。
- `constraints/tang_primer_25k_rtthread.cst`：板级引脚与电气约束。
- `constraints/tang_primer_25k_rtthread.sdc`：50MHz 系统时钟和 QSPI 时钟约束。
- `tang_primer_25k_rtthread_ipgen/src/gowin_gorv32/`：GUI 生成的 GoRV32 IP。
- `firmware/`：RT-Thread BSP、链接脚本、应用与固件构建脚本。
- `scripts/serial_shell_test.ps1`：可配置轮数的严格实板串口回归。
- `reports/`：仿真、Gowin 与实板串口证据。
- `RESULTS.md`：最终资源、时序、哈希、下载与实板结果。

## 验证边界

- `.fs` 当前只验证 FPGA SRAM 下载；未验证将 FPGA 配置固化后掉电冷启动。
- RT-Thread `.bin` 已写入 W25Q64 并 verify，但未单独验证掉电保持后的冷启动。
- 当前 BSP 使用轮询 UART，仅接管 machine timer interrupt；PLIC、UART 中断、heap、device、DFS 和网络栈不在本次范围内。
