# Tang Primer 25K / GoRV32 RT-Thread 固件

这是面向 Gowin GoRV32 1.0 的最小 RT-Thread 4.1.1 BSP。Tang Primer 25K 已通过 UART1 提供 115200 8-N-1 的 FinSH/MSH 命令行。

## 当前状态

- GoRV32 IP 已核验为 64KB SRAM、UART1 开启、UART2 关闭、Flash Burn Address `0x100000`。
- 固件已使用 RV32IMC/ILP32 工具链完成编译、链接和 ELF/XIP 静态自检。
- FPGA 位流已通过 Gowin Programmer operation 2 下载到 SRAM。
- `build/xip/rtthread_gorv32.bin` 已通过 operation 56 写入并校验板载 W25Q64 的物理偏移 `0x100000`。
- COM5 已实测 RT-Thread 4.1.1 的 `msh >`，`version`、`list` 及连续 20 轮 `ps`/`hello` 严格检查全部通过；证据在 `../reports/serial_shell.log`。
- 构建和串口脚本均不会调用 Programmer；本目录没有自动烧录步骤。

## 硬件约定

| 项目 | 配置 |
|---|---|
| FPGA | `GW5A-LV25MG121NC1/I0`，Device B |
| CPU | Gowin GoRV32，RV32IMC，machine mode |
| CPU/UART/MTIME 时钟 | 50MHz |
| XIP 逻辑地址 | `0x80000000` |
| 固件物理 Flash 偏移 | `0x100000` |
| 片上 RAM | `0x40000000`，64KB |
| PLIC | `0xE4000000`，本最小 BSP 暂不使用 |
| CLINT | `0xE6000000` |
| MTIMECMP | `0xE6004000` |
| MTIME | `0xE600BFF8` |
| UART1 | `0xF0200000`，115200 8-N-1 |
| RT-Thread tick | 100Hz |

上述地址和 50MHz 时钟来自本机 Gowin GoRV32 SDK。链接沿用官方 `linker_xip.lds` 的模型：`.text/.rodata` 在 `0x80000000` 就地执行，启动代码只把 `.data` 初值复制到 `0x40000000`，随后清零 `.bss`；并非把完整镜像搬入 RAM。

生成的 IP 必须同时满足：

```verilog
`define ONCHIP_RAM_SIZE 14
`define FLASH_BURN_ADDR 24'h100000
`define UART1_SUPPORT
```

`scripts/build_firmware.ps1` 会在编译前检查这些宏及 IPC 配置；如果 GUI 配置回落到默认值，构建会立即失败。

## BSP 组成

- `startup/crt.S`：设置 `gp`/`sp`、关闭中断、复制 `.data`、清零 `.bss`、设置 `mtvec`，然后进入 RT-Thread。
- `linker/gorv32_xip.ld`：将 `.text/.rodata` 放在 XIP Flash，将 `.data/.bss` 放在 64KB SRAM，并在 RAM 顶部保留 2KB machine trap 栈。
- `bsp/board.c`：按照 Gowin SDK 的 UART 寄存器布局初始化 UART1；轮询收发；以 RV32 安全写序更新 MTIMECMP，并用绝对截止时间维持 100Hz tick。
- `bsp/trap.c`：处理 machine timer interrupt；未接管异常会打印 `mcause/mepc` 后停机。
- `bsp/interrupt_gcc.S`：基于 RT-Thread 4.1.1 E310 trap 入口的本地修正版，在使用 `t0/x5` 前先保存其被中断现场，避免 tick 中断随机破坏线程寄存器。
- `include/rtconfig.h`：启用静态内核、信号量和最小 FinSH/MSH；不启用 heap 和 device 框架；`RT_ALIGN_SIZE=16` 与上下文栈布局一致。
- `app/main.c`：打印板级配置并导出 `hello` MSH 命令。

移植使用 RT-Thread 官方 RISC-V common port 的 `cpuport.c` 与 `context_gcc.S`。trap 入口采用上述本地修正版，没有假设旧 PicoRV32 工程的 ABI 或上下文切换代码与 GoRV32 兼容。

## 构建

默认依赖：

- RT-Thread：`D:\RT-ThreadStudio\repo\Extract\RT-Thread_Source_Code\RT-Thread\4.1.1`
- RISC-V GCC：`D:\RT-ThreadStudio\repo\Extract\ToolChain_Support_Packages\KENDRYTE\RISC-V-GCC-KENDRYTE\8.3.0\bin`

在仓库根目录运行：

```powershell
& .\fpga_rtl\boards\tang_primer_25k_rtthread\firmware\scripts\build_firmware.ps1
```

也可通过 `-RtThreadRoot` 和 `-ToolchainBin` 指定其它安装位置。编译参数固定为 `-march=rv32imc -mabi=ilp32 -mcmodel=medany -fsigned-char`；`-fsigned-char` 是 FinSH 用 `-1` 表示“暂无输入”时的必要条件。

输出目录为 `build/xip`：

- `rtthread_gorv32.elf`：带调试信息的 ELF，入口为逻辑地址 `0x80000000`。
- `rtthread_gorv32.bin`：连续 raw binary；第 0 字节映射到 GoRV32 逻辑 XIP 地址 `0x80000000`，实际写入 W25Q64 的物理偏移是 `0x100000`。
- `rtthread_gorv32.hex`：使用逻辑地址 `0x80000000` 的 Intel HEX，仅用于检查或支持逻辑地址映射的工具；不能直接当作物理起点 `0x100000` 的 Flash HEX 烧录。
- `rtthread_gorv32.map`：链接映射。
- `rtthread_gorv32.lst`：带源代码的反汇编。

`rtthread_gorv32.bin` 不是 FPGA 位流。FPGA 位流是 `../impl/pnr/tang_primer_25k_rtthread.fs`；两者必须分别按正确 operation 和地址处理。

## 串口回归

连接 UART1 后，以 115200、8 data bits、no parity、1 stop bit、无硬件流控打开串口。shell 提示符为：

```text
msh >
```

`hello` 的输出为：

```text
hello from Tang Primer 25K / Gowin GoRV32
```

严格回归脚本会检查 RT-Thread 版本、`list` 内容、完整的 `tshell/tidle0` 行，以及所有响应中是否有非预期控制字符、非 ASCII 字符或替换字符。默认重复 20 轮 `ps`/`hello`：

```powershell
& .\fpga_rtl\boards\tang_primer_25k_rtthread\scripts\serial_shell_test.ps1 `
    -Port COM5 -BaudRate 115200 -TimeoutMs 8000
```

可用 `-StressRounds` 调整为 1 到 1000 轮。2026-08-27 的默认 20 轮实板结果为：

```text
SERIAL RT-THREAD SHELL PASS: COM5 @ 115200 8-N-1;PS_HELLO_ROUNDS=20
```

## 验证边界

- 当前 FPGA `.fs` 只验证了 SRAM 下载；断电后 FPGA 配置不保留，尚未验证将 FPGA 位流固化到配置 Flash 后的冷启动流程。
- RT-Thread 固件已写入外部 Flash 并校验，但本工程未验证掉电保持后的独立冷启动。
- UART1 当前为轮询式 console，PLIC 和 UART 中断未启用。
- 本结果验证最小静态 RT-Thread + FinSH/MSH，不代表完整 RT-Thread device/DFS/network 软件栈已移植。

详细构建、段布局、最终哈希和实板证据见 `BUILD_EVIDENCE.md`；板级实现结果见 `../RESULTS.md`。
