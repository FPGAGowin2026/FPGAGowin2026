# 验证结果

日期：2026-08-27

## 已完成

| 项目 | 结果 | 证据 |
|---|---|---|
| wrapper 仿真 | PASS，POR 延迟与 UART 连通性 | `reports/simulation.log` |
| GoRV32 IP | PASS，64KB SRAM、UART1、Flash `0x100000` | `gorv32_config.vh`、`gowin_gorv32.ipc` |
| RT-Thread 固件构建 | PASS，RT-Thread 4.1.1，RV32IMC/ILP32 | `firmware/BUILD_EVIDENCE.md` |
| ELF/XIP 布局 | PASS，entry `0x80000000`，RAM `0x40000000` | `rtthread_gorv32.elf/.map` |
| Gowin 综合 | PASS，Gowin V1.9.12.03 | `reports/gowin.log` |
| 布局布线与位流 | PASS | `impl/pnr/tang_primer_25k_rtthread.fs` |
| 目标器件 | `GW5A-LV25MG121NC1/I0`，Device B | PnR 报告 |
| 时序 | 50MHz 约束，setup 0、hold 0；Fmax 50.604MHz | timing 报告 |
| FPGA SRAM 下载 | operation 2 PASS，User Code `0x16DF`，Status Code `0x70026020` | Gowin Programmer |
| RT 固件 Flash 写入 | operation 56，W25Q64 物理 `0x100000`，write/verify PASS | Gowin Programmer |
| RT-Thread shell | PASS，COM5 @ 115200 8-N-1 | `reports/serial_shell.log` |
| 串口压力回归 | PASS，`version/list` + 20/20 轮 `ps/hello`，无乱码或控制字符 | 同上 |

仿真通过标记：

```text
PASS: Tang Primer 25K RT-Thread wrapper POR/UART connectivity
```

最终串口通过标记：

```text
SERIAL RT-THREAD SHELL PASS: COM5 @ 115200 8-N-1;PS_HELLO_ROUNDS=20
```

代表性实测响应：

```text
msh >

version
 \ | /
- RT -     Thread Operating System
 / | \     4.1.1 build Aug 27 2026 03:51:34
 2006 - 2022 Copyright by RT-Thread team
msh >

ps
thread       pri  status      sp     stack size max used left tick  error
------------ ---  ------- ---------- ----------  ------  ---------- ---
tshell        20  running 0x00000100 0x00000800    28%   0x00000009 OK
tidle0        31  ready   0x000000a0 0x00000100    62%   0x00000017 OK
msh >

hello
hello from Tang Primer 25K / Gowin GoRV32
msh >
```

串口脚本统计为 20 个 `STRESS_ROUND=n/20 PASS`、20 个 `PASS=ps` 和 20 个 `PASS=hello`。每个响应都通过了完整行正则和字符级检查。

## 关键缺陷与闭环

修复前的短命令验收曾出现“看似可用”，但重复 `ps` 捕获到 `tidle0` 优先级 `31` 被控制字符 `0x05` 取代。根因是 RT-Thread 4.1.1 E310 trap 入口在保存 x5 之前执行 `li t0, 0x80`，而 `t0` 与 x5 是同一寄存器。

最终固件改用 `firmware/bsp/interrupt_gcc.S`：先保存 x5，再使用 t0 构造 MPIE 初值。结合 `RT_ALIGN_SIZE=16`、MTIMECMP 的 RV32 安全写序及绝对 tick 截止时间，重建、重刷后完成 20 轮严格回归，未再观察到字段损坏或非法字符。

## PnR 资源

| 资源 | 使用量 | 利用率 |
|---|---:|---:|
| Logic | 6967 / 23040 | 31% |
| Register | 3117 / 23280 | 14% |
| CLS | 4618 / 11520 | 41% |
| I/O Port | 10 / 86 | 12% |
| BSRAM | 39 / 56 | 70% |
| DSP | 4 / 28 | 15% |

时序报告分析 8151 条路径、11163 个 endpoint；setup/hold violated endpoint 都为 0，sys_clk TNS 均为 0.000。50MHz sys_clk 的实际 Fmax 为 50.604MHz。

## 最终产物

| 文件 | 字节数 | SHA256 |
|---|---:|---|
| `impl/pnr/tang_primer_25k_rtthread.fs` | 7142032 | `897BB4E6E7A6C749C85804494CBBF45C29068BAE8E527907FA6E0886810ED203` |
| `firmware/build/xip/rtthread_gorv32.bin` | 12048 | `3A4C7A1810664F88213F9EC5592243BE25A0A15BA55991465F26037B484596BA` |
| `firmware/build/xip/rtthread_gorv32.elf` | 207068 | `76EAA035E74F0077FC90004C403BF98E99951B68FFCF8ABA3837CDFE548225F2` |
| `firmware/build/xip/rtthread_gorv32.hex` | 33962 | `E371A4776EC248FA8711E6FC0C265B649208AEE698F88FD67162D93F0ECE8D57` |
| `firmware/build/xip/rtthread_gorv32.lst` | 258134 | `1F55E64F892EDCF7896D9AE43E63BD68BBFE8F83FC45A8F8397E9CF59597A5C1` |
| `firmware/build/xip/rtthread_gorv32.map` | 175763 | `A22FFB8145FA8978561204C60E0A7FD497F2CDE143C33AA04CA263B35F992161` |
| `reports/serial_shell.log` | 11523 | `6903B9E493E13D3CC3DFC8E2B1E25FF76957D028A5871FFE8ABEF649514C261A` |

`.fs` 是 FPGA 位流；`.bin` 是从物理 Flash `0x100000` 开始写入的 CPU raw payload。`.hex` 中的 `0x80000000` 是 GoRV32 逻辑 XIP 地址，不是 Programmer 的物理 Flash 起点。

## 尚未证明的内容

- FPGA `.fs` 只完成 SRAM 下载，未验证 FPGA 配置固化及掉电冷启动。
- RT 固件已写入 W25Q64 并 verify，未单独验证掉电保持后的冷启动。
- 未启用或验证 PLIC、UART 中断、heap、RT-Thread device 框架、DFS 和网络栈。
- 当前结果针对生成并核验过的 GoRV32 1.0 IP，不代表 PicoRV32 或其它 RISC-V CPU 可直接 ABI 兼容。
