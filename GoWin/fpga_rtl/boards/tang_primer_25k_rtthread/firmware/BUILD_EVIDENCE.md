# RT-Thread GoRV32 固件构建与实板证据

记录日期：2026-08-27（Asia/Shanghai）

## 结论

最终 trap 修复版固件已在正确 GoRV32 IP 配置下完成编译、链接和静态布局核验；raw XIP 固件已写入并校验 W25Q64 物理偏移 `0x100000`。配套 FPGA 位流下载到 SRAM 后，COM5 @ 115200 8-N-1 的 RT-Thread 4.1.1 FinSH/MSH 已通过 `version`、`list` 和连续 20 轮 `ps`/`hello` 严格回归。

本次最终固件不再使用会破坏 `t0/x5` 被中断现场的 RT-Thread 4.1.1 原始 E310 trap 入口，而由 `bsp/interrupt_gcc.S` 提供已验证的本地修正版。

## 输入版本

- GoRV32 SDK：`Gowin_GoRV32_SDK_V1.0`
- SDK ZIP SHA256：`4D9DDDD4F2F1464141D1900B07F21DC0A704E6A7B43B9F7E9E48570EAD065280`
- RT-Thread：4.1.1，本机 RT-Thread Studio 源码包
- 编译器：xPack GNU RISC-V Embedded GCC 8.3.0
- ABI 参数：`-march=rv32imc -mabi=ilp32 -mcmodel=medany -mstrict-align -fsigned-char`

## IP 配置核验

生成器成功结束后，磁盘文件同时包含：

```text
gorv32_config.vh:3  `define ONCHIP_RAM_SIZE 14
gorv32_config.vh:4  `define FLASH_BURN_ADDR 24'h100000
gorv32_config.vh:5  `define UART1_SUPPORT
gowin_gorv32.ipc:10 Flash_Burn_Address=100000
gowin_gorv32.ipc:16 Enable_UART1=true
gowin_gorv32.ipc:17 Enable_UART2=false
gowin_gorv32.ipc:18 SRAM_Size=64KB
```

生成的 `gowin_gorv32.v` 顶层包含 `UART1_CTSN`、`UART1_RXD`、`UART1_RTSN`、`UART1_TXD` 和 QSPI 端口。

## 构建与静态自检

构建命令：

```powershell
& .\fpga_rtl\boards\tang_primer_25k_rtthread\firmware\scripts\build_firmware.ps1
```

脚本会先拒绝错误的 SRAM/UART/Flash IP 配置，再检查 ELF32、RISC-V、RVC soft-float ABI、`0x80000000` 入口、XIP/RAM LOAD 段和 `0x40010000` 栈顶。最终 ELF 的未解析符号列表为空。

编译警告来自未修改的 RT-Thread 4.1.1/FinSH 源码，包括 FinSH 命令函数指针转换、`cpuport.c` signedness、`kservice.c` fallthrough 和 `thread.c` 静态分析提示；没有 BSP 编译错误、链接错误或未定义符号。

### 段尺寸

```text
section    size   address
.text      9972   0x80000000
.rodata    2020   0x800026f4
.data        56   0x40000000
.bss       4432   0x40000040
```

- raw Flash 初始化镜像：12048 bytes。
- RAM 有效静态数据：4488 bytes；因 `.bss` 对齐，实际地址跨度为 `0x40000000..0x40001190`，即 4496 bytes。
- machine trap 栈：额外保留 2048 bytes，范围 `0x4000F800..0x40010000`。
- 链接脚本断言保证静态区不会与 machine trap 栈重叠。

### ELF 头和装载布局

```text
Class:               ELF32
Machine:             RISC-V
Entry point address: 0x80000000
Flags:               0x1, RVC, soft-float ABI

LOAD  offset 0x001000  VMA 0x80000000  PAddr 0x80000000  FileSiz 0x02ed8  MemSiz 0x02ed8  R E
LOAD  offset 0x004000  VMA 0x40000000  PAddr 0x80002ed8  FileSiz 0x00038  MemSiz 0x00038  RW
LOAD  offset 0x004040  VMA 0x40000040  PAddr 0x80002f10  FileSiz 0x00000  MemSiz 0x01150  RW
```

第一段是 `0x2ED8` 字节的 `.text/.rodata` XIP 内容；第二段是 56 字节 `.data` 初值，因此 raw binary 总计 12048 字节。启动代码把 `.data` 从逻辑 LMA `0x80002ED8` 复制到 SRAM `0x40000000`，随后清零第三段 `.bss`。这与 Gowin SDK 官方 `linker_xip.lds`/`crt.S` 模型一致；完整代码镜像不会搬入 SRAM。

关键符号：

```text
0x40000000 __data_start
0x40000038 __data_end
0x40000040 __bss_start
0x40001190 __bss_end
0x4000f800 __machine_stack_bottom
0x40010000 _sp
0x80000000 crtStart
0x8000006c trap_entry
0x8000014c entry
0x80000520 rtthread_startup
0x80001642 finsh_system_init
0x80002674 __fsymtab_start
0x800026e0 __fsymtab_end
```

FinSH 符号表包含 `hello`、`help`、`ps`、`list`、`list_timer`、`list_sem`、`list_thread`、`version` 和 `clear`。

## trap 入口修复

RT-Thread 4.1.1 的 `libcpu/risc-v/e310/interrupt_gcc.S` 在保存 x5 之前执行 `li t0, 0x80`，而 `t0` 就是 x5，因此每次 machine timer interrupt 都可能把线程原始 x5 覆盖为 `0x80`。短命令偶尔看似正常，但修复前的严格 `ps` 回归已捕获到字段缺字和控制字符，例如 `tidle0` 优先级 `31` 被 `0x05` 取代。

本工程的 `bsp/interrupt_gcc.S` 保持 RT-Thread 原有上下文格式，但在构造 slot 2 的 MPIE 初值之前先执行：

```asm
STORE x5, 5 * REGBYTES(sp)
li    t0, 0x80
STORE t0, 2 * REGBYTES(sp)
```

构建脚本明确编译此本地文件，不再编译上游 E310 版本。最终固件还使用 `RT_ALIGN_SIZE=16`、RV32 规范的 MTIMECMP 写序和绝对 tick 截止时间。

## 最终固件产物

| 文件 | 字节数 | SHA256 |
|---|---:|---|
| `rtthread_gorv32.bin` | 12048 | `3A4C7A1810664F88213F9EC5592243BE25A0A15BA55991465F26037B484596BA` |
| `rtthread_gorv32.elf` | 207068 | `76EAA035E74F0077FC90004C403BF98E99951B68FFCF8ABA3837CDFE548225F2` |
| `rtthread_gorv32.hex` | 33962 | `E371A4776EC248FA8711E6FC0C265B649208AEE698F88FD67162D93F0ECE8D57` |
| `rtthread_gorv32.lst` | 258134 | `1F55E64F892EDCF7896D9AE43E63BD68BBFE8F83FC45A8F8397E9CF59597A5C1` |
| `rtthread_gorv32.map` | 175763 | `A22FFB8145FA8978561204C60E0A7FD497F2CDE143C33AA04CA263B35F992161` |

`rtthread_gorv32.bin` 是 raw XIP 固件，Programmer 的目标物理偏移必须为 `0x100000`。其逻辑执行地址仍为 `0x80000000`。Intel HEX 的扩展线性地址同样描述逻辑地址 `0x80000000`，不能把 `.hex` 当作已经以物理 `0x100000` 起始的 Flash 文件。

## 下载与实板回归

| 对象 | Programmer 操作 | 结果 |
|---|---|---|
| `impl/pnr/tang_primer_25k_rtthread.fs` | operation 2，FPGA SRAM | PASS；User Code `0x16DF`，Status Code `0x70026020` |
| `firmware/build/xip/rtthread_gorv32.bin` | operation 56，W25Q64 物理 `0x100000` | 写入并 verify PASS |

最终 FPGA 位流：

```text
size:   7142032 bytes
SHA256: 897BB4E6E7A6C749C85804494CBBF45C29068BAE8E527907FA6E0886810ED203
```

实板回归命令：

```powershell
& .\fpga_rtl\boards\tang_primer_25k_rtthread\scripts\serial_shell_test.ps1 `
    -Port COM5 -BaudRate 115200 -TimeoutMs 8000
```

脚本先检查 `version` 和 `list`，再执行 20 轮 `ps`/`hello`。每轮要求 `ps` 标题及 `tshell`/`tidle0` 行完整、`hello` 精确整行匹配，并拒绝除 CR/LF 外的控制字符、非 ASCII 字符和 `?` 替换字符。统计结果：20/20 round PASS、20/20 `ps` PASS、20/20 `hello` PASS。

最终通过标记：

```text
SERIAL RT-THREAD SHELL PASS: COM5 @ 115200 8-N-1;PS_HELLO_ROUNDS=20
```

串口证据：`../reports/serial_shell.log`，11523 bytes，SHA256 `6903B9E493E13D3CC3DFC8E2B1E25FF76957D028A5871FFE8ABEF649514C261A`。

## 结论边界

- FPGA 位流只完成 SRAM 下载，尚未验证将 `.fs` 固化到配置 Flash 后的掉电冷启动。
- 固件已写入 W25Q64 并 verify，但尚未单独验证掉电保持后的冷启动。
- 当前 BSP 仅验证轮询 UART console、machine timer tick、静态线程调度和最小 FinSH/MSH；PLIC、UART 中断、heap、device、DFS 与网络栈不在本次范围内。
