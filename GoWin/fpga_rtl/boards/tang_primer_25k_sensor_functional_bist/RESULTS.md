# 验证结果

日期：2026-08-27

## 已完成

| 项目 | 结果 | 证据 |
|---|---|---|
| Icarus 编译与严格自检 | PASS，17/17 | `reports/simulation.log` |
| 片上激励活动位图 | `ACT=0x1FFFF` | 同上 |
| 核心有效事件位图 | `OBS=0x1FFFF` | 同上 |
| 精确结果位图 | `PASS=0x1FFFF` | 同上 |
| Gowin 综合 | PASS | `reports/gowin.log` |
| 布局布线与位流 | PASS | `impl/pnr/tang_primer_25k_sensor_functional_bist.fs` |
| 目标器件 | `GW5A-LV25MG121NC1/I0`，Device B | PnR 报告 |
| 时序 | 50 MHz 约束，setup 0、hold 0；Fmax 123.136 MHz | timing 报告 |
| 实板 SRAM 下载 | PASS，User Code `0x00002E54`，Status Code `0x70026020` | Gowin Programmer，`USB Debugger A` |
| 实板 UART | PASS，COM5 @ 115200 8-N-1，P/B/D 三条命令逐项匹配 | `reports/serial_debug.log` |

仿真唯一通过标记：

```text
PASS: Tang Primer 25K sensor functional BIST (17/17)
PASS=0x1ffff OBS=0x1ffff ACT=0x1ffff
```

实板 UART 通过标记：

```text
PONG;FUNC_BIST=READY
FUNC_BIST=PASS;MODULES=17/17;MODE=ONCHIP_STIMULUS
PASS=0x1FFFF;OBS=0x1FFFF;ACT=0x1FFFF;DONE=1
SERIAL FUNCTIONAL BIST PASS: COM5 @ 115200 8-N-1
```

## PnR 资源

| 资源 | 使用量 | 利用率 |
|---|---:|---:|
| Logic | 2640 / 23040 | 12% |
| Register | 1567 / 23280 | 7% |
| CLS | 1865 / 11520 | 17% |
| DSP | 1 / 28 | 4% |
| I/O Port | 4 / 86 | 5% |

位流 SHA-256：

```text
97A5378AFABAF573FB7E4EF8FCB95DCFE3AB51C90C4FCF2D25AEABA453472DAD
```

## 尚未由本工程证明的内容

- 没有外接任何真实传感器，因此没有真实数据、电气或物理环境证据。
- OV5640 项只验证 DVP capture；SCCB 配置/器件身份由其他工程验证。
- OV7725、MT9V03X、SCC8660 项只验证 DVP capture，不验证各自外部配置链路。
- TSL1401 使用片上数字 ADC 响应模型，不验证模拟 AO、外置 ADC 或采样建立时间。
- HC-SR04 的 5 V echo 必须在真实接线时做电平转换；本测试不涉及该电气路径。
