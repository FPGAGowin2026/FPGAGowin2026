# Tang Primer 25K 实测结果

验证日期：2026-08-27。测试板只用于可移植逻辑自检；没有把 ACG720 的 DDR3、摄像头、显示器、以太网 PHY、音频或 FX2 外设接到 25K。

## 结果

| 门禁 | 结果 |
|---|---|
| Icarus 自检式仿真 | PASS，收到完整动态报告，日志 `sim/run.log` |
| Gowin 综合/P&R/bitstream | PASS，V1.9.12.03，目标 `GW5A-LV25MG121NC1/I0` |
| STA | PASS，50 MHz 约束，Fmax 175.773 MHz，最差 setup slack +14.311 ns，setup/hold 违例 0/0 |
| SRAM 下载 | PASS，JTAG ID `0x0001281B`，User Code `0x000028FE`，Status `0x70026020` |
| COM5 UART | PASS，115200 8N1，清空 FTDI 缓冲后最新一次在 193 ms 内匹配 8 条完整报告（门限至少 2 条） |

最终位流：

```text
impl/pnr/acg720_tutorial_25k_bist.fs
SHA-256 7c858cb651ef3aa3c632dba3757b3bd7bcf1ae1bcfb814ed0a1de84508e16046
```

资源：Logic 584/23040（3%），Register 266/23280（2%），CLS 353/11520（4%），I/O 4/86（5%）。

实收串口行：

```text
ACG720_TUTORIAL_25K B=1 P=1 G=1 C=1 D=1 U=1 ALL=1 S=B1E6
```

字段对应：

- B：第 5 章 `bin_counter`
- P：第 19 章 `pwm_gen`
- G：第 39 章 `rgb2gray`
- C / S：第 53 章 `checksum` 的 PASS 和原始值 `0xB1E6`
- D：第 44 章 `DVP_Capture`，合成 11 帧后检查像素 `0x1234`
- U：第 6/7 章 `uart_byte_tx` / `uart_byte_rx` 片上回环

证据文件：`gowin_build.log`、`hardware_program.log`、`hardware_serial.log`、`impl/pnr/*.rpt.*` 与 `impl/pnr/*tr_content.html`。

SRAM Program 不修改启动 Flash，断电后该配置消失。
