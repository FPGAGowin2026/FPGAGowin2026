# 验证结果

验证日期：2026-08-27。离线门禁完成后又执行了 JTAG SRAM 下载与 COM5 双向 UART 实测；没有写启动 Flash。

## Icarus 自检

命令：

```powershell
pwsh -File .\Scripts\run_sim.ps1
```

结果：PASS。关键输出：

```text
PASS: Tang Primer 25K sensor bus no-device diagnostic
I2C_ERR=0xf SPI_TXN=0xff SPI_REJECT=0xff
```

testbench 还独立确认了：四组 SCL/SDA 均出现过低电平；SPI 的八个 `sensor_ok` 均保持 0；UART 横幅以及 `P/B/D/?` 响应逐字节一致。完整日志：[`reports/simulation.log`](reports/simulation.log)。

## Gowin 综合、布局布线与位流

命令：

```powershell
pwsh -File .\Scripts\run_gowin.ps1
```

结果：PASS。

| 项目 | 结果 |
|---|---:|
| 工具 | Gowin V1.9.12.03 x64 |
| 器件 | GW5A-LV25MG121NC1/I0，Device B |
| Logic | 4260 / 23040（19%） |
| Register | 2621 / 23280（12%） |
| I/O Port | 12 / 86（14%） |
| BSRAM | 2 / 56（4%） |
| 约束频率 | 50.000 MHz |
| 实际 Fmax | 119.221 MHz |
| Setup 违规端点 | 0 |
| Hold 违规端点 | 0 |

位流：[`impl/pnr/tang_primer_25k_sensor_bus_debug.fs`](impl/pnr/tang_primer_25k_sensor_bus_debug.fs)

SHA-256：

```text
132F4BEC380490198D92D75F39BB71788E51C37C150D0F1B73F99C3DCFB6E2DB
```

构建日志：[`reports/gowin.log`](reports/gowin.log)。器件、引脚、资源和时序的原始证据分别保留在 `impl/pnr/*.rpt.txt`、`*.pin.html` 与 `*_tr_content.html`。

## Tang Primer 25K 实板 UART 诊断

- Programmer 通过 `USB Debugger A` 扫描到 ID `0x0001281B`，目标识别为 `GW5A-25B`。
- 对上述 SHA-256 位流执行 `SRAM Program`（operation 2）成功；User Code `0x0000D84B`，Status Code `0x70026020`。
- COM5、115200 8-N-1 下，`P/B/D` 三条命令均返回预期内容：

```text
PONG;BUS_DEBUG=READY
BUS_BIST=PASS;I2C=4/4;SPI=8/8;MODE=NO_DEVICE
I2C_ERR=0xF;SPI_TXN=0xFF;SPI_REJECT=0xFF
```

完整证据：[`reports/serial_debug.log`](reports/serial_debug.log)。重新配置瞬间捕获到一个前导 NUL，随后横幅与全部命令响应完整，脚本退出码为 0。

## 尚未验证

- 4 组 I2C 已在真实 FPGA 封装 GPIO、内部上拉且外部无设备的条件下得到地址 NACK；没有验证接入实物后的 ACK、寄存器值或数据物理正确性。
- SPI 是芯片内部全 1 故障注入，不是外部 SPI 管脚或真实传感器验证。
- 真实传感器数据与电气连接不属于本工程的 PASS 含义。
