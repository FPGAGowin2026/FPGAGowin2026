# ACG720 教程模块 Tang Primer 25K 自检

该工程直接引用教程原始 HDL，在当前连接的 `GW5A-LV25MG121NC1/I0` 测试板上做片上激励与 UART 回报。25K 只用于验证可移植逻辑，不替代最终 ACG720-138K 外设联调。

覆盖：

- 第 5 章 `bin_counter`
- 第 6/7 章 `uart_byte_tx` / `uart_byte_rx`（片上回环，同时驱动 USB-UART）
- 第 19 章 `pwm_gen`
- 第 39 章 `rgb2gray` 的公式路径
- 第 44 章 `DVP_Capture`（合成帧/像素输入）
- 第 53 章 `checksum`

串口参数为 115200、8N1。复位后先输出一个字符 `U` 作为 UART 回环挑战，随后每秒输出：

```text
ACG720_TUTORIAL_25K B=1 P=1 G=1 C=1 D=1 U=1 ALL=1 S=B1E6
```

其中 B/P/G/C/D/U 分别对应上述计数器、PWM、灰度、校验和、DVP、UART 测试，`S=B1E6` 是第 53 章模块实际输出的原始 16 位校验和值。
