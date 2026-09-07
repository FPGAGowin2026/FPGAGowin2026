# ACG720-138K 迁移构建结果

验证日期：2026-08-27。本工程把已在 25K 实板通过的同一教程模块 BIST 迁到最终比赛器件 `GW5AT-LV138PG484AC1/I0`。

| 门禁 | 结果 |
|---|---|
| Gowin 综合 | PASS，顶层 `acg720_tutorial_138k_bist_top` |
| P&R/bitstream | PASS，ACG720 教程引脚 Y18/F15/J21/M15 |
| STA | PASS，50 MHz 约束，Fmax 160.863 MHz，最差 setup slack +13.784 ns，setup/hold 违例 0/0 |
| 138K 实板 UART | 未执行；当前连接的是 25K 测试板 |

最终位流：

```text
impl/pnr/acg720_tutorial_138k_bist.fs
SHA-256 65cf059311762efb9b5e9ffeef065e05e4dd5d9ca30b662d76dbcdcc0c12c31a
```

资源：Logic 574/138240（<1%），Register 266/139095（<1%），CLS 352/69120（<1%），I/O 4/297（2%）。

这证明 BIST 与选中的教程模块能在最终器件上完成实现和时序收敛；不证明摄像头、DDR3、显示、网口、音频、ADC/DAC 或 USB2.0 外设已经在 138K 板上实测。
