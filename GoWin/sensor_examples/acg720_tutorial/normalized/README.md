# 规范化补充工程

这里仅补齐原教程源码包缺失或顶层设置不明确的工程入口，不修改厂商/教程原始 HDL。

- `ch44_dvp_capture`：第 44 章 DVP 8 位数据采集与 RGB565 像素组帧，顶层 `DVP_Capture`。
- `ch44_camera_init`：第 44 章 OV5640 SCCB/I²C 初始化，顶层 `camera_init`。
- `ch12_key_led_tb_compat`：第 12 章原 testbench 把 `key_led_top` 写成了 `key_led`；该兼容别名只修复仿真入口，不改原 HDL。
- `ch48_udp_pll_compat`：第 48 章原 PLL 的 125 MHz/IDIV=1 在 Gowin 1.9.12 中超出 PFD 范围；改为 IDIV=2、MDIV=14，保持 125 MHz 输出，并恢复 CPU/SSPI 配置脚复用。
- `ch54_udp_send_config_pin_compat`：第 54 章不改 RTL，只恢复 ACG720 原工程把 CPU/SSPI 配置脚作为普通 GPIO 的选项。
- `ch56_eth_udp_rx_pll_compat`：第 56 章 125 MHz PLL 参数和配置脚兼容工程。
- `ch61_eth_udp_ddr3_tft_pll_compat`：第 61 章 PLL/配置脚兼容工程。
- `ch62_ad7606_ddr3_rgmii_pll_compat`：第 62 章 PLL/配置脚兼容工程。
- `ch63_acm1030_ddr3_rgmii_pll_compat`：第 63 章 PLL/配置脚兼容工程。

第 44 章两个工程只用于综合检查，因为原章节没有提供板级顶层和 CST。第 48/54/56/61/62/63 章兼容工程均面向 `GW5AT-LV138PG484AC1/I0` 并已生成位流；其中第 56/61/62/63 章仍有时序违例，详见 `../reports/compatibility_pnr_results.md`，不能直接视为可交付固件。
