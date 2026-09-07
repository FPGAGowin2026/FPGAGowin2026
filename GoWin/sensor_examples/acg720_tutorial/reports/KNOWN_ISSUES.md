# 教程原始资料问题与边界

以下均来自对提取副本的复现，不修改 D 盘教程原件。

1. **第 12 章 testbench 顶层名不一致**：`key_led_tb.v` 实例化 `key_led`，实际设计模块名为 `key_led_top`。`normalized/ch12_key_led_tb_compat` 提供同端口别名，原 testbench 在该别名下可编译并运行。
2. **第 15/16 章自动顶层是 testbench**：原工程会自动选择 `ram_tb` / `fifo_ip_tb`；实际核心顶层 `Gowin_SP` / `fifo_top` 已用 `semantic_top_results.csv` 指定重跑并通过。
3. **第 36 章器件不一致**：`ddr3_1v4_hs` 指向 `GW5AST-LV138FPG676AES`，不是比赛板 `GW5AT-LV138PG484AC1/I0`。必须在最终器件上重新生成 DDR IP，不能只改器件字符串。
4. **第 37 章无 CST**：该目录是二端口 DDR3 核心示例，不是完整板级顶层；综合通过，但 P&R 报 PR1085（DQ/DQS 无法自动放到合法 bank）。在没有官方 DDR 引脚映射时不猜 pin。
5. **第 44 章只有散装源码**：原包没有 `.gprj`。已补 `DVP_Capture` 和 `camera_init` 两个 GW5AT-138B 规范化综合工程，均通过；没有伪造板级 CST。
6. **第 51 章有两个独立数据方向**：原工程同时包含 `gmii_to_rgmii` 与 `rgmii_to_gmii`，自动顶层会落到 `Gowin_PLL`。两个转换顶层已分别指定综合并通过。
7. **第 66 章错误启用 testbench**：自动综合 `usb_stream_out_tb` 会因混用 blocking/non-blocking assignment 报 EX3930；指定硬件顶层 `usb_stream_out` 后通过。
8. **Icarus 与 Gowin 加密包装器**：第 16、37、55、60、66 章部分 testbench 依赖 `` `pragma protect`` 的 Gowin IP 文件，Icarus 无法解析；同一设计用 Gowin EDA 重新综合可通过。不能把第三方仿真器解析失败说成 RTL 功能失败。
9. **第 64 章不是 FPGA HDL 工程**：内容是 Cypress FX2 固件、驱动、PC 工具和图像素材。它已归档，但不计入 FPGA 工程通过数。
10. **125 MHz 输入 PLL 与 Gowin 1.9.12 PFD 规则冲突**：第 48、56、61、62、63 章各有一个工程使用 `FCLKIN=125 MHz, IDIV=1, MDIV=7, ODIV=7`，新版工具报 PA2078（PFD=125 MHz 超出 19–81.25 MHz）。规范化副本改为 `IDIV=2, MDIV=14, ODIV=7`，输出仍为 125 MHz；五个兼容工程均完成 P&R 和位流生成，原始源码未改。
11. **配置脚复用选项不会从 `.gprj` 单独恢复**：第 48、54、56、61、62、63 章的 ACG720 板级约束使用 CPU/SSPI 复用脚。命令行只打开 `.gprj` 时默认禁用复用，会报 PR2017/PR2028；兼容构建显式设置 `-use_cpu_as_gpio 1` 与 `-use_sspi_as_gpio 1`。第 54 章不需要改 RTL，只补回这两个工程选项即可生成位流。
12. **位流生成不等于时序收敛**：当前工具重跑的 68 个原始 P&R 成功工程中，40 个 setup/hold 违例端点均为 0，28 个仍有至少一个违例端点；六个兼容工程中 2 个时序干净、4 个仍有违例。逐工程端点数见 `pnr_timing_audit.csv`，不能把 “P&R PASS” 表述为全部时序闭合。

外设边界：原工程 P&R 通过只证明其原 CST/器件可实现，不证明本次实际接入了 DDR3、OV5640、TFT/HDMI、RGMII PHY、ES8388、ADC/DAC 或 FX2。当前实物证据仅来自 Tang Primer 25K 上的片上合成激励与 USB-UART 回报。
