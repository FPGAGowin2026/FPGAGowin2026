# 逻辑集成顶层时钟：RGMII RX 125 MHz，独立系统域 100 MHz。
create_clock -name rgmii_rx_clk -period 8.000 -waveform {0.000 4.000} [get_ports {rgmii_rx_clk_i}]
create_clock -name system_clk -period 10.000 -waveform {0.000 5.000} [get_ports {system_clk_i}]

# net_clk 由 Gowin PLL 原语自动派生；工具报告为 125 MHz，rise/fall=5/1 ns。
# 不用手写 generated clock 覆盖 vendor 模型中的相位参数。

# 只对明确的两级同步器第一级取消同步时序分析；不屏蔽其他跨域路径。
set_false_path -to [get_regs {control_cdc_fifo/wr_gray_rd_sync1_q_*}]
set_false_path -to [get_regs {control_cdc_fifo/rd_gray_wr_sync1_q_*}]
set_false_path -to [get_regs {tx_cdc_fifo/wr_gray_rd_sync1_q_*}]
set_false_path -to [get_regs {tx_cdc_fifo/rd_gray_wr_sync1_q_*}]
set_false_path -to [get_regs {rx_overflow_sync1_q_s0}]
set_false_path -to [get_regs {rx_fifo_full_sync1_q_s0}]
