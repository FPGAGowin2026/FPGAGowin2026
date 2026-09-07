# RGMII RX 为 125 MHz；最终板级顶层必须改为真实系统时钟。
create_clock -name rgmii_rx_clk -period 8.000 -waveform {0.000 4.000} [get_ports {rgmii_rx_clk_i}]
create_generated_clock -name offline_system_clk -source [get_ports {rgmii_rx_clk_i}] -master_clock rgmii_rx_clk -divide_by 2 [get_nets {offline_system_clk_q}]

# robot_network/net_clk 由 Gowin PLL 原语自动派生；保留 vendor 模型的真实相位。

# 即使离线 wrapper 的两域共用时钟源，整帧 FIFO 仍只在同步器第一级收敛异步 Gray 指针。
set_false_path -to [get_regs {robot_network/control_cdc_fifo/wr_gray_rd_sync1_q_*}]
set_false_path -to [get_regs {robot_network/control_cdc_fifo/rd_gray_wr_sync1_q_*}]
set_false_path -to [get_regs {robot_network/tx_cdc_fifo/wr_gray_rd_sync1_q_*}]
set_false_path -to [get_regs {robot_network/tx_cdc_fifo/rd_gray_wr_sync1_q_*}]
set_false_path -to [get_regs {robot_network/rx_overflow_sync1_q_s0}]
set_false_path -to [get_regs {robot_network/rx_fifo_full_sync1_q_s0}]

# PHY 板级时序和 PCB 延迟未确定，因此不猜测 set_input_delay/set_output_delay。
