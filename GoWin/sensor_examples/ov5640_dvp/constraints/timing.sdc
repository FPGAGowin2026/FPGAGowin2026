# 本例 clk_i 直接输出为 24 MHz XCLK。PCLK 由传感器内部 PLL/分频产生，
# 不能假定等于 XCLK；按 OV5640 规格的 DVP 上限 96 MHz 保守约束内部采集域。
# data/href/vsync 相对 PCLK 的板外 delay 依具体模组与走线而变，必须在板级
# 实测后补 set_input_delay；本独立例程不伪造该数值。
create_clock -name camera_xclk_source -period 41.667 -waveform {0.000 20.833} [get_ports {clk_i}]
create_clock -name camera_pclk -period 10.417 -waveform {0.000 5.208} [get_ports {camera_pclk_i}]
