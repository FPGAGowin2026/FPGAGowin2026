# 参数的实际时间基准按 27 MHz 系统时钟设置。
create_clock -name system_clk -period 37.037 -waveform {0.000 18.519} [get_ports {clk}]
