# 顶层参数 CLK_FREQ_HZ 默认为 27 MHz。
create_clock -name system_clk -period 37.037 -waveform {0.000 18.519} [get_ports {clk}]
