# 27 MHz 控制时钟；TSL CLK 是寄存器输出，不是独立时钟域。
create_clock -name system_clk -period 37.037 -waveform {0.000 18.519} [get_ports {clk}]
