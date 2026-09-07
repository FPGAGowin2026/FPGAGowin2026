# Tang Primer 25K 核心板板载 50 MHz 有源晶振。
create_clock -name system_clk -period 20.000 -waveform {0.000 10.000} [get_ports {clk_i}]
