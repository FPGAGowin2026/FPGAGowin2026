# 示例默认使用 27 MHz 系统时钟；板级整合时按实际晶振修改。
create_clock -name system_clk -period 37.037 -waveform {0.000 18.519} [get_ports {clk_i}]
