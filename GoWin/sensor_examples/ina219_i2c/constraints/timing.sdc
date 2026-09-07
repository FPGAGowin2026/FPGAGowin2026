# 启动与采样周期参数按 27 MHz 系统时钟设置。
create_clock -name system_clk -period 37.037 -waveform {0.000 18.519} [get_ports {clk}]
