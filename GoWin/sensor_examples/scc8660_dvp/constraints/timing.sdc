create_clock -name system_clk -period 37.037 -waveform {0.000 18.519} [get_ports {clk}]
create_clock -name camera_pclk -period 40.000 -waveform {0.000 20.000} [get_ports {camera_pclk}]
