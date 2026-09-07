create_clock -name sys_clk -period 20.000 -waveform {0.000 10.000} [get_ports {clk_i}]
create_clock -name qspi_clk -period 40.000 -waveform {0.000 20.000} [get_ports {flash_qspi_clk}]
set_clock_groups -exclusive -group [get_clocks {sys_clk}] -group [get_clocks {qspi_clk}]
set_false_path -from [get_ports {reset_i}]
