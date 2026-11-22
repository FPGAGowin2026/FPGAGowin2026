# Clocks. Camera remains on the board's B1 GPIO; clock insertion is propagated by STA.
create_clock -name clk50 -period 20 [get_ports {clk50m}]
# DVP acceptance envelope: 100 MHz, 50% duty (both half-periods >= 5 ns).
create_clock -name camera_pclk -period 10 -waveform {0 5} [get_ports {camera_pclk}]
# RTL8211F 1G minimum RX period; both 45/55% duty corners.
create_clock -name eth_rx_clk -period 7.2 -waveform {0 3.96} [get_ports {eth_rx_clk}]
create_clock -name eth_rx_clk_duty45 -period 7.2 -waveform {0 3.24} -add [get_ports {eth_rx_clk}]
create_clock -name ddr_ref -period 20 [get_pins {memory_pll/u_pll/PLLA_inst/CLKOUT0}]
create_clock -name ddr_mem -period 5 [get_pins {memory_pll/u_pll/PLLA_inst/CLKOUT1}]
create_generated_clock -name ddr_ui -source [get_pins {memory_pll/u_pll/PLLA_inst/CLKOUT1}] -master_clock ddr_mem -divide_by 4 [get_nets {ui_clk}]
create_clock -name camera_xclk -period 40 [get_pins {video_pll/PLLA_inst/CLKOUT2}]
create_clock -name eth_tx -period 8 [get_pins {video_pll/PLLA_inst/CLKOUT0}]
# DS981: budget up to 400 ps cycle/period jitter for the new >=100 MHz PLL clock.
set_clock_uncertainty -setup 0.4 -to [get_clocks {eth_tx}]

set_clock_groups -asynchronous -group [get_clocks {clk50 ddr_ref}] -group [get_clocks {ddr_mem}] -group [get_clocks {ddr_ui}] -group [get_clocks {camera_pclk}] -group [get_clocks {camera_xclk eth_tx}] -group [get_clocks {eth_rx_clk eth_rx_clk_duty45}]

# Two alternative duty-cycle models for the same PHY clock, never concurrent.
set_clock_groups -exclusive -group [get_clocks {eth_rx_clk}] -group [get_clocks {eth_rx_clk_duty45}]

# OV5640 0x4740[5]=1 launches DVP data on PCLK falling edges.
# Engineering acceptance bounds, NOT an OmniVision guaranteed tCO specification:
# total sensor + module/cable/PCB data-minus-clock delay = [-0.5, 3.0] ns.
# FPGA IODELAY(64) and B1 clock routing remain inside the device timing model.
set_input_delay -clock camera_pclk -clock_fall -min -0.5 [get_ports {camera_data* camera_href camera_vsync}]
set_input_delay -clock camera_pclk -clock_fall -max 3.0 [get_ports {camera_data* camera_href camera_vsync}]

# RTL8211F RXID guarantees >=1.2 ns setup/hold at each edge; PCB skew budget +/-0.2 ns.
# 7.2 ns / 55%: high=3.96, low=3.24; min=1.0, max=H/L-1.2+0.2 ns.
set_input_delay -clock eth_rx_clk -min 1.0 [get_ports {eth_rxd* eth_rx_ctl}]
set_input_delay -clock eth_rx_clk -max 2.96 [get_ports {eth_rxd* eth_rx_ctl}]
set_input_delay -clock eth_rx_clk -clock_fall -min 1.0 -add_delay [get_ports {eth_rxd* eth_rx_ctl}]
set_input_delay -clock eth_rx_clk -clock_fall -max 2.24 -add_delay [get_ports {eth_rxd* eth_rx_ctl}]
# Opposite 45% corner, exclusive with the first model of the same physical clock.
set_input_delay -clock eth_rx_clk_duty45 -min 1.0 -add_delay [get_ports {eth_rxd* eth_rx_ctl}]
set_input_delay -clock eth_rx_clk_duty45 -max 2.24 -add_delay [get_ports {eth_rxd* eth_rx_ctl}]
set_input_delay -clock eth_rx_clk_duty45 -clock_fall -min 1.0 -add_delay [get_ports {eth_rxd* eth_rx_ctl}]
set_input_delay -clock eth_rx_clk_duty45 -clock_fall -max 2.96 -add_delay [get_ports {eth_rxd* eth_rx_ctl}]

# TXID: FPGA clock/data share the same 125 MHz ODDR clock, PHY inserts its delay.
# These delays expose all output data paths and set a one-period (8 ns) pad
# insertion deadline: nearest ideal capture half-edge is 4 ns, minus (-4)=8 ns.
# They are NOT a PHY setup/hold model. Actual TX compliance is same-corner,
# same-launch-edge data-minus-clock pad skew, checked from full routed SDF.
# RGMII source skew +/-0.5 ns minus PCB +/-0.2 ns => FPGA pad skew +/-0.3 ns.
set_output_delay -clock eth_tx -max -4 [get_ports {eth_txd* eth_tx_en eth_tx_clk}]
set_output_delay -clock eth_tx -min 0 [get_ports {eth_txd* eth_tx_en eth_tx_clk}]
set_output_delay -clock eth_tx -clock_fall -max -4 -add_delay [get_ports {eth_txd* eth_tx_en eth_tx_clk}]
set_output_delay -clock eth_tx -clock_fall -min 0 -add_delay [get_ports {eth_txd* eth_tx_en eth_tx_clk}]
# Preserve global critical paths and explicit per-interface setup/hold coverage.
report_timing -setup -max_paths 25 -max_common_paths 1
report_timing -hold -max_paths 25 -max_common_paths 1
report_timing -setup -from [get_ports {camera_data* camera_href camera_vsync}] -max_paths 30 -max_common_paths 3
report_timing -hold -from [get_ports {camera_data* camera_href camera_vsync}] -max_paths 30 -max_common_paths 3
report_timing -setup -from [get_ports {eth_rxd* eth_rx_ctl}] -max_paths 40 -max_common_paths 8
report_timing -hold -from [get_ports {eth_rxd* eth_rx_ctl}] -max_paths 40 -max_common_paths 8
report_timing -setup -to [get_ports {eth_txd* eth_tx_en}] -max_paths 20 -max_common_paths 4
report_timing -hold -to [get_ports {eth_txd* eth_tx_en}] -max_paths 20 -max_common_paths 4
report_timing -setup -to [get_ports {eth_tx_clk}] -max_paths 8 -max_common_paths 8
report_timing -hold -to [get_ports {eth_tx_clk}] -max_paths 8 -max_common_paths 8
