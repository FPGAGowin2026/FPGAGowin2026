//Copyright (C)2014-2023 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//GOWIN Version: 1.9.9 Beta-4
//Created Time: 2023-09-13 18:00:32
create_clock -name clk_50m -period 20 -waveform {0 10} [get_ports {clk}]
