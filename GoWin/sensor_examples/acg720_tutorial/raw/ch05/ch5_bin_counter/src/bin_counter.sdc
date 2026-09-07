//Copyright (C)2014-2023 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//GOWIN Version: 1.9.9 Beta-1
//Created Time: 2023-06-26 10:23:49
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]
