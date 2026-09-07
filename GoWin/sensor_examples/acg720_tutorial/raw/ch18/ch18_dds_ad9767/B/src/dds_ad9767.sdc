//Copyright (C)2014-2023 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//GOWIN Version: 1.9.9 Beta-3
//Created Time: 2023-09-13 10:34:56
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk_50M}]
create_clock -name clk125M -period 8 -waveform {0 4} [get_ports {Clk0}]
