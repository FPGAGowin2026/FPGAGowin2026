//Copyright (C)2014-2025 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//Tool Version: V1.9.11 (64-bit) 
//Created Time: 2025-03-27 18:14:47
create_clock -name ACM -period 20 -waveform {0 10} [get_ports {clk50m}] -add
