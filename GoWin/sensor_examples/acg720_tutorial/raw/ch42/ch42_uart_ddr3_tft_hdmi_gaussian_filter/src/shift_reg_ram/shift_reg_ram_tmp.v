//Copyright (C)2014-2024 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Template file for instantiation
//Tool Version: V1.9.10.03 (64-bit)
//Part Number: GW5AT-LV138PG484AC1/I0
//Device: GW5AT-138
//Device Version: B
//Created Time: Thu Dec 26 11:08:34 2024

//Change the instance name and port connections to the signal names
//--------Copy here to design--------

	shift_reg_ram your_instance_name(
		.clk(clk), //input clk
		.Reset(Reset), //input Reset
		.Din(Din), //input [7:0] Din
		.Q(Q) //output [7:0] Q
	);

//--------Copy end-------------------
