//Copyright (C)2014-2023 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Template file for instantiation
//GOWIN Version: V1.9.9 Beta-3
//Part Number: GW5A-LV25UG324ES
//Device: GW5A-25
//Device Version: A
//Created Time: Fri Sep 08 15:24:38 2023

//Change the instance name and port connections to the signal names
//--------Copy here to design--------

    rom_square your_instance_name(
        .dout(dout_o), //output [13:0] dout
        .clk(clk_i), //input clk
        .oce(oce_i), //input oce
        .ce(ce_i), //input ce
        .reset(reset_i), //input reset
        .ad(ad_i) //input [11:0] ad
    );

//--------Copy end-------------------
