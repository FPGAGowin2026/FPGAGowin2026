//Copyright (C)2014-2023 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Template file for instantiation
//GOWIN Version: V1.9.9 Beta-1
//Part Number: GW5A-LV25UG324ES
//Device: GW5A-25
//Created Time: Fri Jun 30 16:41:23 2023

//Change the instance name and port connections to the signal names
//--------Copy here to design--------

    Gowin_RAM16SDP your_instance_name(
        .dout(dout_o), //output [7:0] dout
        .wre(wre_i), //input wre
        .wad(wad_i), //input [7:0] wad
        .di(di_i), //input [7:0] di
        .rad(rad_i), //input [7:0] rad
        .clk(clk_i) //input clk
    );

//--------Copy end-------------------
