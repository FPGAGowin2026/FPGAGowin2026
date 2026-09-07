//Copyright (C)2014-2024 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: IP file
//Tool Version: V1.9.10.03 (64-bit)
//Part Number: GW5AT-LV138PG484AC1/I0
//Device: GW5AT-138
//Device Version: B
//Created Time: Mon Nov 18 16:42:54 2024

module Gowin_pROM (dout, clk, oce, ce, reset, ad);

output [7:0] dout;
input clk;
input oce;
input ce;
input reset;
input [7:0] ad;

wire [23:0] prom_inst_0_dout_w;
wire gw_gnd;

assign gw_gnd = 1'b0;

pROM prom_inst_0 (
    .DO({prom_inst_0_dout_w[23:0],dout[7:0]}),
    .CLK(clk),
    .OCE(oce),
    .CE(ce),
    .RESET(reset),
    .AD({gw_gnd,gw_gnd,gw_gnd,ad[7:0],gw_gnd,gw_gnd,gw_gnd})
);

defparam prom_inst_0.READ_MODE = 1'b0;
defparam prom_inst_0.BIT_WIDTH = 8;
defparam prom_inst_0.RESET_MODE = "SYNC";
defparam prom_inst_0.INIT_RAM_00 = 256'hD7D5D3D0CECBC9C6C4C1BEBCB9B6B3B0ADAAA7A5A29E9B9895928F8C89868380;
defparam prom_inst_0.INIT_RAM_01 = 256'hFFFFFFFEFEFEFDFDFCFBFAFAF9F8F6F5F4F3F1F0EEEDEBEAE8E6E4E2E0DEDCDA;
defparam prom_inst_0.INIT_RAM_02 = 256'hDCDEE0E2E4E6E8EAEBEDEEF0F1F3F4F5F6F8F9FAFAFBFCFDFDFEFEFEFFFFFFFF;
defparam prom_inst_0.INIT_RAM_03 = 256'h8386898C8F9295989B9EA2A5A7AAADB0B3B6B9BCBEC1C4C6C9CBCED0D3D5D7DA;
defparam prom_inst_0.INIT_RAM_04 = 256'h282A2C2F313436393B3E414346494C4F5255585A5D6164676A6D707376797C80;
defparam prom_inst_0.INIT_RAM_05 = 256'h0000000101010202030405050607090A0B0C0E0F1112141517191B1D1F212325;
defparam prom_inst_0.INIT_RAM_06 = 256'h23211F1D1B1917151412110F0E0C0B0A09070605050403020201010100000000;
defparam prom_inst_0.INIT_RAM_07 = 256'h7C797673706D6A6764615D5A5855524F4C494643413E3B393634312F2C2A2825;

endmodule //Gowin_pROM
