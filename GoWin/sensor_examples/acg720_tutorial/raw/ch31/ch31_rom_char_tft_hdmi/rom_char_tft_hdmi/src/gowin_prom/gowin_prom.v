//Copyright (C)2014-2024 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: IP file
//Tool Version: V1.9.10.03 (64-bit)
//Part Number: GW5AT-LV138PG484AC1/I0
//Device: GW5AT-138
//Device Version: B
//Created Time: Fri Nov 29 14:14:32 2024

module Gowin_pROM (dout, clk, oce, ce, reset, ad);

output [175:0] dout;
input clk;
input oce;
input ce;
input reset;
input [4:0] ad;

wire gw_gnd;

assign gw_gnd = 1'b0;

pROMX9 promx9_inst_0 (
    .DO(dout[35:0]),
    .CLK(clk),
    .OCE(oce),
    .CE(ce),
    .RESET(reset),
    .AD({gw_gnd,gw_gnd,gw_gnd,gw_gnd,ad[4:0],gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd})
);

defparam promx9_inst_0.READ_MODE = 1'b0;
defparam promx9_inst_0.BIT_WIDTH = 36;
defparam promx9_inst_0.RESET_MODE = "SYNC";
defparam promx9_inst_0.INIT_RAM_00 = 288'h0038003C00038003C0000000000000000000000000000000000000000000000000000000;
defparam promx9_inst_0.INIT_RAM_01 = 288'h00860018000C400180004C00180004C001C0804C003C0804C003C08038003C00038003C0;
defparam promx9_inst_0.INIT_RAM_02 = 288'h82018000081030000081030000081030018081FF00180E18200180008600180008600180;
defparam promx9_inst_0.INIT_RAM_03 = 288'h0000000000000000000000000000000000000F83E01800601C03C08201803C0820180180;

pROMX9 promx9_inst_1 (
    .DO(dout[71:36]),
    .CLK(clk),
    .OCE(oce),
    .CE(ce),
    .RESET(reset),
    .AD({gw_gnd,gw_gnd,gw_gnd,gw_gnd,ad[4:0],gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd})
);

defparam promx9_inst_1.READ_MODE = 1'b0;
defparam promx9_inst_1.BIT_WIDTH = 36;
defparam promx9_inst_1.RESET_MODE = "SYNC";
defparam promx9_inst_1.INIT_RAM_00 = 288'h1C18180C3FC7FF003C000000000000000000000000000000000000000000000000000000;
defparam promx9_inst_1.INIT_RAM_01 = 288'h30180C60010180660010180660000180620000180630002180630002180618104180C081;
defparam promx9_inst_1.INIT_RAM_02 = 288'h001800301001800301001800201101800601101800601101800607301FE0600F01818600;
defparam promx9_inst_1.INIT_RAM_03 = 288'h000000000000000000000000000000000000007E0007C0018000C2001800181001800101;

pROMX9 promx9_inst_2 (
    .DO(dout[107:72]),
    .CLK(clk),
    .OCE(oce),
    .CE(ce),
    .RESET(reset),
    .AD({gw_gnd,gw_gnd,gw_gnd,gw_gnd,ad[4:0],gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd})
);

defparam promx9_inst_2.READ_MODE = 1'b0;
defparam promx9_inst_2.BIT_WIDTH = 36;
defparam promx9_inst_2.RESET_MODE = "SYNC";
defparam promx9_inst_2.INIT_RAM_00 = 288'h00000001800000007F000000000000000000000000000000000000000000000000000000;
defparam promx9_inst_2.INIT_RAM_01 = 288'h818000018C300000183C0000018000000018000000018000000018000000018000000018;
defparam promx9_inst_2.INIT_RAM_02 = 288'h00C00001800C00001800C00001800C00001800C00001800C00001800C00001881800001F;
defparam promx9_inst_2.INIT_RAM_03 = 288'h0000000000000000000000000000000000003C000007EC30000018818000018818000018;

pROMX9 promx9_inst_3 (
    .DO(dout[143:108]),
    .CLK(clk),
    .OCE(oce),
    .CE(ce),
    .RESET(reset),
    .AD({gw_gnd,gw_gnd,gw_gnd,gw_gnd,ad[4:0],gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd})
);

defparam promx9_inst_3.READ_MODE = 1'b0;
defparam promx9_inst_3.BIT_WIDTH = 36;
defparam promx9_inst_3.RESET_MODE = "SYNC";
defparam promx9_inst_3.INIT_RAM_00 = 288'h0180018001F801F800008000800000000000000000000000000000000000000000000000;
defparam promx9_inst_3.INIT_RAM_01 = 288'h018001800018001800018001800018001800018001800018001800018001800018001800;
defparam promx9_inst_3.INIT_RAM_02 = 288'h018001803018001803018001803018001803018001803018001803018001801018001801;
defparam promx9_inst_3.INIT_RAM_03 = 288'h0000000000000000000000000000000000001FF81FF80018001800018001801018001801;

pROM prom_inst_4 (
    .DO(dout[175:144]),
    .CLK(clk),
    .OCE(oce),
    .CE(ce),
    .RESET(reset),
    .AD({gw_gnd,gw_gnd,gw_gnd,gw_gnd,ad[4:0],gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd})
);

defparam prom_inst_4.READ_MODE = 1'b0;
defparam prom_inst_4.BIT_WIDTH = 32;
defparam prom_inst_4.RESET_MODE = "SYNC";
defparam prom_inst_4.INIT_RAM_00 = 256'h300C0000FC3F0000000000000000000000000000000000000000000000000000;
defparam prom_inst_4.INIT_RAM_01 = 256'h300C0818300C0C30300C03C0300C0000300C0000300C0000300C0000300C0000;
defparam prom_inst_4.INIT_RAM_02 = 256'h300C3000300C3000300C3000300C3FFC300C300C300C300C300C300C3FFC1808;
defparam prom_inst_4.INIT_RAM_03 = 256'h00000000000000000000000000000000FC3F03E0300C0E18300C1808300C1804;

endmodule //Gowin_pROM
