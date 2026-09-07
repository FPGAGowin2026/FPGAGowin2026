/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Design Name   : 
// Module Name   : shift_register_2taps
// Project Name  : 
// Description   : 基于RAM的移位寄存器，2行缓存
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module shift_register_2taps
#(
  parameter DATA_WIDTH = 16
)
(
  input                   clk,
  input                   Reset,
  input  [DATA_WIDTH-1:0] shiftin,
  input                   shiftin_valid,

  output [DATA_WIDTH-1:0] shiftout,
  output [DATA_WIDTH-1:0] taps1x,
  output [DATA_WIDTH-1:0] taps0x
);
wire clk_ce;
assign shiftout = taps0x;
   DCE DCE(
    .CLKIN(clk),
    .CE(shiftin_valid),
    .CLKOUT(clk_ce)
   );

	shift_reg_ram shift_reg_ram_inst1(
		.clk(clk_ce), //input clk
		.Reset(Reset), //input Reset
		.Din(shiftin), //input [7:0] Din
		.Q(taps1x) //output [7:0] Q
	);

	shift_reg_ram shift_reg_ram_inst2(
		.clk(clk_ce), //input clk
		.Reset(Reset), //input Reset
		.Din(taps1x), //input [7:0] Din
		.Q(taps0x) //output [7:0] Q
	);


endmodule