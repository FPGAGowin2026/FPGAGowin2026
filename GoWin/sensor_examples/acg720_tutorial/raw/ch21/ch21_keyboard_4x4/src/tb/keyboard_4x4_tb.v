`timescale 1ns/1ns
`define CLK_PERIOD
/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: Max
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: keyboaed_4x4
// Module Name: keyboaed_4x4_tb
// Project Name: keyboaed_4x4
// Description: keyboaed_4x4测试文件
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns

module keyboaed_4x4_tb;

	reg clk;
	reg reset_n;
	reg press_key;
	reg [3:0]press_key_value;
	wire [3:0]keyboard_row_x4;
	wire [3:0]keyboard_col_x4;
	
	wire key_flag;
	wire [3:0]key_value;

	initial clk = 1;
	always #10 clk = ~clk;
	
	initial begin
		reset_n = 1'b0;
		press_key = 1'b0;
		press_key_value = 8'h0;
		#201;
		reset_n = 1'b1;
    
		press_key = 1'b1;
		press_key_value = 8'ha;
		#20;
		press_key = 1'b0;
		
		#50000000;
		press_key = 1'b1;
		press_key_value = 8'h8;
		#20;
		press_key = 1'b0;
		
		#50000000;
		press_key = 1'b1;
		press_key_value = 8'hd;
		#20;
		press_key = 1'b0;
		
		#50000000;
		$stop;
	end

	keyboard_4x4_model
	keyboard_4x4_model
	(
		.press_key(press_key),
		.press_key_value(press_key_value),
		.keyboard_col_x4_i(keyboard_col_x4),
		.keyboard_row_x4_o(keyboard_row_x4)
	);

	keyboard_4x4
	keyboard_4x4
	(
		.clk(clk),
		.reset_n(reset_n),
		.keyboard_row_x4_i(keyboard_row_x4),
		.keyboard_col_x4_o(keyboard_col_x4),
		.key_flag(key_flag),
		.key_value(key_value)
	);
	
endmodule
