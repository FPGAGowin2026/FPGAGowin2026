//////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: fsm_hello
// Module Name: fsm_hello_tb
// Project Name: fsm_hello
// Target Devices: XC7A35T-2FGG484I
// Tool Versions: Vivado 2018.3
// Description: 状态机字符串串口传送测试程序
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns
`define CLK_PERIOD 20

module fsm_hello_tb;
	
	reg clk;
	reg reset_n;
  
	reg data_valid;
	reg [7:0]data_in;
	
	wire check_ok;	
	
	fsm_hello fsm_hello(
		.clk(clk),
		.reset_n(reset_n),
		.data_valid(data_valid),
		.data_in(data_in),
		.check_ok(check_ok)
	);
	
	initial clk = 1;
	always#(`CLK_PERIOD/2) clk = ~clk;
	
	initial begin
		reset_n = 0;
		data_valid = 0;
		data_in = 0;
		#(`CLK_PERIOD*20);
		reset_n = 1;
		#(`CLK_PERIOD*20 + 1);

		repeat(2) begin
			gen_char("I");
			#(`CLK_PERIOD);
			gen_char("A");
			#(`CLK_PERIOD);
			gen_char("h");
			#(`CLK_PERIOD);
			gen_char("e");
			#(`CLK_PERIOD);
		gen_char("l");
			#(`CLK_PERIOD);
			gen_char("l");
			#(`CLK_PERIOD);   
			gen_char("l");
			#(`CLK_PERIOD);
			gen_char("h");
			#(`CLK_PERIOD);
			gen_char("e");
			#(`CLK_PERIOD);
			gen_char("l");
			#(`CLK_PERIOD);
		gen_char("l");
			#(`CLK_PERIOD);
			gen_char("o");
			#(`CLK_PERIOD);
			gen_char("e");
			#(`CLK_PERIOD);
			gen_char("h");
			#(`CLK_PERIOD);
			gen_char("h");
			#(`CLK_PERIOD);
			gen_char("o");
			#(`CLK_PERIOD);      
		end

			#200;
			$stop;
	end
  
	task gen_char;
		input [7:0]char;    
		begin
			data_in = char;
			data_valid = 1'b1;
			#(`CLK_PERIOD);
			data_valid = 1'b0;
		end
	endtask

endmodule
