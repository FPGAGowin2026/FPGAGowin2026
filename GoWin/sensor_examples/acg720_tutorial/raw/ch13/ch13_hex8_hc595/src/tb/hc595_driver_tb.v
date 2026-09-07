//////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: hex8
// Module Name: hc595_driver
// Project Name: hex8
// Target Devices: XC7A35T-2FGG484I
// Tool Versions: Vivado 2018.3
// Description: 驱动hc595测试文件，发出数据和选通信号
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns

`define clk_period 20
module hc595_driver_tb;

	reg clk;
	reg reset_n;
	reg [15 : 0] data;	//data to send
	reg s_en;	//send en
	wire sh_cp;	//shift clock
	wire st_cp;	//latch data clock
	wire ds;	//shift serial data
	

	hc595_driver hc595_driver(
		.clk(clk),
		.reset_n(reset_n),
		.data(data),
		.s_en(s_en),
		.sh_cp(sh_cp),
		.st_cp(st_cp),
		.ds(ds)
	);
	initial clk = 1;
	always#(`clk_period/2) clk = ~clk;

	initial begin
		reset_n = 1'b0;
		s_en = 1;
		data = 16'b1010_1111_0110_0101;
		#(`clk_period*20);
		reset_n = 1;
		#(`clk_period*20);
		#5000;
		data = 16'b0101_0101_1010_0101;
		#5000;

		$stop;
	end
endmodule 