//////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: hex8
// Module Name: hex_top
// Project Name: hex8
// Description: hex8项目的顶层文件，协调VIO，hex8和hc595的工作和接口
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module hex_top(
	clk,
	reset_n,
	sh_cp,
	st_cp,
	ds
);

	input clk;	//50M
	input reset_n;
	
	output sh_cp;
	output st_cp;
	output ds;
	
	wire [31:0]disp_data;
	wire [7:0] sel;//数码管位选（选择当前要显示的数码管）
	wire [6:0] seg;//数码管段选（当前要显示的内容)
	
    assign disp_data = 32'h01234567;
	
	hc595_driver hc595_driver(
		.clk(clk),
		.reset_n(reset_n),
		.data({1'd0,seg,sel}),
		.s_en(1'b1),
		.sh_cp(sh_cp),
		.st_cp(st_cp),
		.ds(ds)
	);
	
	hex8 hex8(
		.clk(clk),
		.reset_n(reset_n),
		.en(1'b1),
		.disp_data(disp_data),
		.sel(sel),
		.seg(seg)
	);
	
endmodule
