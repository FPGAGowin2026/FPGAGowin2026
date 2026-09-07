//////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: hex8
// Module Name: hex_top
// Project Name: hex8
// Target Devices: XC7A35T-2FGG484I
// Tool Versions: Vivado 2018.3
// Description: hex8项目的顶层文件，协调VIO，hex8和hc595的工作和接口
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module hex_top_special_for_rtc(
	clk,
	reset_n,
	disp_data,
	sh_cp,
	st_cp,
	ds
);

	input clk;	//50M
	input reset_n;
	input disp_data;
	
	output sh_cp;
	output st_cp;
	output ds;
	
	wire [31:0]disp_data;
	wire [7:0] sel;//数码管位选（选择当前要显示的数码管）
	wire [6:0] seg;//数码管段选（当前要显示的内容)
	
/*	vio_0 vio_0 (
		.clk(clk), 
		.probe_out0(disp_data)  
);*/
	
	hc595_driver hc595_driver(
		.clk(clk),
		.reset_n(reset_n),
		.data({1'd1,seg,sel}),
		.s_en(1'b1),
		.sh_cp(sh_cp),
		.st_cp(st_cp),
		.ds(ds)
	);
	
	hex8_special_for_rtc hex8_special_for_rtc(
		.clk(clk),
		.reset_n(reset_n),
		.en(1'b1),
		.disp_data(disp_data),
		.sel(sel),
		.seg(seg)
	);
	
endmodule
