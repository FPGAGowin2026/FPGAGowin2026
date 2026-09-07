`timescale 1ns/1ns
module hdmi_color_tb();

reg clk;
reg rst_n;
wire tmds_clk_p;
wire tmds_clk_n;
wire [2:0]tmds_data_p;    //rgb
wire [2:0]tmds_data_n;     //rgb

	
	hdmi_color hdmi_color(
		.clk(clk),
		.rst_n(rst_n),
		.tmds_clk_p(tmds_clk_p),
		.tmds_clk_n(tmds_clk_n),
		.tmds_data_p(tmds_data_p),    //rgb
		.tmds_data_n(tmds_data_n)   //rgb
	);

	initial clk = 1;
	always#20 clk = ~clk;
	
	
	initial begin
		rst_n = 0;
		#201;
		rst_n = 1;
		#200;
		#200000000;
	
	
	end

endmodule


