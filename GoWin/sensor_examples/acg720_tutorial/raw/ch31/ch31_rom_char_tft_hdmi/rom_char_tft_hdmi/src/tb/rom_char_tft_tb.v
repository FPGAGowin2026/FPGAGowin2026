
`timescale 1ns/1ns
`define CLK_PERIOD 20

module rom_char_tft_tb();

	reg        clk50M;	       //模块全局时钟输入，50M
	reg        reset_n;        //复位信号输入，低有效

	wire[15:0] TFT_rgb;        //TFT数据输出
	wire       TFT_hs;         //TFT行同步信号
	wire       TFT_vs;         //TFT场同步信号
	wire       TFT_clk;        //TFT像素时钟
	wire       TFT_de;         //TFT数据使能
	wire       TFT_pwm;        //TFT背光控制

	initial clk50M = 1;
	always#(`CLK_PERIOD/2) clk50M = ~clk50M;

	GSR GSR(.GSRI(1'b1));
		
	rom_char_tft_hdmi rom_char_tft_hdmi(
		.clk50M    (clk50M   ),
		.reset_n   (reset_n  ),

		.TFT_rgb   (TFT_rgb  ),
		.TFT_hs    (TFT_hs   ),
		.TFT_vs    (TFT_vs   ),
		.TFT_clk   (TFT_clk  ),
		.TFT_de    (TFT_de   ),
		.TFT_pwm   (TFT_pwm  ),
		.hdmi1_clk_p()  ,
		.hdmi1_clk_n()  ,
		.hdmi1_dat_p()  ,
		.hdmi1_dat_n()  ,
		.hdmi1_oe()  
	);

	initial begin
		reset_n = 0;
		#(`CLK_PERIOD *20 +1);
		reset_n = 1;
    
    @(posedge rom_char_tft_hdmi.disp_driver.Frame_Begin);
    @(posedge rom_char_tft_hdmi.disp_driver.Frame_Begin);
    #200;
    $stop;
	end

endmodule
