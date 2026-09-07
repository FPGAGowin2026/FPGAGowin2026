//////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: music_gen
// Module Name: music_gen_tb
// Project Name: music_gen
// Description: 测试音乐生成文件
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

module music_gen_tb();

	reg clk;  //系统时钟输入，50M
	reg reset_n; //复位信号输入，低有效

	wire beep;	   //pwm输出信号
  
	defparam music_gen.get_pitch.T125ms_MAX_CNT = 24'd62500;

    GSR GSR(.GSRI(1'b1));
	music_gen music_gen(
		.clk(clk),
		.reset_n(reset_n),

		.beep(beep)
	);

	initial clk = 0;
	always #(`CLK_PERIOD/2) clk = ~clk;
	
	initial begin
		reset_n = 0;
		#(`CLK_PERIOD*20 +1);
		reset_n = 1;
		
		wait(music_gen.pitch_num == 512);
		#200;

		$stop;
	end
endmodule
