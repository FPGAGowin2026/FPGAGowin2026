//////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: pwm_gen
// Module Name: pwm_gen_tb
// Project Name: pwm_gen
// Description: 测试pwm_gen
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

module pwm_gen_tb;

	reg clk;	     //系统时钟输入，50M
	reg reset_n;	   //复位信号输入，低有效
	reg pwm_gen_en;	 //pwm产生使能信号
	reg [31:0]counter_arr; //输入32位预重装值
	reg [31:0]counter_ccr; //输入32位输出比较值
	wire pwm_out;	   //pwm输出信号

	pwm_gen pwm_gen(
		.clk(clk),
		.reset_n(reset_n),
		.pwm_gen_en(pwm_gen_en),
		.counter_arr(counter_arr),
		.counter_ccr(counter_ccr),
		.pwm_out(pwm_out)
	);
	
	initial clk = 0;
	always #(`CLK_PERIOD/2) clk = ~clk;
	
	initial begin
		reset_n = 0;
		pwm_gen_en = 0;
		counter_arr = 0;
		counter_ccr = 0;
		#(`CLK_PERIOD*20 +1);
		reset_n = 1;
		#(`CLK_PERIOD*10 +1);

		counter_arr = 1000; //设置输出信号频率为50KHz
		counter_ccr = 400; //设置输出PWM波占空比为40%
		#(`CLK_PERIOD*10);
		pwm_gen_en = 1;	   //启动计数以产生PWM输出
		#100050;
    
		counter_ccr = 700; //设置输出PWM波占空比为70%
		#100050;
		pwm_gen_en = 0;	   //停止计数以关闭PWM输出
    
		counter_arr = 500; //设置输出信号频率为100KHz
		counter_ccr = 250; //设置输出PWM波占空比为50%
		#(`CLK_PERIOD*10);
		pwm_gen_en = 1;	   //启动计数以产生PWM输出
		#50050;
		counter_ccr = 100; //设置输出PWM波占空比为20%
		#50050;
		pwm_gen_en = 0;	   //停止计数以关闭PWM输出

		#200;
		$stop;
	end

endmodule
