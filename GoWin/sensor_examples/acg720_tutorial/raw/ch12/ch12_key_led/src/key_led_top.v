

module key_led_top(
   clk,
	reset_n,
	key_in0,
	key_in1,
	key_in2,
	key_in3,
	led
);

	input clk;   //系统时钟输入，50M
	input reset_n;  //复位信号输入，低有效
	input key_in0;  //按键0输入,自加
	input key_in1;  //按键1输入，自减
	input key_in2;  //按键0输入，左移
	input key_in3;  //按键1输入，右移
	
	output [7:0]led;//led数据显示
	
	wire key_flag0, key_state0;
	wire key_flag1, key_state1;
	wire key_flag2, key_state2;
	wire key_flag3, key_state3;
	wire key_add;
	wire key_sub;
	wire key_shift_l;
	wire key_shift_r;
  
	assign key_add = key_flag0 && (~key_state0);
	assign key_sub = key_flag1 && (~key_state1);
	assign key_shift_l = key_flag2 && (~key_state2);
	assign key_shift_r = key_flag3 && (~key_state3);

	key_filter key_filter0(
		.clk(clk),
		.reset_n(reset_n),
		.key_in(key_in0),
		.key_flag(key_flag0),
		.key_state(key_state0 )
	);
	
	key_filter key_filter1(
		.clk(clk),
		.reset_n(reset_n),
		.key_in(key_in1),
		.key_flag(key_flag1),
		.key_state (key_state1)
	);
  
  key_filter key_filter2(
		.clk(clk),
		.reset_n(reset_n),
		.key_in(key_in2),
		.key_flag(key_flag2),
		.key_state(key_state2)
	);

  key_filter key_filter3(
		.clk(clk),
		.reset_n(reset_n),
		.key_in(key_in3),
		.key_flag(key_flag3),
		.key_state(key_state3)
	);
	
	led_ctrl led_ctrl0(
		.clk(clk),
		.reset_n(reset_n),
		.key_add(key_add),
		.key_sub(key_sub),
		.key_shift_l(key_shift_l),
		.key_shift_r(key_shift_r),
		.led(led)
	);

endmodule
