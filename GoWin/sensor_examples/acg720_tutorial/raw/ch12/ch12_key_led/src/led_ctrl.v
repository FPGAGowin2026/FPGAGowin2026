
module led_ctrl(
	clk,
	reset_n,

	key_add,
	key_sub,
	key_shift_l,
	key_shift_r,
	led
);
	
	input clk;      //模块工作时钟输入，50M
	input reset_n;     //复位信号输入，低有效
	input key_add;     //自加按键
	input key_sub;     //自减按键
	input key_shift_l; //左移按键
	input key_shift_r; //右移按键
	
	output [7:0]led;   //led显示
	
	reg [7:0]led;
    wire reset=~reset_n;
	
	always@(posedge clk or posedge reset)
	if(reset)
		led <= 8'b0000_0000;
	else if(key_add)
		led <= led + 1'b1;
	else if(key_sub)
		led <= led - 1'b1;
	else if(key_shift_l)
		led <= (led << 1);
	else if(key_shift_r)
		led <= (led >> 1);
	else
		led <= led;
		
endmodule
