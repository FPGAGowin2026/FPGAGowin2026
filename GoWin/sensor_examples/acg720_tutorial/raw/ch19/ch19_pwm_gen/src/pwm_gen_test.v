//////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: pwm_gen_test
// Module Name: pwm_gen_test
// Project Name: pwm_gen_test
// Description: 对原始时钟进行分频，然后生成音调的频率
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module pwm_gen_test(
	clk,
	reset_n,
  
	beep
);
	wire reset=~reset_n;
	input clk;
	input reset_n;
  
	output beep;

	reg [31:0]counter_arr;    //预重装值寄存器
	wire [31:0]counter_ccr;    //输出比较值

	reg [24:0]t500ms_dly_cnt; //500ms延时计数器
	reg [2:0]pitch_num;      //音调编号

	parameter T500ms_MAX_CNT  = 25'd25000000;

	localparam
		D1  = 170068, //D调音1 
		D2  = 151515, //D调音2 
		D3  = 142857, //D调音3 
		D4  = 127227, //D调音4 
		D5  = 113379, //D调音5 
		D6  = 101010, //D调音6 
		D7  = 89928 ; //D调音7

  //输出比较值为预重装值一半
		assign counter_ccr = counter_arr >> 1;

	pwm_gen pwm_gen(
		.clk(clk),
		.reset_n(reset_n),
		.pwm_gen_en(1'b1),
		.counter_arr(counter_arr),
		.counter_ccr(counter_ccr),
		.pwm_out(beep)
		);
    
	//500ms延时计数器计数
	always@(posedge clk or posedge reset)
	if(reset)
		t500ms_dly_cnt <= 1'd0;
	else if(t500ms_dly_cnt == T500ms_MAX_CNT - 1'b1)
		t500ms_dly_cnt <= 1'd0;
	else
		t500ms_dly_cnt <= t500ms_dly_cnt + 1'b1;
    
	//每500ms切换一次音调
	always@(posedge clk or posedge reset)
	if(reset)
		pitch_num <= 3'd0;
	else if(t500ms_dly_cnt == T500ms_MAX_CNT - 1'b1)
    pitch_num <= pitch_num + 1'd1;
	else
		pitch_num <= pitch_num;
        
  //根据音调编号给预重装值给相应的值
	always@(posedge clk or posedge reset)
	if(reset)
		counter_arr  = 32'd1;
	else
	begin
		case(pitch_num)
			0 :counter_arr  = 32'd1;
			1 :counter_arr  = D1;
			2 :counter_arr  = D2;
			3 :counter_arr  = D3;
			4 :counter_arr  = D4;
			5 :counter_arr  = D5;
			6 :counter_arr  = D6;
			7 :counter_arr  = D7;
		endcase
	end

endmodule 