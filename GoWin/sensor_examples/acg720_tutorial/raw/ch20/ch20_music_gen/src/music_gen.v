//////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: music_gen
// Module Name: music_gen
// Project Name: music_gen
// Description: PWM波应用，音乐发生器，使用蜂鸣器出声
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module music_gen(
	clk,
	reset_n,

	beep
);
	wire reset;
	input clk;   //系统时钟输入，50MHz
	input reset_n;	 //复位信号输入，低有效
	output beep;     //蜂鸣器控制信号
	
	reg  [31:0]counter_arr;	   //预重装值寄存器
	wire [31:0]counter_ccr;	   //输出比较值
	

	wire [8:0] pitch_num;	     //音乐的音调编号，0——>最大值 循环递增
	wire [4:0] pitch;	         //音乐的音调

	localparam
		DL1 = 340136, //D调低音1
		DL2 = 303030, //D调低音2
		DL3 = 285714, //D调低音3
		DL4 = 255102, //D调低音4
		DL5 = 226244, //D调低音5
		DL6 = 201613, //D调低音6
		DL7 = 179856, //D调低音7

		D1  = 170068, //D调音1 
		D2  = 151515, //D调音2 
		D3  = 142857, //D调音3 
		D4  = 127227, //D调音4 
		D5  = 113379, //D调音5 
		D6  = 101010, //D调音6 
		D7  = 89928 , //D调音7 

		DH1 = 84889,  //D调高音1 
		DH2 = 75643,  //D调高音2 
		DH3 = 71429,  //D调高音3 
		DH4 = 63613,  //D调高音4 
		DH5 = 56689,  //D调高音5 
		DH6 = 50505,  //D调高音6 
		DH7 = 44964;  //D调高音7

    assign reset=~reset_n;
    
	//乐谱存放rom
    Gowin_pROM Gowin_pROM(
        .dout(pitch), //output [4:0] dout
        .clk(clk), //input clk
        .oce(1'b1), //input oce
        .ce(1'b1), //input ce
        .reset(reset), //input reset
        .ad(pitch_num) //input [8:0] ad
    );

	get_pitch get_pitch(
		.clk(clk),
		.reset_n(reset_n),

		.pitch_num(pitch_num)
	);

  //根据rom存储输出不同的音调输出不同的预置数	
	always@(posedge clk or posedge reset)
	if(reset)
		counter_arr  = 32'd1;
	else
	begin
		case(pitch)
			5'b01_001:counter_arr  = DL1;  //D调低音1
			5'b01_010:counter_arr  = DL2;  //D调低音2
			5'b01_011:counter_arr  = DL3;  //D调低音3
			5'b01_100:counter_arr  = DL4;  //D调低音4
			5'b01_101:counter_arr  = DL5;  //D调低音5	
			5'b01_110:counter_arr  = DL6;  //D调低音6
			5'b01_111:counter_arr  = DL7;  //D调低音7

			5'b10_001:counter_arr  = D1;   //D调音1			
			5'b10_010:counter_arr  = D2;   //D调音2				
			5'b10_011:counter_arr  = D3;   //D调音3				
			5'b10_100:counter_arr  = D4;   //D调音4				
			5'b10_101:counter_arr  = D5;   //D调音5	
			5'b10_110:counter_arr  = D6;   //D调音6	
			5'b10_111:counter_arr  = D7;   //D调音7

			5'b11_001:counter_arr  = DH1;  //D调高音1			
			5'b11_010:counter_arr  = DH2;  //D调高音2
			5'b11_011:counter_arr  = DH3;  //D调高音3
			5'b11_100:counter_arr  = DH4;  //D调高音4	
			5'b11_101:counter_arr  = DH5;  //D调高音5
			5'b11_110:counter_arr  = DH6;  //D调高音6
			5'b11_111:counter_arr  = DH7;  //D调高音7

			default:  counter_arr  = 32'd1;//休止符
		endcase
	end

	//设置输出比较值为预重装值一半
	assign counter_ccr = counter_arr >> 1;

	pwm_gen pwm_gen(
		.clk(clk),
		.reset_n(reset_n),
		.pwm_gen_en(1'b1),
		.counter_arr(counter_arr),
		.counter_ccr(counter_ccr),
		.pwm_out(beep)
	);

endmodule
