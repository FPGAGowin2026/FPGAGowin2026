//////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: music_gen
// Module Name: get_pitch
// Project Name: music_gen
// Description:	生成音乐的一拍
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module get_pitch(
	clk,
	reset_n,
  
	pitch_num
);
	wire reset;
	input clk;    //系统时钟输入＿50MHz
	input reset_n;	 //复位信号输入，低有效
  
	output reg[8:0] pitch_num; //获取音符rom的地址编号

	parameter T125ms_MAX_CNT  = 24'd6250000;

	reg [23:0] t125ms_dly_cnt; //125ms延时计数器,每个节拍的时间
  
    assign reset=~reset_n;

	//125ms延时计数器计数
	always@(posedge clk or posedge reset)
	if(reset)
		t125ms_dly_cnt <= 1'd0;
	else if(t125ms_dly_cnt == T125ms_MAX_CNT - 1'b1)
		t125ms_dly_cnt <= 1'd0;
	else
		t125ms_dly_cnt <= t125ms_dly_cnt + 1'b1;
	
	//每125ms切换一次音符
	always@(posedge clk or posedge reset)
	if(reset)
		pitch_num <= 9'd0;
	else if(t125ms_dly_cnt == T125ms_MAX_CNT - 1'b1)
		pitch_num <= pitch_num + 1'd1;
	else
		pitch_num <= pitch_num;

endmodule