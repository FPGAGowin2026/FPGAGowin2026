//////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: www.corecourse.cn
// 
// Create Date: 2019/05/01 00:00:00
// Design Name: 
// Module Name: tft_ctrl
// Project Name: tft_ctrl
// Target Devices: XC7A35T-2FGG484I
// Tool Versions: Vivado 2018.3
// Description: 5寸/4.3寸TFT显示屏控制器，默认5寸
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

`include "../define/define.v"

module tft_ctrl
(
	input            clk_ctrl       , //时钟输入
	input            reset_n        , //异步复位信号，低有效
  
	input     [15:0] disp_data      , //待显示数据
  output           disp_data_req  , //
	output reg[11:0] visible_hcount , //TFT可见区域行扫描计数器
	output reg[11:0] visible_vcount , //TFT可见区域场扫描计数器
  output reg       frame_begin    , //一帧图像起始标识信号，clk_ctrl时钟域

	output    [15:0] TFT_rgb        , //TFT数据输出
	output reg       TFT_hs         , //TFT行同步信号
	output reg       TFT_vs         , //TFT场同步信号
	output           TFT_clk        , //TFT像素时钟
	output           TFT_de         , //TFT数据使能
	output           TFT_pwm          //TFT背光控制
);

`include "tft_timing_parameter.vh"
//可见显示区计数的起始和结束计数值
localparam H_DATA_BEGIN = H_Sync_Pulse + H_Back_Porch + H_Left_Border - 1'b1;
localparam H_DATA_END   = H_Visible_Area + H_Sync_Pulse + H_Back_Porch + H_Left_Border - 1'b1;
localparam V_DATA_BEGIN	= V_Sync_Pulse + V_Back_Porch + V_Top_Border - 1'b1;
localparam V_DATA_END   = V_Visible_Area + V_Sync_Pulse + V_Back_Porch + V_Top_Border - 1'b1;

  reg       visible_flag;   //可见区域标定
	reg [11:0]hcount;         //行扫描计数
	reg [11:0]vcount;	        //场扫描计数
	wire      hcount_ov;      //行扫描计数满
	wire      vcount_ov;      //场扫描计数满
  reg       TFT_vs_dly1;    //TFT场同步信号打一拍后信号

	assign TFT_clk  = clk_ctrl;
  assign TFT_de   = visible_flag;
	assign TFT_pwm  = reset_n;
	assign TFT_rgb  = visible_flag ? disp_data : 16'h0000; 

  assign disp_data_req = visible_flag;
	
	//TFT驱动部分
	//行扫描计数
	assign hcount_ov = (hcount == H_Whole_Line - 1'b1)?1'b1:1'b0;

	always@(posedge clk_ctrl or negedge reset_n)
	begin
		if(!reset_n)
			hcount <= 12'd0;
		else if(hcount_ov)
			hcount <= 12'd0;
		else 
			hcount <= hcount + 1'b1;
	end

	//场扫描计数
	assign vcount_ov = (vcount == V_Whole_Frame - 1'b1)?1'b1:1'b0;

	always@(posedge clk_ctrl or negedge reset_n)
	begin
		if(!reset_n)
			vcount <= 12'd0;
		else if(hcount_ov)
		begin
			if(vcount_ov)
				vcount <= 12'd0;
			else 
				vcount <= vcount + 1'b1;		
		end
		else
		vcount <= vcount;	
	end

	always@(posedge clk_ctrl or negedge reset_n)
	begin
		if(!reset_n)
			TFT_hs <= 1'b0;
		else if(hcount == H_Sync_Pulse - 1'b1)
			TFT_hs <= 1'b1;
		else if(hcount == H_Whole_Line - 1'b1)
			TFT_hs <= 1'b0;
    else
      TFT_hs <= TFT_hs;
	end

	always@(posedge clk_ctrl or negedge reset_n)
	begin
		if(!reset_n)
			TFT_vs <= 1'b0;
		else if(hcount_ov && vcount == V_Sync_Pulse - 1'b1)
			TFT_vs <= 1'b1;
		else if(hcount_ov && vcount == V_Whole_Frame - 1'b1)
			TFT_vs <= 1'b0;
    else
      TFT_vs <= TFT_vs;
	end

	always@(posedge clk_ctrl or negedge reset_n)
	begin
		if(!reset_n)
			visible_flag <= 1'b0;
		else if((vcount >= V_DATA_BEGIN) && (vcount < V_DATA_END))
      if((hcount >= H_DATA_BEGIN) && (hcount < H_DATA_END))
        visible_flag <= 1'b1;
      else
        visible_flag <= 1'b0;
    else
      visible_flag <= 1'b0;
	end

	always@(posedge clk_ctrl or negedge reset_n)
	begin
		if(!reset_n)
			visible_hcount <= 1'b0;
		else if((hcount >= H_DATA_BEGIN) && (hcount < H_DATA_END))
			visible_hcount <= hcount - H_DATA_BEGIN;
		else
			visible_hcount <= 1'b0;
	end   
  
	always@(posedge clk_ctrl or negedge reset_n)
	begin
		if(!reset_n)
			visible_vcount <= 1'b0;
		else if((vcount >= V_DATA_BEGIN) && (vcount < V_DATA_END))
      visible_vcount <= vcount - V_DATA_BEGIN;
		else
			visible_vcount <= 1'b0;
	end 

	always@(posedge clk_ctrl)
	begin
    TFT_vs_dly1 <= TFT_vs;
  end

	always@(posedge clk_ctrl or negedge reset_n)
	begin
		if(!reset_n)
			frame_begin <= 1'b0;
		else if(!TFT_vs_dly1 && TFT_vs)
			frame_begin <= 1'b1;
		else
			frame_begin <= 1'b0;
	end
	
endmodule 