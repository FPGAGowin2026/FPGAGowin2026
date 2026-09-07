/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: Max
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: keyboard_4x4
// Module Name: keyboard_4x4
// Project Name: keyboard_4x4
// Description: 4x4矩阵键盘驱动
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module keyboard_4x4(
	clk,
	reset_n,

	keyboard_row_x4_i,
	keyboard_col_x4_o,

	key_flag,
	key_value
);
	wire reset;
	input clk;
	input reset_n;
	input [3:0]keyboard_row_x4_i;
	
	output [3:0]keyboard_col_x4_o;
	output key_flag;
	output [3:0]key_value;

	reg [3:0]keyboard_col_x4_o;
	reg key_flag;
	reg [3:0]key_value;
	reg delay20ms_en;
	reg [19:0]delay20ms_cnt;
	reg delay20ms_flag;
	reg [10:0]state;
	reg [3:0]keyboard_row_reg;
	reg [3:0]keyboard_col_reg;
	reg [1:0]col_scan_cnt;
	reg [7:0]key_value_pre;
	reg key_flag_pre;

	localparam 
	WAIT_P = 8'b00000001,
	P_FILTER = 8'b00000010,
	READ_ROW_P = 8'b00000100,
	COL_SCAN = 8'b00001000,
	PRESS_RESULT = 8'b00010000,
	WAIT_R = 8'b00100000,
	R_FILTER = 8'b01000000,
	READ_ROW_R = 8'b10000000;
    
    assign reset=~reset_n;
    
//按键采用延时消抖，产生20ms延时计数器，由延时使能控制
	always@(posedge clk or posedge reset)
	if(reset)
		delay20ms_cnt <= 20'd0;
	else if(delay20ms_en)
		delay20ms_cnt <= delay20ms_cnt + 1'b1;
	else
		delay20ms_cnt <= 20'd0;

	always@(posedge clk or posedge reset)
	if(reset)
		delay20ms_flag <= 1'b0;
	else if(delay20ms_cnt == 20'd999999)
		delay20ms_flag <= 1'b1;
	else
		delay20ms_flag <= 1'b0;

//状态机实现
	always@(posedge clk or posedge reset)
	if(reset)begin
		state <= WAIT_P;
		delay20ms_en <= 1'b0;
		keyboard_col_x4_o <= 4'b0000;
		keyboard_row_reg <= 4'b1111;
		keyboard_col_reg <= 4'b0000; 
		col_scan_cnt <= 2'd0;
		key_flag_pre <= 1'b0;
		key_value_pre <= 8'd0;
	end
	else begin
		case(state)
			WAIT_P: begin
				delay20ms_en <= 1'b0;
				keyboard_col_x4_o <= 4'b0000;
				keyboard_row_reg <= 4'b1111;
				keyboard_col_reg <= 4'b0000;
				col_scan_cnt <= 2'd0;        
				key_flag_pre <= 1'b0;
				key_value_pre <= 8'd0;
				if(keyboard_row_x4_i != 4'b1111)begin //检测到有按键按下，至少有一行的信号为低
					state <= P_FILTER;           //进入到消抖状态
					delay20ms_en <= 1'b1;               //使能20ms延时
					end
				else begin
					state <= WAIT_P;			
					delay20ms_en <= 1'b0;
					end
				end
				
			P_FILTER:
				if(delay20ms_flag) begin  //20ms延时到，进入到再次确认是否有按键按下状态
					state <= READ_ROW_P;
					delay20ms_en <= 1'b0;
					end
				else begin
					state <= P_FILTER;				
					delay20ms_en <= 1'b1;   //20ms延时未到，继续计数
				end
			
			READ_ROW_P:
				if(keyboard_row_x4_i != 4'b1111)begin    //至少有一行的信号为低，说明确实检测到有按键按下
					state <= COL_SCAN;	       //进入到列扫描状态，进一步确认是哪个按键按下
					keyboard_row_reg  <= keyboard_row_x4_i;//寄存此时的行信号
					keyboard_col_x4_o <= 4'b1110;
					end
				else begin                               //未有行信号为0，说明只是抖动，并未真的有按键按下
					state <= WAIT_P;
				end
			
			COL_SCAN:        
				if(keyboard_row_x4_i == keyboard_row_reg) begin //找到是哪个按键按下
					state <= PRESS_RESULT;
					keyboard_col_reg <= keyboard_col_x4_o;
					end
				else if(col_scan_cnt == 2'd3) begin             //扫描结束，未找到哪个按键按下，认为异常回到初始态
					state <= WAIT_P;
					col_scan_cnt <= 2'd0;
					end
				else begin
					keyboard_col_x4_o <= {keyboard_col_x4_o[2:0],1'b1};
					col_scan_cnt <= col_scan_cnt + 1'b1;
					end

			PRESS_RESULT: begin //记录扫描确认按键后的结果      
				state <= WAIT_R;
				keyboard_col_x4_o <= 4'b0000;
				key_flag_pre <= 1'b1;
				key_value_pre <= {keyboard_row_reg,keyboard_col_reg};
				end

			WAIT_R: begin                  //等待按键松开
				key_flag_pre <= 1'b0;
				if(keyboard_row_x4_i == 4'b1111)begin  //行信号全为高，说明检测到按键松开
					state <= R_FILTER;
					delay20ms_en <= 1'b1;
					end
				else begin
					state <= WAIT_R;
					delay20ms_en <= 1'b0;
					end
				end
			
			R_FILTER:					
				if(delay20ms_flag) begin     //延时20ms到,进入再次确认按键松开状态
					state <= READ_ROW_R;
					delay20ms_en <= 1'b0;
				end
				else begin
					state <= R_FILTER;
					delay20ms_en <= 1'b1;
				end

			READ_ROW_R:
				if(keyboard_row_x4_i == 4'b1111)   //行信号全为高，确认按键已经松开
					state <= WAIT_P;
				else                               //按键并未真的松开，是抖动
					state <= WAIT_R;
			default:state <= WAIT_P;
		endcase	
	end
	
	always@(posedge clk or posedge reset)
	if(reset)
		key_value <= 4'd0;
	else if(key_flag_pre)begin
		case(key_value_pre)  //{keyboard_row_reg,keyboard_col_reg}
			8'b1110_1110 : key_value <= 4'h0;
			8'b1110_1101 : key_value <= 4'h1;
			8'b1110_1011 : key_value <= 4'h2;
			8'b1110_0111 : key_value <= 4'h3;
			8'b1101_1110 : key_value <= 4'h4;
			8'b1101_1101 : key_value <= 4'h5;
			8'b1101_1011 : key_value <= 4'h6;
			8'b1101_0111 : key_value <= 4'h7;
			8'b1011_1110 : key_value <= 4'h8;
			8'b1011_1101 : key_value <= 4'h9;
			8'b1011_1011 : key_value <= 4'ha;
			8'b1011_0111 : key_value <= 4'hb;
			8'b0111_1110 : key_value <= 4'hc;
			8'b0111_1101 : key_value <= 4'hd;
			8'b0111_1011 : key_value <= 4'he;
			8'b0111_0111 : key_value <= 4'hf;
			default : key_value <= key_value;
		endcase
	end

	always@(posedge clk)
		key_flag <= key_flag_pre;
	
endmodule
