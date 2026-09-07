/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: Max
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: keyboard_4x4
// Module Name: keyboard_4x4_model
// Project Name: keyboard_4x4
// Description: 矩阵键盘模拟按键按下和松开过程
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns

module keyboard_4x4_model
(
	press_key,
	press_key_value,
	keyboard_col_x4_i,
	keyboard_row_x4_o
);
	input press_key;
	input [3:0]press_key_value;
	input [3:0]keyboard_col_x4_i;
	output [3:0]keyboard_row_x4_o;

	reg [3:0]keyboard_row_x4_o_tmp;

	pullup(keyboard_row_x4_o[0]);
	pullup(keyboard_row_x4_o[1]);
	pullup(keyboard_row_x4_o[2]);
	pullup(keyboard_row_x4_o[3]);

	assign keyboard_row_x4_o[0] = ~keyboard_row_x4_o_tmp[0] ? 1'b0 : 1'bz;
	assign keyboard_row_x4_o[1] = ~keyboard_row_x4_o_tmp[1] ? 1'b0 : 1'bz;
	assign keyboard_row_x4_o[2] = ~keyboard_row_x4_o_tmp[2] ? 1'b0 : 1'bz;
	assign keyboard_row_x4_o[3] = ~keyboard_row_x4_o_tmp[3] ? 1'b0 : 1'bz;

	reg [15:0]myrand;

	initial begin
    while(1) begin
		keyboard_row_x4_o_tmp = 4'b1111;
		@(posedge press_key)
			case(press_key_value)
				4'h0: press_key_task(0,0);
				4'h1: press_key_task(0,1);
				4'h2: press_key_task(0,2);
				4'h3: press_key_task(0,3);
				4'h4: press_key_task(1,0);
				4'h5: press_key_task(1,1);
				4'h6: press_key_task(1,2);
				4'h7: press_key_task(1,3);
				4'h8: press_key_task(2,0);
				4'h9: press_key_task(2,1);
				4'ha: press_key_task(2,2);
				4'hb: press_key_task(2,3);
				4'hc: press_key_task(3,0);
				4'hd: press_key_task(3,1);
				4'he: press_key_task(3,2);
				4'hf: press_key_task(3,3);
			endcase
		end
	end

	task press_key_task;
		input [1:0] row;
		input [1:0] col;
		begin
			keyboard_row_x4_o_tmp = 4'b1111;
			//模拟按键按下
			keyboard_row_x4_o_tmp[row] = keyboard_col_x4_i[col];
			//模拟按键按下抖动
			repeat(20)begin
			myrand = {$random} % 65536;
			#myrand keyboard_row_x4_o_tmp[row] = ~keyboard_row_x4_o_tmp[row];
			end
			//按键已经按下并稳定22ms(20ms以内认为是抖动，仿真中大于20ms即可)
			repeat(22000000)begin
			keyboard_row_x4_o_tmp[row] = keyboard_col_x4_i[col];
			#1;
			end
			//模拟按键松开过程的抖动
			repeat(20)begin
			myrand = {$random} % 65536;
			#myrand keyboard_row_x4_o_tmp[row] = ~keyboard_row_x4_o_tmp[row];
			end
			//按键松开
			keyboard_row_x4_o_tmp = 4'b1111;
			#22000000;		
		end
	endtask
	
endmodule
