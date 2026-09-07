//////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: 
// Module Name: fsm_hello
// Project Name: 
// Description: 实现字符串“hello”检测，每检测成功一次产生一个时钟周期脉冲check_ok
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module fsm_hello(
	clk,
	reset_n,

	data_valid,
	data_in,

	check_ok
);

	input clk;          //模块全局时钟输入，50M
	input reset_n;         //复位信号输入，低有效
	wire reset=~reset_n;
	
	input data_valid; //输入字符数据有效标识
	input [7:0]data_in;    //字符数据输入
	
	output check_ok;   //字符串检测成功标识
	reg check_ok;
	
	localparam
		CHECK_h= 5'b0_0001,
		CHECK_e= 5'b0_0010,
		CHECK_l1= 5'b0_0100,
		CHECK_l2= 5'b0_1000,
		CHECK_o= 5'b1_0000;

	reg [4:0]state;
	
	always@(posedge clk or posedge reset)
	if(reset)begin
		check_ok <= 1'b0;
		state <= CHECK_h;
	end
	else begin
		case(state)
			CHECK_h:
				begin
				check_ok <= 1'b0;
				if(data_valid && data_in == "h") 
				state <= CHECK_e;
				else
				state <= CHECK_h;
				end

			CHECK_e:
				if(data_valid && data_in == "e") 
				state <= CHECK_l1;
				else if(data_valid && data_in == "h")
				state <= CHECK_e;
				else if(data_valid)
				state <= CHECK_h;
				else
				state <= CHECK_e;
					
			CHECK_l1:
				if(data_valid && data_in == "l")
				state <= CHECK_l2;
				else if(data_valid && data_in == "h")
				state <= CHECK_e;
				else if(data_valid)
				state <= CHECK_h;
				else
				state <= CHECK_l1;
					
			CHECK_l2:
				if(data_valid && data_in == "l")
				state <= CHECK_o;
				else if(data_valid && data_in == "h")
				state <= CHECK_e;
				else if(data_valid)
				state <= CHECK_h;
				else
				state <= CHECK_l2;
					
			CHECK_o:
				if(data_valid && data_in == "h")
				state <= CHECK_e;
				else if(data_valid)begin
					state <= CHECK_h;
					if(data_in == "o")
						check_ok <= 1'b1;
					else
						check_ok <= 1'b0;
				end
				else
					state <= CHECK_o;
				
			default:state <= CHECK_h;
		endcase		
	end

endmodule
