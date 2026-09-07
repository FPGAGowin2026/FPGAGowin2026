`timescale 1ns/1ns

module tlv5618(
	Clk,
	Rst_n,
	
	DAC_DATA,
	Start,
	Set_Done,
	
	DAC_CS_N,
	DAC_DIN,
	DAC_SCLK,
	DAC_State
);
	
	parameter fCLK = 50;
	parameter DIV_PARAM = 4;

	input Clk;
	input Rst_n;
	input [15:0]DAC_DATA;
	input Start;
	output reg Set_Done;
	
	output reg DAC_CS_N;
	output reg DAC_DIN;
	output reg DAC_SCLK;
	output DAC_State;

	assign DAC_State = DAC_CS_N;
	
	reg [15:0]r_DAC_DATA;
	
	reg [3:0]DIV_CNT;//分频计数器
	
	reg [5:0]SCLK_GEN_CNT;//SCLK生成暨序列机计数器
	
	reg en;//转换使能信号
	
	wire trans_done; //转换序列完成标志信号
	
	always@(posedge Clk or negedge Rst_n)
	if(!Rst_n)
		en  <= #1  1'b0;
	else if(trans_done)
		en  <= #1  1'b0;
	else if(Start)
		en  <= #1  1'b1;
	else
		en  <= #1  en;

	always@(posedge Clk or negedge Rst_n)
	if(!Rst_n)
		r_DAC_DATA  <= #1  16'd0;
	else if(Start)//收到开始发送命令时，寄存DAC_DATA值	
		r_DAC_DATA  <= #1  DAC_DATA;
	else
		r_DAC_DATA  <= #1  r_DAC_DATA;

	//生成2倍SCLK使能时钟计数器
	always@(posedge Clk or negedge Rst_n)
	if(!Rst_n)
		DIV_CNT  <= #1  4'd0;
	else if(en)begin
		if(DIV_CNT == (DIV_PARAM - 'd1))
			DIV_CNT  <= #1  4'd0;
		else 
			DIV_CNT  <= #1  DIV_CNT + 'd1;
	end else	
		DIV_CNT  <= #1  4'd0;
    reg flag   ;
    always@(posedge Clk) 
        flag <= DIV_CNT == (DIV_PARAM - 'd1);
		
	//生成序列计数器
	always@(posedge Clk or negedge Rst_n)
	if(!Rst_n)
		SCLK_GEN_CNT  <= #1  6'd0;
	else if(flag)begin
		if(SCLK_GEN_CNT == 6'd33)
			SCLK_GEN_CNT  <= #1  6'd0;
		else
			SCLK_GEN_CNT  <= #1  SCLK_GEN_CNT + 1'd1;
	end else
		SCLK_GEN_CNT  <= #1  SCLK_GEN_CNT;

				
	//依次将数据移出到DAC芯片		
	always@(posedge Clk or negedge Rst_n)
	if(!Rst_n)begin
		DAC_DIN  <= #1  1'b1;
		DAC_SCLK  <= #1  1'b0;
		DAC_CS_N <= #1 1'b1;
	end else if (flag) begin
		case(SCLK_GEN_CNT)
			0:
				begin
					DAC_CS_N <= #1 1'b0;
					DAC_DIN  <= #1 r_DAC_DATA[15];
					DAC_SCLK  <= #1  1'b1;
				end
		
			1,3,5,7,9,11,13,15,17,19,21,23,25,27,29,31:
				begin
					DAC_SCLK  <= #1  1'b0;
				end
			
			2:  begin DAC_DIN  <= #1 r_DAC_DATA[14]; DAC_SCLK  <= #1 1'b1; end
			4:  begin DAC_DIN  <= #1 r_DAC_DATA[13]; DAC_SCLK  <= #1 1'b1; end
			6:  begin DAC_DIN  <= #1 r_DAC_DATA[12]; DAC_SCLK  <= #1 1'b1; end
			8:  begin DAC_DIN  <= #1 r_DAC_DATA[11]; DAC_SCLK  <= #1 1'b1; end			
			10: begin DAC_DIN  <= #1 r_DAC_DATA[10]; DAC_SCLK  <= #1 1'b1; end
			12: begin DAC_DIN  <= #1 r_DAC_DATA[9];  DAC_SCLK  <= #1 1'b1; end
			14: begin DAC_DIN  <= #1 r_DAC_DATA[8];  DAC_SCLK  <= #1 1'b1; end
			16: begin DAC_DIN  <= #1 r_DAC_DATA[7];  DAC_SCLK  <= #1 1'b1; end	
			18: begin DAC_DIN  <= #1 r_DAC_DATA[6];  DAC_SCLK  <= #1 1'b1; end
			20: begin DAC_DIN  <= #1 r_DAC_DATA[5];  DAC_SCLK  <= #1 1'b1; end				
			22: begin DAC_DIN  <= #1 r_DAC_DATA[4];  DAC_SCLK  <= #1 1'b1; end
			24: begin DAC_DIN  <= #1 r_DAC_DATA[3];  DAC_SCLK  <= #1 1'b1; end
			26: begin DAC_DIN  <= #1 r_DAC_DATA[2];  DAC_SCLK  <= #1 1'b1; end
			28: begin DAC_DIN  <= #1 r_DAC_DATA[1];  DAC_SCLK  <= #1 1'b1; end			
			30: begin DAC_DIN  <= #1 r_DAC_DATA[0];  DAC_SCLK  <= #1 1'b1; end
			
			32: DAC_SCLK  <= #1 1'b1; 
			33: DAC_CS_N  <= #1 1'b1;
			default:;
		endcase
	end

	assign trans_done = flag && SCLK_GEN_CNT == 33;
	
	always@(posedge Clk or negedge Rst_n)
	if(!Rst_n)
		Set_Done <= 1'b0;
	else if(trans_done)
		Set_Done <= 1'b1;
	else
		Set_Done <= 1'b0;
	
endmodule