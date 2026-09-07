`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/09/27 23:48:17
// Design Name: 
// Module Name: FX2_SF_Loopback
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module FX2_SF_Loopback(
	clk,
	reset_n,
	fx2_fdata,      //  FX2型USB2.0芯片的SlaveFIFO的数据线
	fx2_faddr,      //  FX2型USB2.0芯片的SlaveFIFO的FIFO地址线
	fx2_slrd,       //  FX2型USB2.0芯片的SlaveFIFO的读控制信号，低电平有效
	fx2_slwr,       //  FX2型USB2.0芯片的SlaveFIFO的写控制信号，低电平有效
	fx2_sloe,       //  FX2型USB2.0芯片的SlaveFIFO的输出使能信号，低电平有效
	fx2_flagc,      //  FX2型USB2.0芯片的端点6满标志
	fx2_flagb,      //  FX2型USB2.0芯片的端点2空标志
	fx2_ifclk,      //  FX2型USB2.0芯片的接口时钟信号
	fx2_pkt_end,	//数据包结束标志信号
	fx2_clear,      //  FX2型USB2.0芯片的清除信号
	fx2_slcs
);
    input clk;
	input reset_n;
	inout [15:0]fx2_fdata;
	input fx2_flagb;
	input fx2_flagc;
	input fx2_ifclk;
    
	output [1:0]fx2_faddr;
	output fx2_sloe;
	output fx2_slwr;
	output fx2_slrd;
	output fx2_pkt_end;
	output fx2_slcs;
    output fx2_clear; 
	reg slrd_n;
	reg slwr_n;
	reg sloe_n;

	reg [15:0] fifo_data_in;
	wire [15:0] fifo_data_out;
	reg [15:0] data_out;

	reg [1:0]faddr_n;

	parameter [1:0] loop_back_idle       = 2'd0;
	parameter [1:0] loop_back_read       = 2'd1;
	parameter [1:0] loop_back_wait_flagc = 2'd2;
	parameter [1:0] loop_back_write      = 2'd3;

	reg [1:0]current_loop_back_state;
	reg [1:0]next_loop_back_state;

	wire fifo_full;
	wire fifo_empty;
	wire fifo_push;
	wire fifo_pop;
    
    assign fx2_clear = 0;
	assign fx2_slwr = slwr_n;
	assign fx2_slrd = slrd_n;
	assign fx2_sloe = sloe_n;
	assign fx2_faddr = faddr_n;
	assign fx2_pkt_end = 1'b1;
	assign fx2_slcs = 1'b0;

	assign fx2_fdata = data_out;

	always@(*)begin
		if((current_loop_back_state == loop_back_idle) | (current_loop_back_state == loop_back_read))begin
			faddr_n = 2'b00;
		end else begin
			faddr_n = 2'b10;
		end
	end	

	//读控制信号产生逻辑
	always@(*)begin
		if((current_loop_back_state == loop_back_read) & (fx2_flagb == 1'b1))begin
			slrd_n = 1'b0;
			sloe_n = 1'b0;
		end else begin
			slrd_n = 1'b1;
			sloe_n = 1'b1;
		end
	end	

	assign fifo_push = ((slrd_n == 1'b0) & (fifo_full == 1'b0));

	always@(*)begin
		if(slrd_n == 1'b0)
			fifo_data_in = fx2_fdata;
		else
			fifo_data_in = 16'd0;
	end	

	//写控制信号产生逻辑
	always@(*)begin
		if((current_loop_back_state == loop_back_write) & (fx2_flagc == 1'b1) & (fifo_empty == 1'b0))
			slwr_n <= 1'b0;
		else
			slwr_n <= 1'b1;
	end

	assign fifo_pop = ((slwr_n == 1'b0) & (fifo_empty == 1'b0));

	//回送模式状态机
	always@(posedge fx2_ifclk, negedge reset_n) begin
		if(reset_n == 1'b0)
			current_loop_back_state <= loop_back_idle;
		else
			current_loop_back_state <= next_loop_back_state;
	end

	//回送模式状态机组合逻辑
	always@(*) begin
		next_loop_back_state = current_loop_back_state;
		case(current_loop_back_state)
			loop_back_idle:begin
				if(fx2_flagb == 1'b1)
					next_loop_back_state = loop_back_read;
				else
					next_loop_back_state = loop_back_idle;
			end
			loop_back_read:begin
				if(fx2_flagb == 1'b0)
					next_loop_back_state = loop_back_wait_flagc;
				else
					next_loop_back_state = loop_back_read;
			end
			loop_back_wait_flagc:begin
				if(fx2_flagc == 1'b1)
					next_loop_back_state = loop_back_write;
				else 
					next_loop_back_state = loop_back_wait_flagc;
			end
			loop_back_write:begin
				if((fx2_flagc == 1'b0) | (fifo_empty == 1'b1))
					next_loop_back_state = loop_back_idle;
				else 
					next_loop_back_state = loop_back_write;
			end
			default: 
				next_loop_back_state = loop_back_idle;
		endcase		
	end

	always@(*)begin
		if(slwr_n == 1'b1)
			data_out = 16'dz;
		else
			data_out = fifo_data_out;
	end
    
    wire fifo_flush;
    assign fifo_flush = current_loop_back_state == loop_back_idle;
	//回送缓存fifo
	fifo_512x8 fifo_inst(
        .din	     (fifo_data_in),	
        .write_busy (fifo_push),
        .fifo_full  (fifo_full), 
        .dout	     (fifo_data_out),
        .read_busy  (fifo_pop),
        .fifo_empty (fifo_empty),
        .fifo_clk   (fx2_ifclk),
        .reset_   (reset_n),
        .fifo_flush (fifo_flush)
	);
	
	// 以下部分的代码用于ILA调试
	wire clk_96m;//生成96M时钟用于ILA采样
//	pll pll_inst(
//        .clk_out1(clk_96m),
//        .clk_in1(clk)
//    );
	

    Gowin_PLL Gowin_PLL(
        .clkout0(clk_96m), //output clkout0
        .clkin(clk) //input clkin
    );

//	ila_0 ila_0_inst(
//        .clk(clk_96m), // input wire clk
//        .probe0(fx2_fdata), // input wire [15:0]  probe0  
//        .probe1(fx2_faddr), // input wire [1:0]  probe1 
//        .probe2(fx2_flagb), // input wire [0:0]  probe2 
//        .probe3(fx2_flagc), // input wire [0:0]  probe3 
//        .probe4(fx2_sloe), // input wire [0:0]  probe4 
//        .probe5(fx2_slwr), // input wire [0:0]  probe5 
//        .probe6(fx2_slrd), // input wire [0:0]  probe6 
//        .probe7(fifo_empty), // input wire [0:0]  probe7 
//        .probe8(fifo_full), // input wire [0:0]  probe8 
//        .probe9(fifo_flush) // input wire [0:0]  probe9
//    );

endmodule
