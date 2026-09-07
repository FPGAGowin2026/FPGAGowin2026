`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/10/10 13:57:20
// Design Name: 
// Module Name: FX2_Stream_IN_JPEG
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

module FX2_Stream_IN_JPEG(
	reset_n,
	fx2_fdata, //  FX2型USB2.0芯片的SlaveFIFO的数据线
	fx2_faddr, //  FX2型USB2.0芯片的SlaveFIFO的FIFO地址线
	fx2_slrd,  //  FX2型USB2.0芯片的SlaveFIFO的读控制信号，低电平有效
	fx2_slwr,  //  FX2型USB2.0芯片的SlaveFIFO的写控制信号，低电平有效
	fx2_sloe,  //  FX2型USB2.0芯片的SlaveFIFO的输出使能信号，低电平有效
	fx2_flagc, //  FX2型USB2.0芯片的端点6满标志
	fx2_flagb, //  FX2型USB2.0芯片的端点2空标志
	fx2_ifclk, //  FX2型USB2.0芯片的接口时钟信号
	fx2_clear, //  FX2型USB2.0芯片的清除信号
	fx2_pkt_end,	//数据包结束标志信号
	fx2_slcs	
);

	input reset_n;
	output [15:0]fx2_fdata;
	input fx2_flagb;
	input fx2_flagc;
	input fx2_ifclk;
    output fx2_clear;
    
	output [1:0]fx2_faddr;
	output fx2_sloe;
	output fx2_slwr;
	output fx2_slrd;
	output fx2_pkt_end;
	output fx2_slcs;
	
	wire [15:0]fifo_wrdata;
	reg fifo_wrreq;
	wire [10:0]fifo_usedw;

	reg [13:0]rom_address;
	assign fx2_clear = 0;
	
	//例化USB Stream in数据发送模块
	usb_stream_in usb_stream_in(
		.fx2_fdata(fx2_fdata),                               
		.fx2_faddr(fx2_faddr),                               
		.fx2_slrd(fx2_slrd),                                
		.fx2_slwr(fx2_slwr),                                
		.fx2_sloe(fx2_sloe), 
		.fx2_flagc(fx2_flagc),                               
		.fx2_flagb(fx2_flagb),                               
		.fx2_ifclk(fx2_ifclk),  
		.fx2_pkt_end(fx2_pkt_end),		
		.fx2_slcs(fx2_slcs),	
		.reset_n(reset_n),
		.fifo_wrdata(fifo_wrdata),
		.fifo_wrreq(fifo_wrreq),
		.fifo_usedw(fifo_usedw)
	);
	


    Gowin_pROM Gowin_pROM(
        .dout(fifo_wrdata), //output [10239:0] dout
        .clk(fx2_ifclk), //input clk
        .oce(1'b1), //input oce
        .ce(1'b1), //input ce
        .reset(~reset_n), //input reset
        .ad(rom_address) //input [3:0] ad
    );
	
	always@(posedge fx2_ifclk, negedge reset_n)
	if(!reset_n)begin
		fifo_wrreq <= 0;
		rom_address <= 0;
	end
	else if(fifo_usedw < 512)begin
		fifo_wrreq <= 1;
		if(rom_address >= 10239)
			rom_address <= 0;
		else
			rom_address <= rom_address + 1'b1;
	end
	else
		fifo_wrreq <= 0;

endmodule
