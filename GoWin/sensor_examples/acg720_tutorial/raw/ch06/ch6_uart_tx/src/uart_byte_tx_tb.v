//////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: uart_tx
// Module Name: uart_byte_tx_tb
// Project Name: uart_tx
// Description: 测试串口输出
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns
`define CLK_PERIOD 20

module uart_byte_tx_tb;

	reg clk;
	reg reset_n;
	reg [7:0]data_byte;
	reg send_en;
	reg [2:0]baud_set;
	
	wire uart_tx;
	wire tx_done;
	wire uart_state;
	
	uart_byte_tx uart_byte_tx(
		.clk(clk),
		.reset_n(reset_n),

		.data_byte(data_byte),
		.send_en(send_en),
		.baud_set(baud_set),

		.uart_tx(uart_tx),
		.tx_done(tx_done),
		.uart_state(uart_state)
	);
	
	initial clk = 1;
	always#(`CLK_PERIOD/2)clk = ~clk;
	
	initial begin
		reset_n = 1'b0;
		data_byte = 8'd0;
		send_en = 1'd0;
		baud_set = 3'd4;
		#(`CLK_PERIOD*500 + 1 )
		reset_n = 1'b1;
		#(`CLK_PERIOD*50);
    
    //send first byte
		data_byte = 8'haa;
		send_en = 1'd1;
		#`CLK_PERIOD;
		send_en = 1'd0;

		@(posedge tx_done)
		#(`CLK_PERIOD*5000);
    
    //send second byte
		data_byte = 8'h55;
		send_en = 1'd1;
		#`CLK_PERIOD;
		send_en = 1'd0;

		@(posedge tx_done)
		#(`CLK_PERIOD*5000);
		$stop;	
	end

endmodule
