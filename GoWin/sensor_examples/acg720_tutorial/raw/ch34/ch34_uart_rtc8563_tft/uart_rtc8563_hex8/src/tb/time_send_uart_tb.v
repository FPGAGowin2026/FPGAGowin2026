/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2019/05/01 00:00:00
// Design Name: 
// Module Name: time_send_uart_tb
// Project Name: 
// Target Devices: XC7A35T-2FGG484I
// Tool Versions: Vivado 2018.3
// Description: time_send_uart模块仿真文件
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

module time_send_uart_tb();

  reg         clk50M       ; //系统时钟输入，50M
  reg         reset_n      ; //复位信号输入，低有效
  reg  [47:0] date_time    ;
  reg         read_done    ;
  wire        uart_tx_done ;
  wire [7:0]  uart_tx_data ;
  wire        uart_tx_en   ;
  wire        uart_tx      ;

	initial clk50M = 1;
	always#(`CLK_PERIOD/2)clk50M = ~clk50M;

  initial begin
    reset_n = 1'b0;
    date_time = 48'd0;
    read_done = 1'b0;
    #(`CLK_PERIOD*20 + 1 );
    reset_n = 1'b1;
    #(`CLK_PERIOD*50);
    date_time = 48'h19_05_01_10_23_56;
    #30000000;
    read_done = 1;
    #(`CLK_PERIOD  );
    read_done = 0;
    #1000000;
    read_done = 1;
    #(`CLK_PERIOD  );
    read_done = 0;
    
    date_time = 48'h19_06_10_12_45_12;
    #30000000;
    read_done = 1;
    #(`CLK_PERIOD  );
    read_done = 0;
    #1000000;
    read_done = 1;
    #(`CLK_PERIOD  );
    read_done = 0;    
    #1000000;
    $stop;	
  end
  
  time_send_uart time_send_uart(
    .clk(clk50M),
    .reset(~reset_n),
    .date_time_en(read_done),
    .date_time(date_time),
    .uart_tx_done(uart_tx_done),
    .uart_tx_data(uart_tx_data),
    .uart_tx_en(uart_tx_en)
  );
  
  uart_byte_tx uart_byte_tx
  (
    .clk       (clk50M       ),
    .reset_n   (reset_n      ),      
    .data_byte (uart_tx_data ),
    .send_en   (uart_tx_en   ),
    .baud_set  (3'd0         ),    
    .uart_tx   (uart_tx      ),
    .tx_done   (uart_tx_done ),
    .uart_state(             )
  );

endmodule
