/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2019/05/01 00:00:00
// Design Name: 
// Module Name: uart_adjust_time_tb
// Project Name: 
// Target Devices: XC7A35T-2FGG484I
// Tool Versions: Vivado 2018.3
// Description: uart_adjust_time模块仿真文件
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

module uart_adjust_time_tb();

reg         clk50M         ;
reg         reset_n        ;
reg  [7:0]  uart_tx_data   ;
reg         uart_tx_en     ;
reg         set_done       ;
wire        uart_tx_done   ;
wire        uart_rx        ;
wire [7:0]  uart_rx_data   ; //串口接收数据
wire        uart_data_valid; //串口接收数据有效标识
wire        set_time       ; //设置时间使能
wire [23:0] set_time_data  ; //设置时间数据
wire        set_date       ; //设置日期使能
wire [31:0] set_date_data  ; //设置日期数据
reg  [87:0] adjust_cmd_data_reg;

integer     i;

//defparam uart_adjust_time.DLY_CNT_MAX = 500;

  initial clk50M = 1;
  always#(`CLK_PERIOD/2)clk50M = ~clk50M;

  initial begin
    reset_n = 1'b0;
    uart_tx_data = 8'd0;
    uart_tx_en = 1'b0;
    adjust_cmd_data_reg = 88'd0;
    set_done = 1'b0;
    #(`CLK_PERIOD*20 + 1 );
    reset_n = 1'b1;
    #(`CLK_PERIOD*50);
    
    send_adjust_cmd(88'h01030204_191111_01_235930);
    #1000000;
    set_done = 1;
    #(`CLK_PERIOD  );
    set_done = 0;
    #1000000;
    set_done = 1;
    #(`CLK_PERIOD  );
    set_done = 0;
     
    send_adjust_cmd(88'h01030204_191113_03_124956);
    #1000000;
    set_done = 1;
    #(`CLK_PERIOD  );
    set_done = 0;
    #1000000;
    set_done = 1;
    #(`CLK_PERIOD  );
    set_done = 0;
    #4000000;
    $stop;	
  end

task send_adjust_cmd;
  input [87:0] adjust_cmd_data;
  begin
    adjust_cmd_data_reg = adjust_cmd_data;
    for(i=11;i>0;i=i-1) begin
    uart_tx_data = adjust_cmd_data_reg[87:80];
    uart_tx_en = 1'b1;
    #(`CLK_PERIOD);
    uart_tx_en = 1'b0; 
    @(posedge uart_tx_done);
    adjust_cmd_data_reg = adjust_cmd_data_reg << 8;
    #40;
    end  
  end
endtask

  uart_byte_tx uart_byte_tx
  (
    .clk    (clk50M       ),
    .reset_n   (reset_n      ),      
    .data_byte (uart_tx_data ),
    .send_en   (uart_tx_en   ),
    .baud_set  (3'd0         ),    
    .uart_tx   (uart_rx      ),
    .tx_done   (uart_tx_done ),
    .uart_state(             )
  );

  uart_byte_rx uart_byte_rx
  (
    .clk   (clk50M          ),
    .reset_n  (reset_n         ),
    .baud_set (3'd0            ),
    .uart_rx  (uart_rx         ),
    .data_byte(uart_rx_data    ),
    .rx_done  (uart_data_valid )
  );
  
    uart_adjust_time uart_adjust_time
  (
    //clock reset
    .clk            (clk50M          ), //系统时钟输入＿50M
    .reset_n        (reset_n         ), //复位信号输入，低有效
    .uart_rx_data   (uart_rx_data    ), //串口接收数据
    .uart_data_valid(uart_data_valid ), //串口接收数据有效标识
    .set_done       (set_done        ),
    .set_time       (set_time        ), //设置时间使能
    .set_time_data  (set_time_data   ), //设置时间数据
    .set_date       (set_date        ), //设置日期使能
    .set_date_data  (set_date_data   )  //设置日期数据
  );


endmodule
