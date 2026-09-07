/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2019/05/01 00:00:00
// Design Name: 
// Module Name: eth_rx_ctrl_tb
// Project Name: 
// Target Devices: XC7A35T-2FGG484I
// Tool Versions: Vivado 2018.3
// Description: 
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

module eth_rx_ctrl_tb;

  reg         clk;
  reg         reset_n;

  wire        fifo_full;
  reg         fifo_wr;
  reg  [7:0]  fifo_din;

  reg         one_pkt_done;
  reg         pkt_error;

  wire [15:0] rx_data;
  wire        rx_data_valid; 

  eth_rx_ctrl
  #(
    .O_DATA_WIDTH (16  )
  )eth_rx_ctrl
  (
    .clk          (clk          ),
    .reset        (~reset_n      ),

    .fifo_full    (fifo_full    ),
    .fifo_wr      (fifo_wr      ),
    .fifo_din     (fifo_din     ),

    .one_pkt_done (one_pkt_done ),
    .pkt_error    (pkt_error    ),

    .rx_data      (rx_data      ),
    .rx_data_valid(rx_data_valid)  
  );
  
  initial clk = 1'b1;
  always #(`CLK_PERIOD/2) clk = ~clk;

  initial begin
    reset_n = 1'b0;
    fifo_wr = 0;
    fifo_din = 0;
    one_pkt_done = 0;
    pkt_error = 0;
    #201;
    reset_n = 1'b1;
    #200;

    write_data(0,50);
    #2000;
    write_data(100,50);
    #2000;
    write_data_error(200,50);
    #2000;
    write_data(300,50);
    #2000;
    write_data_error(400,50);
    #2000;
    write_data(500,50);
    #2000;
    $stop;
  end
  
  task write_data;
    input [7:0]data_begin;
    input [10:0]data_cnt;
    begin
      fifo_wr = 0;
      fifo_din = data_begin;
      one_pkt_done = 0;
      pkt_error = 0;
      
      @(posedge clk);
      #1 fifo_wr = 1;
      repeat(data_cnt) begin
        @(posedge clk);
        #1 fifo_din = fifo_din + 1;
      end
      fifo_wr = 0;
      @(posedge clk);
      #1 one_pkt_done = 1;
      @(posedge clk);
      #1 one_pkt_done = 0;   
    end
  endtask

  task write_data_error;
    input [7:0]data_begin;
    input [10:0]data_cnt;
    begin
      fifo_wr = 0;
      fifo_din = data_begin;
      one_pkt_done = 0;
      pkt_error = 0;
      
      @(posedge clk);
      #1 fifo_wr = 1;
      repeat(data_cnt) begin
        @(posedge clk);
        #1 fifo_din = fifo_din + 1;
      end
      fifo_wr = 0;
      @(posedge clk);#1;
      one_pkt_done = 1;
      pkt_error = 1;
      @(posedge clk);#1;
      one_pkt_done = 0;
      pkt_error = 0;
    end
  endtask

endmodule