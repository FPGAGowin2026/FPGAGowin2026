/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : uart_byte_rx_tb
// Description   : 串口接收模块仿真
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ns
`define CLK_PERIOD 20

module uart_byte_rx_tb;

  reg       clk50m;
  reg       reset;

  reg [2:0] baud_set;

  reg [7:0] data_byte_tx;
  reg       send_en;
  wire      tx_done;
  wire      uart_state;

  wire[7:0] data_byte_rx;
  wire      rx_done;

  wire      uart_tx_rx;

  uart_byte_tx uart_byte_tx(
    .clk       (clk50m      ),
    .reset     (reset       ),

    .data_byte (data_byte_tx),
    .send_en   (send_en     ),
    .baud_set  (baud_set    ),

    .uart_tx   (uart_tx_rx  ),
    .tx_done   (tx_done     ),
    .uart_state(uart_state  )
  );

  uart_byte_rx uart_byte_rx(
    .clk       (clk50m      ),
    .reset     (reset       ),

    .baud_set  (baud_set    ),
    .uart_rx   (uart_tx_rx  ),

    .data_byte (data_byte_rx),
    .rx_done   (rx_done     )
  );

  initial clk50m = 1;
  always#(`CLK_PERIOD/2)clk50m = ~clk50m;

  initial begin
    reset = 1'b1;
    data_byte_tx = 8'd0;
    send_en = 1'd0;
    baud_set = 3'd4;
    #(`CLK_PERIOD*20 + 1 );
    reset = 1'b0;
    #(`CLK_PERIOD*50);

    //send first byte
    data_byte_tx = 8'haa;
    send_en = 1'd1;
    #`CLK_PERIOD;
    send_en = 1'd0;

    @(posedge tx_done)
    #(`CLK_PERIOD*5000);

    //send second byte
    data_byte_tx = 8'h55;
    send_en = 1'd1;
    #`CLK_PERIOD;
    send_en = 1'd0;

    @(posedge tx_done)
    #(`CLK_PERIOD*5000);
    $stop;
  end

`ifdef SAVE_WAVE_FILE
  // dump fsdb file for debussy
  initial
  begin
    $fsdbDumpfile("wave.fsdb");
    $fsdbDumpvars;
  end
`endif

endmodule
