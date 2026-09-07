`timescale 1ns/1ps

// 8-N-1 UART 发送器。data_valid_i 与 data_ready_o 同拍握手。
module bringup_uart_tx #(
    parameter integer CLK_FREQ_HZ=50000000,parameter integer BAUD_RATE=115200
)(input wire clk,input wire rst_n,input wire [7:0] data_i,input wire data_valid_i,
  output wire data_ready_o,output reg tx_o,output reg busy_o);
    localparam integer SAFE_BAUD_RATE=(BAUD_RATE<1)?1:BAUD_RATE;
    localparam integer RAW_CLKS_PER_BIT=(CLK_FREQ_HZ+(SAFE_BAUD_RATE/2))/SAFE_BAUD_RATE;
    localparam integer CLKS_PER_BIT=(RAW_CLKS_PER_BIT<2)?2:RAW_CLKS_PER_BIT;
    reg [31:0] clock_count;reg [3:0] bit_index;reg [9:0] frame_shift;
    assign data_ready_o=!busy_o;
    always @(posedge clk or negedge rst_n) begin
      if(!rst_n)begin clock_count<=0;bit_index<=0;frame_shift<=10'h3ff;tx_o<=1;busy_o<=0;end
      else if(!busy_o)begin clock_count<=0;bit_index<=0;tx_o<=1;
        if(data_valid_i)begin frame_shift<={1'b1,data_i,1'b0};tx_o<=0;busy_o<=1;end end
      else if(clock_count>=(CLKS_PER_BIT-1))begin clock_count<=0;
        if(bit_index==9)begin bit_index<=0;tx_o<=1;busy_o<=0;end
        else begin bit_index<=bit_index+1'b1;tx_o<=frame_shift[bit_index+1'b1];end end
      else clock_count<=clock_count+1'b1;
    end
endmodule
