/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: time_send_uart
// Module Name: time_send_uart
// Project Name: time_send_uart
// Target Devices: XC7A35T-2FGG484I
// Tool Versions: Vivado 2018.3
// Description: 串口时钟发送模块
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module time_send_uart(
  //clock reset
  clk        , //系统时钟输入，50M
  reset      , //复位信号输入，高有效

  date_time_en  ,
  date_time     ,

  uart_tx_done  ,
  uart_tx_data  ,
  uart_tx_en
);

  input clk;       //系统时钟输入，50M
  input reset;   //复位信号输入，低有效

  input date_time_en;
  input [47:0]date_time;

  input uart_tx_done;
  output [7:0]uart_tx_data;
  output uart_tx_en;

  reg [7:0]uart_tx_data;
  reg uart_tx_en;
  reg [47:0]date_time_latch;
  reg [47:0]date_time_latch_dly1;
  reg date_time_en_dly1;
  reg time_data_change;
  reg [111:0]data_time_ascii;
  reg [4:0]send_byte_cnt;
  reg uart_tx_en_pre;

//数据发生变化就通过串口发送一次数据
  always@(posedge clk or posedge reset)
  begin
    if(reset)
      date_time_latch <= 'd0;
    else if(date_time_en)
      date_time_latch <= date_time;
  end

  always@(posedge clk or posedge reset)
  begin
    if(reset)
      time_data_change <= 1'b0;
    else if(date_time_en & (date_time_latch != date_time))
      time_data_change <= 1'b1;
    else
      time_data_change <= 1'b0;
  end
  
  //bcd translate ascii
  always @ (posedge clk)
  if (time_data_change) begin
    data_time_ascii[111:104] <= 4'd2 + 8'h30;//"2"
    data_time_ascii[103:96 ] <= 4'd0 + 8'h30;//"0"
    data_time_ascii[95 :88 ] <= date_time_latch[47:44] + 8'h30;//YY
    data_time_ascii[87 :80 ] <= date_time_latch[43:40] + 8'h30;
    data_time_ascii[79 :72 ] <= date_time_latch[39:36] + 8'h30;//MM
    data_time_ascii[71 :64 ] <= date_time_latch[35:32] + 8'h30;
    data_time_ascii[63 :56 ] <= date_time_latch[31:28] + 8'h30;//DD
    data_time_ascii[55 :48 ] <= date_time_latch[27:24] + 8'h30;
    data_time_ascii[47 :40 ] <= date_time_latch[23:20] + 8'h30;//hh
    data_time_ascii[39 :32 ] <= date_time_latch[19:16] + 8'h30;
    data_time_ascii[31 :24 ] <= date_time_latch[15:12] + 8'h30;//mm
    data_time_ascii[23 :16 ] <= date_time_latch[11:8]  + 8'h30;
    data_time_ascii[15 :8  ] <= date_time_latch[7:4]   + 8'h30;//ss
    data_time_ascii[7  :0  ] <= date_time_latch[3:0]   + 8'h30;
  end

  always@(posedge clk or posedge reset)
  begin
    if(reset)
      send_byte_cnt <= 'd0;
    else if(time_data_change)
      send_byte_cnt <= 'd0;
    else if(uart_tx_done)
      send_byte_cnt <= send_byte_cnt + 1'b1;
    else
      send_byte_cnt <= send_byte_cnt;
  end

  always@(posedge clk or posedge reset)
  begin
    if(reset)
      uart_tx_en_pre <= 1'b0;
    else if(time_data_change || (uart_tx_done && send_byte_cnt <= 'd18))
      uart_tx_en_pre <= 1'b1;
    else
      uart_tx_en_pre <= 1'b0;
  end

  always@(posedge clk or posedge reset)
  begin
    if(reset)
    begin
      uart_tx_data <= 'd0;
      uart_tx_en <= 1'b0;
    end
    else if(uart_tx_en_pre)
    begin
      case(send_byte_cnt)
        5'd0 : begin uart_tx_data <= data_time_ascii[111:104];  uart_tx_en <= 1'b1; end  //2 YYYY
        5'd1 : begin uart_tx_data <= data_time_ascii[103:96 ];  uart_tx_en <= 1'b1; end  //0
        5'd2 : begin uart_tx_data <= data_time_ascii[95 :88 ];  uart_tx_en <= 1'b1; end  //1
        5'd3 : begin uart_tx_data <= data_time_ascii[87 :80 ];  uart_tx_en <= 1'b1; end  //9
        5'd4 : begin uart_tx_data <= 8'h2D;                     uart_tx_en <= 1'b1; end  //-
        5'd5 : begin uart_tx_data <= data_time_ascii[79 :72 ];  uart_tx_en <= 1'b1; end  //0 MM
        5'd6 : begin uart_tx_data <= data_time_ascii[71 :64 ];  uart_tx_en <= 1'b1; end  //1
        5'd7 : begin uart_tx_data <= 8'h2D;                     uart_tx_en <= 1'b1; end  //-
        5'd8 : begin uart_tx_data <= data_time_ascii[63 :56 ];  uart_tx_en <= 1'b1; end  //0 DD
        5'd9 : begin uart_tx_data <= data_time_ascii[55 :48 ];  uart_tx_en <= 1'b1; end  //1
        5'd10: begin uart_tx_data <= 8'h20;                    uart_tx_en <= 1'b1; end  //space
        5'd11: begin uart_tx_data <= data_time_ascii[47 :40 ]; uart_tx_en <= 1'b1; end  //1 hh
        5'd12: begin uart_tx_data <= data_time_ascii[39 :32 ]; uart_tx_en <= 1'b1; end  //2
        5'd13: begin uart_tx_data <= 8'h3A;                    uart_tx_en <= 1'b1; end  //:
        5'd14: begin uart_tx_data <= data_time_ascii[31 :24 ]; uart_tx_en <= 1'b1; end  //0 mm
        5'd15: begin uart_tx_data <= data_time_ascii[23 :16 ]; uart_tx_en <= 1'b1; end  //0
        5'd16: begin uart_tx_data <= 8'h3A;                    uart_tx_en <= 1'b1; end  //:
        5'd17: begin uart_tx_data <= data_time_ascii[15 :8  ]; uart_tx_en <= 1'b1; end  //0 ss
        5'd18: begin uart_tx_data <= data_time_ascii[7  :0  ]; uart_tx_en <= 1'b1; end  //0
        5'd19: begin uart_tx_data <= 8'h0A;                    uart_tx_en <= 1'b1; end  // LF(换行)
        default:begin uart_tx_data <= 'd0;                     uart_tx_en <= 1'b0; end
      endcase
    end
    else
    begin
      uart_tx_data <= uart_tx_data;
      uart_tx_en <= 1'b0;
    end
end

endmodule
