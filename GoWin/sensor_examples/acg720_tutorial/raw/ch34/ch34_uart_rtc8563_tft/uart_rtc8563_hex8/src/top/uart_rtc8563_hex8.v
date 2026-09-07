//////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 22:48:59
// Design Name: uart_rtc8563_hex8
// Module Name: uart_rtc8563_hex8
// Project Name: uart_rtc8563_hex8
// Target Devices: XC7A35T-2FGG484I
// Tool Versions: Vivado 2018.3
// Description: uart_rtc8563_hex8顶层调度文件
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module uart_rtc8563_hex8(
  //external clock, reset
  input clk50M,
  input reset_n,
  //key
  input key_in0,
  //LED
  output [1:0]led,
  //uart interface
  input uart_rx,
  output uart_tx,
  //hex8 interface
  output sh_cp,
  output st_cp,
  output ds,
  //pcf8563 interface
  output i2c_sclk,
  inout i2c_sdat
);

  wire disp_data_sel;
  wire [7:0]uart_rx_data; //串口接收数据
  wire uart_data_valid; //串口接收数据有效标识
  wire uart_tx_done;
  wire [7:0]uart_tx_data;
  wire uart_tx_en;
  wire set_done;
  wire set_time; //设置时间使能
  wire [23:0]set_time_data; //设置时间数据
  wire set_date; //设置日期使能
  wire [31:0]set_date_data; //设置日期数据
  reg  [25:0]read_tcnt;
  wire read;
  wire [23:0]time_data;
  wire [31:0]date_data;  
  wire read_done;
  wire [31:0]disp_data;
  wire [47:0]date_time;

  assign led[0] = ~disp_data_sel;
  assign led[1] = read_done;
 
  key_filter key_filter
  (
    .clk(clk50M),
    .reset_n(reset_n),
    .key_in(key_in0),
    .key_flag(),
    .key_state(disp_data_sel)
  );

  uart_byte_rx uart_byte_rx
  (
    .clk(clk50M),
    .reset_n(reset_n),
    .baud_set (3'd0),
    .uart_rx(uart_rx),
    .data_byte(uart_rx_data),
    .rx_done(uart_data_valid )
  );

  uart_byte_tx uart_byte_tx
  (
    .clk(clk50M),
    .reset_n(reset_n),
    .data_byte(uart_tx_data),
    .send_en(uart_tx_en),
    .baud_set(3'd0),    
    .uart_tx(uart_tx),
    .tx_done(uart_tx_done),
    .uart_state()
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

  assign date_time = {date_data[31:16],date_data[7:0],time_data};

  time_send_uart time_send_uart(
    .clk(clk50M),
    .reset(~reset_n),
    .date_time_en(read_done),
    .date_time(date_time),
    .uart_tx_done(uart_tx_done),
    .uart_tx_data(uart_tx_data),
    .uart_tx_en(uart_tx_en)
  );

  always@(posedge clk50M or negedge reset_n)
  begin
    if(!reset_n)
    read_tcnt <= 26'd0;
    else if(read_tcnt == 26'd14_999_999)
    read_tcnt <= 26'd0;
    else
    read_tcnt <= read_tcnt + 1'd1;
  end

  assign read = (read_tcnt == 26'd14_999_999);

  pcf8563_ctrl pcf8563_ctrl(
    .Clk(clk50M),
    .Rst_n(reset_n),
    .Set_Time(set_time),
    .Time_to_Set(set_time_data),
    .Set_Date(set_date),
    .Date_to_Set(set_date_data),
    .Read(read),
    .Time_Read(time_data),
    .Date_Read(date_data),
    .Set_Done(set_done),
    .Read_Done(read_done),
    .i2c_sclk(i2c_sclk),
    .i2c_sdat(i2c_sdat)
  ); 

  assign disp_data = disp_data_sel ? {time_data[23:16],4'hf,time_data[15:8] ,4'hf,time_data[7:0]} :
                                     {8'h20,date_data[31:24],date_data[23:16],date_data[7:0]};

  hex_top_special_for_rtc hex_top_special_for_rtc(
    .clk (clk50M),
    .reset_n (reset_n),
    .disp_data (disp_data),
    .sh_cp (sh_cp),
    .st_cp (st_cp),
    .ds (ds)
  );

endmodule
