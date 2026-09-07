/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: uart_rtc8563_hex8
// Module Name: uart_adjust_time
// Project Name: uart_rtc8563_hex8
// Target Devices: XC7A35T-2FGG484I
// Tool Versions: Vivado 2018.3
// Description: 串口输入的时间调整模块
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module uart_adjust_time(
    //System clock reset
    input clk, //系统时钟输入，50M
    input reset_n, //复位信号输入，低有效
    
    input [7:0]uart_rx_data   , //串口接收数据
    input uart_data_valid, //串口接收数据有效标识

    input set_done,//时间或日期设置完成
    output reg set_time       , //设置时间使能
    output  [23:0]set_time_data  , //设置时间数据
    output set_date       , //设置日期使能
    output  [31:0]set_date_data    //设置日期数据
);

//调整时间的串口接收素数据协议命令格式
//           header         + 年+月+日+周+时+分+秒
//0x01 + 0x03 + 0x02 + 0x04 + 年+月+日+周+时+分+秒
  reg [87:0]rx_cmd_data;
  wire      cmd_checked;     //如果为真，表示检测到帧头，开始将收到的时间校准到时钟芯片
  reg       cmd_checked_dly1;
  reg       set_state;       //0:设置时间，1:设置日期

  always @ (posedge clk or negedge reset_n)
  begin
    if (!reset_n)
      rx_cmd_data <= 'd0;
    else if (uart_data_valid)
      rx_cmd_data <= {rx_cmd_data[79:0], uart_rx_data};
    else
      rx_cmd_data <= rx_cmd_data;
  end

  assign cmd_checked = (rx_cmd_data[87:56] == 32'h01030204);

  //接收到正确的时间调整命令后，先对时间进行调整，时间设置完成后再对日期进行调整更新
  always @ (posedge clk or negedge reset_n)
  begin
    if (!reset_n)
      cmd_checked_dly1 <= 1'b0;
    else
      cmd_checked_dly1 <= cmd_checked;
  end

  //第1步设置时间，确保设置成功
  always @ (posedge clk or negedge reset_n)
  begin
    if (!reset_n)
      set_time <= 1'b0;
    else if(set_done)
      set_time <= 1'b0;
    else if(cmd_checked && (!cmd_checked_dly1))
      set_time <= 1'b1;
  end

  assign set_time_data = rx_cmd_data[23:0];               //时间调整数据，00:23:20

  always @ (posedge clk or negedge reset_n)
  begin
    if (!reset_n)
      set_state <= 1'b0;
    else if(set_time)
      set_state <= 1'b1;
    else if(set_date)
      set_state <= 1'b0;
  end

  //第2步，时间调整完成后使能设置日期
  assign set_date = (set_done & (set_state == 1)) ? 1'b1 : 1'b0;
  assign set_date_data = {rx_cmd_data[55:40], rx_cmd_data[31:24], rx_cmd_data[39:32]};//日期调整数据，19-05-01

endmodule
