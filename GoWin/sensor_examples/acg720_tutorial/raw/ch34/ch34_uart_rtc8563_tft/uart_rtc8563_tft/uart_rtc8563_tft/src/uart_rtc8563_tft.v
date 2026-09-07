/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2019/05/01 00:00:00
// Design Name: 
// Module Name: uart_rtc8563_tft
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

`include "..\define\define.v"

module uart_rtc8563_tft(
  //external clock, reset
  input         clk50M,
  input         reset_n,
  //key
  input         key_in0,
  //LED
  output [2:0]  led,
  //uart interface              
  input         uart_rx,
  output        uart_tx,
  //TFT interface
	output [15:0] TFT_rgb,
	output        TFT_hs,
	output        TFT_vs,
	output        TFT_clk,
	output        TFT_de,
	output        TFT_pwm,
  //pcf8563 interface
	output        i2c_sclk,
	inout         i2c_sdat
);
//设置待显示背景颜色和字符颜色
  parameter DISP_BACK_COLOR = 16'hFFFF; //背景白色
  parameter DISP_CHAR_COLOR = 16'h001F; //显示字符黑色
//在define.v文件设置选择屏幕尺寸
`ifdef SEL_TFT_4_3
  parameter TFT_WIDTH  = 480;
  parameter TFT_HEIGHT = 272;
`else
  parameter TFT_WIDTH  = 800;
  parameter TFT_HEIGHT = 480;
`endif
//字符显示在屏幕中间位置
  parameter DISP_HBEGIN = (TFT_WIDTH  - 160)/2;
  parameter DISP_VBEGIN = (TFT_HEIGHT - 32 )/2;

wire        sys_clk50M     ;
wire        clk33M         ;
wire        clk9M          ;
wire        pll_locked     ;
wire        sys_rst_n      ;
wire        tft_ctrl_clk   ;
wire        disp_data_sel  ;
wire [7:0]  uart_rx_data   ; //串口接收数据
wire        uart_data_valid; //串口接收数据有效标识
wire        uart_tx_done   ;
wire [7:0]  uart_tx_data   ;
wire        uart_tx_en     ;
wire        set_done       ;
wire        set_time       ; //设置时间使能
wire [23:0] set_time_data  ; //设置时间数据
wire        set_date       ; //设置日期使能
wire [31:0] set_date_data  ; //设置日期数据
reg  [25:0] read_tcnt      ;
wire        read           ;
wire [23:0] time_data      ;
wire [31:0] date_data      ;	
wire        read_done      ;
wire [39:0] disp_data_pre  ;
reg  [39:0] disp_data      ;
reg         scan_en_pulse  ;
wire [47:0] date_time      ;

  assign sys_rst_n = pll_locked;
  assign led[0]    = pll_locked;
  assign led[1]    = ~disp_data_sel;
  assign led[2]    = read_done;

`ifdef SEL_TFT_4_3
  assign tft_ctrl_clk = clk9M;
`else
  assign tft_ctrl_clk = clk33M;
`endif

    pll_tft_4 pll_tft_4(
        .lock(pll_locked), //output lock
        .clkout0(sys_clk50M), //output clkout0
        .clkout1(clk9M), //output clkout1
        .clkin(clk50M), //input clkin
        .reset(~reset_n) //input reset
    );

    pll_tft_5 pll_tft_5(
        .clkout0(clk33M), //output clkout0
        .clkin(sys_clk50M) //input clkin
    );
 
  key_filter key_filter
  (
    .clk50M   (sys_clk50M    ),
    .reset_n  (sys_rst_n     ),
    .key_in   (key_in0       ),
    .key_flag (              ),
    .key_state(disp_data_sel )
  );

  uart_byte_rx uart_byte_rx
  (
    .clk50M   (sys_clk50M      ),
    .reset_n  (sys_rst_n       ),
    .baud_set (3'd0            ),
    .uart_rx  (uart_rx         ),
    .data_byte(uart_rx_data    ),
    .rx_done  (uart_data_valid )
  );

  uart_byte_tx uart_byte_tx
  (
    .clk50M    (sys_clk50M   ),
    .reset_n   (sys_rst_n    ),      
    .data_byte (uart_tx_data ),
    .send_en   (uart_tx_en   ),
    .baud_set  (3'd0         ),    
    .uart_tx   (uart_tx      ),
    .tx_done   (uart_tx_done ),
    .uart_state(             )
  );

  uart_adjust_time uart_adjust_time
  (
    //clock reset
    .clk            (sys_clk50M      ), //系统时钟输入＿50M
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
    .clk(sys_clk50M),
    .reset(~reset_n),
    .date_time_en(read_done),
    .date_time(date_time),
    .uart_tx_done(uart_tx_done),
    .uart_tx_data(uart_tx_data),
    .uart_tx_en(uart_tx_en)
  );

  always@(posedge sys_clk50M or negedge sys_rst_n)
  begin
    if(!sys_rst_n)
      read_tcnt <= 26'd0;
    else if(read_tcnt == 26'd14_999_999)
      read_tcnt <= 26'd0;
    else
      read_tcnt <= read_tcnt + 1'd1;
  end

  assign read = (read_tcnt == 26'd14_999_999);

  pcf8563_ctrl pcf8563_ctrl(
    .Clk        (sys_clk50M    ),
    .Rst_n      (sys_rst_n     ),
    .Set_Time   (set_time      ),
    .Time_to_Set(set_time_data ),
    .Set_Date   (set_date      ),
    .Date_to_Set(set_date_data ),
    .Read       (read          ),
    .Time_Read  (time_data     ),
    .Date_Read  (date_data     ),
    .Set_Done   (set_done      ),
    .Read_Done  (read_done     ),
    .i2c_sclk   (i2c_sclk      ),
    .i2c_sdat   (i2c_sdat      )
  ); 

  assign disp_data_pre = disp_data_sel ? {4'hf,time_data[23:16],4'hb,time_data[15:8] ,4'hb,time_data[7:0],4'hf} : // 10:10:10 
                                         {8'h20,date_data[31:24],4'ha,date_data[23:16],4'ha,date_data[7:0]};    //2019-11-11

  always@(posedge sys_clk50M or negedge sys_rst_n)
  begin
    if(!sys_rst_n)
      disp_data <= 40'd0;
    else
      disp_data <= disp_data_pre;
  end

  always@(posedge sys_clk50M or negedge sys_rst_n)
  begin
    if(!sys_rst_n)
      scan_en_pulse <= 1'b0;
    else if(disp_data != disp_data_pre)
      scan_en_pulse <= 1'b1;
    else
      scan_en_pulse <= 1'b0;
  end

  dync_char_tft
  #(
    .TFT_WIDTH  (TFT_WIDTH ),
    .TFT_HEIGHT (TFT_HEIGHT)
  )dync_char_tft
  (
    .clk             (sys_clk50M      ),
    .reset_n         (sys_rst_n       ),
    .tft_ctrl_clk    (tft_ctrl_clk    ),
    .char_disp_hbegin(DISP_HBEGIN     ),
    .char_disp_vbegin(DISP_VBEGIN     ),
    .disp_back_color (DISP_BACK_COLOR ),
    .disp_char_color (DISP_CHAR_COLOR ),  
    .scan_en_pulse   (scan_en_pulse   ),
    .disp_data       (disp_data       ),
    .TFT_rgb         (TFT_rgb         ),
    .TFT_hs          (TFT_hs          ),
    .TFT_vs          (TFT_vs          ),
    .TFT_clk         (TFT_clk         ),
    .TFT_de          (TFT_de          ),
    .TFT_pwm         (TFT_pwm         )
  );

endmodule
