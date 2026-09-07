/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : uart_ddr3_tft_tb
// Description   : uart_ddr3_tft_hdmi模块仿真
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ns
`define CLK_PERIOD 20

module uart_ddr3_tft_hdmi_tb();
  reg           clk50m;
  reg           reset_n;

  wire [15:0]   TFT_rgb; //TFT数据输出
  wire          TFT_hs;  //TFT行同步信号
  wire          TFT_vs;  //TFT场同步信号
  wire          TFT_clk; //TFT像素时钟
  wire          TFT_de;  //TFT数据使能
  wire          TFT_pwm; //TFT背光控制

  wire [13:0]   ddr3_addr;
  wire [2:0]    ddr3_ba;
  wire          ddr3_cas_n;
  wire [0:0]    ddr3_ck_n;
  wire [0:0]    ddr3_ck_p;
  wire [0:0]    ddr3_cke;
  wire          ddr3_ras_n;
  wire          ddr3_reset_n;
  wire          ddr3_we_n;
  wire [15:0]   ddr3_dq;
  wire [1:0]    ddr3_dqs_n;
  wire [1:0]    ddr3_dqs_p;
  wire [0:0]    ddr3_cs_n;
  wire [1:0]    ddr3_dm;
  wire [0:0]    ddr3_odt;

  initial clk50m = 1;
  always #(`CLK_PERIOD/2) clk50m = ~clk50m;

  GSR GSR(.GSRI(1'b1));
  
  initial begin
    reset_n = 1'b0;
    #201;
    reset_n = 1'b1;

    @(posedge uart_ddr3_tft_hdmi.ddr3_init_done)
    @(posedge TFT_vs);
    @(posedge TFT_vs);
    @(posedge TFT_vs);
    $stop;
  end

  uart_ddr3_tft_hdmi #(
    .USE_TEST_DATA ( "TRUE" )
  )uart_ddr3_tft_hdmi
  (
    //System clock reset
    .clk50m         (clk50m         ), //系统时钟输入，50M
    .reset_n        (reset_n        ), //复位信号输入，低有效
    //led
    .led            (               ), //指示灯
    //Uart interface                
    .uart_rx        (               ), //串口输入信号
    //TFT Interface                 
    .TFT_rgb        (TFT_rgb        ), //TFT数据输出
    .TFT_hs         (TFT_hs         ), //TFT行同步信号
    .TFT_vs         (TFT_vs         ), //TFT场同步信号
    .TFT_clk        (TFT_clk        ), //TFT像素时钟
    .TFT_de         (TFT_de         ), //TFT数据使能
    .TFT_pwm        (TFT_pwm        ), //TFT背光控制
    // Memory interface ports       
    .O_ddr_addr     (ddr3_addr      ), // output [13:0]   ddr3_addr
    .O_ddr_ba       (ddr3_ba        ), // output [2:0]    ddr3_ba
    .O_ddr_cs_n     (ddr3_cs_n      ), // output          ddr3_cas_n
    .O_ddr_ras_n    (ddr3_ras_n		), // output [0:0]    ddr3_ck_n
    .O_ddr_cas_n    (ddr3_cas_n     ), // output [0:0]    ddr3_ck_p
    .O_ddr_we_n     (ddr3_we_n      ), // output [0:0]    ddr3_cke
    .O_ddr_clk      (ddr3_ck_p		), // output          ddr3_ras_n
    .O_ddr_clk_n    (ddr3_ck_n   	), // output          ddr3_reset_n
    .O_ddr_cke      (ddr3_cke       ), // output          ddr3_we_n
    .O_ddr_odt      (ddr3_odt       ), // inout [15:0]    ddr3_dq
    .O_ddr_reset_n  (ddr3_reset_n   ), // inout [1:0]     ddr3_dqs_n
    .O_ddr_dqm      (ddr3_dm     	), // inout [1:0]     ddr3_dqs_p
    .IO_ddr_dq      (ddr3_dq        ), // output [0:0]    ddr3_cs_n
    .IO_ddr_dqs     (ddr3_dqs_p     ), // output [1:0]    ddr3_dm
    .IO_ddr_dqs_n   (ddr3_dqs_n     )  // output [0:0]    ddr3_odt
  );

  ddr3_model ddr3_model
  (
    .rst_n  (ddr3_reset_n ),
    .ck     (ddr3_ck_p    ),
    .ck_n   (ddr3_ck_n    ),
    .cke    (ddr3_cke     ),
    .cs_n   (ddr3_cs_n    ),
    .ras_n  (ddr3_ras_n   ),
    .cas_n  (ddr3_cas_n   ),
    .we_n   (ddr3_we_n    ),
    .dm_tdqs(ddr3_dm      ),
    .ba     (ddr3_ba      ),
    .addr   (ddr3_addr    ),
    .dq     (ddr3_dq      ),
    .dqs    (ddr3_dqs_p   ),
    .dqs_n  (ddr3_dqs_n   ),
    .tdqs_n (             ),
    .odt    (ddr3_odt     )
  );

endmodule
