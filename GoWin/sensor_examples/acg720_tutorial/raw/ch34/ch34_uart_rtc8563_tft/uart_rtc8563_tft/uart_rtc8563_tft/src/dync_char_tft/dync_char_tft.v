/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2019/05/01 00:00:00
// Design Name: 
// Module Name: dync_char_tft
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

module dync_char_tft
#(
  parameter TFT_WIDTH  = 800,
  parameter TFT_HEIGHT = 480 
)
(
  clk,
  reset_n,
  tft_ctrl_clk,

  char_disp_hbegin,
  char_disp_vbegin,
  disp_back_color,
  disp_char_color,  
  scan_en_pulse,
  disp_data,

	TFT_rgb,
	TFT_hs,
	TFT_vs,
	TFT_clk,
	TFT_de,
	TFT_pwm
);
  input         clk;         //模块工作时钟输入
	input         reset_n;     //复位信号输入，低有效
  input         tft_ctrl_clk;//TFT屏控制时钟

  input  [15:0] char_disp_hbegin; //待显示图片左上角第一个像素点在TFT屏的行向坐标
  input  [15:0] char_disp_vbegin; //待显示图片左上角第一个像素点在TFT屏的场向坐标
  input  [15:0] disp_back_color;  //显示的背景颜色
  input  [15:0] disp_char_color;  //显示字符的颜色  
  input         scan_en_pulse;
  input  [39:0] disp_data; 
  
	output [15:0] TFT_rgb;  //TFT数据输出
	output        TFT_hs;   //TFT行同步信号
	output        TFT_vs;   //TFT场同步信号
	output        TFT_clk;  //TFT像素时钟
	output        TFT_de;   //TFT数据使能
	output        TFT_pwm;  //TFT背光控制

	wire [15:0]  tft_disp_data;
  wire         tft_disp_data_req;
	wire [11:0]  visible_hcount;
	wire [11:0]  visible_vcount;
  wire         frame_begin;
  
  wire [8:0]   extr_data_addr;
  wire [15:0]  extr_data;
  wire [15:0]  update_data;
  wire [9:0]   update_data_wr;
  wire [4:0]   update_data_addr;
  wire [4:0]   ram_addrb;
  wire [159:0] ram_doutb; 

    num_lib_rom num_lib_rom(
        .dout(extr_data), //output [15:0] dout
        .clk(clk), //input clk
        .oce(1'd1), //input oce
        .ce(1'd1), //input ce
        .reset(~reset_n), //input reset
        .ad(extr_data_addr) //input [8:0] ad
    );

assign update_data = extr_data;

  data_scan_ctrl data_scan_ctrl
  (
    .clk               (clk              ),
    .reset_n           (reset_n          ),
    .scan_en_pulse     (scan_en_pulse    ),
    .disp_data         (disp_data        ),
    .extr_data_addr    (extr_data_addr   ),
    .update_data_wr    (update_data_wr   ),
    .update_data_addr  (update_data_addr )
  );

genvar i;
generate
  for (i=0; i < 10; i=i+1)
  begin: num_dpram
//    num_dpram num_dpram_inst (
//      .clka  (clk               ), // input wire clka
//      .wea   (update_data_wr[i] ), // input wire [0 : 0] wea
//      .addra (update_data_addr  ), // input wire [4 : 0] addra
//      .dina  (update_data       ), // input wire [15 : 0] dina
//      .clkb  (tft_ctrl_clk      ), // input wire clkb
//      .addrb (ram_addrb         ), // input wire [4 : 0] addrb
//      .doutb (ram_doutb[16*(i+1)-1:16*i] )  // output wire [15 : 0] doutb
//    );

    Gowin_SDPB Gowin_SDPB(
        .dout(ram_doutb[16*(i+1)-1:16*i]), //output [15:0] dout
        .clka(clk), //input clka
        .cea(update_data_wr[i] ), //input cea
        .clkb(tft_ctrl_clk), //input clkb
        .ceb(1'd1), //input ceb
        .oce(1'd1), //input oce
        .reset(~reset_n), //input reset
        .ada(update_data_addr), //input [4:0] ada
        .din(update_data), //input [15:0] din
        .adb(ram_addrb) //input [4:0] adb
    );

  end
endgenerate

  char_extract
  #(
    .H_Visible_area   (TFT_WIDTH ), //屏幕显示区域宽度
    .V_Visible_area   (TFT_HEIGHT), //屏幕显示区域高度
    .CHAR_WIDTH       (   16     ), //单个字符显示宽度
    .CHAR_HEIGHT      (   32     ), //单个字符显示高度
    .ROW_DISP_CHAR_NUM(   10     ), //一行显示字符个数
    .COL_DISP_CHAR_NUM(   1      ), //显示字符行数
    .CHAR_ROM_ADDR_W  (   5      ), //存储字符ROM地址位宽，log2(CHAR_HEIGHT * COL_DISP_CHAR_NUM)
    .DISP_DATA_W      (   16     )  //图片像素点数据位宽
  )char_extract
  (
    .clk_ctrl        (tft_ctrl_clk     ),
    .reset_n         (reset_n          ),
    .char_disp_hbegin(char_disp_hbegin ),
    .char_disp_vbegin(char_disp_vbegin ),
    .disp_back_color (disp_back_color  ),
    .disp_char_color (disp_char_color  ),
    .rom_addra       (ram_addrb        ),
    .rom_data        (ram_doutb        ),
    .frame_begin     (frame_begin      ),
    .disp_data_req   (tft_disp_data_req),
    .visible_hcount  (visible_hcount   ),
    .visible_vcount  (visible_vcount   ),
    .disp_data       (tft_disp_data    )
  );

  tft_ctrl tft_ctrl
  (
    .clk_ctrl      (tft_ctrl_clk      ),
    .reset_n       (reset_n           ),

    .disp_data     (tft_disp_data     ),
    .disp_data_req (tft_disp_data_req ),
    .visible_hcount(visible_hcount    ),
    .visible_vcount(visible_vcount    ),
    .frame_begin   (frame_begin       ),

    .TFT_rgb       (TFT_rgb           ),
    .TFT_hs        (TFT_hs            ),
    .TFT_vs        (TFT_vs            ),
    .TFT_clk       (TFT_clk           ),
    .TFT_de        (TFT_de            ),
    .TFT_pwm       (TFT_pwm           )
  );

endmodule
