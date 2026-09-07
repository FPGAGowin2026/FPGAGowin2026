/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: rom_char_tft
// Module Name: rom_char_tft
// Project Name: rom_char_tft
// Target Devices: XC7A35T-2FGG484I
// Tool Versions: Vivado 2018.3
// Description: rom_char_tft项目的顶层调度文件，支持锁相环，液晶驱动和字符显示的协调工作
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module rom_char_tft_hdmi(
	clk50M,
	reset_n,

	TFT_rgb,
	TFT_hs,
	TFT_vs,
	TFT_clk,
	TFT_de,
	TFT_pwm,
	
	  //hdmi1 interface
    hdmi1_clk_p   ,
    hdmi1_clk_n   ,
    hdmi1_dat_p   ,
    hdmi1_dat_n   ,
    hdmi1_oe      
);

	input         clk50M;   //系统时钟输入，50M
	input         reset_n;  //复位信号输入，低有效
  
	output [15:0] TFT_rgb;  //TFT数据输出
	output        TFT_hs;   //TFT行同步信号
	output        TFT_vs;   //TFT场同步信号
	output        TFT_clk;  //TFT像素时钟
	output        TFT_de;   //TFT数据使能
	output        TFT_pwm;  //TFT背光控制
	
   //hdmi1 interface
    output          hdmi1_clk_p   ;
    output          hdmi1_clk_n   ;
    output [2:0]    hdmi1_dat_p   ;
    output [2:0]    hdmi1_dat_n   ;
    output          hdmi1_oe      ;
	
	//设置待显示字符尺寸，和存储字符ROM的地址位宽
	parameter CHAR_WIDTH        = 16; //单个字符显示宽度
	parameter CHAR_HEIGHT       = 32; //单个字符显示高度
	parameter ROW_DISP_CHAR_NUM = 14; //一行显示字符个数
	parameter COL_DISP_CHAR_NUM = 2 ; //显示字符行数
	parameter CHAR_ROM_ADDR_W   = 6 ; //存储字符ROM地址位宽，log2(CHAR_HEIGHT * COL_DISP_CHAR_NUM)
	parameter DISP_BACK_COLOR = 16'hFFFF; //背景白色
	parameter DISP_CHAR_COLOR = 16'h0000; //显示字符黑色
	//设置TFT屏幕尺寸
	parameter TFT_WIDTH  = 800;
	parameter TFT_HEIGHT = 480;
	//显示字符串的总的像素点宽度和高度
	localparam DISP_CHAR_TOTAL_W = CHAR_WIDTH  * ROW_DISP_CHAR_NUM;
	localparam DISP_CHAR_TOTAL_H = CHAR_HEIGHT * COL_DISP_CHAR_NUM;
	//字符显示在屏幕中间位置
	localparam DISP_HBEGIN = (TFT_WIDTH  - DISP_CHAR_TOTAL_W)/2;
	localparam DISP_VBEGIN = (TFT_HEIGHT - DISP_CHAR_TOTAL_H)/2;

	wire                         pll_locked;
	wire                         clk_ctrl;
	wire                         tft_reset_n;
	wire [15:0]                  disp_data;
	wire                         disp_data_req;
	wire [11:0]                  visible_hcount;
	wire [11:0]                  visible_vcount;

	wire [CHAR_ROM_ADDR_W-1:0]   rom_addra;
	wire [DISP_CHAR_TOTAL_W-1:0] rom_data;
  
	assign tft_reset_n = pll_locked;
	wire tft_reset_p;
    wire [4:0]Disp_Red;
    wire [5:0]Disp_Green;
    wire [4:0]Disp_Blue;
    wire [15:0]TFT_rgb;
    wire Frame_Begin;
    wire loc_clk33M;
    wire loc_clk165M;

    Gowin_PLL Gowin_PLL(
        .lock(pll_locked), //output lock
        .clkout0(loc_clk33M), //output clkout0
        .clkout1(loc_clk165M), //output clkout1
        .clkin(clk50M), //input clkin
        .reset(~reset_n) //input reset
    );
  
  assign clk_ctrl = loc_clk33M;
  

    Gowin_pROM Gowin_pROM(
        .dout(rom_data), //output [15:0] dout
        .clk(clk_ctrl), //input clk
        .oce(1'b1), //input oce
        .ce(1'b1), //input ce
        .reset(~reset_n), //input reset
        .ad(rom_addra) //input [15:0] ad
    ); 

  char_extract
  #(
    .H_Visible_area   (TFT_WIDTH        ), //屏幕显示区域宽度
    .V_Visible_area   (TFT_HEIGHT       ), //屏幕显示区域高度
    .CHAR_WIDTH       (CHAR_WIDTH       ), //单个字符显示宽度
    .CHAR_HEIGHT      (CHAR_HEIGHT      ), //单个字符显示高度
    .ROW_DISP_CHAR_NUM(ROW_DISP_CHAR_NUM), //一行显示字符个数
    .COL_DISP_CHAR_NUM(COL_DISP_CHAR_NUM), //显示字符行数
    .CHAR_ROM_ADDR_W  (CHAR_ROM_ADDR_W  ), //存储字符ROM地址位宽，log2(CHAR_HEIGHT * COL_DISP_CHAR_NUM)
    .DISP_DATA_W      (     16          )  //图片像素点数据位宽
  )char_extract
  (
    .clk_ctrl        (clk_ctrl       ),
    .reset_n         (tft_reset_n    ),
    .char_disp_hbegin(DISP_HBEGIN    ),
    .char_disp_vbegin(DISP_VBEGIN    ),
    .disp_back_color (DISP_BACK_COLOR),
    .disp_char_color (DISP_CHAR_COLOR),
    .rom_addra       (rom_addra      ),
    .rom_data        (rom_data       ),
    .Frame_Begin     (Frame_Begin    ),
    .disp_data_req   (disp_data_req  ),
    .visible_hcount  (visible_hcount ),
    .visible_vcount  (visible_vcount ),
    .disp_data       (disp_data      )
  );

	disp_driver disp_driver
	(
		.ClkDisp(clk_ctrl),
		.Rst_p(tft_reset_p),

		.Data(disp_data),
		.DataReq(disp_data_req),

		.H_Addr(visible_hcount),
		.V_Addr(visible_vcount),
                            
		.Disp_HS(TFT_hs),
		.Disp_VS(TFT_vs),
		.Disp_Red(Disp_Red),
		.Disp_Green(Disp_Green),
		.Disp_Blue(Disp_Blue),
		.Frame_Begin(Frame_Begin),
                            
		.Disp_DE(TFT_de),
		.Disp_PCLK(TFT_clk)
	);
	assign tft_reset_p = ~pll_locked; //锁相环提供的TFT屏复位信号进行取反，满足高电平复位	
	assign TFT_rgb={Disp_Red,Disp_Green,Disp_Blue};
//	assign TFT_clk=Disp_PCLK;信号转换，按各上层文件的变量声明情况决定变量名称和取舍
	assign TFT_pwm=1'b1;
	
  //HDMI显示
  wire pixelclk;
  wire pixelclk5x;
  wire [7:0]disp_red;
  wire [7:0]disp_green;
  wire [7:0]disp_blue;
  wire disp_hs;
  wire disp_vs;
  wire disp_de;
  wire hdmi1_oe;
  
  assign pixelclk   = loc_clk33M;
  assign pixelclk5x = loc_clk165M;
  assign disp_red   = {TFT_rgb[15:11],3'b0};
  assign disp_green = {TFT_rgb[10:5],2'b0};
  assign disp_blue  = {TFT_rgb[4:0],3'b0};
  assign disp_hs    = TFT_hs;
  assign disp_vs    = TFT_vs;
  assign disp_de    = TFT_de;
  assign hdmi1_oe   = 1'b1;

	dvi_encoder u_dvi_encoder(
		.pixelclk   (pixelclk),// system clock
		.pixelclk5x (pixelclk5x),// system clock x5
		.rst_n      (pll_locked),// reset
		.blue_din   (disp_blue),// Blue data in
		.green_din  (disp_green),// Green data in
		.red_din    (disp_red),// Red data in
		.hsync      (disp_hs),// hsync data
		.vsync      (disp_vs),// vsync data
		.de         (disp_de),// data enable
		.tmds_clk_p (hdmi1_clk_p),
		.tmds_clk_n (hdmi1_clk_n),
		.tmds_data_p(hdmi1_dat_p),//rgb
		.tmds_data_n(hdmi1_dat_n) //rgb
	);



endmodule