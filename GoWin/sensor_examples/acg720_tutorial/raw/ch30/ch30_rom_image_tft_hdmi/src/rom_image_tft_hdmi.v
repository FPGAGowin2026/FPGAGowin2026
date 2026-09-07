/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: rom_image_tft
// Module Name: rom_image_tft
// Project Name: rom_image_tft
// Description: rom_image_tft_hdmi的顶层调度文件，协调锁相环、驱动、存储数据和屏的工作
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module rom_image_tft_hdmi(
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
    hdmi1_dat_n   
//    hdmi1_oe      
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
//    output          hdmi1_oe      ;

    //设置待显示图片尺寸，和存储图片ROM的地址位宽，显示背景颜色
    parameter DISP_IMAGE_W    = 200;
    parameter DISP_IMAGE_H    = 200;
    parameter ROM_ADDR_WIDTH  = 16; //需根据图片存储所需ROM深度决定ROM的地址位宽
    parameter DISP_BACK_COLOR = 16'hFFFF; //白色
    //设置屏幕尺寸
    parameter TFT_WIDTH  = 800;
    parameter TFT_HEIGHT = 480;
    //图片显示在屏幕中间位置
    parameter DISP_HBEGIN = (TFT_WIDTH  - DISP_IMAGE_W)/2;
    parameter DISP_VBEGIN = (TFT_HEIGHT - DISP_IMAGE_H)/2;

    wire                      pll_locked;
    wire                      clk_ctrl;
    wire                      tft_reset_n;
    wire [ROM_ADDR_WIDTH-1:0] rom_addra;
    wire [15:0]               disp_data;
    wire [15:0]               rom_data;
    wire                      disp_data_req;
    wire [11:0]               visible_hcount;
    wire [11:0]               visible_vcount;
    wire tft_reset_p;
    wire [4:0]Disp_Red;
    wire [5:0]Disp_Green;
    wire [4:0]Disp_Blue;
    wire [15:0]TFT_rgb;
    wire Frame_Begin;
    wire loc_clk33M;
    wire loc_clk165M;
    assign tft_reset_n = pll_locked;
    
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

/*
  //SIM
  assign rom_data = rom_addra;
*/
    image_extract
    #(
        .H_Visible_area (TFT_WIDTH      ), //屏幕显示区域宽度
        .V_Visible_area (TFT_HEIGHT     ), //屏幕显示区域高度
        .IMG_WIDTH      (DISP_IMAGE_W   ), //图片宽度
        .IMG_HEIGHT     (DISP_IMAGE_W   ), //图片高度
        .IMG_DATA_WIDTH (16             ), //图片像素点位宽
        .ROM_ADDR_WIDTH (ROM_ADDR_WIDTH )  //存储图片ROM的地址位宽
    )image_extract
    (
        .clk_ctrl       (clk_ctrl       ),
        .reset_n        (tft_reset_n    ),
        .img_disp_hbegin(DISP_HBEGIN    ),
        .img_disp_vbegin(DISP_VBEGIN    ),
        .disp_back_color(DISP_BACK_COLOR),
        .rom_addra      (rom_addra      ),
        .rom_data       (rom_data       ),
        .Frame_Begin    (Frame_Begin    ),
        .disp_data_req  (disp_data_req  ),
        .visible_hcount (visible_hcount ),
        .visible_vcount (visible_vcount ),
        .disp_data      (disp_data      )
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
//  wire hdmi1_oe;
  
  assign pixelclk   = loc_clk33M;
  assign pixelclk5x = loc_clk165M;
  assign disp_red   = {TFT_rgb[15:11],3'b0};
  assign disp_green = {TFT_rgb[10:5],2'b0};
  assign disp_blue  = {TFT_rgb[4:0],3'b0};
  assign disp_hs    = TFT_hs;
  assign disp_vs    = TFT_vs;
  assign disp_de    = TFT_de;
//  assign hdmi1_oe   = 1'b1;


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