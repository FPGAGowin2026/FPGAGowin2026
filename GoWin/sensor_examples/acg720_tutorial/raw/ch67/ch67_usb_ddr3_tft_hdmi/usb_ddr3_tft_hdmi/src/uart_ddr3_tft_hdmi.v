/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2023/01/31 00:00:00
// Module Name   : uart_ddr3_tft_hdmi
// Description   : 串口传图DDR3缓存TFT屏和双HDMI显示
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module uart_ddr3_tft_hdmi #(

  //显示器分辨率设置(使用小梅哥显示屏模块，分辨率800*480)
  parameter DISP_WIDTH    = 800,
  parameter DISP_HEIGHT   = 480
)
(
    //System clock reset
    input           clk50m        , //系统时钟输入，50MHz
    input           reset_n       , //复位信号输入
    //LED
    output [2:0]    led           ,

      //USB interface              
  input  [15:0]   fx2_fdata     , //  FX2型USB2.0芯片的SlaveFIFO的数据线
  output [1:0]    fx2_faddr     , //  FX2型USB2.0芯片的SlaveFIFO的FIFO地址线
  output          fx2_slrd      , //  FX2型USB2.0芯片的SlaveFIFO的读控制信号，低电平有效
  output          fx2_slwr      , //  FX2型USB2.0芯片的SlaveFIFO的写控制信号，低电平有效
  output          fx2_sloe      , //  FX2型USB2.0芯片的SlaveFIFO的输出使能信号，低电平有效
  input           fx2_flagc     , //  FX2型USB2.0芯片的端点6满标志
  input           fx2_flagb     , //  FX2型USB2.0芯片的端点2空标志
  input           fx2_ifclk     , //  FX2型USB2.0芯片的接口时钟信号
  output          fx2_pkt_end   , //  数据包结束标志信号
  output          fx2_slcs      ,


    //TFT Interface
    output [15:0]   TFT_rgb       , //TFT数据输出
    output          TFT_hs        , //TFT行同步信号
    output          TFT_vs        , //TFT场同步信号
    output          TFT_clk       , //TFT像素时钟
    output          TFT_de        , //TFT数据使能
    output          TFT_pwm       , //TFT背光控制

    //hdmi interface
    output          hdmi_clk_p   ,
    output          hdmi_clk_n   ,
    output [2:0]    hdmi_dat_p   ,
    output [2:0]    hdmi_dat_n   ,



    //ddr
    output [13:0] O_ddr_addr        ,
    output [2:0] O_ddr_ba           ,
    output O_ddr_cs_n               ,
    output O_ddr_ras_n              ,
    output O_ddr_cas_n              ,
    output O_ddr_we_n               ,
    output O_ddr_clk                ,
    output O_ddr_clk_n              ,
    output O_ddr_cke                ,
    output O_ddr_odt                ,
    output O_ddr_reset_n            ,
    output [1:0] O_ddr_dqm          ,
    inout [15:0] IO_ddr_dq          ,
    inout [1:0] IO_ddr_dqs          ,
    inout [1:0] IO_ddr_dqs_n        
);


//*********************************
//Internal connect
//*********************************
  //clock
  wire          pll_locked;
  wire          loc_clk50m;
  wire          loc_clk200m;
  wire          loc_clk33m;
  wire          loc_clk9m;
  wire          loc_clk165m;
  //reset
  wire          g_rst_p;


  //
  wire [15:0]   image_data;
  wire          image_data_valid;
  wire [15:0]   image_data_hcnt;
  wire [15:0]   image_data_vcnt;
  wire          image_data_hs;
  wire          image_data_vs;
  wire          frame_rx_done_flip;
  //wr_fifo Interface
  wire          wrfifo_clr;
  wire [15:0]   wrfifo_din;
  wire          wrfifo_wren;
  //rd_fifo Interface
  wire          rdfifo_clr;
  wire          rdfifo_rden;
  wire [15 :0]  rdfifo_dout;
  //mig Interface 
  wire          ddr3_rst_n;
  wire          ddr3_init_done;
  wire          pll_stop;
  //tft
  wire          clk_disp;
  wire          disp_sof;
  //hdmi
  wire          pixelclk;
  wire          pixelclk5x;
  wire  [7:0]   disp_red;
  wire  [7:0]   disp_green;
  wire  [7:0]   disp_blue;
  wire          disp_hs;
  wire          disp_vs;
  wire          disp_de;



  assign ddr3_rst_n = pll_locked;  
  assign g_rst_p    = ~ddr3_init_done;
  assign led        = {frame_rx_done_flip,ddr3_init_done,pll_locked};

    ddr3_pll ddr3_pll(
        .lock(pll_locked), //output lock
        .clkout0(loc_clk50m), //output clkout0
        .clkout1(loc_clk200m), //output clkout1
        .clkin(clk50m), //input clkin
        .reset(~reset_n), //input reset
        .enclk0(1'b1), //input enclk0
        .enclk1(1'b1), //input enclk1
        .enclk2(pll_stop) //input enclk2
    );



    pll pll(
        .clkout0(loc_clk165m), //output clkout0
        .clkout1(loc_clk9m), //output clkout1
        .clkin(loc_clk50m) //input clkin
    );

  //-------------------------------------------
  //USB接收数据
  //-------------------------------------------  

    wire [15:0] usb_data;
	wire usb_data_valid;
	reg source_ready;

    usb_stream_out usb_ctl(
    .fx2_fdata(fx2_fdata),                               
    .fx2_faddr(fx2_faddr),                               
    .fx2_slrd(fx2_slrd),                                
    .fx2_slwr(fx2_slwr),                                
    .fx2_sloe(fx2_sloe), 
    .fx2_flagc(fx2_flagc),                               
    .fx2_flagb(fx2_flagb),                               
    .fx2_ifclk(fx2_ifclk),
    .fx2_pkt_end(fx2_pkt_end),                               
    .fx2_slcs(fx2_slcs),  		
    .reset_n(!wrfifo_clr),
    .data_out(usb_data),
    .data_valid(usb_data_valid),
    .source_ready(source_ready)
	);

	wire [11:0]wrfifo_wr_cnt;
//	根据FIFO中剩余存储深度决定是否允许从FX2中读取新一包数据
	always @ (posedge fx2_ifclk or posedge wrfifo_clr)
	if(wrfifo_clr)
		source_ready <= 0;
	else if(wrfifo_wr_cnt >= 2048)
		source_ready <= 0;
	else 
		source_ready <= 1;

    CLKDIV u_clkdiv
    (.RESETN(pll_locked)
    ,.HCLKIN(loc_clk165m) //clk  x5
    ,.CLKOUT(loc_clk33m)    //clk  x1
    ,.CALIB (1'b1)
    );
    defparam u_clkdiv.DIV_MODE="5";


//  image_rx_ctrl#(
//    .DISP_WIDTH        (DISP_WIDTH         ),
//    .DISP_HEIGHT       (DISP_HEIGHT        )
//  )
//  image_rx_ctrl(
//   .clk                (loc_clk50m         ),
//   .reset_p            (g_rst_p            ),
//   .image_data_valid   (image_data_valid   ),
//   .image_data_hcnt    (image_data_hcnt    ),
//   .image_data_vcnt    (image_data_vcnt    ),
//   .image_data_hs      (image_data_hs      ),
//   .image_data_vs      (image_data_vs      ),
//   .frame_rx_done_flip (frame_rx_done_flip )
//  );

  //TFT屏幕显示
  assign clk_disp = loc_clk33m;
  assign TFT_pwm  = 1'b1;

  disp_driver #(
    .AHEAD_CLK_CNT ( 0 )
  )disp_driver
  (
    .ClkDisp     (clk_disp       ),
    .Rst_n       (~g_rst_p       ),

    .Data        (rdfifo_dout    ),
    .DataReq     (rdfifo_rden    ),

    .H_Addr      (               ),
    .V_Addr      (               ),
    .Disp_Sof    (disp_sof       ),

    .Disp_HS     (TFT_hs         ),
    .Disp_VS     (TFT_vs         ),
    .Disp_Red    (TFT_rgb[15:11] ),
    .Disp_Green  (TFT_rgb[10:5]  ),
    .Disp_Blue   (TFT_rgb[4:0]    ),

    .Disp_DE     (TFT_de         ),
    .Disp_PCLK   (TFT_clk        )
  );

  assign wrfifo_clr  = g_rst_p;
  assign wrfifo_din  = {usb_data[8+:5],usb_data[0+:3],usb_data[15-:3],usb_data[7-:5]};
  assign wrfifo_wren = usb_data_valid;
  assign rdfifo_clr  = disp_sof;

    wire [27:0] app_addr_max = DISP_WIDTH * DISP_HEIGHT;
    wire [7:0] burst_len = DISP_WIDTH[10:2];

    ddr3_ctrl_2port ddr3_ctrl_2port(
        .clk(loc_clk50m)                 ,      //50M时钟信号
        .pll_stop(pll_stop)                ,
        .pll_lock(pll_locked)            ,
        .clk_200m(loc_clk200m)            ,      //DDR3参考时钟信号
        .sys_rst_n(reset_n)           ,      //外部复位信号
        .init_calib_complete(ddr3_init_done) ,    //DDR初始化完成信号

        //用户接口
        .rd_load()             ,   //输出源更新信号
        .wr_load()             ,   //输入源更新信号
        .app_addr_rd_min(28'd0)     ,   //读DDR3的起始地址
        .app_addr_rd_max(app_addr_max)     ,   //读DDR3的结束地址
        .rd_bust_len(burst_len)         ,   //从DDR3中读数据时的突发长度
        .app_addr_wr_min(28'd0)     ,   //写DD3的起始地址
        .app_addr_wr_max(app_addr_max)     ,   //写DDR的结束地址
        .wr_bust_len(burst_len)         ,   //向D                    DR3中写数据时的突发长度

        .wr_clk(fx2_ifclk)             ,//wr_fifo的写时钟信号
        .wfifo_wren(wrfifo_wren)          , //wr_fifo的写使能信号
        .wfifo_din(wrfifo_din)           , //写入到wr_fifo中的数据
        .rd_clk(clk_disp)              , //rd_fifo的读时钟信号
        .rfifo_rden(rdfifo_rden)          , //rd_fifo的读使能信号
        .rfifo_dout(rdfifo_dout)          , //rd_fifo读出的数据信号 

        //DDR3   
        .ddr3_dq(IO_ddr_dq)             ,   //DDR3 数据
        .ddr3_dqs_n(IO_ddr_dqs_n)          ,   //DDR3 dqs负
        .ddr3_dqs_p(IO_ddr_dqs)          ,   //DDR3 dqs正  
        .ddr3_addr(O_ddr_addr)           ,   //DDR3 地址   
        .ddr3_ba(O_ddr_ba)             ,   //DDR3 banck 选择
        .ddr3_ras_n(O_ddr_ras_n)          ,   //DDR3 行选择
        .ddr3_cas_n(O_ddr_cas_n)          ,   //DDR3 列选择
        .ddr3_we_n(O_ddr_we_n)           ,   //DDR3 读写选择
        .ddr3_reset_n(O_ddr_reset_n)        ,   //DDR3 复位
        .ddr3_ck_p(O_ddr_clk)          ,   //DDR3 时钟正
        .ddr3_ck_n(O_ddr_clk_n)           ,   //DDR3 时钟负
        .ddr3_cke(O_ddr_cke)            ,   //DDR3 时钟使能
        .ddr3_cs_n(O_ddr_cs_n)           ,   //DDR3 片选
        .ddr3_dm(O_ddr_dqm)             ,   //DDR3_dm
        .ddr3_odt(O_ddr_odt)                //DDR3_odt   
    );

  //HDMI显示
  assign pixelclk   = loc_clk33m;
  assign pixelclk5x = loc_clk165m;
  assign disp_red   = {TFT_rgb[15:11],3'b0};
  assign disp_green = {TFT_rgb[10:5],2'b0};
  assign disp_blue  = {TFT_rgb[4:0],3'b0};
  assign disp_hs    = TFT_hs;
  assign disp_vs    = TFT_vs;
  assign disp_de    = TFT_de;

	dvi_encoder u_dvi_encoder(
		.pixelclk(pixelclk),// system clock
		.pixelclk5x(pixelclk5x),// system clock x5
		.rst_n(~g_rst_p),// reset
		.blue_din(disp_blue),// Blue data in
		.green_din(disp_green),// Green data in
		.red_din(disp_red),// Red data in
		.hsync(disp_hs),// hsync data
		.vsync(disp_vs),// vsync data
		.de(disp_de),// data enable
		.tmds_clk_p(hdmi_clk_p),
		.tmds_clk_n(hdmi_clk_n),
		.tmds_data_p(hdmi_dat_p),//rgb
		.tmds_data_n(hdmi_dat_n) //rgb
	);


endmodule