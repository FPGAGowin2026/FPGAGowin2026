/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2023/01/31 00:00:00
// Module Name   : uart_ddr3_tft_hdmi
// Description   : 串口传图DDR3缓存TFT屏和HDMI显示
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module uart_ddr3_tft_hdmi_sobel_filter #(
  //是否使用测试数据
  parameter USE_TEST_DATA = "FALSE", //"TRUE"：使用测试数据 or "FALSE"：使用串口传的数据
  //显示器分辨率设置(使用小梅哥显示屏模块，分辨率800*480)
  parameter DISP_WIDTH    = 800,
  parameter DISP_HEIGHT   = 480
)
(
    //System clock reset
    input           clk50m        , //系统时钟输入，50MHz
    input           reset_n       , //复位信号输入
    input           key_in0       ,
    //LED
    output [2:0]    led           ,
    //Uart interface              
    input           uart_rx       , //串口输入信号
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

    output  uart_rx_flag,
//    output  [7:0] uart_data_img,
//    output uart_byte_vaild_r0,
//    output image_data_valid_r0,

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

wire [7:0] uart_data_img;
wire uart_byte_vaild_r0;
wire image_data_valid_r0;
//*********************************
//Internal connect
//*********************************
  //clock
  wire          pll_locked;
  wire          loc_clk50m;
  wire          loc_clk400m;
  wire          loc_clk33m;
  wire          loc_clk165m;
  //reset
  wire          g_rst_p;
  //uart Interface
  wire [7:0]    uart_byte;
  wire          uart_byte_vaild;
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

  assign uart_rx_flag = uart_rx;
  assign uart_data_img = uart_byte;
  assign uart_byte_vaild_r0 = uart_byte_vaild;
  assign image_data_valid_r0 = image_data_valid;

  assign ddr3_rst_n = pll_locked;  
  assign g_rst_p    = ~ddr3_init_done;
  assign led        = {frame_rx_done_flip,ddr3_init_done,pll_locked};

    ddr3_pll ddr3_pll(
        .lock(pll_locked), //output lock
        .clkout0(loc_clk50m), //output clkout0
        .clkout1(loc_clk400m), //output clkout1
        .clkin(clk50m), //input clkin
        .reset(~reset_n), //input reset
        .enclk0(1'b1), //input enclk0
        .enclk1(1'b1), //input enclk1
        .enclk2(pll_stop) //input enclk2
    );

    pll pll(
        .clkout0(loc_clk165m), //output clkout0
        .clkin(loc_clk50m) //input clkin
    );

    CLKDIV u_clkdiv
    (.RESETN(pll_locked)
    ,.HCLKIN(loc_clk165m) //clk  x5
    ,.CLKOUT(loc_clk33m)    //clk  x1
    ,.CALIB (1'b1)
    );
    defparam u_clkdiv.DIV_MODE="5";

generate
  if (USE_TEST_DATA == "TRUE") begin: use_test_data_inst
  //---------------------------------------------
  //产生测试数据,仿真或测试用
  //---------------------------------------------
    test_dat_gen #(
      .DISP_WIDTH    (DISP_WIDTH*2    ),
      .DISP_HEIGHT   (DISP_HEIGHT     ),
      .DATA_WIDTH    (8               )
    )
    test_dat_gen_inst(
      .clk           (loc_clk50m      ),
      .reset         (g_rst_p         ),
      .gen_en        (1'b1            ),
      .test_dat      (uart_byte       ),
      .test_dat_vaild(uart_byte_vaild ),
      .test_dat_hs   (                ),
      .test_dat_vs   (                ),
      .test_dat_col  (                ),
      .test_dat_row  (                )
    );
  //-------------------------------------------
  end
  else begin: use_uart_data_inst
  //-------------------------------------------
  //串口接收数据
  //-------------------------------------------  

//	uart_byte_rx uart_byte_rx(
//		.Clk(loc_clk50m),
//		.Reset_n(~g_rst_p),
//		.uart_rx(uart_rx),
//		.rx_data(uart_byte),
//		.rx_done(uart_byte_vaild)
//	);

//  end
//  endgenerate

  uart_byte_rx#(
    .CLK_FRQ(50_000_000)
  )
  uart_byte_rx(
    .Clk      (loc_clk50m     ),
    .Reset_n    (~g_rst_p          ),
    .Baud_Set (3'd5            ), //1562500bps
    .uart_rx  (uart_rx         ),
    
    .Data(uart_byte       ),
    .Rx_Done  (uart_byte_vaild )
  );

  end
  endgenerate

    bit8_trans_bit16 bit8_trans_bit16
    (
        .clk             (loc_clk50m      ),
        .reset_p         (g_rst_p         ),

        .bit8_in         (uart_byte       ),
        .bit8_in_valid   (uart_byte_vaild ),

        .bit16_out       (image_data      ),
        .bit16_out_valid (image_data_valid)
    );

  image_rx_ctrl#(
    .DISP_WIDTH        (DISP_WIDTH         ),
    .DISP_HEIGHT       (DISP_HEIGHT        )
  )
  image_rx_ctrl(
   .clk                (loc_clk50m         ),
   .reset_p            (g_rst_p            ),
   .image_data_valid   (image_data_valid   ),
   .image_data_hcnt    (image_data_hcnt    ),
   .image_data_vcnt    (image_data_vcnt    ),
   .image_data_hs      (image_data_hs      ),
   .image_data_vs      (image_data_vs      ),
   .frame_rx_done_flip (frame_rx_done_flip )
  );

    //---------------------------------------------
//image processing
//---------------------------------------------
  wire [7:0] image_gray_data;
  wire       image_gray_data_valid;
  wire [15:0]image_stitche_data;
  wire       image_stitche_data_valid;
  wire       image_gray_data_hs;
  wire       image_gray_data_vs; 
  reg  [7:0] sobel_threshold;
  wire       sobel_data;
  wire       sobel_data_valid; 
  reg        key_in0_flip;

  rgb2gray
  #(
    .PROC_METHOD ( "FORMULA" )//"AVERAGE"     :求平均法
                              //or "FORMULA"  :直接公式法
                              //or "LUT"      :查找表法
  )rgb2gray
  (
    .clk        (loc_clk50m                ),//input
    .reset_p    (g_rst_p                   ),//input
    .rgb_valid  (image_data_valid          ),//input
    .rgb_hs     (image_data_hs             ),//input
    .rgb_vs     (image_data_vs             ),//input
    .red_8b_i   ({image_data[15:11],3'b000}),//input     [7:0]
    .green_8b_i ({image_data[10:5],2'b00}  ),//input     [7:0]
    .blue_8b_i  ({image_data[4:0],3'b000}  ),//input     [7:0]

    .gray_8b_o  (image_gray_data           ),//output    [7:0]
    .gray_valid (image_gray_data_valid     ),//output reg
    .gray_hs    (image_gray_data_hs        ),//output reg
    .gray_vs    (image_gray_data_vs        ) //output reg
  );

    //threshold adjust
  wire key0_flag;
  wire key0_state;

  key_filter key_filter(
    .clk       (loc_clk50m),//input       模块全局时钟输入，50M
    .reset_n   (~g_rst_p   ),//input       复位信号输入
    .key_in    (key_in0   ),//input       按键输入

    .key_flag  (key0_flag ),//output reg  按键标志信号
    .key_state (key0_state) //output reg  按键状态信号
  );
 
  always@(posedge loc_clk50m or posedge g_rst_p)
    if(g_rst_p)
      sobel_threshold <= 'd128;
    else if(~key0_state && key0_flag)
      sobel_threshold <= sobel_threshold + 'd8;

  always@(posedge loc_clk50m or posedge g_rst_p)
    if(g_rst_p)
      key_in0_flip <= 1'b0;
    else if(~key0_state && key0_flag)
      key_in0_flip <= ~key_in0_flip;

  sobel
  #(
    .DATA_WIDTH ( 8 )
  )sobel
  (
    .clk            (loc_clk50m              ),//input     pixel clk
    .reset_p        (g_rst_p                 ),//input
    .data_in        (image_gray_data         ),//input     [DATA_WIDTH-1:0]
    .data_in_valid  (image_gray_data_valid   ),//input
    .data_in_hs     (image_gray_data_hs      ),//input
    .data_in_vs     (image_gray_data_vs      ),//input
    .threshold      (sobel_threshold         ),//input     [DATA_WIDTH-1:0]

    .data_out       (sobel_data              ),//output    [DATA_WIDTH-1:0]
    .data_out_valid (sobel_data_valid        ),//output reg
    .data_out_hs    (                        ),//output reg
    .data_out_vs    (                        ) //output reg
  );
  
    image_stitche_x
    #(
        .DATA_WIDTH        ( 16          ),  //16 or 24
        //image1_in: 400*480
        .IMAGE1_WIDTH_IN   ( DISP_WIDTH/2),
        .IMAGE1_HEIGHT_IN  ( DISP_HEIGHT ),
        //image2_in: 400*480
        .IMAGE2_WIDTH_IN   ( DISP_WIDTH/2),
        .IMAGE2_HEIGHT_IN  ( DISP_HEIGHT ),
        //image_out: 800*480
        .IMAGE_WIDTH_OUT   ( DISP_WIDTH  ),
        .IMAGE_HEIGHT_OUT  ( DISP_HEIGHT )
    )image_stitche_x(
      .clk_image1_in      (loc_clk50m           ),
      .clk_image2_in      (loc_clk50m           ),
      .clk_image_out      (loc_clk50m           ),
      .reset_p            (g_rst_p              ),

      .rst_busy_o         (                     ),
      .image_in_ready_o   (                     ),

      .image1_data_pixel_i({image_gray_data[7:3],image_gray_data[7:2],image_gray_data[7:3]}),
      .image1_data_valid_i(image_gray_data_valid),

      .image2_data_pixel_i({16{sobel_data}} ),
      .image2_data_valid_i(sobel_data_valid),

      .data_out_ready_i   (1'b0                 ),
      .data_pixel_o       (image_stitche_data   ),
      .data_valid_o       (image_stitche_data_valid)
    );

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
    .Disp_Blue    (TFT_rgb[15:11] ),
    .Disp_Green  (TFT_rgb[10:5]  ),
    .Disp_Red   (TFT_rgb[4:0]   ),

    .Disp_DE     (TFT_de         ),
    .Disp_PCLK   (TFT_clk        )
  );

    assign wrfifo_din  = image_stitche_data;
    assign wrfifo_wren = image_stitche_data_valid;

    wire [27:0] app_addr_max = DISP_WIDTH * DISP_HEIGHT;
    wire [7:0] burst_len = DISP_WIDTH[10:3];

    ddr3_ctrl_2port ddr3_ctrl_2port(
        .clk(loc_clk50m)                 ,      //50M时钟信号
        .pll_stop(pll_stop)                , 
        .pll_lock(pll_locked)            ,
        .clk_400m(loc_clk400m)            ,      //DDR3参考时钟信号
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
        .wr_bust_len(burst_len)         ,   //向DDR3中写数据时的突发长度

        .wr_clk(loc_clk50m)             ,//wr_fifo的写时钟信号
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
  assign disp_blue   = {TFT_rgb[15:11],3'b0};
  assign disp_green = {TFT_rgb[10:5],2'b0};
  assign disp_red  = {TFT_rgb[4:0],3'b0};
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