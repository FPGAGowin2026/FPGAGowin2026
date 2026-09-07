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

module eth_udp_rgmii_ddr3_tft800x480
(
    //System clock reset
    input           clk50m        , //系统时钟输入，50MHz
    input           reset_n       , //复位信号输入
    //LED
    output [1:0]    led           ,

    //eth interface              
    input           rgmii_rx_clk_i,
    input  [3:0]    rgmii_rxd     ,
    input           rgmii_rxdv    ,
    output 		    eth_rst_n     , 

    //TFT Interface
    output [15:0]   TFT_rgb       , //TFT数据输出
    output          TFT_hs        , //TFT行同步信号
    output          TFT_vs        , //TFT场同步信号
    output          TFT_clk       , //TFT像素时钟
    output          TFT_de        , //TFT数据使能
    output          TFT_pwm       , //TFT背光控制

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
//Parameter
//*********************************
  localparam LOCAL_MAC  = 48'h00_0a_35_01_fe_c0;
  localparam LOCAL_IP   = 32'hc0_a8_00_02;
  localparam LOCAL_PORT = 16'd5000;

  localparam DISP_WIDTH  = 800;
  localparam DISP_HEIGHT = 480;

//*********************************
//Internal connect
//*********************************
    //clock
    wire pll_locked;
    wire loc_clk50m;
    wire loc_clk200m;
    wire loc_clk33m;
 
    //reset
    wire g_rst_p;

    //eth
    wire rgmii_rx_clk;
    wire gmii_rx_clk;
    wire [7:0] gmii_rxd;
    wire gmii_rxdv;

    wire eth_clk125m;
    wire eth_fifo_full;
    wire eth_fifo_wr;
    wire [7 :0] eth_fifo_din;
    wire one_pkt_done;
    wire pkt_error;

    wire [15:0]rx_data;
    wire rx_data_valid;
    
    wire [15:0] image_data;
    wire image_data_valid;
    wire frame_dis_pulse;

    //ddr
    wire ddr3_init_done;
    wire rd_load;
    wire wr_load;
    wire wr_clk;
    wire wfifo_wren;
    wire [15:0] wfifo_din;
    wire rd_clk; 
    wire rfifo_rden;
    wire [15:0] rfifo_dout;

    //tft
    wire disp_data_req;
    wire [15:0] disp_data;
    wire disp_sof;  

    assign eth_rst_n = 1'd1;
    assign ddr3_rst_n = pll_locked;  
    assign g_rst_p    = ~ddr3_init_done;
    assign led        = {ddr3_init_done,pll_locked};

    ddr3_pll pll(
        .lock(pll_locked), //output lock
        .clkout0(loc_clk50m), //output clkout0
        .clkout1(loc_clk200m), //output clkout1
        .clkout2(loc_clk33m), //output clkout2
        .clkin(clk50m), //input clkin
        .reset(~reset_n) //input reset
    );

    pll eth_pll (
        .clkout0(rgmii_rx_clk), //output clkout0
        .clkin(rgmii_rx_clk_i) //input clkin
    );

      rgmii_to_gmii rgmii_to_gmii(
          .reset(g_rst_p),

          .rgmii_rx_clk(rgmii_rx_clk),
          .rgmii_rxd(rgmii_rxd),
          .rgmii_rxdv(rgmii_rxdv),

          .gmii_rx_clk(gmii_rx_clk),
          .gmii_rxdv(gmii_rxdv),
          .gmii_rxd(gmii_rxd),
          .gmii_rxer( )
    );

    //以太网接收
    eth_udp_rx_gmii eth_udp_rx_gmii(
        .reset_p         (g_rst_p               ),

        .local_mac       (LOCAL_MAC             ),
        .local_ip        (LOCAL_IP              ),
        .local_port      (LOCAL_PORT            ),

        .clk125m_o       (eth_clk125m           ),
        .exter_mac       (                      ),
        .exter_ip        (                      ),
        .exter_port      (                      ),
        .rx_data_length  (                      ),
        .data_overflow_i (eth_fifo_full         ),
        .payload_valid_o (eth_fifo_wr           ),
        .payload_dat_o   (eth_fifo_din          ),

        .one_pkt_done    (one_pkt_done          ),
        .pkt_error       (pkt_error             ),
        .debug_crc_check (                      ),

        .gmii_rx_clk     (gmii_rx_clk           ),
        .gmii_rxdv       (gmii_rxdv             ),
        .gmii_rxd        (gmii_rxd              )
    );

      eth_rx_ctrl
      #(
        .O_DATA_WIDTH (16  )
      )eth_rx_ctrl
      (
        .clk          (eth_clk125m  ),
        .reset        (g_rst_p      ),

        .fifo_full    (eth_fifo_full),
        .fifo_wr      (eth_fifo_wr  ),
        .fifo_din     (eth_fifo_din ),

        .one_pkt_done (one_pkt_done ),
        .pkt_error    (pkt_error    ),

        .rx_data      (rx_data      ),
        .rx_data_valid(rx_data_valid)  
      );


      frame_check
      #(
        .IMAGE_WIDTH  ( DISP_WIDTH   ),
        .IMAGE_HEIGHT ( DISP_HEIGHT  ),
        .DATA_WIDTH   (      16      )
      )frame_check
      (
        .clk            (eth_clk125m      ),
        .reset          (g_rst_p          ),
        .data_i         (rx_data          ),
        .data_i_valid   (rx_data_valid    ),
        .data_o_valid   (image_data_valid ),
        .data_o         (image_data       ),
        .frame_dis_pulse(frame_dis_pulse  )
      );


    wire [27:0] app_addr_max = DISP_WIDTH * DISP_HEIGHT;
    wire [7:0] burst_len = DISP_WIDTH[10:3];

    assign wr_load = g_rst_p | frame_dis_pulse;
    assign rd_load = disp_sof;

    assign wr_clk = eth_clk125m;
    assign wfifo_wren = image_data_valid;
    assign wfifo_din = image_data;

    assign rd_clk = loc_clk33m;
    assign rfifo_rden = disp_data_req;
    assign disp_data = rfifo_dout;

    ddr3_ctrl_2port ddr3_ctrl_2port(
        .clk(loc_clk50m)                 ,      //50M时钟信号
        .pll_lock(pll_locked)            ,
        .clk_200m(loc_clk200m)            ,      //DDR3参考时钟信号
        .sys_rst_n(reset_n)           ,      //外部复位信号
        .init_calib_complete(ddr3_init_done) ,    //DDR初始化完成信号

        //用户接口
        .rd_load(rd_load),   //输出源更新信号
        .wr_load(wr_load),   //输入源更新信号
        .app_addr_rd_min(28'd0)     ,   //读DDR3的起始地址
        .app_addr_rd_max(app_addr_max)     ,   //读DDR3的结束地址
        .rd_bust_len(burst_len)         ,   //从DDR3中读数据时的突发长度
        .app_addr_wr_min(28'd0)     ,   //写DD3的起始地址
        .app_addr_wr_max(app_addr_max)     ,   //写DDR的结束地址
        .wr_bust_len(burst_len)         ,   //向DDR3中写数据时的突发长度

        .wr_clk(wr_clk)             ,//wr_fifo的写时钟信号
        .wfifo_wren(wfifo_wren)          , //wr_fifo的写使能信号
        .wfifo_din(wfifo_din)           , //写入到wr_fifo中的数据
        .rd_clk(rd_clk)              , //rd_fifo的读时钟信号
        .rfifo_rden(rfifo_rden)          , //rd_fifo的读使能信号
        .rfifo_dout(rfifo_dout)          , //rd_fifo读出的数据信号 

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

    disp_driver disp_driver
    (
        .ClkDisp     (loc_clk33m     ),
        .Rst_n       (~g_rst_p       ),

        .Data        (disp_data      ),
        .DataReq     (disp_data_req  ),

        .H_Addr      (               ),
        .V_Addr      (               ),

        .Disp_HS     (TFT_hs         ),
        .Disp_VS     (TFT_vs         ),
        .Disp_Red    ( TFT_rgb[4:0] ),
        .Disp_Green  (TFT_rgb[10:5]  ),
        .Disp_Blue   ( TFT_rgb[15:11] ),
        .Disp_Sof    (disp_sof       ),
        .Disp_DE     (TFT_de         ),
        .Disp_PCLK   (TFT_clk        )
    );
    assign TFT_pwm = 1'b1;


endmodule