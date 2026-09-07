module acm1030_ddr3_rgmii(
    //System clock reset
    input       clk50m        , //系统时钟输入，50MHz
    input       reset_n       , //复位信号输入

    //led
    output [2:0]led           ,

    //ACM1030
    input [9:0]ad_in1,         
	input [9:0]ad_in2,         
	output  ad_clk1,         
	output  ad_clk2,            


    //eth_rx
    input         rgmii_rx_clk_i,
    input  [3:0]  rgmii_rxd,
    input         rgmii_rxdv,
    output 		  eth_rst_n, 
    output        eth_mdc,
    output        eth_mdio, 

    //eth_tx
    output        rgmii_tx_clk,
    output  [3:0] rgmii_txd,
    output        rgmii_txen,

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
    //Set IMAGE Size  
    parameter LOCAL_MAC  = 48'h00_0a_35_01_fe_c0;
    parameter LOCAL_IP   = 32'hc0_a8_00_02;
    parameter LOCAL_PORT = 16'd5000;

    wire          g_rst_p;
    wire          clk_200M;
    wire          pll_lock;

    //ddr
    wire init_calib_complete;
    
    wire loc_clk50m;

    //eth_rx
    wire rgmii_rx_clk;
    wire gmii_rx_clk;
    wire [7:0] gmii_rxd;
    wire gmii_rxdv;
    
    wire clk125m_o;
    wire [7:0]payload_dat_o;
    wire payload_valid_o;
    wire one_pkt_done;

    //fifo_rx
    wire rx_empty;
    wire fifo_rd_req;
    wire [7:0]rxdout;

    //rxcmd
    wire cmdvalid_0;
    wire [7:0]address_0;
    wire [31:0]cmd_data_0; 

    wire [1:0]ChannelSel;
    wire [31:0]DataNum;
    wire [31:0]ADC_Speed_Set;   
    wire RestartReq;


    //state_ctrl
    wire rdfifo_empty;
    wire [15:0] rdfifo_dout;
    wire wrfifo_full;
    wire wr_load;
    wire rd_load;
    wire rdfifo_rden;
    wire eth_fifo_wrreq;
    wire [15:0] eth_fifo_wrdata;

    //DDR
    wire wrfifo_clk;
    wire wrfifo_wren;
    wire [15:0] wrfifo_din;
    wire rdfifo_clk;

    wire [27:0] app_addr_max = 256*1024*1024-1;
    wire [7:0] burst_len = 8'd128;

    //FIFO_TX
    wire payload_req_o;
    wire [10:0] eth_fifo_usedw;
    wire [11:0] rd_data_count;
    wire [7:0] dout;
    wire eth_fifo_tx_empty;

    //eth_tx
    wire tx_done;
    wire tx_en_pulse;
    wire [15:0] lenth_val;
 
    wire gmii_tx_clk;
    wire[7:0] gmii_txd;
    wire gmii_txen;

    //ACM1030
	wire	adc_data_en;
	wire ad_sample_en;
    wire [15:0]ad_out;
    wire ad_out_valid;

    assign eth_rst_n = 1;
    assign eth_mdc = 1;
    assign eth_mdio = 1;

    assign  g_rst_p = ~pll_lock;

    assign led = {~g_rst_p,init_calib_complete,pll_lock};

    assign ad_clk1 = loc_clk50m;
    assign ad_clk2 = loc_clk50m;

    Gowin_PLL Gowin_PLL(
        .lock(pll_lock), //output lock
        .clkout0(loc_clk50m), //output clkout0
        .clkout1(clk_200M), //output clkout1
        .clkin(clk50m), //input clkin
        .reset(~reset_n), //input reset
        .enclk0(1'b1), //input enclk0
        .enclk1(1'b1), //input enclk1
        .enclk2(pll_stop) //input enclk2
    );
    
    //以太网接收数据
    
    eth_pll eth_pll(
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

        .clk125m_o       (clk125m_o             ),
        .exter_mac       (             ),
        .exter_ip        (              ),
        .exter_port      (            ),
        .rx_data_length  (        ),
        .data_overflow_i (         ),
        .payload_valid_o (payload_valid_o      ),
        .payload_dat_o   (payload_dat_o        ),

        .one_pkt_done    (one_pkt_done           ),
        .pkt_error       (            ),
        .debug_crc_check (                      ),

        .gmii_rx_clk     (gmii_rx_clk           ),
        .gmii_rxdv       (gmii_rxdv             ),
        .gmii_rxd        (gmii_rxd              )
    );
    
    //FIFO存储以太网发送过来的命令帧
    fifo_rx fifo_rx(
        .Data(payload_dat_o), //input [7:0] Data
        .Reset(g_rst_p), //input Reset
        .WrClk(clk125m_o), //input WrClk
        .RdClk(loc_clk50m), //input RdClk
        .WrEn(payload_valid_o), //input WrEn
        .RdEn(fifo_rd_req), //input RdEn
        .Q(rxdout), //output [7:0] Q
        .Empty(rx_empty), //output Empty
        .Full() //output Full
    );

    eth_cmd eth_cmd (
        .clk(loc_clk50m),
        .reset_n(~g_rst_p),
        .fifo_rd_req(fifo_rd_req),
        .rx_empty(rx_empty),
        .fifodout(rxdout),
        .cmdvalid(cmdvalid_0),
        .address(address_0),
        .cmd_data(cmd_data_0)
    );

	cmd_rx cmd_rx_0(
		.clk(loc_clk50m),
		.reset_n(~g_rst_p),
		.cmdvalid(cmdvalid_0),
		.cmd_addr(address_0),
		.cmd_data(cmd_data_0),
		
		.ChannelSel(ChannelSel),
		.DataNum(DataNum),
		.ADC_Speed_Set(ADC_Speed_Set),
		.RestartReq(RestartReq)
	);

    
    reg RestartReq_0_d0,RestartReq_0_d1;
    reg [31:0]Number_d0,Number_d1;

    always@(posedge clk125m_o)
    begin
        Number_d0 <= DataNum;
        Number_d1 <= Number_d0;

        RestartReq_0_d0 <= RestartReq;
        RestartReq_0_d1 <= RestartReq_0_d0;
    end

      speed_ctrl speed_ctrl(
          .clk(loc_clk50m),
          .reset_n(reset_n),
          .ad_sample_en(ad_sample_en),
          .adc_data_en(adc_data_en),
          .div_set(ADC_Speed_Set)
      );

   //双通道的数据输出模块：AD1030采样的数据是10位的
   ad_10bit_to_16bit ad_10bit_to_16bit
   (
         .clk   (loc_clk50m ),
         .ad_sample_en(ad_sample_en),
         .ch_sel(ChannelSel ),
         .ad_in1(ad_in1     ),
         .ad_in2(ad_in2     ),
         .ad_out(ad_out     ),
         .ad_out_valid(ad_out_valid)    
   );

    state_ctrl state_ctrl(
        .clk(loc_clk50m),
        .reset(g_rst_p),
        .ddr3_init_done(init_calib_complete),  //DDR初始化完成标志信号
        .start_sample(RestartReq), //ADC启动采集标志信号
        .set_sample_num(DataNum),//需要采集的数量，32位，4G
        .rdfifo_empty(rdfifo_empty), //DDR双端口模块读FIFO为空标志信号
        .rdfifo_dout(rdfifo_dout), //从DDR中读出的16位数据
        .wrfifo_full(wrfifo_full),  //DDR写FIFO为满标志信号
        .adc_data_en(adc_data_en),  //ADC输出数据使能信号
        .ad_out_valid(ad_out_valid),	 //ADC输出数据有效信号
        .wr_load(wr_load),  //DDR双端口模块的写FIFO清除信号
        .rd_load(rd_load),  //DDR双端口模块的读FIFO清除信号
        .rdfifo_rden(rdfifo_rden), //DDR双端口模块的读使能信号
        .ad_sample_en(ad_sample_en),   //ADC采样使能标志信号
        .eth_fifo_wrreq(eth_fifo_wrreq), //以太网发送fifo_tx的写请求信号
        .eth_fifo_wrdata(eth_fifo_wrdata)    //需要以太网发送fifo_tx中写入的数据
      );

    assign wrfifo_clk = loc_clk50m;
    assign wrfifo_wren = ad_out_valid && adc_data_en;
    assign wrfifo_din = ad_out;

    assign rdfifo_clk = loc_clk50m;
    

    ddr3_ctrl_2port ddr3_ctrl_2port(
        .clk(loc_clk50m)                 ,      //50M时钟信号
		.pll_stop(pll_stop), //output pll_stop
        .pll_lock(pll_lock)            ,
        .clk_200m(clk_200M)            ,      //DDR3参考时钟信号
        .sys_rst_n(reset_n)           ,      //外部复位信号
        .init_calib_complete(init_calib_complete) ,    //DDR初始化完成信号

        //用户接口
        .rd_load(rd_load)             ,   //输出源更新信号
        .wr_load(wr_load)             ,   //输入源更新信号
        .app_addr_rd_min(28'd0)     ,   //读DDR3的起始地址
        .app_addr_rd_max(app_addr_max)     ,   //读DDR3的结束地址
        .rd_bust_len(burst_len)         ,   //从DDR3中读数据时的突发长度
        .app_addr_wr_min(28'd0)     ,   //写DD3的起始地址
        .app_addr_wr_max(app_addr_max)     ,   //写DDR的结束地址
        .wr_bust_len(burst_len)         ,   //向DDR3中写数据时的突发长度

        .wr_clk(wrfifo_clk)             ,//wr_fifo的写时钟信号
        .wfifo_wren(wrfifo_wren)          , //wr_fifo的写使能信号
        .wfifo_din(wrfifo_din)           , //写入到wr_fifo中的数据
        .wrfifo_full(wrfifo_full),
        .rd_clk(rdfifo_clk)              , //rd_fifo的读时钟信号
        .rfifo_rden(rdfifo_rden)          , //rd_fifo的读使能信号
        .rdfifo_empty(rdfifo_empty),
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
        .ddr3_cs_n(O_ddr_cs_n)        ,   //DDR3 片选
        .ddr3_dm(O_ddr_dqm)             ,   //DDR3_dm
        .ddr3_odt(O_ddr_odt)                //DDR3_odt   
    );
    

	//以太网发送FIFO
	fifo_tx fifo_tx(
		.Data({eth_fifo_wrdata[7:0],eth_fifo_wrdata[15:8]}), //input [15:0] Data
		.Reset(g_rst_p), //input Reset
		.WrClk(loc_clk50m), //input WrClk
		.RdClk(clk125m_o), //input RdClk
		.WrEn(eth_fifo_wrreq), //input WrEn
		.RdEn(payload_req_o), //input RdEn
		.Wnum(eth_fifo_usedw), //output [10:0] Wnum
		.Rnum(rd_data_count), //output [11:0] Rnum
		.Q(dout), //output [7:0] Q
		.Empty(eth_fifo_tx_empty), //output Empty
		.Full() //output Full
	);


    //以太网发送控制模块
    eth_send_ctrl eth_send_ctrl(
        .clk125M(clk125m_o),     
        .reset_n(~g_rst_p),  //模块的复位信号
        .eth_tx_done(tx_done),    //以太网一个包发送完毕信号
        .restart_req(RestartReq_0_d1),
        .fifo_rd_cnt(rd_data_count), //从FIFO中读取的数据个数
        .total_data_num(Number_d1),  //需要发送的数据总数
        .pkt_tx_en(tx_en_pulse),   //以太网发送使能信号
        .pkt_length(lenth_val)  //以太网需要发送的数据的长度
    ); 

    //以太网发送模块
    eth_udp_tx_gmii eth_udp_tx_gmii
    (
        .clk125m       (clk125m_o               ),
        .reset_p       (g_rst_p               ),

        .tx_en_pulse   (tx_en_pulse           ),
        .tx_done       (tx_done               ),

        .dst_mac       (48'hFF_FF_FF_FF_FF_FF            ),
        .src_mac       (LOCAL_MAC             ), 
        .dst_ip        (32'hc0_a8_00_03             ),
        .src_ip        (LOCAL_IP              ),
        .dst_port      (16'd6102           ),
        .src_port      (LOCAL_PORT            ),


        .data_length   (lenth_val        ),

        .payload_req_o (payload_req_o        ),
        .payload_dat_i (dout        ),

        .gmii_tx_clk   (gmii_tx_clk           ),
        .gmii_txen     (gmii_txen             ),
        .gmii_txd      (gmii_txd              )
    );

     gmii_to_rgmii gmii_to_rgmii(
      .reset_n(~g_rst_p),

      .gmii_tx_clk(gmii_tx_clk),
      .gmii_txd(gmii_txd),
      .gmii_txen(gmii_txen),
      .gmii_txer(1'b0),

      .rgmii_tx_clk(rgmii_tx_clk),
      .rgmii_txd(rgmii_txd),
      .rgmii_txen(rgmii_txen)
    );

endmodule