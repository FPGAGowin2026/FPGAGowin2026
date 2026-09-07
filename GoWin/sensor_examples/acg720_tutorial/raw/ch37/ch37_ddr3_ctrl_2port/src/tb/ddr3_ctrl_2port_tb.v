/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : ddr3_ctrl_2port_tb
// Description   : ddr3_ctrl_2port模块仿真文件
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/100ps

module ddr3_ctrl_2port_tb();

  reg           ddr3_clk200m;
  reg           ddr3_rst_n;
  wire          ddr3_init_done;
  //wr_fifo Interface
  reg           wr_load;
  reg           wrfifo_clk;
  reg           wrfifo_wren;
  reg    [15:0] wrfifo_din;
  wire          wrfifo_full;
  //rd_fifo Interface
  reg           rd_load;
  reg           rdfifo_clk;
  reg           rdfifo_rden;
  wire   [15:0] rdfifo_dout;
  wire          rdfifo_empty;

   wire                  ddr_cs;
   wire                  ddr_ras;
   wire                  ddr_cas;
   wire                  ddr_we;
   wire                  ddr_ck;
   wire                  ddr_ck_n;
   wire                  ddr_cke;
   wire                  ddr_odt;
   wire                  ddr_reset_n;
   wire [2-1:0]          ddr_dm;	//DM_WIDTH=2
   wire [16-1:0]         ddr_dq;	//DQ_WIDTH=16
   wire [2-1:0]          ddr_dqs;	//DQS_WIDTH=2
   wire [2-1:0]          ddr_dqs_n;
   wire [14-1:0]         ddr_addr;		//ROW_WIDTH=14
   wire [3-1:0]          ddr_bank;		//BANK_WIDTH=3

  reg   pll_lock;
  reg   clk;


  initial ddr3_clk200m = 1'b1;
  always #2.5 ddr3_clk200m = ~ddr3_clk200m;

  initial wrfifo_clk = 1'b1;
  always #5 wrfifo_clk = ~wrfifo_clk;

  initial rdfifo_clk = 1'b1;
  always #5 rdfifo_clk = ~rdfifo_clk;
  
  initial clk = 1'b1;
  always #10 clk = ~clk;


  initial begin
    ddr3_rst_n = 1'b0;
    wr_load  = 1'b1;
    wrfifo_wren = 1'b0;
    wrfifo_din  = 16'd0;
    rd_load  = 1'b1;
    rdfifo_rden = 1'b0;
    pll_lock   = 1'b0;
    #201;
    ddr3_rst_n = 1'b1;
    pll_lock   = 1'b1;
    #200;
    wr_load  = 1'b0;
    rd_load  = 1'b0;
    @(posedge ddr3_init_done);
    #200;
    wr_data(16'd1,16'd1024);
    #2000;
    rd_load  = 1'b1;
    #20;
    rd_load  = 1'b0;
    #20000;
    rd_data(16'd1024);
	
	
    #15000;
    $stop;
  end

  task wr_data;
    input [15:0]data_begin;
    input [15:0]wr_data_cnt;
    begin
      wrfifo_wren = 1'b0;
      wrfifo_din  = data_begin;
      @(posedge wrfifo_clk);
      #1 wrfifo_wren = 1'b1;
      repeat(wr_data_cnt)
      begin        
        @(posedge wrfifo_clk);
        wrfifo_din = wrfifo_din + 1'b1;
      end
      #1 wrfifo_wren = 1'b0;
    end
  endtask

  task rd_data;
    input [15:0]rd_data_cnt;
    begin
      rdfifo_rden = 1'b0;
      @(posedge rdfifo_clk);
		  #1 rdfifo_rden = 1'b1;
		  repeat(rd_data_cnt)
		  begin
			@(posedge rdfifo_clk);
		  end 
      #1 rdfifo_rden = 1'b0;
    end
  endtask

    wire [27:0] app_addr_max = 28'd1024;
    wire [7:0] burst_len = 8'd64;

    ddr3_ctrl_2port ddr3_ctrl_2port(
        .clk(clk)                 ,      //50M时钟信号
        .pll_lock(pll_lock)            ,
        .clk_200m(ddr3_clk200m)            ,      //DDR3参考时钟信号
        .sys_rst_n(ddr3_rst_n)           ,      //外部复位信号
        .init_calib_complete(ddr3_init_done) ,    //DDR初始化完成信号

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
        .wrfifo_full(),
        .rd_clk(rdfifo_clk)              , //rd_fifo的读时钟信号
        .rfifo_rden(rdfifo_rden)          , //rd_fifo的读使能信号
        .rdfifo_empty(),
        .rfifo_dout(rdfifo_dout)          , //rd_fifo读出的数据信号 

        //DDR3   
        .ddr3_dq(ddr_dq)             ,   //DDR3 数据
        .ddr3_dqs_n(ddr_dqs_n)          ,   //DDR3 dqs负
        .ddr3_dqs_p(ddr_dqs)          ,   //DDR3 dqs正  
        .ddr3_addr(ddr_addr)           ,   //DDR3 地址   
        .ddr3_ba(ddr_bank)             ,   //DDR3 banck 选择
        .ddr3_ras_n(ddr_ras)          ,   //DDR3 行选择
        .ddr3_cas_n(ddr_cas)          ,   //DDR3 列选择
        .ddr3_we_n(ddr_we)           ,   //DDR3 读写选择
        .ddr3_reset_n(ddr_reset_n)        ,   //DDR3 复位
        .ddr3_ck_p(ddr_ck)          ,   //DDR3 时钟正
        .ddr3_ck_n(ddr_ck_n)           ,   //DDR3 时钟负
        .ddr3_cke(ddr_cke)            ,   //DDR3 时钟使能
        .ddr3_cs_n(ddr_cs)           ,   //DDR3 片选
        .ddr3_dm(ddr_dm)             ,   //DDR3_dm
        .ddr3_odt(ddr_odt)                //DDR3_odt   
    );


    ddr3_model ddr3_model(
        .rst_n            (ddr_reset_n),
        .ck               (ddr_ck),
        .ck_n             (ddr_ck_n),
        .cke              (ddr_cke),
        .cs_n             (ddr_cs),
        .ras_n            (ddr_ras),
        .cas_n            (ddr_cas),
        .we_n             (ddr_we),
        .dm_tdqs          (ddr_dm),
        .ba               (ddr_bank),
        .addr             (ddr_addr),
        .dq               (ddr_dq),
        .dqs              (ddr_dqs),
        .dqs_n            (ddr_dqs_n),
        .tdqs_n           (),
        .odt              (ddr_odt)
    );

endmodule