/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : fx2_stream_in_ov5640_rgb
// Description   : 摄像头采集，USB发送
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module fx2_stream_in_ov5640_rgb(
  //System clock reset
  input       clk50m        , //系统时钟输入，50MHz
  input       reset_n       , //复位信号输入
  //usb interface
  output      fx2_clear     ,
  ////usb_ctrl
  inout [15:0]fx2_fdata     ,
  input       fx2_flagb     ,
  input       fx2_flagc     ,
  input       fx2_ifclk     ,
  
  output [1:0]fx2_faddr     ,
  output      fx2_sloe      ,
  output      fx2_slwr      ,
  output      fx2_slrd      ,
  output      fx2_pkt_end   , 
  output      fx2_slcs      ,
  //camera interface
  output      camera_sclk   ,
  inout       camera_sdat   ,
  input       camera_vsync  ,
  input       camera_href   ,
  input       camera_pclk   ,
  output      camera_xclk   ,
  input  [7:0]camera_data   ,
  output      camera_rst_n  ,
  output      camera_pwdn   ,
  //led
  output [1:0]led         
);
  //*****************************
  //Set IMAGE Size  
  parameter IMAGE_WIDTH  = 800;
  parameter IMAGE_HEIGHT = 480;
  //*****************************
  //clock
  wire          pll_locked;
  wire          loc_clk50m;
  wire          loc_clk24m;
  //reset
  wire          g_rst_p;
  //camera interface
  wire          camera_init_done;
  wire          pclk_bufg_o;
  wire [15:0]   image_data;
  wire          image_data_valid;
  wire          image_data_hs;
  wire          image_data_vs;
  wire [11:0]   image_data_xaddr;
  wire [11:0]   image_data_yaddr;

  reg           pixel_data_valid;
  reg  [15:0]   pixel_data;
  
  //USB Ctrl
  wire [1:0]fx2_faddr0;//FX2型USB2.0芯片的SlaveFIFO的FIFO地址线
  wire fx2_slrd0;//FX2型USB2.0芯片的SlaveFIFO的读控制信号，低电平有效
  wire fx2_slwr0;//FX2型USB2.0芯片的SlaveFIFO的写控制信号，低电平有效
  wire fx2_sloe0;//FX2型USB2.0芯片的SlaveFIFO的输出使能信号，低电平有效
  wire fx2_flagc0;//FX2型USB2.0芯片的端点6满标志
  wire fx2_flagb0; //FX2型USB2.0芯片的端点2空标志
  wire fx2_pkt_end0;//数据包结束标志信号
  wire fx2_slcs0;
  
  wire [15:0]fx2_fdata1;
  wire [1:0]fx2_faddr1;//FX2型USB2.0芯片的SlaveFIFO的FIFO地址线
  wire fx2_slrd1;//FX2型USB2.0芯片的SlaveFIFO的读控制信号，低电平有效
  wire fx2_slwr1;//FX2型USB2.0芯片的SlaveFIFO的写控制信号，低电平有效
  wire fx2_sloe1;//FX2型USB2.0芯片的SlaveFIFO的输出使能信号，低电平有效
  wire fx2_flagc1;//FX2型USB2.0芯片的端点6满标志
  wire fx2_flagb1;//FX2型USB2.0芯片的端点2空标志
  wire fx2_pkt_end1;//数据包结束标志信号
  wire fx2_slcs1;
  
  wire inout_switch;    
  //0: read由pc向FPGA下发指令   1:write由FPGA向fx2芯片继而向pc上传数据
  wire rw_switch;
      
  wire usb_fifo_full;
  wire          usb_fifo_wrempty; 
  wire [15:0]   usb_fifo_wrdata;
  wire          usb_fifo_wrreq;
  wire [10:0]   usb_fifo_usedw;
 
  assign rw_switch = (inout_switch || (usb_fifo_wrempty == 0 ))?1'd1:1'd0;
  //inout_switch为1是rw_switch为1的充分条件，
  //如果usb_fifo_wrempty没有为1，
  //说明usbfifo还没有读完，需要再给它一点读完的时间直到收到它为0的反馈
  assign fx2_fdata=rw_switch? fx2_fdata1 : 16'dz;

  assign fx2_faddr=rw_switch ? fx2_faddr1 : fx2_faddr0;
  assign fx2_sloe=rw_switch ? fx2_sloe1 : fx2_sloe0;
  assign fx2_slwr=rw_switch ? fx2_slwr1 : fx2_slwr0;
  assign fx2_slrd=rw_switch ? fx2_slrd1 : fx2_slrd0;
  assign fx2_pkt_end=rw_switch ? fx2_pkt_end1 : fx2_pkt_end0;
  assign fx2_slcs=rw_switch ? fx2_slcs1 : fx2_slcs0;
  

  assign g_rst_p     = ~pll_locked;
  assign led         = {camera_init_done,pll_locked};
  
//  pll pll
//  (
//     Clock out ports
//    .clk_out1 (loc_clk50m  ), // output clk_out1
//    .clk_out2 (loc_clk24m ), // output clk_out2
//     Status and control signals
//    .resetn   (reset_n     ), // input reset
//    .locked   (pll_locked  ), // output locked
//     Clock in ports
//    .clk_in1  (clk50m      )  // input clk_in1
//  );

    Gowin_PLL Gowin_PLL(
        .lock(pll_locked), //output lock
        .clkout0(loc_clk50m), //output clkout0
        .clkout1(loc_clk24m), //output clkout1
        .clkin(clk50m), //input clkin
        .reset(~reset_n) //input reset
    );
  
  wire [15:0]usb_data_out;
  wire usb_data_valid;
  
  //USB数据流发送控制模块：不断的将端点2中的数据读取出来，数据读取后直接作为端口输出
  usb_stream_out usb_stream_out(
    .clk (loc_clk50m),
    .fx2_fdata (fx2_fdata), //  FX2型USB2.0芯片的SlaveFIFO的数据线
    .fx2_faddr (fx2_faddr0), //  FX2型USB2.0芯片的SlaveFIFO的FIFO地址线
    .fx2_slrd (fx2_slrd0),  //  FX2型USB2.0芯片的SlaveFIFO的读控制信号，低电平有效
    .fx2_slwr (fx2_slwr0),  //  FX2型USB2.0芯片的SlaveFIFO的写控制信号，低电平有效
    .fx2_sloe (fx2_sloe0),  //  FX2型USB2.0芯片的SlaveFIFO的输出使能信号，低电平有效
    .fx2_flagc (fx2_flagc ), //  FX2型USB2.0芯片的端点6满标志
    .fx2_flagb (fx2_flagb ), //  FX2型USB2.0芯片的端点2空标志
    .fx2_ifclk (fx2_ifclk ), //  FX2型USB2.0芯片的接口时钟信号
    .fx2_pkt_end (fx2_pkt_end0),	//数据包结束标志信号
    .fx2_slcs (fx2_slcs0),
    .reset_n (reset_n),
    .data_out (usb_data_out),	
    //经过FPGA接收了的USB数据.这个数据从pc经过fx2芯片提供给FPGA
    .data_valid (usb_data_valid),	
    //经过FPGA接收了的USB数据有效标志信号.FPGA只要在读数据，这个信号一直拉高输出给FX2
    .source_ready (~rw_switch)	
    //外部数据消费者数据接收允许信号，例如FPGA中的缓存FIFO中有足够的空间存储一帧USB数据，则允许从Slave FIFO中去读取数据。当FPGA有能力读取一帧数据，则向外
   ); 
   
  wire [7:0]cmd_addr;
  wire [31:0]cmd_data;
  wire cmdvalid;
   ////////////下方模块对接收的信号进行解析，输出地址、数据、有效，然后通过地址判断这个数据是采样起始信号，采样数量，还是采样通道////////////
  usb_cmd usb_cmd_inst(
    .Clk (fx2_ifclk),
    .Reset_n (~g_rst_p),
    .rx_data (usb_data_out),
    .rx_done (usb_data_valid),
    .address (cmd_addr),
    .data (cmd_data),
    .cmdvalid (cmdvalid)
  );
  
  wire start_sample;
  //命令解析函数
 usb_cmd_rx usb_cmd_rx(
    .clk(fx2_ifclk),
    .reset(g_rst_p),
    .cmdvalid(cmdvalid),
    .cmd_addr(cmd_addr),
    .cmd_data(cmd_data),
    .start_sample(start_sample)
 );

  assign camera_xclk = loc_clk24m;
  camera_init
  #(
    .SYS_CLOCK      ( 50_000_000   ),//系统时钟采用50MHz
    .SCL_CLOCK      ( 400_000      ),//SCL总线时钟采用400kHz
    .CAMERA_TYPE    ( "ov5640"     ),//"ov5640" or "ov7725"
    .IMAGE_TYPE     ( 0            ),// 0: RGB; 1: JPEG
    .IMAGE_WIDTH    ( IMAGE_WIDTH  ),// 图片宽度
    .IMAGE_HEIGHT   ( IMAGE_HEIGHT ),// 图片高度
    .IMAGE_FLIP_EN  ( 0            ),// 0: 不翻转，1: 上下翻转
    .IMAGE_MIRROR_EN( 0            ) // 0: 不镜像，1: 左右镜像
  )camera_init
  (
    .Clk         (loc_clk50m       ),
    .Rst_n       (~g_rst_p         ),
    .Init_Done   (camera_init_done ),
    .camera_rst_n(camera_rst_n     ),
    .camera_pwdn (camera_pwdn      ),
    .i2c_sclk    (camera_sclk      ),
    .i2c_sdat    (camera_sdat      )
  );

  
//  BUFG BUFG_inst (
//    .O(pclk_bufg_o ), // 1-bit output: Clock output
//    .I(camera_pclk )  // 1-bit input: Clock input
//  );
  assign pclk_bufg_o = camera_pclk;

 wire ImageState;
  DVP_Capture DVP_Capture(
    .Rst_n      (~g_rst_p         ),//input
    .PCLK       (pclk_bufg_o      ),//input
    .Vsync      (camera_vsync     ),//input
    .Href       (camera_href      ),//input
    .Data       (camera_data      ),//input     [7:0]

    .ImageState (ImageState       ),//output reg
    .DataValid  (image_data_valid ),//output
    .DataPixel  (image_data       ),//output    [15:0]
    .DataHs     (image_data_hs    ),//output
    .DataVs     (image_data_vs    ),//output
    .Xaddr      (image_data_xaddr ),//output    [11:0],start is 1
    .Yaddr      (image_data_yaddr ) //output    [11:0],start is 1
  );
  
    reg[4:0] start_sample_r;
    always@(posedge fx2_ifclk)
    begin
        start_sample_r <= {start_sample_r[3:0],start_sample};           //拓宽start_sample信号，确保能收到启动开始信号，使摄像头的数据通过USB进行传输
    end

  reg        image_data_vs_dly1;
  reg [15:0] image_data_dly1;
  reg        image_data_valid_dly1;

  always@(posedge pclk_bufg_o)
  begin
    image_data_vs_dly1    <= image_data_vs;
    image_data_dly1       <= image_data;
    image_data_valid_dly1 <= image_data_valid;
  end

    usb_send_ctrl usb_send_ctrl
    (
      .reset_p          (~reset_n)         ,

      .clk              (pclk_bufg_o)              ,  //pclk_bufg_o
      .data_i           (image_data_dly1),
      .data_valid_i     (image_data_valid_dly1)     ,
      .start_sample     (|start_sample_r)     , //按位或，只要检测到start_sample_r中某一时刻得到start_sample信号就启动传输
      .camera_vs_start  (~image_data_vs_dly1 && image_data_vs),
      
      .usb_fifo_full    (usb_fifo_full),
      .inout_switch     (inout_switch)      ,
      .usb_fifo_wrreq   (usb_fifo_wrreq)    , //USB FIFO的写请求信号，产生该信号,USB开始发送数据
      .fx2_clear        (fx2_clear)         ,//USB清除
      .usb_fifo_wrdata  (usb_fifo_wrdata)    //USB需要发送的数据
      );
    
  //USB数据流发送控制模块
  usb_stream_in usb_stream_in(
    .reset_n (~g_rst_p),
    .fx2_fdata (fx2_fdata1), //FX2型USB2.0芯片的SlaveFIFO的数据线
    .fx2_faddr (fx2_faddr1), //FX2型USB2.0芯片的SlaveFIFO的FIFO地址线
    .fx2_slrd (fx2_slrd1), //FX2型USB2.0芯片的SlaveFIFO的读控制信号，低电平有效
    .fx2_slwr (fx2_slwr1), //FX2型USB2.0芯片的SlaveFIFO的写控制信号，低电平有效
    .fx2_sloe (fx2_sloe1), //FX2型USB2.0芯片的SlaveFIFO的输出使能信号，低电平有效
    .fx2_flagc (fx2_flagc), //FX2型USB2.0芯片的端点6满标志
    .fx2_flagb (fx2_flagb), //FX2型USB2.0芯片的端点2空标志
    .fx2_ifclk (fx2_ifclk), //FX2型USB2.0芯片的接口时钟信号
    .fx2_pkt_end (fx2_pkt_end1), //数据包结束标志信号
    .fx2_slcs (fx2_slcs1),

    .usb_fifo_wrclk (pclk_bufg_o),
    .usb_fifo_wrdata(usb_fifo_wrdata),
    .usb_fifo_full(usb_fifo_full),
    .usb_fifo_wrreq (usb_fifo_wrreq),
    .usb_fifo_usedw (usb_fifo_usedw),
    .usb_fifo_wrempty(usb_fifo_wrempty),
    .usb_fifo_rst(fx2_clear)
  );
  
endmodule