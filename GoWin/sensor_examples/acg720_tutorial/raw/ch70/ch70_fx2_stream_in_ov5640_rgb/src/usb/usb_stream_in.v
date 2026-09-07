/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : usb_stream_in
// Description   : USB数据流发送控制模块
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module usb_stream_in(
  reset_n,
  fx2_fdata,    //FX2型USB2.0芯片的SlaveFIFO的数据线
  fx2_faddr,    //FX2型USB2.0芯片的SlaveFIFO的FIFO地址线
  fx2_slrd,     //FX2型USB2.0芯片的SlaveFIFO的读控制信号，低电平有效
  fx2_slwr,     //FX2型USB2.0芯片的SlaveFIFO的写控制信号，低电平有效
  fx2_sloe,     //FX2型USB2.0芯片的SlaveFIFO的输出使能信号，低电平有效
  fx2_flagc,    //FX2型USB2.0芯片的端点6满标志
  fx2_flagb,    //FX2型USB2.0芯片的端点2空标志
  fx2_ifclk,    //FX2型USB2.0芯片的接口时钟信号
  fx2_pkt_end,  //数据包结束标志信号
  fx2_slcs,

  usb_fifo_rst,
  usb_fifo_wrclk,
  usb_fifo_wrdata,
  usb_fifo_full,
  usb_fifo_wrreq,
  usb_fifo_usedw,
  usb_fifo_wrempty
);

  input reset_n;
  input usb_fifo_rst;
  output [15:0]fx2_fdata;
  input fx2_flagb;
  input fx2_flagc;
  input fx2_ifclk;

  output [1:0]fx2_faddr;
  output fx2_sloe;
  output fx2_slwr;
  output fx2_slrd;
  output fx2_pkt_end;
  output fx2_slcs;

  input usb_fifo_wrclk;
  input [15:0]usb_fifo_wrdata;
  input usb_fifo_wrreq;
  output [10:0]usb_fifo_usedw;
  output usb_fifo_wrempty;
  output usb_fifo_full;
  
  wire slrd_n;
  reg slwr_n;
  wire sloe_n;
  reg slrd_d_n;
  wire empty;

  wire [15:0] data_out1;

  parameter stream_in_idle   = 1'b0;
  parameter stream_in_write  = 1'b1;

  reg current_stream_in_state;
  reg next_stream_in_state;

  assign fx2_slwr = slwr_n;
  assign fx2_slrd = slrd_n;
  assign fx2_sloe = sloe_n;
  assign fx2_pkt_end = 1'b1;
  assign fx2_slcs = 1'b0;
  assign fx2_fdata[15:0] = data_out1[15:0];
  assign fx2_faddr = 2'b10;
  
  assign usb_fifo_wrempty=empty;

  assign slrd_n = 1;
  assign sloe_n = 1;

  //write control signal generation
  always@(*)begin
    if((current_stream_in_state == stream_in_write) & (fx2_flagc == 1'b1) & (~empty))
      slwr_n <= 1'b0;
    else
      slwr_n <= 1'b1;
  end

  //Stream_IN mode state machine 
  always@(posedge fx2_ifclk, negedge reset_n) begin
    if(reset_n == 1'b0)
      current_stream_in_state <= stream_in_idle;
        else
      current_stream_in_state <= next_stream_in_state;
  end

  //Stream_IN mode state machine combo
  always@(*) begin
    next_stream_in_state = current_stream_in_state;
    case(current_stream_in_state)
      stream_in_idle:begin
        if(fx2_flagc == 1'b1)
          next_stream_in_state = stream_in_write;
        else
          next_stream_in_state = stream_in_idle;
      end
      stream_in_write:begin
        if(fx2_flagc == 1'b0)
          next_stream_in_state = stream_in_idle;
        else
          next_stream_in_state = stream_in_write;
      end
      default:
        next_stream_in_state = stream_in_idle;
    endcase
  end

//  fifo fifo(
//    .rst (usb_fifo_rst),// input wire srst
//    .wr_clk (usb_fifo_wrclk),            // input wire wr_clk
//    .rd_clk (fx2_ifclk),            // input wire rd_clk
//    .din (usb_fifo_wrdata),// input wire [15 : 0] din
//    .wr_en (usb_fifo_wrreq),// input wire wr_en
//    .rd_en (~slwr_n),// input wire rd_en
//    .dout (data_out1),// output wire [15 : 0] dout
//    .full (usb_fifo_full ),// output wire full
//    .empty (empty),// output wire empty
//    .wr_data_count (usb_fifo_usedw),// output wire [10 : 0] data_count
//    .wr_rst_busy ( ),  // output wire wr_rst_busy
//    .rd_rst_busy ( )  // output wire rd_rst_busy
//  );

	fifo_top fifo_top(
		.Data(usb_fifo_wrdata), //input [15:0] Data
		.Reset(usb_fifo_rst), //input Reset
		.WrClk(usb_fifo_wrclk), //input WrClk
		.RdClk(fx2_ifclk), //input RdClk
		.WrEn(usb_fifo_wrreq), //input WrEn
		.RdEn(~slwr_n), //input RdEn
		.Wnum(usb_fifo_usedw), //output [10:0] Wnum
		.Almost_Empty(), //output Almost_Empty
		.Almost_Full(), //output Almost_Full
		.Q(data_out1), //output [15:0] Q
		.Empty(empty), //output Empty
		.Full(usb_fifo_full) //output Full
	);

endmodule
