/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2022/10/23 00:00:00
// Module Name   : eth_rx_ctrl
// Description   : 以太网接收控制模块，将以太网接收的报文数据存入FIFO中
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module eth_rx_ctrl
#(
  parameter O_DATA_WIDTH = 16
)
(
  input                        clk          ,
  input                        reset        ,

  output                       fifo_full    ,
  input                        fifo_wr      ,
  input     [7:0]              fifo_din     ,

  input                        one_pkt_done ,
  input                        pkt_error    ,

  output reg[O_DATA_WIDTH-1:0] rx_data      ,
  output reg                   rx_data_valid
);

    reg                   wrfifo_sel;
    wire                  rdfifo_sel;

    reg                   fifo0_srst;
    wire                  fifo0_wr_en;
    reg                   fifo0_rd_en;
    wire[O_DATA_WIDTH-1:0]fifo0_dout;
    wire                  fifo0_full;
    wire[10:0]            fifo0_rd_data_cnt;

    reg                   fifo1_srst;
    wire                  fifo1_wr_en;
    reg                   fifo1_rd_en;
    wire[O_DATA_WIDTH-1:0]fifo1_dout;
    wire                  fifo1_full;
    wire[10:0]            fifo1_rd_data_cnt;

    reg                   fifo0_rd_en_dly1;
    reg                   fifo1_rd_en_dly1;

	rx_buf_fifo rx_buf_fifo_0(
		.Data(fifo_din), //input [7:0] Data
		.Reset(fifo0_srst), //input Reset
		.WrClk(clk), //input WrClk
		.RdClk(clk), //input RdClk
		.WrEn(fifo0_wr_en), //input WrEn
		.RdEn(fifo0_rd_en), //input RdEn
		.Rnum(fifo0_rd_data_cnt), //output [10:0] Rnum
		.Q(fifo0_dout), //output [15:0] Q
		.Empty(), //output Empty
		.Full(fifo0_full) //output Full
	);

	rx_buf_fifo rx_buf_fifo_1(
		.Data(fifo_din), //input [7:0] Data
		.Reset(fifo1_srst), //input Reset
		.WrClk(clk), //input WrClk
		.RdClk(clk), //input RdClk
		.WrEn(fifo1_wr_en), //input WrEn
		.RdEn(fifo1_rd_en), //input RdEn
		.Rnum(fifo1_rd_data_cnt), //output [10:0] Rnum
		.Q(fifo1_dout), //output [15:0] Q
		.Empty(), //output Empty
		.Full(fifo1_full) //output Full
	);


    assign fifo0_wr_en = fifo_wr && (wrfifo_sel == 1'b0);
    assign fifo1_wr_en = fifo_wr && (wrfifo_sel == 1'b1);

    assign fifo_full = (wrfifo_sel == 1'b0) ? fifo0_full : fifo1_full;

    always@(posedge clk or posedge reset)
    begin
      if(reset)
        wrfifo_sel <= 1'b0;
      else if(one_pkt_done)
        wrfifo_sel <= ~wrfifo_sel;
      else
        wrfifo_sel <= wrfifo_sel;
    end

    assign rdfifo_sel = ~wrfifo_sel;

    always@(posedge clk or posedge reset)
    begin
      if(reset)
        fifo0_srst <= 1'b1;
      else if(one_pkt_done && pkt_error && (wrfifo_sel == 1'b0))
        fifo0_srst <= 1'b1;
      else
        fifo0_srst <= 1'b0;
    end

    always@(posedge clk or posedge reset)
    begin
      if(reset)
        fifo1_srst <= 1'b1;
      else if(one_pkt_done && pkt_error && (wrfifo_sel == 1'b1))
        fifo1_srst <= 1'b1;
      else
        fifo1_srst <= 1'b0;
    end

    always@(posedge clk or posedge reset)
    begin
      if(reset)
        fifo0_rd_en <= 1'b0;
      else if((fifo0_rd_data_cnt > 11'd2) && (rdfifo_sel == 1'b0))
        fifo0_rd_en <= 1'b1;
      else
        fifo0_rd_en <= 1'b0;
    end

    always@(posedge clk or posedge reset)
    begin
      if(reset)
        fifo1_rd_en <= 1'b0;
      else if((fifo1_rd_data_cnt > 11'd2) && (rdfifo_sel == 1'b1))
        fifo1_rd_en <= 1'b1;
      else
        fifo1_rd_en <= 1'b0;
    end

    always@(posedge clk)
    begin
      fifo0_rd_en_dly1 <= fifo0_rd_en;
      fifo1_rd_en_dly1 <= fifo1_rd_en;
    end

    always@(posedge clk or posedge reset)
    begin
      if(reset)
        rx_data <= 'd0;
      else if(fifo0_rd_en_dly1)
        rx_data <= fifo0_dout;
      else if(fifo1_rd_en_dly1)
        rx_data <= fifo1_dout;
      else
        rx_data <= 'd0;
    end

    always@(posedge clk or posedge reset)
    begin
      if(reset)
        rx_data_valid <= 1'b0;
      else if(fifo0_rd_en_dly1)
        rx_data_valid <= 1'b1;
      else if(fifo1_rd_en_dly1)
        rx_data_valid <= 1'b1;
      else
        rx_data_valid <= 1'b0;
    end

endmodule