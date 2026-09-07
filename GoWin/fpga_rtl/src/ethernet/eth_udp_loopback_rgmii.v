/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
//
// Create Date   : 2021/12/20 00:00:00
// Module Name   : eth_udp_loopback_rgmii
// Description   : 以太网rgmii环回测试顶层
//
// Dependencies  :
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
/////////////////////////////////////////////////////////////////////////////////

module eth_udp_loopback_rgmii(
  input         reset_n,
  //eth_rx
  input         rgmii_rx_clk_i,
  input  [3:0]  rgmii_rxd,
  input         rgmii_rxdv,
  output 		eth_rst_n,

  //eth_tx
  output        rgmii_tx_clk,
  output  [3:0] rgmii_txd,
  output        rgmii_txen

);
parameter LOCAL_MAC  = 48'h00_0a_35_01_fe_c0;
parameter LOCAL_IP   = 32'hc0_a8_00_02;
parameter LOCAL_PORT = 16'd5000;
//eth_rx
wire        reset_p;
wire        clk125m;
wire [47:0] exter_mac;
wire [31:0] exter_ip;
wire [15:0] exter_port;
wire [15:0] rx_data_length;
wire        data_overflow;
wire        payload_buffer_full;
wire [7:0]  rx_payload_dat;
wire        rx_payload_valid;
wire        rx_pkt_done;
wire        rx_pkt_err;
reg  [3:0]  pkt_right_cnt;
reg  [3:0]  pkt_err_cnt;
//eth_tx
wire        tx_done;
wire        tx_en_pulse;
wire [47:0] tx_dst_mac;
wire [31:0] tx_dst_ip;
wire [15:0] tx_dst_port;
wire [15:0] tx_data_length;
wire [7:0]  tx_payload_dat;
wire        tx_payload_req;

//eth_msg_buf interface
 wire        eth_msg_wr_en;
 wire [111:0]eth_msg_din;
 reg         eth_msg_rd_en;
 wire [111:0]eth_msg_dout;
 wire [4 : 0]eth_msg_dat_cnt;
 wire        eth_msg_full;
 reg         eth_msg_rd_en_dly1;
 reg         eth_tx_state;  //1:tx  0:idle
// gmii interface
 wire gmii_rx_clk;
 wire [7:0] gmii_rxd;
 wire gmii_rxdv;

 wire gmii_tx_clk;
 wire [7:0] gmii_txd;
 wire gmii_txen;

  wire rgmii_rx_clk;
 wire pll_locked;
 assign eth_rst_n = 1;
 assign data_overflow = payload_buffer_full || eth_msg_full;


    Gowin_PLL Gowin_PLL(
        .clkin(rgmii_rx_clk_i), //input  clkin
        .init_clk(rgmii_rx_clk_i), //input  init_clk
        .clkout0(rgmii_rx_clk) //output  clkout0
);

  assign reset_p = ~reset_n;

  always@(posedge clk125m or posedge reset_p)
  begin
    if(reset_p)
      pkt_right_cnt <= 4'd0;
    else if(~rx_pkt_err && rx_pkt_done)
      pkt_right_cnt <= pkt_right_cnt + 1'b1;
    else
      pkt_right_cnt <= pkt_right_cnt;
  end

  always@(posedge clk125m or posedge reset_p)
  begin
    if(reset_p)
      pkt_err_cnt <= 4'd0;
    else if(rx_pkt_err && rx_pkt_done)
      pkt_err_cnt <= pkt_err_cnt + 1'b1;
    else
      pkt_err_cnt <= pkt_err_cnt;
  end

  rgmii_to_gmii rgmii_to_gmii(
  .reset(reset_p),

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
    .reset_p         (reset_p               ),

    .local_mac       (LOCAL_MAC             ),
    .local_ip        (LOCAL_IP              ),
    .local_port      (LOCAL_PORT            ),

    .clk125m_o       (clk125m               ),
    .exter_mac       (exter_mac             ),
    .exter_ip        (exter_ip              ),
    .exter_port      (exter_port            ),
    .rx_data_length  (rx_data_length        ),
    .data_overflow_i (data_overflow         ),
    .payload_valid_o (rx_payload_valid      ),
    .payload_dat_o   (rx_payload_dat        ),

    .one_pkt_done    (rx_pkt_done           ),
    .pkt_error       (rx_pkt_err            ),
    .debug_crc_check (                      ),

    .gmii_rx_clk     (gmii_rx_clk           ),
    .gmii_rxdv       (gmii_rxdv             ),
    .gmii_rxd        (gmii_rxd              )
  );

	// 接收负载在 FCS 判定前仅为“暂存”数据；合法帧才提交给发送侧，
	// 坏帧回滚整帧写指针，避免残留字节污染下一条合法 UDP 报文。
	eth_packet_data_buf eth_packet_data_buf(
		.clk(clk125m),
		.reset_p(reset_p),
		.frame_data(rx_payload_dat),
		.frame_wr_en(rx_payload_valid),
		.frame_done(rx_pkt_done),
		.frame_accept(~rx_pkt_err),
		.rd_en(tx_payload_req),
		.q(tx_payload_dat),
		.empty(),
		.full(payload_buffer_full)
	);

  //同时对报文中MAC、IP等消息数据缓存
  // 元数据与负载共用同一 FCS 提交条件，坏帧不进入发送描述符队列。
  assign eth_msg_wr_en = rx_pkt_done && !rx_pkt_err;
  assign eth_msg_din   = {exter_mac,exter_ip,exter_port,rx_data_length};

	eth_msg_buf eth_msg_buf(
		.Data(eth_msg_din), //input [111:0] Data
		.Clk(clk125m), //input Clk
		.WrEn(eth_msg_wr_en), //input WrEn
		.RdEn(eth_msg_rd_en), //input RdEn
		.Reset(reset_p), //input Reset
		.Wnum(eth_msg_dat_cnt), //output [5:0] Wnum
		.Q(eth_msg_dout), //output [111:0] Q
		.Empty(), //output Empty
		.Full(eth_msg_full) //output Full
	);

  always@(posedge clk125m or posedge reset_p)
  begin
    if(reset_p)
      eth_tx_state <= 1'b0;
    else if(tx_done)
      eth_tx_state <= 1'b0;
    else if(eth_msg_dat_cnt >0)
      eth_tx_state <= 1'b1;
    else
      eth_tx_state <= eth_tx_state;
  end

  always@(posedge clk125m or posedge reset_p)
  begin
    if(reset_p)
      eth_msg_rd_en <= 1'b0;
    else if((eth_tx_state == 1'b0)&&(eth_msg_dat_cnt >0))
      eth_msg_rd_en <= 1'b1;
    else
      eth_msg_rd_en <= 1'b0;
  end

  always@(posedge clk125m or posedge reset_p)
  begin
    if(reset_p)
      eth_msg_rd_en_dly1 <= 1'b0;
    else
      eth_msg_rd_en_dly1 <= eth_msg_rd_en;
  end

  assign tx_en_pulse    = eth_msg_rd_en_dly1;
  assign tx_dst_mac     = eth_msg_dout[111:64];
  assign tx_dst_ip      = eth_msg_dout[63:32];
  assign tx_dst_port    = eth_msg_dout[31:16];
  assign tx_data_length = eth_msg_dout[15:0];

  eth_udp_tx_gmii eth_udp_tx_gmii
  (
    .clk125m       (clk125m               ),
    .reset_p       (reset_p               ),

    .tx_en_pulse   (tx_en_pulse           ),
    .tx_done       (tx_done               ),

    .dst_mac       (tx_dst_mac            ),
    .src_mac       (LOCAL_MAC             ),
    .dst_ip        (tx_dst_ip             ),
    .src_ip        (LOCAL_IP              ),
    .dst_port      (tx_dst_port           ),
    .src_port      (LOCAL_PORT            ),


    .data_length   (tx_data_length        ),

    .payload_req_o (tx_payload_req        ),
    .payload_dat_i (tx_payload_dat        ),

    .gmii_tx_clk   (gmii_tx_clk           ),
    .gmii_txen     (gmii_txen             ),
    .gmii_txd      (gmii_txd              )
  );

 gmii_to_rgmii gmii_to_rgmii(
  .reset_n(reset_n),

  .gmii_tx_clk(gmii_tx_clk),
  .gmii_txd(gmii_txd),
  .gmii_txen(gmii_txen),
  .gmii_txer(1'b0),

  .rgmii_tx_clk(rgmii_tx_clk),
  .rgmii_txd(rgmii_txd),
  .rgmii_txen(rgmii_txen)
);
endmodule

// 帧感知负载 FIFO：写入区在 frame_done 前不可读，FCS 通过后原子提交，
// FCS/溢出失败则把写指针回退到本帧起点。接口保持 FWFT 读语义。
module eth_packet_data_buf(
  input         clk,
  input         reset_p,
  input  [7:0]  frame_data,
  input         frame_wr_en,
  input         frame_done,
  input         frame_accept,
  input         rd_en,
  output reg [7:0] q,
  output        empty,
  output        full
);
  parameter ADDR_WIDTH = 13;
  localparam DEPTH = (1 << ADDR_WIDTH);

  reg [7:0] mem [0:DEPTH-1];
  reg [ADDR_WIDTH-1:0] wr_ptr;
  reg [ADDR_WIDTH-1:0] frame_start_ptr;
  reg [ADDR_WIDTH-1:0] rd_ptr;
  reg [ADDR_WIDTH:0] committed_count;
  reg [ADDR_WIDTH:0] speculative_count;

  wire write_fire;
  wire read_fire;
  wire [ADDR_WIDTH-1:0] prefetch_addr;

  assign empty = (committed_count == 0);
  assign full = ((committed_count + speculative_count) == DEPTH);
  assign write_fire = frame_wr_en && !full;
  assign read_fire = rd_en && !empty;
  assign prefetch_addr = read_fire ? (rd_ptr + 1'b1) : rd_ptr;

  always @(posedge clk)
  begin
    if(write_fire)
      mem[wr_ptr] <= frame_data;
  end

  // 寄存式预取保持外部 FWFT 语义，同时让存储体可按同步读 RAM 综合。
  always @(posedge clk)
  begin
    if(reset_p)
      q <= 8'h00;
    else
      q <= mem[prefetch_addr];
  end

  always @(posedge clk or posedge reset_p)
  begin
    if(reset_p)
    begin
      wr_ptr            <= {ADDR_WIDTH{1'b0}};
      frame_start_ptr   <= {ADDR_WIDTH{1'b0}};
      rd_ptr            <= {ADDR_WIDTH{1'b0}};
      committed_count   <= {(ADDR_WIDTH+1){1'b0}};
      speculative_count <= {(ADDR_WIDTH+1){1'b0}};
    end
    else
    begin
      if(write_fire)
        wr_ptr <= wr_ptr + 1'b1;

      if(read_fire)
        rd_ptr <= rd_ptr + 1'b1;

      if(frame_done)
      begin
        speculative_count <= {(ADDR_WIDTH+1){1'b0}};
        if(frame_accept)
        begin
          frame_start_ptr <= wr_ptr;
          committed_count <= committed_count + speculative_count - read_fire;
        end
        else
        begin
          wr_ptr <= frame_start_ptr;
          committed_count <= committed_count - read_fire;
        end
      end
      else
      begin
        if(write_fire)
          speculative_count <= speculative_count + 1'b1;
        if(read_fire)
          committed_count <= committed_count - 1'b1;
      end
    end
  end
endmodule
