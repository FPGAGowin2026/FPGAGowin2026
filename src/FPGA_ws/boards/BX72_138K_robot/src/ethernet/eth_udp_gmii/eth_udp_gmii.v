// ==================== eth_udp_checksums.v ====================
/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
//
// Create Date   : 2019/05/01 00:00:00
// Module Name   : ip_checksum
// Description   : ip头部校验模块
//
// Dependencies  :
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
/////////////////////////////////////////////////////////////////////////////////

module ip_checksum(
  input           clk            ,
  input           reset_p        ,

  input           cal_en         ,

  input   [3:0]   IP_ver         ,
  input   [3:0]   IP_hdr_len     ,
  input   [7:0]   IP_tos         ,
  input   [15:0]  IP_total_len   ,
  input   [15:0]  IP_id          ,
  input           IP_rsv         ,
  input           IP_df          ,
  input           IP_mf          ,
  input   [12:0]  IP_frag_offset ,
  input   [7:0]   IP_ttl         ,
  input   [7:0]   IP_protocol    ,
  input   [31:0]  src_ip         ,
  input   [31:0]  dst_ip         ,

  output  [15:0]  checksum         ,
  output reg      cal_done_o
);

  reg  [16:0]sum01_q, sum23_q, sum45_q, sum67_q;
  reg  [15:0]dst_ip_low_q;
  reg  pair_valid_q, suma_valid_q, sumb_valid_q;
  reg  [19:0]suma;
  reg  [16:0]sumb;
  reg  [15:0]checksum_q;

  // 九个16位首部字按树形分组相加，避免长串行加法器。
  wire [15:0] word0 = {IP_ver,IP_hdr_len,IP_tos};
  wire [15:0] word3 = {IP_rsv,IP_df,IP_mf,IP_frag_offset};
  wire [15:0] word4 = {IP_ttl,IP_protocol};
  wire [16:0] sum01 = {1'b0,word0} + {1'b0,IP_total_len};
  wire [16:0] sum23 = {1'b0,IP_id} + {1'b0,word3};
  wire [16:0] sum45 = {1'b0,word4} + {1'b0,src_ip[31:16]};
  wire [16:0] sum67 = {1'b0,src_ip[15:0]} + {1'b0,dst_ip[31:16]};
  wire [17:0] sum0123 = {1'b0,sum01_q} + {1'b0,sum23_q};
  wire [17:0] sum4567 = {1'b0,sum45_q} + {1'b0,sum67_q};
  wire [18:0] sum01234567 = {1'b0,sum0123} + {1'b0,sum4567};
  wire [19:0] suma_next = {1'b0,sum01234567} + {4'd0,dst_ip_low_q};

  always@(posedge clk or posedge reset_p)
  if(reset_p) begin
    sum01_q<=17'd0;sum23_q<=17'd0;sum45_q<=17'd0;sum67_q<=17'd0;
    dst_ip_low_q<=16'd0;
    pair_valid_q<=1'b0;suma_valid_q<=1'b0;sumb_valid_q<=1'b0;cal_done_o<=1'b0;
    suma<=20'd0;sumb<=17'd0;checksum_q<=16'd0;
  end else begin
    // 先寄存四组首部字对和有效脉冲，把总长度输入与后续折叠树隔开一拍。
    pair_valid_q<=cal_en;
    if(cal_en) begin
      sum01_q<=sum01;sum23_q<=sum23;sum45_q<=sum45;sum67_q<=sum67;
      dst_ip_low_q<=dst_ip[15:0];
    end
    suma_valid_q<=pair_valid_q;
    if(pair_valid_q)suma<=suma_next;
    sumb_valid_q<=suma_valid_q;
    if(suma_valid_q)sumb<={1'b0,suma[19:16]}+{1'b0,suma[15:0]};
    cal_done_o<=sumb_valid_q;
    if(sumb_valid_q)checksum_q<=~(sumb[15:0]+{15'd0,sumb[16]});
  end

  assign checksum = checksum_q;

endmodule

////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 1999-2008 Easics NV.
// This source file may be used and distributed without restriction
// provided that this copyright statement is not removed from the file
// and that any derivative work contains the original copyright notice
// and the associated disclaimer.
//
// THIS SOURCE FILE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
// WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Purpose : synthesizable CRC function
//   * polynomial: x^32 + x^26 + x^23 + x^22 + x^16 + x^12 + x^11 + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x^1 + 1
//   * data width: 8
//
// Info : tools@easics.be
//        http://www.easics.com
////////////////////////////////////////////////////////////////////////////////

module crc32_d8
(
  input           clk         ,
  input           reset_p     ,

  input    [7:0]  data        ,
  input           crc_init    ,
  input           crc_en      ,
  output   [31:0] crc_result
);

wire   [7:0]   data_i;
reg    [31:0]  crc_result_o;

assign data_i = {data[ 0],data[ 1],data[ 2],data[ 3], data[ 4],data[ 5],data[ 6],data[ 7]};

assign crc_result = ~{crc_result_o[00],crc_result_o[01],crc_result_o[02],crc_result_o[03],crc_result_o[04],crc_result_o[05],crc_result_o[06],crc_result_o[07],
                      crc_result_o[08],crc_result_o[09],crc_result_o[10],crc_result_o[11],crc_result_o[12],crc_result_o[13],crc_result_o[14],crc_result_o[15],
                      crc_result_o[16],crc_result_o[17],crc_result_o[18],crc_result_o[19],crc_result_o[20],crc_result_o[21],crc_result_o[22],crc_result_o[23],
                      crc_result_o[24],crc_result_o[25],crc_result_o[26],crc_result_o[27],crc_result_o[28],crc_result_o[29],crc_result_o[30],crc_result_o[31]};

always @(posedge clk or posedge reset_p)
begin
  if(reset_p)
    crc_result_o <= 32'hffff_ffff;
  else if(crc_init)
    crc_result_o <= 32'hffff_ffff;
  else if(crc_en)
    crc_result_o <= nextCRC32_D8( data_i, crc_result_o);
  else
    crc_result_o <= crc_result_o;
end

  // polynomial: x^32 + x^26 + x^23 + x^22 + x^16 + x^12 + x^11 + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x^1 + 1
  // data width: 8
  // convention: the first serial bit is D[7]
  function [31:0] nextCRC32_D8;

    input [7:0] Data;
    input [31:0] crc;
    reg [7:0] d;
    reg [31:0] c;
    reg [31:0] newcrc;
    reg [7:0] mix;
  begin
    d = Data;
    c = crc;
    // 先共享 data 与高位 CRC 的混合项，再并行求多项式余数。
    mix[0] = d[0] ^ c[24]; mix[1] = d[1] ^ c[25];
    mix[2] = d[2] ^ c[26]; mix[3] = d[3] ^ c[27];
    mix[4] = d[4] ^ c[28]; mix[5] = d[5] ^ c[29];
    mix[6] = d[6] ^ c[30]; mix[7] = d[7] ^ c[31];
    newcrc[0] = mix[6] ^ mix[0];
    newcrc[1] = mix[7] ^ mix[6] ^ mix[1] ^ mix[0];
    newcrc[2] = mix[7] ^ mix[6] ^ mix[2] ^ mix[1] ^ mix[0];
    newcrc[3] = mix[7] ^ mix[3] ^ mix[2] ^ mix[1];
    newcrc[4] = mix[6] ^ mix[4] ^ mix[3] ^ mix[2] ^ mix[0];
    newcrc[5] = mix[7] ^ mix[6] ^ mix[5] ^ mix[4] ^ mix[3] ^ mix[1] ^ mix[0];
    newcrc[6] = mix[7] ^ mix[6] ^ mix[5] ^ mix[4] ^ mix[2] ^ mix[1];
    newcrc[7] = mix[7] ^ mix[5] ^ mix[3] ^ mix[2] ^ mix[0];
    newcrc[8] = c[0] ^ mix[4] ^ mix[3] ^ mix[1] ^ mix[0];
    newcrc[9] = c[1] ^ mix[5] ^ mix[4] ^ mix[2] ^ mix[1];
    newcrc[10] = c[2] ^ mix[5] ^ mix[3] ^ mix[2] ^ mix[0];
    newcrc[11] = c[3] ^ mix[4] ^ mix[3] ^ mix[1] ^ mix[0];
    newcrc[12] = c[4] ^ mix[6] ^ mix[5] ^ mix[4] ^ mix[2] ^ mix[1] ^ mix[0];
    newcrc[13] = c[5] ^ mix[7] ^ mix[6] ^ mix[5] ^ mix[3] ^ mix[2] ^ mix[1];
    newcrc[14] = c[6] ^ mix[7] ^ mix[6] ^ mix[4] ^ mix[3] ^ mix[2];
    newcrc[15] = c[7] ^ mix[7] ^ mix[5] ^ mix[4] ^ mix[3];
    newcrc[16] = c[8] ^ mix[5] ^ mix[4] ^ mix[0];
    newcrc[17] = c[9] ^ mix[6] ^ mix[5] ^ mix[1];
    newcrc[18] = c[10] ^ mix[7] ^ mix[6] ^ mix[2];
    newcrc[19] = c[11] ^ mix[7] ^ mix[3];
    newcrc[20] = c[12] ^ mix[4];
    newcrc[21] = c[13] ^ mix[5];
    newcrc[22] = c[14] ^ mix[0];
    newcrc[23] = c[15] ^ mix[6] ^ mix[1] ^ mix[0];
    newcrc[24] = c[16] ^ mix[7] ^ mix[2] ^ mix[1];
    newcrc[25] = c[17] ^ mix[3] ^ mix[2];
    newcrc[26] = c[18] ^ mix[6] ^ mix[4] ^ mix[3] ^ mix[0];
    newcrc[27] = c[19] ^ mix[7] ^ mix[5] ^ mix[4] ^ mix[1];
    newcrc[28] = c[20] ^ mix[6] ^ mix[5] ^ mix[2];
    newcrc[29] = c[21] ^ mix[7] ^ mix[6] ^ mix[3];
    newcrc[30] = c[22] ^ mix[7] ^ mix[4];
    newcrc[31] = c[23] ^ mix[5];
    nextCRC32_D8 = newcrc;
  end
  endfunction

endmodule

// ==================== eth_udp_tx_gmii.v ====================
/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: Max
// Web: www.corecourse.cn
//
// Create Date: 2020/07/20 00:00:00
// Design Name: eth_udp_tx_gmii
// Module Name: eth_udp_tx_gmii
// Project Name: eth_udp_tx_gmii
// Target Devices: XC7A35T-2FGG484I
// Tool Versions: Vivado 2018.3
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

module eth_udp_tx_gmii(
  clk125m,
  reset_p,

  tx_en_pulse,
  tx_done,

  dst_mac,
  src_mac,
  dst_ip,
  src_ip,
  dst_port,
  src_port,

  data_length,

  payload_req_o,
  payload_req_telemetry_o,
  payload_dat_i,

  gmii_tx_clk,
  gmii_txen,
  gmii_txd
);

  input          clk125m;
  input          reset_p;

  input          tx_en_pulse;
  output reg     tx_done;

  input [47:0]   dst_mac;
  input [47:0]   src_mac;
  input [31:0]   dst_ip;
  input [31:0]   src_ip;
  input [15:0]   dst_port;
  input [15:0]   src_port;

  input [15:0]   data_length;

  output         payload_req_o;
  output         payload_req_telemetry_o;
  input     [7:0]payload_dat_i;

  output         gmii_tx_clk;
//  (*IOB = "TRUE"*) output  reg[7:0] gmii_txd;
//  (*IOB = "TRUE"*) output  reg      gmii_txen;
  output  reg[7:0] gmii_txd;
  output  reg      gmii_txen;

  parameter ETH_type       = 16'h0800,
            IP_ver         = 4'h4,
            IP_hdr_len     = 4'h5,
            IP_tos         = 8'h00,
            IP_id          = 16'h0000,
            IP_rsv         = 1'b0,
            IP_df          = 1'b0,
            IP_mf          = 1'b0,
            IP_frag_offset = 13'h0000,
            IP_ttl         = 8'h40,
            IP_protocol    = 8'h11;

  localparam
    IDLE          = 8'b00000001,
    TX_PREAMBLE   = 8'b00000010,
    TX_ETH_HEADER = 8'b00000100,
    TX_IP_HEADER  = 8'b00001000,
    TX_UDP_HEADER = 8'b00010000,
    TX_DATA       = 8'b00100000,
    TX_FILL_DATA  = 8'b01000000,
    TX_CRC        = 8'b10000000;

  wire [15:0] IP_total_len;
  wire [15:0] IP_check_sum;
  wire [15:0] udp_length;
  wire [15:0] udp_check_sum = 16'h0000;

  reg  [47:0] dst_mac_reg;
  reg  [47:0] src_mac_reg;
  reg  [15:0] dst_port_reg;
  reg  [15:0] src_port_reg;
  reg  [31:0] dst_ip_reg;
  reg  [31:0] src_ip_reg;
  reg  [15:0] data_length_reg;
  reg  [15:0] IP_total_len_reg;
  reg  [15:0] udp_length_reg;

  reg  [3:0]  cnt_preamble;
  reg  [3:0]  cnt_eth_header;
  reg  [4:0]  cnt_ip_header;
  reg  [3:0]  cnt_udp_header;
  reg  [15:0] cnt_data;
  reg         cnt_data_upper_increment;
  reg  [4:0]  cnt_fill_data;
  reg  [1:0]  cnt_crc;
  reg  [7:0]  tx_data;
  reg         tx_en;
  reg  [7:0]  curr_state;
  reg  [7:0]  next_state;
  // 输出握手副本与 curr_state 同拍更新，保持请求相位且隔离跨模块负载。
  reg         payload_req_shadow /* synthesis syn_preserve = 1 */;

  wire        crc_state;
  reg         crc_state_dly1;
  reg         crc_state_dly2;
  reg  [1:0]  cnt_crc_dly1;
  reg  [1:0]  cnt_crc_dly2;
  reg         tx_en_dly1;
  reg  [7:0]  tx_data_dly1;
  reg         crc_init;
  reg         crc_en_temp;
  reg         crc_en;
  reg  [7:0]  crc_in;
  wire [31:0] crc_result;

  reg  [7:0]  gmii_txd_renewcrc;
  reg         gmii_txen_renewcrc;

  assign gmii_tx_clk = clk125m;

  assign udp_length = data_length + 8'd8;   //udp_header: 8byte
  assign IP_total_len = udp_length + 8'd20; //ip_header: 20byte

  ip_checksum ip_checksum(
    .clk            (clk125m        ),
    .reset_p        (reset_p        ),

    .cal_en         (tx_en_pulse    ),

    .IP_ver         (IP_ver         ),
    .IP_hdr_len     (IP_hdr_len     ),
    .IP_tos         (IP_tos         ),
    .IP_total_len   (IP_total_len   ),
    .IP_id          (IP_id          ),
    .IP_rsv         (IP_rsv         ),
    .IP_df          (IP_df          ),
    .IP_mf          (IP_mf          ),
    .IP_frag_offset (IP_frag_offset ),
    .IP_ttl         (IP_ttl         ),
    .IP_protocol    (IP_protocol    ),
    .src_ip         (src_ip         ),
    .dst_ip         (dst_ip         ),

    .checksum       (IP_check_sum   ),
    .cal_done_o     (               )
  );

  //����̫������Դ/Ŀ�Ĳ����Ĵ�
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    dst_mac_reg  <= 48'd0;
    src_mac_reg  <= 48'd0;
    dst_ip_reg   <= 32'd0;
    src_ip_reg   <= 32'd0;
    dst_port_reg <= 16'd0;
    src_port_reg <= 16'd0;
  end
  else if(tx_en_pulse)
  begin
    dst_mac_reg  <= dst_mac;
    src_mac_reg  <= src_mac;
    dst_port_reg <= dst_port;
    src_port_reg <= src_port;
    dst_ip_reg   <= dst_ip;
    src_ip_reg   <= src_ip;
  end

  //����̫���������ݲ��ֳ��Ȳ����Ĵ�
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    data_length_reg  <= 16'h0;
    IP_total_len_reg <= 16'h0;
    udp_length_reg   <= 16'h0;
  end
  else if(tx_en_pulse)
  begin
    data_length_reg  <= data_length;
    IP_total_len_reg <= IP_total_len;
    udp_length_reg   <= udp_length;
  end

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    curr_state <= IDLE;
  else
    curr_state <= next_state;

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    payload_req_shadow <= 1'b0;
  else
    payload_req_shadow <= (next_state == TX_DATA);

  always@(*)
  begin
    case(curr_state)
      IDLE:
        if(tx_en_pulse)
          next_state = TX_PREAMBLE;
        else
          next_state = IDLE;

      TX_PREAMBLE:
        if(cnt_preamble == 4'd7)
          next_state = TX_ETH_HEADER;
        else
          next_state = TX_PREAMBLE;

      TX_ETH_HEADER:
        if(cnt_eth_header == 4'd13)
          next_state = TX_IP_HEADER;
        else
          next_state = TX_ETH_HEADER;

      TX_IP_HEADER:
        if(cnt_ip_header == 5'd19)
          next_state = TX_UDP_HEADER;
        else
          next_state = TX_IP_HEADER;

      TX_UDP_HEADER:
        if(cnt_udp_header == 4'd7)
          next_state = TX_DATA;
        else
          next_state = TX_UDP_HEADER;

      TX_DATA:
        if(data_length_reg <5'd18 && cnt_data == data_length_reg - 1'b1)
          next_state = TX_FILL_DATA;
        else if(cnt_data == data_length_reg - 1'b1)
          next_state = TX_CRC;
        else
          next_state = TX_DATA;

      TX_FILL_DATA:
        if(cnt_fill_data == 5'd17 - data_length_reg)
          next_state = TX_CRC;
        else
          next_state = TX_FILL_DATA;

      TX_CRC:
        if(cnt_crc == 2'd3)
          next_state = IDLE;
        else
          next_state = TX_CRC;

      default:next_state = IDLE;

    endcase
  end

  //cnt_preamble
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    cnt_preamble <= 4'd0;
  else if(curr_state == TX_PREAMBLE)
    cnt_preamble <= cnt_preamble + 1'b1;
  else
    cnt_preamble <= 4'd0;

  //cnt_eth_header
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    cnt_eth_header <= 4'd0;
  else if(curr_state == TX_ETH_HEADER)
    cnt_eth_header <= cnt_eth_header + 1'b1;
  else
    cnt_eth_header <= 4'd0;

  //cnt_ip_header
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    cnt_ip_header <= 5'd0;
  else if(curr_state == TX_IP_HEADER)
    cnt_ip_header <= cnt_ip_header + 1'b1;
  else
    cnt_ip_header <= 5'd0;

  //cnt_udp_header
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    cnt_udp_header <= 4'd0;
  else if(curr_state == TX_UDP_HEADER)
    cnt_udp_header <= cnt_udp_header + 1'b1;
  else
    cnt_udp_header <= 4'd0;

  //cnt_data
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    cnt_data <= 16'd0;
    cnt_data_upper_increment <= 1'b0;
  end
  else if(curr_state == TX_DATA)
  begin
    // 上半字节进位预先一拍生成，保持16位计数及原有逐字节相位。
    cnt_data[7:0] <= cnt_data[7:0] + 1'b1;
    cnt_data[15:8] <= cnt_data[15:8] + cnt_data_upper_increment;
    cnt_data_upper_increment <= cnt_data[7:0] == 8'hfe;
  end
  else
  begin
    cnt_data <= 16'd0;
    cnt_data_upper_increment <= 1'b0;
  end

  //cnt_fill_data
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    cnt_fill_data <= 5'd0;
  else if(curr_state == TX_FILL_DATA)
    cnt_fill_data <= cnt_fill_data + 1'b1;
  else
    cnt_fill_data <= 5'd0;

  //cnt_crc
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    cnt_crc <= 5'd0;
  else if(curr_state == TX_CRC)
    cnt_crc <= cnt_crc + 1'b1;
  else
    cnt_crc <= 5'd0;

  //tx_data,tx_en
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    tx_en   <= 1'b0;
    tx_data <= 8'h00;
  end
  else
  begin
    case(curr_state)
      IDLE:
      begin
        tx_en   <= 1'b0;
        tx_data <= 8'h00;
      end

      TX_PREAMBLE:
      begin
        tx_en <= 1'b1;
        if(cnt_preamble <= 4'd6)
          tx_data <= 8'h55;
        else
          tx_data <= 8'hd5;
      end

      TX_ETH_HEADER:
      begin
        tx_en <= 1'b1;
        case(cnt_eth_header)
          4'd0:   tx_data <= dst_mac_reg[47:40];
          4'd1:   tx_data <= dst_mac_reg[39:32];
          4'd2:   tx_data <= dst_mac_reg[31:24];
          4'd3:   tx_data <= dst_mac_reg[23:16];
          4'd4:   tx_data <= dst_mac_reg[15:8];
          4'd5:   tx_data <= dst_mac_reg[7:0];
          4'd6:   tx_data <= src_mac_reg[47:40];
          4'd7:   tx_data <= src_mac_reg[39:32];
          4'd8:   tx_data <= src_mac_reg[31:24];
          4'd9:   tx_data <= src_mac_reg[23:16];
          4'd10:  tx_data <= src_mac_reg[15:8];
          4'd11:  tx_data <= src_mac_reg[7:0];
          4'd12:  tx_data <= ETH_type[15:8];
          4'd13:  tx_data <= ETH_type[7:0];
          default:tx_data <= 8'h00;
        endcase
      end

      TX_IP_HEADER:
      begin
        tx_en <= 1'b1;
        case(cnt_ip_header)
          5'd0:   tx_data <= {IP_ver,IP_hdr_len};
          5'd1:   tx_data <= IP_tos;
          5'd2:   tx_data <= IP_total_len_reg[15:8];
          5'd3:   tx_data <= IP_total_len_reg[7:0];
          5'd4:   tx_data <= IP_id[15:8];
          5'd5:   tx_data <= IP_id[7:0];
          5'd6:   tx_data <= {IP_rsv,IP_df,IP_mf,IP_frag_offset[12:8]};
          5'd7:   tx_data <= IP_frag_offset[7:0];
          5'd8:   tx_data <= IP_ttl;
          5'd9:   tx_data <= IP_protocol;
          5'd10:  tx_data <= IP_check_sum[15:8];
          5'd11:  tx_data <= IP_check_sum[7:0];
          5'd12:  tx_data <= src_ip_reg[31:24];
          5'd13:  tx_data <= src_ip_reg[23:16];
          5'd14:  tx_data <= src_ip_reg[15:8];
          5'd15:  tx_data <= src_ip_reg[7:0];
          5'd16:  tx_data <= dst_ip_reg[31:24];
          5'd17:  tx_data <= dst_ip_reg[23:16];
          5'd18:  tx_data <= dst_ip_reg[15:8];
          5'd19:  tx_data <= dst_ip_reg[7:0];
          default:tx_data <= 8'h00;
        endcase
      end

      TX_UDP_HEADER:
      begin
        tx_en <= 1'b1;
        case(cnt_udp_header)
          4'd0:   tx_data <= src_port_reg[15:8];
          4'd1:   tx_data <= src_port_reg[7:0];
          4'd2:   tx_data <= dst_port_reg[15:8];
          4'd3:   tx_data <= dst_port_reg[7:0];
          4'd4:   tx_data <= udp_length_reg[15:8];
          4'd5:   tx_data <= udp_length_reg[7:0];
          4'd6:   tx_data <= udp_check_sum[15:8];
          4'd7:   tx_data <= udp_check_sum[7:0];
          default:tx_data <= 8'h00;
        endcase
      end

      TX_DATA:
      begin
        tx_en <= 1'b1;
        tx_data <= payload_dat_i;
      end

      TX_FILL_DATA:
      begin
        tx_en <= 1'b1;
        tx_data <= 8'h00;
      end

      TX_CRC:
      begin
        tx_en <= 1'b1;
        tx_data <= 8'h00;
      end

      default:
      begin
        tx_en   <= 1'b0;
        tx_data <= 8'h00;
      end
    endcase
  end

  //payload_req_o
  assign payload_req_o = payload_req_shadow;
  // 状态寄存器与握手副本同拍更新，遥测读取复用该状态以分担扇出。
  assign payload_req_telemetry_o = (curr_state == TX_DATA);

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    crc_init <= 1'b0;
  else if (tx_en_pulse && (curr_state == IDLE))
    crc_init <= 1'b1;
  else
    crc_init <= 1'b0;

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    crc_en_temp <= 1'b0;
  else if (curr_state == TX_ETH_HEADER || curr_state == TX_IP_HEADER ||
           curr_state == TX_UDP_HEADER || curr_state == TX_FILL_DATA || curr_state == TX_DATA)
    crc_en_temp <= 1'b1;
  else
    crc_en_temp <= 1'b0;

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    crc_en <= 1'b0;
  else if (crc_en_temp)
    crc_en <= 1'b1;
  else
    crc_en <= 1'b0;

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    crc_in <= 8'h00;
  else if(crc_en_temp)
    crc_in <= tx_data;
  else
    crc_in <= crc_in;

  crc32_d8 crc32_d8
  (
    .clk         (clk125m    ),
    .reset_p     (reset_p    ),

    .data        (crc_in     ),
    .crc_init    (crc_init   ),
    .crc_en      (crc_en     ),
    .crc_result  (crc_result )//latency=1
  );

  assign crc_state = curr_state == TX_CRC;

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    tx_en_dly1   <= 1'b0;
    tx_data_dly1 <= 8'h00;
  end
  else
  begin
    tx_en_dly1   <= tx_en;
    tx_data_dly1 <= tx_data;
  end

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    crc_state_dly1 <= 1'b0;
    crc_state_dly2 <= 1'b0;
    cnt_crc_dly1   <= 2'd0;
    cnt_crc_dly2   <= 2'd0;
  end
  else
  begin
    crc_state_dly1 <= crc_state;
    crc_state_dly2 <= crc_state_dly1;
    cnt_crc_dly1   <= cnt_crc;
    cnt_crc_dly2   <= cnt_crc_dly1;
  end

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    gmii_txd_renewcrc <= 8'h00;
  else if(crc_state_dly2)
  begin
    case(cnt_crc_dly2)
      2'd0:gmii_txd_renewcrc <= crc_result[7:0];
      2'd1:gmii_txd_renewcrc <= crc_result[15:8];
      2'd2:gmii_txd_renewcrc <= crc_result[23:16];
      2'd3:gmii_txd_renewcrc <= crc_result[31:24];
    endcase
  end
  else
    gmii_txd_renewcrc <= tx_data_dly1;

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    gmii_txen_renewcrc <= 1'b0;
  else if(tx_en_dly1)
    gmii_txen_renewcrc <= 1'b1;
  else
    gmii_txen_renewcrc <= 1'b0;

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    gmii_txen <= 1'b0;
    gmii_txd  <= 8'h00;
  end
  else
  begin
    gmii_txen <= gmii_txen_renewcrc;
    gmii_txd  <= gmii_txd_renewcrc;
  end

  //tx_done
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    tx_done <= 1'b0;
  else if(curr_state == TX_CRC && cnt_crc == 2'd3)
    tx_done <= 1'b1;
  else
    tx_done <= 1'b0;

endmodule

// ==================== eth_udp_rx_gmii.v ====================
/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
//
// Create Date   : 2019/05/01 00:00:00
// Module Name   : eth_udp_rx_gmii
// Description   : 以太网接收模块，gmii接口
//
// Dependencies  :
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
/////////////////////////////////////////////////////////////////////////////////

module eth_udp_rx_gmii(
  reset_p,

  local_mac,
  local_ip,
  local_port,

  clk125m_o,
  exter_mac,
  exter_ip,
  exter_port,
  rx_data_length,
  data_overflow_i,
  payload_valid_o,
  payload_dat_o,

  one_pkt_done,
  pkt_error,
  debug_crc_check,

  gmii_rx_clk,
  gmii_rxdv,
  gmii_rxd,
  rx_reset_stage0_i,
  rx_async_reset_p_i
);
  input              reset_p;

  input     [47:0]   local_mac;
  input     [31:0]   local_ip;
  input     [15:0]   local_port;

  output             clk125m_o;
  output reg[47:0]   exter_mac;
  output reg[31:0]   exter_ip;
  output reg[15:0]   exter_port;
  output reg[15:0]   rx_data_length;
  input              data_overflow_i;
  output reg         payload_valid_o;
  output reg[7:0]    payload_dat_o;

  output reg         one_pkt_done;
  output reg         pkt_error;
  output    [31:0]   debug_crc_check;

  input        gmii_rx_clk;
  (*IOB = "TRUE"*) input  [7:0] gmii_rxd;
  (*IOB = "TRUE"*) input        gmii_rxdv;
  input              rx_reset_stage0_i;
  input              rx_async_reset_p_i;

  parameter ETH_type       = 16'h0800,
            IP_ver         = 4'h4,
            IP_hdr_len     = 4'h5,
            IP_protocol    = 8'h11;
  parameter ACCEPT_BROADCAST_MAC = 1'b1;
  // 默认共享原 reset，旧实例不连接新增端口时行为保持不变。
  parameter SPLIT_IP_CHECKSUM_RESET = 1'b0;

  localparam
    IDLE              = 12'b000000000001,
    RX_PREAMBLE       = 12'b000000000010,
    RX_ETH_HEADER     = 12'b000000000100,
    RX_IP_HEADER      = 12'b000000001000,
    RX_UDP_HEADER     = 12'b000000010000,
    RX_DATA           = 12'b000000100000,
    RX_DRP_DATA       = 12'b000001000000,
    RX_CRC            = 12'b000010000000,
    UDP_CHECKSUM_SUM  = 12'b000100000000,
    UDP_CHECKSUM_FOLD = 12'b001000000000,
    UDP_CHECKSUM_TEST = 12'b010000000000,
    PKT_CHECK         = 12'b100000000000;

  wire        clk125m;
  reg  [7:0]  reg_gmii_rxd;
  reg         reg_gmii_rxdv;
  reg  [7:0]  rx_data_dly1;
  reg  [7:0]  rx_data_dly2;
  reg         rx_datav_dly1;
  reg         rx_datav_dly2;
  reg  [47:0] local_mac_reg;
  reg  [31:0] local_ip_reg;
  reg  [15:0] local_port_reg;
  reg  [11:0] curr_state;
  reg  [11:0] next_state;

  reg         reg_data_overflow;

  reg  [47:0] rx_dst_mac;
  reg  [47:0] rx_src_mac;
  reg  [15:0] rx_eth_type;
  reg         eth_header_check_ok;

  reg  [3:0]  rx_ip_ver;
  reg  [3:0]  rx_ip_hdr_len;
  reg  [7:0]  rx_ip_tos;
  reg  [15:0] rx_total_len;
  reg  [15:0] rx_ip_id;
  reg         rx_ip_rsv;
  reg         rx_ip_df;
  reg         rx_ip_mf;
  reg  [12:0] rx_ip_frag_offset;
  reg  [7:0]  rx_ip_ttl;
  reg  [7:0]  rx_ip_protocol;
  reg  [15:0] rx_ip_check_sum;
  reg  [31:0] rx_src_ip;
  reg  [31:0] rx_dst_ip;
  reg         ip_checksum_cal_en;
  wire [15:0] cal_check_sum;
  wire        ip_checksum_done;
  reg         ip_header_check_ok;
  wire        ip_checksum_reset_p;

  generate
    if (SPLIT_IP_CHECKSUM_RESET) begin : g_ip_checksum_reset_split
      // D 取 RX reset synchronizer 的 stage0，和主 stage1 同源同拍释放。
      // 保留副本使综合器不把局部 reset 合并回高扇出的主 stage1。
      (* ASYNC_REG="TRUE", syn_keep=1, syn_preserve=1 *) reg ip_checksum_reset_stage1_q;
      always @(posedge clk125m or posedge rx_async_reset_p_i)
        if (rx_async_reset_p_i) ip_checksum_reset_stage1_q <= 1'b1;
        else ip_checksum_reset_stage1_q <= rx_reset_stage0_i;
      assign ip_checksum_reset_p = ip_checksum_reset_stage1_q;
    end else begin : g_ip_checksum_reset_shared
      assign ip_checksum_reset_p = reset_p;
    end
  endgenerate

  reg  [15:0] rx_src_port;
  reg  [15:0] rx_dst_port;
  reg  [15:0] rx_udp_length;
  reg  [15:0] rx_udp_checksum;
  reg         udp_header_check_ok;
  reg  [16:0] udp_checksum_sum;
  reg  [7:0]  udp_checksum_high_byte;
  reg         udp_checksum_odd_byte;
  reg  [16:0] udp_checksum_total_q;
  reg         udp_checksum_ok_q;
  reg         frame_truncated;

  reg         crc_init;
  reg         crc_en;
  reg  [7:0]  crc_data;
  wire [31:0] crc_check;

  reg  [3:0]  cnt_preamble;
  reg  [3:0]  cnt_eth_header;
  reg  [4:0]  cnt_ip_header;
  reg  [3:0]  cnt_udp_header;
  reg  [15:0] cnt_data;
  reg         cnt_data_upper_increment;
  reg  [4:0]  cnt_drp_data;
  // Payload boundary arithmetic is prepared while the IPv4 header is still
  // arriving, so the RX state transition only compares registered values.
  reg  [15:0] rx_payload_length_stage;
  reg  [15:0] rx_data_last_count;
  reg  [4:0]  rx_drop_last_count;
  reg         rx_payload_is_short;

  assign clk125m_o = clk125m;
  assign debug_crc_check = crc_check;

//  BUFG BUFG_pclk (
//    .O(clk125m        ), // 1-bit output: Clock output
//    .I(gmii_rx_clk    )  // 1-bit input: Clock input
//  );

  assign clk125m = gmii_rx_clk;

  //将本地MAC、IP、PORT寄存
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    local_mac_reg  <= 48'h00_00_00_00_00_00;
    local_ip_reg   <= 32'h00_00_00_00;
    local_port_reg <= 16'h00_00;
  end
  else if(curr_state == IDLE)
  begin
    local_mac_reg  <= local_mac;
    local_ip_reg   <= local_ip;
    local_port_reg <= local_port;
  end

  //将以太网输入的接收信号寄存
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    reg_gmii_rxd  <= 8'h00;
    reg_gmii_rxdv <= 1'b0;
  end
  else
  begin
    reg_gmii_rxd  <= gmii_rxd;
    reg_gmii_rxdv <= gmii_rxdv;
  end

  //将以太网输入的接收信号寄存后打拍
  always@(posedge clk125m)
  begin
    rx_data_dly1  <= reg_gmii_rxd;
    rx_data_dly2  <= rx_data_dly1;
    rx_datav_dly1 <= reg_gmii_rxdv;
    rx_datav_dly2 <= rx_datav_dly1;
  end

  //cnt_preamble
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    cnt_preamble <= 4'd0;
  else if(curr_state == RX_PREAMBLE && rx_data_dly2 == 8'h55)
    cnt_preamble <= cnt_preamble + 1'b1;
  else
    cnt_preamble <= 4'd0;

  //cnt_eth_header
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    cnt_eth_header <= 4'd0;
  else if(curr_state == RX_ETH_HEADER)
    cnt_eth_header <= cnt_eth_header + 1'b1;
  else
    cnt_eth_header <= 4'd0;

  // Destination MAC is only inspected after all six bytes have arrived, so
  // it does not need the high-fanout asynchronous parser reset.
  always@(posedge clk125m)
  if(curr_state == RX_ETH_HEADER)
  begin
    case(cnt_eth_header)
      4'd0 :rx_dst_mac[47:40] <= rx_data_dly2;
      4'd1 :rx_dst_mac[39:32] <= rx_data_dly2;
      4'd2 :rx_dst_mac[31:24] <= rx_data_dly2;
      4'd3 :rx_dst_mac[23:16] <= rx_data_dly2;
      4'd4 :rx_dst_mac[15:8]  <= rx_data_dly2;
      4'd5 :rx_dst_mac[7:0]   <= rx_data_dly2;
      default: ;
    endcase
  end

  always@(posedge clk125m or posedge reset_p)
  if(reset_p) begin
    rx_src_mac  <= 48'h00_00_00_00_00_00;
    rx_eth_type <= 16'h0000;
  end else if(curr_state == RX_ETH_HEADER)
  begin
    case(cnt_eth_header)

      4'd6 :rx_src_mac[47:40] <= rx_data_dly2;
      4'd7 :rx_src_mac[39:32] <= rx_data_dly2;
      4'd8 :rx_src_mac[31:24] <= rx_data_dly2;
      4'd9 :rx_src_mac[23:16] <= rx_data_dly2;
      4'd10:rx_src_mac[15:8]  <= rx_data_dly2;
      4'd11:rx_src_mac[7:0]   <= rx_data_dly2;

      4'd12:rx_eth_type[15:8] <= rx_data_dly2;
      4'd13:rx_eth_type[7:0]  <= rx_data_dly2;
      default: ;
    endcase
  end

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    eth_header_check_ok <= 1'b0;
  else if(rx_eth_type == ETH_type &&
          (rx_dst_mac == local_mac_reg ||
           (ACCEPT_BROADCAST_MAC && rx_dst_mac == 48'hFF_FF_FF_FF_FF_FF)))
    eth_header_check_ok <= 1'b1;
  else
    eth_header_check_ok <= 1'b0;

  //cnt_ip_header
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    cnt_ip_header <= 5'd0;
  else if(curr_state == RX_IP_HEADER)
    cnt_ip_header <= cnt_ip_header + 1'b1;
  else
    cnt_ip_header <= 5'd0;

  //ip_header
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    {rx_ip_ver,rx_ip_hdr_len}     <= 8'h0;
    rx_ip_tos                     <= 8'h0;
    rx_total_len                  <= 16'h0;
    rx_ip_id                      <= 16'h0;
    {rx_ip_rsv,rx_ip_df,rx_ip_mf} <= 3'h0;
    rx_ip_frag_offset             <= 13'h0;
    rx_ip_ttl                     <= 8'h0;
    rx_ip_protocol                <= 8'h0;
    rx_ip_check_sum               <= 16'h0;
    rx_src_ip                     <= 32'h0;
    rx_dst_ip                     <= 32'h0;
  end
  else if(curr_state == RX_IP_HEADER)
  begin
    case(cnt_ip_header)
      5'd0:   {rx_ip_ver,rx_ip_hdr_len}                             <= rx_data_dly2;
      5'd1:   rx_ip_tos                                             <= rx_data_dly2;
      5'd2:   rx_total_len[15:8]                                    <= rx_data_dly2;
      5'd3:   rx_total_len[7:0]                                     <= rx_data_dly2;
      5'd4:   rx_ip_id[15:8]                                        <= rx_data_dly2;
      5'd5:   rx_ip_id[7:0]                                         <= rx_data_dly2;
      5'd6:   {rx_ip_rsv,rx_ip_df,rx_ip_mf,rx_ip_frag_offset[12:8]} <= rx_data_dly2;
      5'd7:   rx_ip_frag_offset[7:0]                                <= rx_data_dly2;
      5'd8:   rx_ip_ttl                                             <= rx_data_dly2;
      5'd9:   rx_ip_protocol                                        <= rx_data_dly2;
      5'd10:  rx_ip_check_sum[15:8]                                 <= rx_data_dly2;
      5'd11:  rx_ip_check_sum[7:0]                                  <= rx_data_dly2;
      5'd12:  rx_src_ip[31:24]                                      <= rx_data_dly2;
      5'd13:  rx_src_ip[23:16]                                      <= rx_data_dly2;
      5'd14:  rx_src_ip[15:8]                                       <= rx_data_dly2;
      5'd15:  rx_src_ip[7:0]                                        <= rx_data_dly2;
      5'd16:  rx_dst_ip[31:24]                                      <= rx_data_dly2;
      5'd17:  rx_dst_ip[23:16]                                      <= rx_data_dly2;
      5'd18:  rx_dst_ip[15:8]                                       <= rx_data_dly2;
      5'd19:  rx_dst_ip[7:0]                                        <= rx_data_dly2;
      default: ;
    endcase
  end

  //udp_header: 8byte
  //ip_header: 20byte
  //rx_data_length = rx_total_len - udp_header - ip_header;
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    rx_data_length <= 16'd0;
    rx_payload_length_stage <= 16'd0;
    rx_data_last_count <= 16'hffff;
    rx_drop_last_count <= 5'd0;
    rx_payload_is_short <= 1'b0;
  end
  else begin
    // Count 3 is the final byte of IPv4 total_length.  Use the registered
    // high byte and incoming low byte, then derive terminal counters one
    // clock later while the rest of the IP header is being parsed.
    if(curr_state == RX_IP_HEADER && cnt_ip_header == 5'd3)
      rx_payload_length_stage <= {rx_total_len[15:8],rx_data_dly2} - 16'd28;
    if(curr_state == RX_IP_HEADER && cnt_ip_header == 5'd4) begin
      rx_data_last_count <= rx_payload_length_stage - 16'd1;
      rx_drop_last_count <= 5'd17 - rx_payload_length_stage[4:0];
      rx_payload_is_short <= rx_payload_length_stage < 16'd18;
    end
    if(curr_state == RX_IP_HEADER && cnt_ip_header == 5'd19)
      rx_data_length <= rx_total_len - 8'd20 - 8'd8;
  end

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    ip_checksum_cal_en <= 1'b0;
  else if(curr_state == RX_IP_HEADER && cnt_ip_header == 5'd19)
    ip_checksum_cal_en <= 1'b1;
  else
    ip_checksum_cal_en <= 1'b0;

  ip_checksum ip_checksum(
    .clk            (clk125m           ),
    .reset_p        (ip_checksum_reset_p),

    .cal_en         (ip_checksum_cal_en),

    .IP_ver         (rx_ip_ver         ),
    .IP_hdr_len     (rx_ip_hdr_len     ),
    .IP_tos         (rx_ip_tos         ),
    .IP_total_len   (rx_total_len      ),
    .IP_id          (rx_ip_id          ),
    .IP_rsv         (rx_ip_rsv         ),
    .IP_df          (rx_ip_df          ),
    .IP_mf          (rx_ip_mf          ),
    .IP_frag_offset (rx_ip_frag_offset ),
    .IP_ttl         (rx_ip_ttl         ),
    .IP_protocol    (rx_ip_protocol    ),
    .src_ip         (rx_src_ip         ),
    .dst_ip         (rx_dst_ip         ),

    .checksum       (cal_check_sum     ),
    .cal_done_o     (ip_checksum_done  )
  );

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    ip_header_check_ok <= 1'b0;
  else if(ip_checksum_done &&
          {IP_ver,IP_hdr_len,IP_protocol,cal_check_sum,local_ip_reg} ==
          {rx_ip_ver,rx_ip_hdr_len,rx_ip_protocol,rx_ip_check_sum,rx_dst_ip} &&
          !rx_ip_rsv && !rx_ip_mf && rx_ip_frag_offset == 13'd0 &&
          rx_total_len >= 16'd28)
    ip_header_check_ok <= 1'b1;
  else
    ip_header_check_ok <= 1'b0;

  //cnt_udp_header
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    cnt_udp_header <= 4'd0;
  else if(curr_state == RX_UDP_HEADER)
    cnt_udp_header <= cnt_udp_header + 1'b1;
  else
    cnt_udp_header <= 4'd0;

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    rx_src_port  <= 16'h0;
    rx_dst_port  <= 16'h0;
    rx_udp_length<= 16'h0;
    rx_udp_checksum <= 16'h0;
  end
  else if(curr_state == RX_UDP_HEADER)
  begin
    case(cnt_udp_header)
      4'd0: rx_src_port[15:8]   <= rx_data_dly2;
      4'd1: rx_src_port[7:0]    <= rx_data_dly2;
      4'd2: rx_dst_port[15:8]   <= rx_data_dly2;
      4'd3: rx_dst_port[7:0]    <= rx_data_dly2;
      4'd4: rx_udp_length[15:8] <= rx_data_dly2;
      4'd5: rx_udp_length[7:0]  <= rx_data_dly2;
      4'd6: rx_udp_checksum[15:8] <= rx_data_dly2;
      4'd7: rx_udp_checksum[7:0]  <= rx_data_dly2;
      default: ;
    endcase
  end

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    udp_header_check_ok <= 1'b0;
  else if(rx_dst_port == local_port_reg && rx_udp_length >= 16'd8 &&
          rx_udp_length == (rx_total_len - 16'd20))
    udp_header_check_ok <= 1'b1;
  else
    udp_header_check_ok <= 1'b0;

  //cnt_data
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    cnt_data <= 16'd0;
    cnt_data_upper_increment <= 1'b0;
  end
  else if(curr_state == RX_DATA)
  begin
    cnt_data[7:0] <= cnt_data[7:0] + 1'b1;
    cnt_data[15:8] <= cnt_data[15:8] + cnt_data_upper_increment;
    cnt_data_upper_increment <= cnt_data[7:0] == 8'hfe;
  end
  else
  begin
    cnt_data <= 16'd0;
    cnt_data_upper_increment <= 1'b0;
  end

  //cnt_drp_data
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    cnt_drp_data <= 5'd0;
  else if(curr_state == RX_DRP_DATA)
    cnt_drp_data <= cnt_drp_data + 1'b1;
  else
    cnt_drp_data <= 5'd0;

  // IPv4 允许 UDP checksum 为零；非零值必须覆盖伪首部、UDP 首部和负载。
  // 伪首部随 IP 字节分拍累加，避免目的 IP 一次穿过多级加法器。
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    udp_checksum_sum       <= 17'd0;
    udp_checksum_high_byte <= 8'd0;
    udp_checksum_odd_byte  <= 1'b0;
  end
  else if(curr_state == RX_IP_HEADER && cnt_ip_header == 5'd15)
  begin
    udp_checksum_sum <= {1'b0, rx_src_ip[31:16]} +
                        {1'b0, rx_src_ip[15:8], rx_data_dly2};
    udp_checksum_high_byte <= 8'd0;
    udp_checksum_odd_byte  <= 1'b0;
  end
  else if(curr_state == RX_IP_HEADER && cnt_ip_header == 5'd17)
    udp_checksum_sum <= {1'b0, udp_checksum_sum[15:0]} +
                        {1'b0, rx_dst_ip[31:24], rx_data_dly2} +
                        udp_checksum_sum[16];
  else if(curr_state == RX_IP_HEADER && cnt_ip_header == 5'd19)
    udp_checksum_sum <= {1'b0, udp_checksum_sum[15:0]} +
                        {1'b0, rx_dst_ip[15:8], rx_data_dly2} +
                        udp_checksum_sum[16];
  else if(curr_state == RX_UDP_HEADER && cnt_udp_header == 4'd0)
  begin
    udp_checksum_sum <= {1'b0, udp_checksum_sum[15:0]} +
                        {9'd0, IP_protocol} + udp_checksum_sum[16];
    udp_checksum_high_byte <= rx_data_dly2;
    udp_checksum_odd_byte  <= 1'b1;
  end
  else if(curr_state == RX_UDP_HEADER || curr_state == RX_DATA)
  begin
    if(udp_checksum_odd_byte)
    begin
      udp_checksum_sum <= {1'b0, udp_checksum_sum[15:0]} +
                          {1'b0, udp_checksum_high_byte, rx_data_dly2} +
                          udp_checksum_sum[16];
      udp_checksum_odd_byte <= 1'b0;
    end
    else
    begin
      udp_checksum_high_byte <= rx_data_dly2;
      udp_checksum_odd_byte  <= 1'b1;
    end
  end

  // 帧尾加入长度。奇数字节只占高8位；校验和为全1时，
  // 低8位加进位必须等于ff且不产生进位，高8位与奇数字节互补。
  // 因此可直接比较两个字节，无需再串联16位进位链。
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    udp_checksum_total_q <= 17'd0;
    udp_checksum_ok_q    <= 1'b0;
  end
  else
  begin
    if(curr_state == UDP_CHECKSUM_SUM)
      udp_checksum_total_q <= {1'b0, udp_checksum_sum[15:0]} +
        {1'b0, rx_udp_length} + udp_checksum_sum[16];
    if(curr_state == UDP_CHECKSUM_FOLD)
      udp_checksum_ok_q <= (rx_udp_checksum == 16'h0000) ||
        ((udp_checksum_total_q[7:0] == (udp_checksum_total_q[16] ? 8'hfe : 8'hff)) &&
         (udp_checksum_total_q[15:8] ==
          (udp_checksum_odd_byte ? ~udp_checksum_high_byte : 8'hff)));
  end

  //FSM
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    curr_state <= IDLE;
  else
    curr_state <= next_state;

  always@(*)
  begin
    case(curr_state)
      IDLE:
        if(!rx_datav_dly2 && rx_datav_dly1)
          next_state = RX_PREAMBLE;
        else
          next_state = IDLE;

      RX_PREAMBLE:
        if(rx_data_dly2 == 8'hd5 && cnt_preamble > 4'd5)
          next_state = RX_ETH_HEADER;
        else if(cnt_preamble > 4'd7)
          next_state = IDLE;
        else
          next_state = RX_PREAMBLE;

      RX_ETH_HEADER:
        if(cnt_eth_header == 4'd13)
          next_state = RX_IP_HEADER;
        else
          next_state = RX_ETH_HEADER;

      RX_IP_HEADER:
        if(cnt_ip_header == 5'd2 && eth_header_check_ok == 1'b0)
          next_state = IDLE;
        else if(cnt_ip_header == 5'd19)
          next_state = RX_UDP_HEADER;
        else
          next_state = RX_IP_HEADER;

      RX_UDP_HEADER:
        // ip_checksum 的部分和/折叠均分拍，到第 6 个 UDP 首部字节前裁决。
        if(cnt_udp_header == 4'd5 && ip_header_check_ok == 1'b0)
          next_state = IDLE;
        else if(cnt_udp_header == 4'd7 && udp_header_check_ok == 1'b0)
          next_state = IDLE;
        else if(cnt_udp_header == 4'd7)
          next_state = RX_DATA;
        else
          next_state = RX_UDP_HEADER;

      RX_DATA:
        if(rx_payload_is_short && (cnt_data == rx_data_last_count))
          next_state = RX_DRP_DATA;
        else if(cnt_data == rx_data_last_count)
          next_state = RX_CRC;
        else
          next_state = RX_DATA;

      RX_DRP_DATA:
        if(cnt_drp_data == rx_drop_last_count)
          next_state = RX_CRC;
        else
          next_state = RX_DRP_DATA;

      RX_CRC:
        if(rx_datav_dly2 == 1'b0)
          next_state = UDP_CHECKSUM_SUM;
        else
          next_state = RX_CRC;

      UDP_CHECKSUM_SUM:
        next_state = UDP_CHECKSUM_FOLD;

      UDP_CHECKSUM_FOLD:
        next_state = UDP_CHECKSUM_TEST;

      UDP_CHECKSUM_TEST:
        next_state = PKT_CHECK;

      PKT_CHECK:
        next_state = IDLE;

      default:next_state = IDLE;

    endcase
    // 帧中途撤销 RX_DV 必须结束本帧；不能按伪造长度继续吞掉下一帧。
    if(!rx_datav_dly2 && (curr_state==RX_PREAMBLE || curr_state==RX_ETH_HEADER ||
       curr_state==RX_IP_HEADER || curr_state==RX_UDP_HEADER ||
       curr_state==RX_DATA || curr_state==RX_DRP_DATA))
      next_state=PKT_CHECK;
  end

  always @(posedge clk125m or posedge reset_p)
    if(reset_p)frame_truncated<=1'b0;
    else if(curr_state==IDLE)frame_truncated<=1'b0;
    else if(!rx_datav_dly2 && (curr_state==RX_PREAMBLE || curr_state==RX_ETH_HEADER ||
       curr_state==RX_IP_HEADER || curr_state==RX_UDP_HEADER ||
       curr_state==RX_DATA || curr_state==RX_DRP_DATA))frame_truncated<=1'b1;

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    crc_init <= 1'b0;
  else if (rx_datav_dly1 && (~rx_datav_dly2))
    crc_init <= 1'b1;
  else
    crc_init <= 1'b0;

  //crc_en
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    crc_en <= 1'b0;
  else if(curr_state == IDLE)
    crc_en <= 1'b0;
  else if (curr_state != RX_PREAMBLE && rx_datav_dly2)
    crc_en <= 1'b1;
  else
    crc_en <= 1'b0;

  //crc_data
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    crc_data <= 8'd0;
  else
    crc_data <= rx_data_dly2;

  crc32_d8 crc32_d8
  (
    .clk         (clk125m       ),
    .reset_p     (reset_p       ),

    .data        (crc_data      ),
    .crc_init    (crc_init      ),
    .crc_en      (crc_en        ),
    .crc_result  (crc_check     )//latency=1
  );

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    reg_data_overflow <= 1'b0;
  else if(curr_state == IDLE)
    reg_data_overflow <= 1'b0;
  else if(curr_state == RX_DATA && data_overflow_i == 1'b1)
    reg_data_overflow <= 1'b1;
  else
    reg_data_overflow <= reg_data_overflow;

  //payload output
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    payload_valid_o <= 1'b0;
    payload_dat_o   <= 8'h0;
  end
  else if(curr_state == RX_DATA)
  begin
    payload_valid_o <= 1'b1;
    payload_dat_o   <= rx_data_dly2;
  end
  else
  begin
    payload_valid_o <= 1'b0;
    payload_dat_o   <= 8'h0;
  end

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    exter_mac  <= 48'h0;
    exter_ip   <= 32'h0;
    exter_port <= 16'h0;
  end
  else if(curr_state == PKT_CHECK)
  begin
    exter_mac  <= rx_src_mac;
    exter_ip   <= rx_src_ip;
    exter_port <= rx_src_port;
  end

  //done
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    one_pkt_done <= 1'b0;
    pkt_error    <= 1'b0;
  end
  else if(curr_state == PKT_CHECK)
  begin
    one_pkt_done <= 1'b1;
    if(crc_check == 32'h2144DF1C && reg_data_overflow == 1'b0 &&
       udp_header_check_ok && udp_checksum_ok_q && !frame_truncated)
      pkt_error  <= 1'b0;
    else
      pkt_error  <= 1'b1;
  end
  else
  begin
    one_pkt_done <= 1'b0;
    pkt_error    <= 1'b0;
  end

endmodule
