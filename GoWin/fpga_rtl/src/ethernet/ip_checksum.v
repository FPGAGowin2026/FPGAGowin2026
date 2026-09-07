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

  output  [15:0]  checksum
);

  reg  [18:0] sum_a_q;
  reg  [18:0] sum_b_q;
  reg  [19:0] suma;
  reg  [16:0] sumb_q;
  reg  [15:0] checksum_q;

  // 九个 16 位字先分为两组寄存，再合并。避免 ID/长度等动态字段
  // 经过一整条多操作数加法链后才到 checksum 寄存器。
  always@(posedge clk or posedge reset_p)
  if(reset_p)
  begin
    sum_a_q <= 19'd0;
    sum_b_q <= 19'd0;
    suma    <= 20'd0;
  end
  else
  begin
    if(cal_en)
    begin
      sum_a_q <= {3'd0,IP_ver,IP_hdr_len,IP_tos}+
                 {3'd0,IP_total_len}+{3'd0,IP_id}+
                 {3'd0,IP_rsv,IP_df,IP_mf,IP_frag_offset}+
                 {3'd0,IP_ttl,IP_protocol};
      sum_b_q <= {3'd0,src_ip[31:16]}+{3'd0,src_ip[15:0]}+
                 {3'd0,dst_ip[31:16]}+{3'd0,dst_ip[15:0]};
    end
    suma <= {1'b0,sum_a_q}+{1'b0,sum_b_q};
  end

  // 折叠结果分两拍寄存；RX/TX 状态机在使用前都有充足空档。
  always@(posedge clk or posedge reset_p)
  if(reset_p)
  begin
    sumb_q     <= 17'd0;
    checksum_q <= 16'd0;
  end
  else
  begin
    sumb_q     <= {13'd0,suma[19:16]} + suma[15:0];
    checksum_q <= ~(sumb_q[15:0] + sumb_q[16]);
  end

  assign checksum = checksum_q;

endmodule
