/////////////////////////////////////////////////////////////////////////////////
// Company: 姝︽眽鑺矾鎭掔鎶�鏈夐檺鍏徃
// Engineer: 灏忔鍝ュ洟闃�
// Web: www.corecourse.cn
// 
// Create Date: 2019/05/01 00:00:00
// Design Name: 
// Module Name: data_scan_ctrl
// Project Name: 
// Target Devices: XC7A35T-2FGG484I
// Tool Versions: Vivado 2018.3
// Description: data_scan_ctrl浠跨湡鏂囦欢
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module data_scan_ctrl
(
  input          clk,
  input          reset_n,

  input          scan_en_pulse,
  input    [39:0]disp_data,

  output reg[8:0]extr_data_addr,
  output reg[9:0]update_data_wr,
  output reg[4:0]update_data_addr
);
reg       scan_state;
reg       extr_data_done;
reg [8:0] addr_cnt;
reg [8:0] addr_cnt_dly1;
reg [8:0] addr_cnt_dly2;
reg [8:0] addr_cnt_dly3;
reg [9:0] extr_data_cs;
reg [9:0] extr_data_cs_dly1;
reg [9:0] extr_data_cs_dly2;
reg [8:0] extr_data_addr_offset;
reg [3:0] extr_data_num;

  always@(posedge clk or negedge reset_n)
  begin
    if(~reset_n)
      scan_state <= 1'b0;
    else if(addr_cnt == 9'd319)
      scan_state <= 1'b0;
    else if(scan_en_pulse)
      scan_state <= 1'b1;
    else
      scan_state <= scan_state;
  end

  always@(posedge clk or negedge reset_n)
  begin
    if(~reset_n)
      addr_cnt <= 'd0;
    else if(scan_state == 1'b1)
      addr_cnt <= addr_cnt + 1'b1;
    else
      addr_cnt <= 'd0;
  end

  always@(posedge clk)
  begin
      addr_cnt_dly1 <= addr_cnt;
      addr_cnt_dly2 <= addr_cnt_dly1;
      addr_cnt_dly3 <= addr_cnt_dly2;
  end

  always@(posedge clk or negedge reset_n)
  begin
    if(~reset_n)
      extr_data_done <= 1'b0;
    else if(addr_cnt[4:0] == 5'h1f)
      extr_data_done <= 1'b1;
    else
      extr_data_done <= 1'b0;
  end

  always@(posedge clk or negedge reset_n)
  begin
    if(~reset_n)
      extr_data_cs <= 10'b00_0000_0000;
    else if(scan_en_pulse)
      extr_data_cs <= 10'b00_0000_0001;
    else if(extr_data_done)
      extr_data_cs <= extr_data_cs << 1'b1;
    else
      extr_data_cs <= extr_data_cs;
  end

  always@(posedge clk)
  begin
      extr_data_cs_dly1 <= extr_data_cs;
      extr_data_cs_dly2 <= extr_data_cs_dly1;
  end

  always@(*)
  begin
    case(extr_data_cs)
      10'b00_0000_0001: extr_data_num = disp_data[3:0];
      10'b00_0000_0010: extr_data_num = disp_data[7:4];
      10'b00_0000_0100: extr_data_num = disp_data[11:8];
      10'b00_0000_1000: extr_data_num = disp_data[15:12];
      10'b00_0001_0000: extr_data_num = disp_data[19:16];
      10'b00_0010_0000: extr_data_num = disp_data[23:20];
      10'b00_0100_0000: extr_data_num = disp_data[27:24];
      10'b00_1000_0000: extr_data_num = disp_data[31:28];
      10'b01_0000_0000: extr_data_num = disp_data[35:32];
      10'b10_0000_0000: extr_data_num = disp_data[39:36];
      default     : extr_data_num = 4'h0;
    endcase
  end

  always@(posedge clk or negedge reset_n)
  begin
    if(~reset_n)
      extr_data_addr_offset <= 'd0;
    else
      case(extr_data_num)
        4'h0   :extr_data_addr_offset <= 'd0;
        4'h1   :extr_data_addr_offset <= 'd32;
        4'h2   :extr_data_addr_offset <= 'd64;
        4'h3   :extr_data_addr_offset <= 'd96;
        4'h4   :extr_data_addr_offset <= 'd128;
        4'h5   :extr_data_addr_offset <= 'd160;
        4'h6   :extr_data_addr_offset <= 'd192;
        4'h7   :extr_data_addr_offset <= 'd224;
        4'h8   :extr_data_addr_offset <= 'd256;
        4'h9   :extr_data_addr_offset <= 'd288;
        4'ha   :extr_data_addr_offset <= 'd320;
        4'hb   :extr_data_addr_offset <= 'd352;
        default:extr_data_addr_offset <= 'd384;
      endcase
  end

  always@(posedge clk or negedge reset_n)
  begin
    if(~reset_n)
      extr_data_addr <= 'd0;
    else
      extr_data_addr <= extr_data_addr_offset + addr_cnt_dly2[4:0];
  end

  always@(posedge clk or negedge reset_n)
  begin
    if(~reset_n)
      update_data_addr <= 'd0;
    else
      update_data_addr <= addr_cnt_dly3[4:0];
  end

  always@(posedge clk or negedge reset_n)
  begin
    if(~reset_n)
      update_data_wr <= 'd0;
    else
      update_data_wr <= extr_data_cs_dly2;
  end

endmodule