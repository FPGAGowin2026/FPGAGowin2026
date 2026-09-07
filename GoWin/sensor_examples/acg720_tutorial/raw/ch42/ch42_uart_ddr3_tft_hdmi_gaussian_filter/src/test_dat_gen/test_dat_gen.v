/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : test_dat_gen
// Description   : 图像测试数据产生模块
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module test_dat_gen #(
  parameter CLK_FRQ     = 50_000_000,
  parameter DISP_WIDTH  = 800       ,
  parameter DISP_HEIGHT = 480       ,
  parameter DATA_WIDTH  = 16        ,
  parameter DISP_MODE   = 2'b00     
)
(
  input                  clk,
  input                  reset,
  input                  gen_en,
  output [DATA_WIDTH-1:0]test_dat,
  output                 test_dat_vaild,
  output                 test_dat_hs,
  output                 test_dat_vs,
  output [15:0]          test_dat_col,
  output [15:0]          test_dat_row
);

localparam COL_CNT_MAX   = DISP_WIDTH  + 50;
localparam ROW_CNT_MAX   = DISP_HEIGHT + 10;
localparam T40ms_CNT_MAX = 40_000_000/(1_000_000_000/CLK_FRQ) - 1;

  reg  [15:0]           col_data_cnt;
  reg  [15:0]           row_data_cnt;

  reg                   test_dat_hs_0;
  reg                   test_dat_vs_0;
  reg  [15:0]           test_dat_col_0;
  reg  [15:0]           test_dat_row_0;
  reg                   test_dat_vaild_0;
  reg  [DATA_WIDTH-1:0] test_dat_0;

  wire                  test_dat_hs_1;
  wire                  test_dat_vs_1;
  wire [15:0]           test_dat_col_1;
  wire [15:0]           test_dat_row_1;
  wire                  test_dat_vaild_1;
  wire [DATA_WIDTH-1:0] test_dat_1;

  wire                  test_dat_hs_2;
  wire                  test_dat_vs_2;
  wire [15:0]           test_dat_col_2;
  wire [15:0]           test_dat_row_2;
  wire                  test_dat_vaild_2;
  wire [DATA_WIDTH-1:0] test_dat_2;

  wire                  test_dat_hs_3;
  wire                  test_dat_vs_3;
  wire [15:0]           test_dat_col_3;
  wire [15:0]           test_dat_row_3;
  wire                  test_dat_vaild_3;
  wire [DATA_WIDTH-1:0] test_dat_3;

  reg  [31:0]           t40ms_cnt;
  reg                   t40ms_flag;
  reg                   direction_x;
  reg  [15:0]           img_disp_hbegin;
  reg  [15:0]           img_disp_vbegin;
  wire [15:0]           rom_addr;
  wire [15:0]           rom_data;

generate
  case (DISP_MODE)
    2'd0: begin: test_out0
      assign test_dat_hs    = test_dat_hs_0;
      assign test_dat_vs    = test_dat_vs_0;
      assign test_dat_vaild = test_dat_vaild_0;
      assign test_dat_col   = test_dat_col_0;
      assign test_dat_row   = test_dat_row_0;
      assign test_dat       = test_dat_0;
    end
    2'd1: begin: test_out1
      assign test_dat_hs    = test_dat_hs_1;
      assign test_dat_vs    = test_dat_vs_1;
      assign test_dat_vaild = test_dat_vaild_1;
      assign test_dat_col   = test_dat_col_1;
      assign test_dat_row   = test_dat_row_1;
      assign test_dat       = test_dat_1;
    end
    2'd2: begin: test_out2
      assign test_dat_hs    = test_dat_hs_2;
      assign test_dat_vs    = test_dat_vs_2;
      assign test_dat_vaild = test_dat_vaild_2;
      assign test_dat_col   = test_dat_col_2;
      assign test_dat_row   = test_dat_row_2;
      assign test_dat       = test_dat_2;
    end
    2'd3: begin: test_out3
      assign test_dat_hs    = test_dat_hs_3;
      assign test_dat_vs    = test_dat_vs_3;
      assign test_dat_vaild = test_dat_vaild_3;
      assign test_dat_col   = test_dat_col_3;
      assign test_dat_row   = test_dat_row_3;
      assign test_dat       = test_dat_3;
    end
  endcase
endgenerate

  always@(posedge clk or posedge reset)
  begin
    if(reset)
      col_data_cnt <= 16'd0;
    else if(!gen_en)
      col_data_cnt <= 16'd0;
    else if(col_data_cnt >= COL_CNT_MAX)
      col_data_cnt <= 16'd0;
    else
      col_data_cnt <= col_data_cnt + 1'b1;
  end

  always@(posedge clk or posedge reset)
  begin
    if(reset)
      row_data_cnt <= 16'd0;
    else if(col_data_cnt == COL_CNT_MAX)
      if(row_data_cnt >= ROW_CNT_MAX)
        row_data_cnt <= 16'd0;
      else
        row_data_cnt <= row_data_cnt + 1'b1;
    else
      row_data_cnt <= row_data_cnt;
  end

  //test out 0
  always@(posedge clk)
  begin
      test_dat_hs_0    <= (col_data_cnt > 'd0)  && (col_data_cnt <= DISP_WIDTH  + 'd20) && 
                          (row_data_cnt > 'd5)  && (row_data_cnt <= DISP_HEIGHT + 'd5);
      test_dat_vs_0    <= (row_data_cnt > 'd5)  && (row_data_cnt <= DISP_HEIGHT + 'd5);
      test_dat_col_0   <= col_data_cnt - 'd10;
      test_dat_row_0   <= row_data_cnt - 'd5;
      test_dat_vaild_0 <= (col_data_cnt > 'd10) && (col_data_cnt <= DISP_WIDTH  + 'd10) && 
                          (row_data_cnt > 'd5)  && (row_data_cnt <= DISP_HEIGHT + 'd5);
      test_dat_0       <= col_data_cnt - 'd10;
  end

  //test out 1
  assign test_dat_hs_1    = test_dat_hs_0;
  assign test_dat_vs_1    = test_dat_vs_0;
  assign test_dat_col_1   = test_dat_col_0;
  assign test_dat_row_1   = test_dat_row_0;
  assign test_dat_vaild_1 = test_dat_vaild_0;
  assign test_dat_1       = row_data_cnt - 'd5;

  //test out 2
  assign test_dat_hs_2    = test_dat_hs_0;
  assign test_dat_vs_2    = test_dat_vs_0;
  assign test_dat_col_2   = test_dat_col_0;
  assign test_dat_row_2   = test_dat_row_0;
  assign test_dat_vaild_2 = test_dat_vaild_0;

  colour_bar_dat_gen
  #(
    .DISP_WIDTH  (DISP_WIDTH  ),
    .DISP_HEIGHT (DISP_HEIGHT ),
    .DATA_WIDTH  (DATA_WIDTH  )
  )colour_bar_dat_gen_inst
  (
    .disp_h_addr  (test_dat_col_2[11:0]-1'b1 ),
    .disp_v_addr  (test_dat_row_2[11:0]-1'b1 ),
    .disp_data_req(test_dat_vaild_2          ),
    .disp_data    (test_dat_2                )
  );

  //test out 3
  assign test_dat_hs_3    = test_dat_hs_0;
  assign test_dat_vs_3    = test_dat_vs_0;
  assign test_dat_col_3   = test_dat_col_0;
  assign test_dat_row_3   = test_dat_row_0;
  assign test_dat_vaild_3 = test_dat_vaild_0;

  rom_image_extract
  #(
    .DISP_WIDTH     (DISP_WIDTH     ), //屏幕显示区域宽度
    .DISP_HEIGHT    (DISP_HEIGHT    ), //屏幕显示区域高度
    .IMG_WIDTH      (200            ), //图片宽度
    .IMG_HEIGHT     (200            ), //图片高度
    .IMG_DATA_WIDTH (DATA_WIDTH     ), //图片像素点位宽
    .ROM_ADDR_WIDTH (16             )  //存储图片ROM的地址位宽
  )rom_image_extract_inst
  (
    .clk            (clk                            ),
    .reset          (reset                          ),
    .img_disp_hbegin(img_disp_hbegin                ),
    .img_disp_vbegin(img_disp_vbegin                ),
    .disp_back_color(test_dat_2                     ),
    .rom_addr       (rom_addr                       ),
    .rom_data       (rom_data                       ),
    .disp_data_vs   (test_dat_vs_3                  ),
    .disp_data_hcnt (test_dat_col_3[11:0]           ),
    .disp_data_vcnt (test_dat_row_3[11:0]           ),
    .disp_data_req  (test_dat_vaild_3               ),
    .disp_data      (test_dat_3                     )
  );

  rom_image rom_image_inst (
    .clk  (clk       ),// input wire clka
    .addr (rom_addr  ),// input wire [15 : 0] addra
    .dout (rom_data  ) // output wire [15 : 0] douta
  );

  always@(posedge clk or posedge reset)
  begin
    if(reset)
      t40ms_cnt <= 'd0;
    else if(t40ms_cnt >= T40ms_CNT_MAX)
      t40ms_cnt <= 'd0;
    else
      t40ms_cnt <= t40ms_cnt + 1'b1;
  end

  always@(posedge clk or posedge reset)
  begin
    if(reset)
      t40ms_flag <= 1'b0;
    else if(t40ms_cnt >= T40ms_CNT_MAX)
      t40ms_flag <= 1'b1;
    else
      t40ms_flag <= 1'b0;
  end

  always@(posedge clk or posedge reset)
  begin
    if(reset)
      direction_x <= 1'b0;
    else if(img_disp_hbegin >= 500)
      direction_x <= 1'b1;
    else if(img_disp_hbegin <= 100)
      direction_x <= 1'b0;
    else
      direction_x <= direction_x;
  end

  always@(posedge clk or posedge reset)
  begin
    if(reset)
      img_disp_hbegin <= 16'd100;
    else if(t40ms_flag && (!direction_x))
      img_disp_hbegin <= img_disp_hbegin + 5;
    else if(t40ms_flag && direction_x )
      img_disp_hbegin <= img_disp_hbegin - 5;
  end

  always@(*)
    img_disp_vbegin = 140;

endmodule