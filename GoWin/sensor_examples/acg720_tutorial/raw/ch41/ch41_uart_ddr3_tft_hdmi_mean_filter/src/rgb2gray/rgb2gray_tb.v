/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/04/10 00:00:00
// Module Name   : rgb2gray_tb
// Description   : 图像处理之彩色图像灰度化仿真
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ns
`define CLK_PERIOD 20

module rgb2gray_tb();
  reg       clk;
  reg       rst_n;
  reg       rgb_valid;
  reg       rgb_hs;
  reg       rgb_vs;
  reg [7:0] red_8b_i;
  reg [7:0] green_8b_i;
  reg [7:0] blue_8b_i;
  reg [7:0] comp1_gray;
  reg [7:0] comp2_gray;

  initial clk = 1'b1;
  always #(`CLK_PERIOD/2)clk = ~clk;

  initial begin
    reset_p    = 1;
    rgb_valid  = 0;
    rgb_hs     = 0;
    rgb_vs     = 0;
    red_8b_i   = 0;
    green_8b_i = 0;
    blue_8b_i  = 0;
    #(`CLK_PERIOD*200+1);
    reset_p    = 0;
    red_8b_i   = 56;
    green_8b_i = 124;
    blue_8b_i  = 203;
    #2000;

    rgb_hs    = 1;
    rgb_vs    = 1;
    rgb_valid = 1;
    repeat(256)begin
      #(`CLK_PERIOD)
      red_8b_i   = red_8b_i   + 1;
      green_8b_i = green_8b_i + 1;
      blue_8b_i  = blue_8b_i  + 1;
    end
    rgb_valid = 0;
    rgb_hs    = 0;
    rgb_vs    = 0;

    #2000;
    $stop;  
  end

  always@(posedge clk)
  begin
    if(rgb_valid == 1'b1)
      comp1_gray <= (red_8b_i + green_8b_i + blue_8b_i)/3;  
    else
      comp1_gray <= 0;
  end

  always@(posedge clk)
  begin
    if(rgb_valid == 1'b1)
      comp2_gray <= red_8b_i*0.299 + green_8b_i*0.587 + blue_8b_i*0.114;  
    else
      comp2_gray <= 0;
  end

  rgb2gray
  #(
    .PROC_METHOD("AVERAGE")  //"AVERAGE"     :求平均法
                             //or "FORMULA"  :直接公式法
                             //or "LUT"      :查找表法
  )rgb2gray_average
  (
    .clk       (clk       ),
    .reset_p   (reset_p   ),
    .rgb_valid (rgb_valid ),
    .rgb_hs    (rgb_hs    ),
    .rgb_vs    (rgb_vs    ),
    .red_8b_i  (red_8b_i  ),
    .green_8b_i(green_8b_i),
    .blue_8b_i (blue_8b_i ),
    .gray_8b_o (          ),
    .gray_valid(          ),
    .gray_hs   (          ),
    .gray_vs   (          ) 
  );

  rgb2gray
  #(
    .PROC_METHOD("FORMULA")  //"AVERAGE"     :求平均法
                             //or "FORMULA"  :直接公式法
                             //or "LUT"      :查找表法
  )rgb2gray_formula
  (
    .clk       (clk       ),
    .reset_p   (reset_p   ),
    .rgb_valid (rgb_valid ),
    .rgb_hs    (rgb_hs    ),
    .rgb_vs    (rgb_vs    ),
    .red_8b_i  (red_8b_i  ),
    .green_8b_i(green_8b_i),
    .blue_8b_i (blue_8b_i ),
    .gray_8b_o (          ),
    .gray_valid(          ),
    .gray_hs   (          ),
    .gray_vs   (          ) 
  );

  rgb2gray
  #(
    .PROC_METHOD(  "LUT"  )  //"AVERAGE"     :求平均法
                             //or "FORMULA"  :直接公式法
                             //or "LUT"      :查找表法
  )rgb2gray_lut
  (
    .clk       (clk       ),
    .reset_p   (reset_p   ),
    .rgb_valid (rgb_valid ),
    .rgb_hs    (rgb_hs    ),
    .rgb_vs    (rgb_vs    ),
    .red_8b_i  (red_8b_i  ),
    .green_8b_i(green_8b_i),
    .blue_8b_i (blue_8b_i ),
    .gray_8b_o (          ),
    .gray_valid(          ),
    .gray_hs   (          ),
    .gray_vs   (          ) 
  );

endmodule 