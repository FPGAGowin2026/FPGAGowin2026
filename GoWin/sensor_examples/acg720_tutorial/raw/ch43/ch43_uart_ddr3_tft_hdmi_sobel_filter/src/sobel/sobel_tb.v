/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : sobel_tb
// Description   : sobel模块仿真
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

module sobel_tb();

  reg       clk;    //pixel clk
  reg       reset_p;
  reg [7:0] data_in;
  reg       data_in_valid;
  reg       data_in_hs;
  reg       data_in_vs;
  wire      data_out;
  wire      data_out_valid;
  wire      data_out_hs;
  wire      data_out_vs;

initial clk = 1'b1;
always #(`CLK_PERIOD/2) clk = ~clk;

initial begin
  reset_p = 1'b1;
  data_in = 8'd0;
  data_in_valid = 1'b0;
  data_in_hs = 1'b0;
  data_in_vs = 1'b0;
  #201;
  reset_p = 1'b0;
  #200;

  data_in_vs = 1'b1;
  repeat(480) begin
    #500;
    data_in_hs = 1'b1;
    #500;
    repeat(400*2) begin
      data_in_valid = ~data_in_valid;
      data_in = $random % 256;
      #(`CLK_PERIOD);
    end
    #500;
    data_in_hs = 1'b0;
  end
  data_in_vs = 1'b0;
  #(`CLK_PERIOD);
  data_in_vs = 1'b1;

  #2000;
  $stop;
end

sobel
#(
  .DATA_WIDTH ( 8 )
)sobel
(
  .clk            (clk           ),    //pixel clk
  .reset_p        (reset_p       ),
  .data_in        (data_in       ),
  .data_in_valid  (data_in_valid ),
  .data_in_hs     (data_in_hs    ),
  .data_in_vs     (data_in_vs    ),
  .threshold      (8'd128        ),

  .data_out       (data_out      ),
  .data_out_valid (data_out_valid),
  .data_out_hs    (data_out_hs   ),
  .data_out_vs    (data_out_vs   )
);

endmodule