/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Design Name   : 
// Module Name   : shift_register_2taps
// Project Name  : 
// Description   : shift_register_2taps模块仿真文件
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ns
`define CLK_PERIOD 40

module shift_register_2taps_tb();

  reg         clk;
  reg  [15:0] shiftin;
  reg         shiftin_valid;

  wire [15:0] shiftout;
  wire [15:0] taps1x;
  wire [15:0] taps0x;

initial clk = 1'b1;
always #(`CLK_PERIOD/2) clk = ~clk;

initial begin
  shiftin_valid = 1'b0;
  shiftin = 'd0;
  #201;
  repeat(800*6) begin
    shiftin_valid = 1'b1;
    shiftin = shiftin + 1'b1;
    #(`CLK_PERIOD);
  end
  #200;
  $stop;
end

shift_register_2taps
shift_register_2taps_inst
(
  .clk           (clk          ),
  .shiftin       (shiftin      ),
  .shiftin_valid (shiftin_valid),
 
  .shiftout      (shiftout     ),
  .taps1x        (taps1x       ),
  .taps0x        (taps0x       )
);

endmodule