/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : shift_register_2taps
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

  reg        clk;
  reg        reset_p;
  reg  [7:0] shiftin;
  reg        shiftin_valid;

  wire [7:0] shiftout;
  wire [7:0] taps1x;
  wire [7:0] taps0x;

initial clk = 1'b1;
always #(`CLK_PERIOD/2) clk = ~clk;

initial begin
  shiftin_valid = 1'b0;
  shiftin = 'd0;
  reset_p = 1'd1;
  #201;
  reset_p = 1'd0;
  #2000;
  repeat(3) begin
    shiftin = 'd0;
    repeat(400) begin
      shiftin_valid = 1'b1;
      shiftin = shiftin + 1'b1;
      #(`CLK_PERIOD);
    end
  end
  #200;
  $stop;
end

shift_register_2taps
shift_register_2taps_inst
(
  .clk           (clk          ),
  .Reset		 (reset_p	   ),
  .shiftin       (shiftin      ),
  .shiftin_valid (shiftin_valid),
 
  .shiftout      (shiftout     ),
  .taps1x        (taps1x       ),
  .taps0x        (taps0x       )
);

endmodule