
/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : test_dat_gen
// Description   : 图像测试数据产生模块仿真
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

module test_dat_gen_tb();

  reg         clk;
  reg         reset;
  reg         gen_en;
  wire [15:0] test_dat;
  wire        test_dat_vaild;
  wire        test_dat_hs;
  wire        test_dat_vs;
  wire [15:0] test_dat_col;
  wire [15:0] test_dat_row;

  test_dat_gen #(
    .DISP_WIDTH  ( 800 ),
    .DISP_HEIGHT ( 480 ),
    .DATA_WIDTH  ( 16  ),
    .DISP_MODE   ( 2'd3)
  )test_dat_gen_inst
  (
    .clk            (clk            ),
    .reset          (reset          ),
    .gen_en         (gen_en         ),
    .test_dat       (test_dat       ),
    .test_dat_vaild (test_dat_vaild ),
    .test_dat_hs    (test_dat_hs    ),
    .test_dat_vs    (test_dat_vs    ),
    .test_dat_col   (test_dat_col   ),
    .test_dat_row   (test_dat_row   )
  );

  initial clk = 1'b1;
  always#(`CLK_PERIOD/2)clk = ~clk;

  initial begin
    reset = 1'b1;
    gen_en = 1'b0;
    #(`CLK_PERIOD*10+1'b1);
    reset = 1'b0;
    #(`CLK_PERIOD*200);
    gen_en = 1'b1;

    @(posedge test_dat_vs);
    @(posedge test_dat_vs);
    #5000;
    $stop;
  end

`ifdef SAVE_WAVE_FILE
  // dump fsdb file for debussy
  initial
  begin
    $fsdbDumpfile("wave.fsdb");
    $fsdbDumpvars;
  end
`endif

endmodule