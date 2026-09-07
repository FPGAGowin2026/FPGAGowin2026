/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2019/05/01 00:00:00
// Design Name: 
// Module Name: data_scan_ctrl_tb
// Project Name: 
// Target Devices: XC7A35T-2FGG484I
// Tool Versions: Vivado 2018.3
// Description: data_scan_ctrl仿真文件
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns
`define CLK_PERIOD 20

module data_scan_ctrl_tb();
  reg clk;
  reg reset_n;

  reg scan_en_pulse;
  reg [39:0]disp_data;

  wire [8:0]extr_data_addr;
  wire [9:0]update_data_wr;
  wire [4:0]update_data_addr;

  initial clk = 1;
  always#(`CLK_PERIOD/2)clk = ~clk;

  initial begin
    reset_n = 1'b0;
    disp_data = 40'h0123456789;
    scan_en_pulse = 1'b0;
    #(`CLK_PERIOD*20 + 1 );
    reset_n = 1'b1;
    #(`CLK_PERIOD*50);

    scan_en_pulse = 1'b1;
    #(`CLK_PERIOD)
    scan_en_pulse = 1'b0;

    #20000;
    disp_data = 40'h3402567891;
    scan_en_pulse = 1'b1;
    #(`CLK_PERIOD)
    scan_en_pulse = 1'b0;

    #20000;
    $stop;	
  end

data_scan_ctrl data_scan_ctrl
(
  .clk               (clk              ),
  .reset_n           (reset_n          ),
  .scan_en_pulse     (scan_en_pulse    ),
  .disp_data         (disp_data        ),
  .extr_data_addr    (extr_data_addr   ),
  .update_data_wr    (update_data_wr   ),
  .update_data_addr  (update_data_addr )
);

endmodule