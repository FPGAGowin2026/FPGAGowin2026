/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: Max
// Web: www.corecourse.cn
// 
// Create Date: 2019/05/01 00:00:00
// Design Name: 
// Module Name: frame_check_tb
// Project Name: 
// Target Devices: XC7A35T-2FGG484I
// Tool Versions: Vivado 2018.3
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ns
`define PERIOD_CLK 8

module frame_check_tb();
reg       clk;
reg       reset_n;
reg [15:0]data_i;
reg       data_i_valid;
wire      data_o_valid;
wire[15:0]data_o;
wire      frame_dis_pulse;

integer i;

initial clk = 1'b1;
always #(`PERIOD_CLK/2)clk = ~clk;

initial begin
  reset_n = 1'b0;
  data_i = 16'd0;
  data_i_valid = 1'b0;
  #(`PERIOD_CLK*200+1)
  reset_n = 1'b1;
  #200;
  
  for(i=0;i<50;i=i+1) begin
    @(posedge clk)
    #1 data_i_valid = 1'b1;
    data_i = i;
    repeat(50) begin
    @(posedge clk)
    #1 data_i = 16'h0505;  
    end
    data_i_valid = 1'b0;

    #200;   
  end

  for(i=0;i<100;i=i+2) begin
    @(posedge clk)
    #1 data_i_valid = 1'b1;
    data_i = i;
    repeat(50) begin
    @(posedge clk)
    #1 data_i = 16'hffff;  
    end
    data_i_valid = 1'b0;

    #200;   
  end

  for(i=0;i<50;i=i+1) begin
    @(posedge clk)
    #1 data_i_valid = 1'b1;
    data_i = i;
    repeat(50) begin
    @(posedge clk)
    #1 data_i = 16'h0a0a;  
    end
    data_i_valid = 1'b0;

    #200;   
  end

  #200;
  $stop;
end

frame_check
#(
  .IMAGE_WIDTH  (  50 ),
  .IMAGE_HEIGHT (  50 ),
  .DATA_WIDTH   (  16 )
)frame_check
(
  .clk            (clk             ),
  .reset          (~reset_n         ),
  .data_i         (data_i          ),
  .data_i_valid   (data_i_valid    ),
  .data_o_valid   (data_o_valid    ),
  .data_o         (data_o          ),
  .frame_dis_pulse(frame_dis_pulse )
);

endmodule