//////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Module Name: rom_tb
// Description:rom_ip项目测试 
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

module rom_tb;

	reg clk;
	reg [7:0]addr;
    reg reset_n;
  
	integer i = 0;
  
	initial clk = 1;
	always #(`CLK_PERIOD/2) clk = ~clk;


	wire [7:0]dout;

    GSR GSR(.GSRI(1'b1));

   Gowin_pROM Gowin_pROM(
        .dout(dout), //output [7:0] dout
        .clk(clk), //input clk
        .oce(1'b1), //input oce
        .ce(1'b1), //input ce
        .reset(~reset_n), //input reset
        .ad(addr) //input [7:0] ad
    );
  
	initial begin
        reset_n = 0;
        #201
        reset_n = 1;
		addr = 0;
		#21;
		for(i=0;i<2560;i=i+1)begin
			#`CLK_PERIOD;
		addr = addr + 1'b1;
		end
		#(`CLK_PERIOD * 50);
		$stop;	
	end

endmodule