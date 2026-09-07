//////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: key_filter
// Module Name: key_model
// Project Name: key_filter
// Target Devices: XC7A35T-2FGG484I
// Tool Versions: Vivado 2018.3
// Description: 按键消抖模块测试文件
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ns

module key_model(
	key_press,
	key_out
);
	input key_press;
	output key_out;
	
	reg key_out;
	reg [15:0]myrand;
	
	initial begin
	key_out = 1'b1;
    while(1)
		begin
		  @(posedge key_press);
		  key_gen;
		end
	end
	
	task key_gen;
	begin
    key_out = 1'b1;
    repeat(50)begin
      myrand = {$random}%65536;//0~65535;
      #myrand key_out = ~key_out;			
    end
    key_out = 0;
    #25000000;
    
    repeat(50)begin
      myrand = {$random}%65536;//0~65535;
      #myrand key_out = ~key_out;			
    end
    key_out = 1;
    #25000000;		
	end	
	endtask

endmodule
