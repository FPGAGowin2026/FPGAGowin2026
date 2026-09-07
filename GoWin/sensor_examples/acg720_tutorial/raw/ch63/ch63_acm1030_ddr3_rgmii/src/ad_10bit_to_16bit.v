/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2022/07/20 00:00:00
// Module Name   : ad1030_10bit_to_16bit
// Description   : 将ADC采集数据进行有符号数位宽扩展
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////
`define SIM
module ad_10bit_to_16bit(
  clk,
  ad_sample_en,
  ch_sel,
  ad_in1,
  ad_in2,
  ad_out,
  ad_out_valid
);

  input clk;
  input ad_sample_en;
  input [1:0]ch_sel;
  input[9:0] ad_in1;
  input[9:0] ad_in2;
  output[15:0] ad_out;
  output ad_out_valid;
  reg[15:0] ad_out;
  reg ad_out_valid;
  
  //用于仿真或产生测试数据
  reg [9:0]adc_test_data;
  //测试数据，当ad_sample_en为1时，锁相环生成的50M时钟每个周期使adc_test_data加1
  always@(posedge clk)
    adc_test_data <= ad_sample_en ? (adc_test_data + 1'b1) : 10'd0;

    wire [9:0]s_ad_in1;
	wire [9:0]s_ad_in2;  
	
	assign s_ad_in1 = ad_in1 + 10'd512;
	assign s_ad_in2 = ad_in2 + 10'd512;

  always @(posedge clk)
  if(ad_sample_en && ch_sel == 2'b01)
    ad_out<={4'd0,s_ad_in1,2'd0};//这样补0为了适应上位机
  else if(ad_sample_en && ch_sel == 2'b10)
    ad_out<={4'd0,s_ad_in2,2'd0};//
  else if(ad_sample_en && ch_sel == 2'b00)
    ad_out<={4'd0,adc_test_data,2'd0};
  else
    ad_out <= 16'd0;

  always @(posedge clk)
    ad_out_valid <= ad_sample_en;

endmodule
