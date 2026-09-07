/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : sort
// Description   : 排序模块
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module sort
#(
  parameter DATA_WIDTH = 8
)
(
  input                      clk,    //pixel clk
  input                      reset_p,
  input                      data_in_valid,
  input     [DATA_WIDTH-1:0] data0_in,
  input     [DATA_WIDTH-1:0] data1_in,
  input     [DATA_WIDTH-1:0] data2_in,

  output reg[DATA_WIDTH-1:0] data_max_out,
  output reg[DATA_WIDTH-1:0] data_mid_out,
  output reg[DATA_WIDTH-1:0] data_min_out,
  output reg                 data_out_valid
);

always @(posedge clk or posedge reset_p) begin
  if(reset_p) begin
    data_max_out <= 'd0;
    data_mid_out <= 'd0;
    data_min_out <= 'd0;
  end
  else if(data_in_valid) begin
    if((data0_in >= data1_in) && (data0_in >= data2_in)) begin
      data_max_out <= data0_in;

      if(data1_in >= data2_in) begin
        data_mid_out <= data1_in;
        data_min_out <= data2_in;
      end
      else begin
        data_mid_out <= data2_in;
        data_min_out <= data1_in;
      end
    end
    else if((data1_in > data0_in) && (data1_in >= data2_in)) begin
      data_max_out <= data1_in;

      if(data0_in >= data2_in) begin
        data_mid_out <= data0_in;
        data_min_out <= data2_in;
      end 
      else begin
        data_mid_out <= data2_in;
        data_min_out <= data0_in;
      end
    end
    else if((data2_in > data0_in) && (data2_in > data1_in)) begin
      data_max_out <= data2_in;

      if(data0_in >= data1_in) begin
        data_mid_out <= data0_in;
        data_min_out <= data1_in;
      end 
      else begin
        data_mid_out <= data1_in;
        data_min_out <= data0_in;
      end
    end
  end
end

always @(posedge clk or posedge reset_p)
  if(reset_p)
    data_out_valid <= 1'b0;
  else if(data_in_valid)
    data_out_valid <= 1'b1;
  else
    data_out_valid <= 1'b0;

endmodule
