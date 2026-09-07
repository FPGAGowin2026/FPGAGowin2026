/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2023/01/02 00:00:00
// Module Name   : image_rx_ctrl
// Description   : 
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module image_rx_ctrl # (
  parameter DISP_WIDTH  = 800 ,
  parameter DISP_HEIGHT = 480
)
(
  input             clk                ,
  input             reset_p            ,
  input             image_data_valid   ,
  output reg [15:0] image_data_hcnt    ,
  output reg [15:0] image_data_vcnt    ,
  output reg        image_data_hs      ,
  output reg        image_data_vs      ,
  output reg        frame_rx_done_flip 
);

  //generate image data hs or vs
  always@(posedge clk or posedge reset_p)
    if(reset_p)
      image_data_hcnt <= 'd0;
    else if(image_data_valid) begin
      if(image_data_hcnt == (DISP_WIDTH - 1'b1))
        image_data_hcnt <= 'd0;
      else
        image_data_hcnt <= image_data_hcnt + 1'b1;
    end

  always@(posedge clk or posedge reset_p)
    if(reset_p)
      image_data_vcnt <= 'd0;
    else if(image_data_valid) begin
      if(image_data_hcnt == (DISP_WIDTH - 1'b1)) begin
        if(image_data_vcnt == (DISP_HEIGHT - 1'b1))
          image_data_vcnt <= 'd0;
        else
          image_data_vcnt <= image_data_vcnt + 1'b1;
      end
    end
  //hs
  always@(posedge clk or posedge reset_p)
    if(reset_p)
      image_data_hs <= 1'b0;
    else if(image_data_valid && image_data_hcnt == (DISP_WIDTH - 1'b1))
      image_data_hs <= 1'b0;
    else
      image_data_hs <= 1'b1;
  //vs
  always@(posedge clk or posedge reset_p)
    if(reset_p)
      image_data_vs <= 1'b0;
    else if(image_data_valid && image_data_hcnt == (DISP_WIDTH - 1'b1) &&
            image_data_vcnt == (DISP_HEIGHT - 1'b1))
      image_data_vs <= 1'b0;
    else
      image_data_vs <= 1'b1;

  always@(posedge clk or posedge reset_p)
    if(reset_p)
      frame_rx_done_flip <= 1'b0;
    else if(image_data_valid && image_data_hcnt == (DISP_WIDTH - 1'b1) &&
            image_data_vcnt == (DISP_HEIGHT - 1'b1))
      frame_rx_done_flip <= ~frame_rx_done_flip;

endmodule
