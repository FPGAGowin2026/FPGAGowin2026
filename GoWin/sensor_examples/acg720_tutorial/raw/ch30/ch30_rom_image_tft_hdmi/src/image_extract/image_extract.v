/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: rom_image_tft
// Module Name: image_extract
// Project Name: rom_image_tft
// Description: 图片描述文件，行列控制和颜色执行
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module image_extract
#(
  parameter H_Visible_area = 800, //整个屏幕显示区域宽度
  parameter V_Visible_area = 480, //整个屏幕显示区域高度
  parameter IMG_WIDTH      = 160, //图片宽度
  parameter IMG_HEIGHT     = 120, //图片高度
  parameter IMG_DATA_WIDTH = 16,  //图片像素点位宽
  parameter ROM_ADDR_WIDTH = 16   //存储图片ROM的地址位宽
)
(
  clk_ctrl,
  reset_n,
  img_disp_hbegin,
  img_disp_vbegin,
  disp_back_color,
  Frame_Begin,
  rom_addra,
  rom_data,
  disp_data_req,
  visible_hcount,
  visible_vcount,
  disp_data
);

  input                       clk_ctrl       ; //时钟输入，与TFT屏时钟保持一致
  input                       reset_n        ; //复位信号，低电平有效

  input  [15:0]               img_disp_hbegin; //待显示图片左上角第一个像素点在TFT屏的行向坐标
  input  [15:0]               img_disp_vbegin; //待显示图片左上角第一个像素点在TFT屏的场向坐标
  input  [IMG_DATA_WIDTH-1:0] disp_back_color; //显示的背景颜色
  
  output [ROM_ADDR_WIDTH-1:0] rom_addra      ; //读图片数据ROM地址
  input  [IMG_DATA_WIDTH-1:0] rom_data       ; //读图片数据ROM数据

  input                       Frame_Begin    ; //一帧图像起始标识信号，clk_ctrl时钟域
  input                       disp_data_req  ; //
  input  [11:0]               visible_hcount ; //TFT可见区域行扫描计数器
  input  [11:0]               visible_vcount ; //TFT可见区域场扫描计数器
  output [IMG_DATA_WIDTH-1:0] disp_data      ; //待显示图片数据
  
  reg   [ROM_ADDR_WIDTH-1:0]  rom_addra      ; //读图片数据rom地址 
  
  wire       h_exceed;
  wire       v_exceed;
  wire       img_h_disp;
  wire       img_v_disp;
  wire       img_disp;
  wire [15:0]hcount_max;

  //判断设置的显示的起始位置是否会导致显示超出范围
  assign h_exceed = img_disp_hbegin + IMG_WIDTH  > H_Visible_area - 1'b1;
  assign v_exceed = img_disp_vbegin + IMG_HEIGHT > V_Visible_area - 1'b1;
  //不同的设置情况，显示区域做不同的处理
  assign img_h_disp = h_exceed ? (visible_hcount >= img_disp_hbegin && visible_hcount < H_Visible_area):
                                 (visible_hcount >= img_disp_hbegin && visible_hcount < img_disp_hbegin + IMG_WIDTH);  
  
  assign img_v_disp = v_exceed ? (visible_vcount >= img_disp_vbegin && visible_vcount < V_Visible_area):
                                 (visible_vcount >= img_disp_vbegin && visible_vcount < img_disp_vbegin + IMG_HEIGHT);
     
  assign img_disp = disp_data_req && img_h_disp && img_v_disp;
  
  assign hcount_max = h_exceed ? (H_Visible_area - 1'b1):(img_disp_hbegin + IMG_WIDTH - 1'b1);
     
  always@(posedge clk_ctrl or negedge reset_n)
  begin
    if(!reset_n)
      rom_addra <= 'd0;
    else if(Frame_Begin)
      rom_addra <= 'd0; 
    else if(img_disp)
    begin
      if(visible_hcount == hcount_max)
        rom_addra <= rom_addra + (img_disp_hbegin + IMG_WIDTH - hcount_max);
      else
        rom_addra <= rom_addra + 1'b1;
    end    
    else
      rom_addra <= rom_addra;
  end

  assign disp_data = img_disp ? rom_data : disp_back_color;

 endmodule