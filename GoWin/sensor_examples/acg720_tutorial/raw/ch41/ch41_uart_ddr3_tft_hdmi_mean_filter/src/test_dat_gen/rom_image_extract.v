/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : rom_image_extract
// Description   : 图像rom提取模块
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module rom_image_extract
#(
  parameter DISP_WIDTH     = 800, //整个屏幕显示区域宽度
  parameter DISP_HEIGHT    = 480, //整个屏幕显示区域高度
  parameter IMG_WIDTH      = 160, //图片宽度
  parameter IMG_HEIGHT     = 120, //图片高度
  parameter IMG_DATA_WIDTH = 16 , //图片像素点位宽
  parameter ROM_ADDR_WIDTH = 16   //存储图片ROM的地址位宽
)
(
  input                          clk            , //时钟输入
  input                          reset          , //复位信号

  input     [15:0]               img_disp_hbegin, //待显示图片左上角第一个像素点在TFT屏的行向坐标
  input     [15:0]               img_disp_vbegin, //待显示图片左上角第一个像素点在TFT屏的场向坐标
  input     [IMG_DATA_WIDTH-1:0] disp_back_color, //显示的背景颜色
  
  output reg[ROM_ADDR_WIDTH-1:0] rom_addr       , //读图片数据ROM地址
  input     [IMG_DATA_WIDTH-1:0] rom_data       , //读图片数据ROM数据

  input                          disp_data_vs   , //场信号
  input     [11:0]               disp_data_hcnt , //可见区域行扫描计数器
  input     [11:0]               disp_data_vcnt , //可见区域场扫描计数器
  input                          disp_data_req  , //
  output    [IMG_DATA_WIDTH-1:0] disp_data        //待显示图片数据
);
  wire       h_exceed;
  wire       v_exceed;
  wire       img_h_disp;
  wire       img_v_disp;
  wire       img_disp;
  wire [15:0]hcount_max;
  reg        disp_data_vs_dly1;

  //判断设置的显示的起始位置是否会导致显示超出范围
  assign h_exceed = img_disp_hbegin + IMG_WIDTH  > DISP_WIDTH - 1'b1;
  assign v_exceed = img_disp_vbegin + IMG_HEIGHT > DISP_HEIGHT - 1'b1;
  //不同的设置情况，显示区域做不同的处理
  assign img_h_disp = h_exceed ? (disp_data_hcnt >= img_disp_hbegin && disp_data_hcnt < DISP_WIDTH):
                                 (disp_data_hcnt >= img_disp_hbegin && disp_data_hcnt < img_disp_hbegin + IMG_WIDTH);

  assign img_v_disp = v_exceed ? (disp_data_vcnt >= img_disp_vbegin && disp_data_vcnt < DISP_HEIGHT):
                                 (disp_data_vcnt >= img_disp_vbegin && disp_data_vcnt < img_disp_vbegin + IMG_HEIGHT);

  assign img_disp = disp_data_req && img_h_disp && img_v_disp;

  assign hcount_max = h_exceed ? (DISP_WIDTH - 1'b1):(img_disp_hbegin + IMG_WIDTH - 1'b1);

  always@(posedge clk)
    disp_data_vs_dly1 <= disp_data_vs;

  always@(posedge clk or posedge reset)
  begin
    if(reset)
      rom_addr <= 'd0;
    else if(!disp_data_vs && disp_data_vs_dly1)
      rom_addr <= 'd0; 
    else if(img_disp)
    begin
      if(disp_data_hcnt == hcount_max)
        rom_addr <= rom_addr + (img_disp_hbegin + IMG_WIDTH - hcount_max);
      else
        rom_addr <= rom_addr + 1'b1;
    end
    else
      rom_addr <= rom_addr;
  end

  assign disp_data = img_disp ? rom_data : disp_back_color;

endmodule