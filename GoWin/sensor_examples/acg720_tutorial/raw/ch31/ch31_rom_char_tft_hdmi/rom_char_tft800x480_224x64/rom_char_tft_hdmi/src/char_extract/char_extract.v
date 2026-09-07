/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2019/05/01 00:00:00
// Design Name: 
// Module Name: char_extract
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

module char_extract
#(
  parameter H_Visible_area    = 800, //屏幕显示区域宽度
  parameter V_Visible_area    = 480, //屏幕显示区域高度
  parameter CHAR_WIDTH        = 16 , //单个字符显示宽度
  parameter CHAR_HEIGHT       = 32 , //单个字符显示高度
  parameter ROW_DISP_CHAR_NUM = 10 , //一行显示字符个数
  parameter COL_DISP_CHAR_NUM = 2  , //显示字符行数
  parameter CHAR_ROM_ADDR_W   = 5  , //存储字符ROM地址位宽，log2(CHAR_HEIGHT * COL_DISP_CHAR_NUM)
  parameter DISP_DATA_W       = 16   //图片像素点数据位宽
)
(
  clk_ctrl,
  reset_n,
  char_disp_hbegin,
  char_disp_vbegin,
  disp_back_color,
  disp_char_color,
  Frame_Begin,
  rom_addra,
  rom_data,
  disp_data_req,
  visible_hcount,
  visible_vcount,
  disp_data
);
localparam DISP_CHAR_TOTAL_W = CHAR_WIDTH  * ROW_DISP_CHAR_NUM;
localparam DISP_CHAR_TOTAL_H = CHAR_HEIGHT * COL_DISP_CHAR_NUM;

  input                         clk_ctrl;         //时钟输入，与TFT屏时钟保持一致
  input                         reset_n;          //复位信号，低电平有效

  input  [15:0]                 char_disp_hbegin; //待显示图片左上角第一个像素点在TFT屏的行向坐标
  input  [15:0]                 char_disp_vbegin; //待显示图片左上角第一个像素点在TFT屏的场向坐标
  input  [DISP_DATA_W-1:0]      disp_back_color;  //显示的背景颜色
  input  [DISP_DATA_W-1:0]      disp_char_color;  //显示字符的颜色

  input                         Frame_Begin;      //一帧图像起始标识信号，clk_ctrl时钟域
  output[CHAR_ROM_ADDR_W-1:0]   rom_addra;        //读字符数据rom地址                  
  input [DISP_CHAR_TOTAL_W-1:0] rom_data;         //读出字符数据
  input                         disp_data_req;    //
  input  [11:0]                 visible_hcount;   //TFT可见区域行扫描计数器
  input  [11:0]                 visible_vcount;   //TFT可见区域场扫描计数器
  output [DISP_DATA_W-1:0]      disp_data;        //待显示数据

  reg  [CHAR_ROM_ADDR_W-1:0]   rom_addra;        //读图片数据rom地址  
  wire                         h_exceed;
  wire                         v_exceed;
  wire                         char_h_disp;
  wire                         char_v_disp;
  wire                         char_disp;
  wire [15:0]                  hcount_max;
  reg  [DISP_CHAR_TOTAL_W-1:0] char_flag;

  //判断设置的显示的起始位置是否会导致显示超出范围
  assign h_exceed = char_disp_hbegin + DISP_CHAR_TOTAL_W > H_Visible_area - 1'b1;
  assign v_exceed = char_disp_vbegin + DISP_CHAR_TOTAL_H > V_Visible_area - 1'b1;
  //不同的设置情况，显示区域做不同的处理
  assign char_h_disp = h_exceed ? (visible_hcount >= char_disp_hbegin && visible_hcount < H_Visible_area):
                                  (visible_hcount >= char_disp_hbegin && visible_hcount < char_disp_hbegin + DISP_CHAR_TOTAL_W);  
  
  assign char_v_disp = v_exceed ? (visible_vcount >= char_disp_vbegin && visible_vcount < V_Visible_area):
                                  (visible_vcount >= char_disp_vbegin && visible_vcount < char_disp_vbegin + DISP_CHAR_TOTAL_H);
     
  assign char_disp = disp_data_req && char_h_disp && char_v_disp;
  
  assign hcount_max = h_exceed ? (H_Visible_area - 1'b1):(char_disp_hbegin + DISP_CHAR_TOTAL_W - 1'b1);
     
  always@(posedge clk_ctrl or negedge reset_n)
  begin
    if(!reset_n)
      rom_addra <= 'd0;
    else if(Frame_Begin)
      rom_addra <= 'd0; 
    else if((visible_hcount == hcount_max) && char_disp)
      rom_addra <= rom_addra + 1'b1;
    else
      rom_addra <= rom_addra;
  end

  always@(posedge clk_ctrl or negedge reset_n)
  begin
    if(!reset_n)
      char_flag <= 'd0;
    else if(char_disp)
      char_flag <= {char_flag[DISP_CHAR_TOTAL_W-2:0],1'b0};
    else
      char_flag <= rom_data;
  end

  assign disp_data = (char_disp && char_flag[DISP_CHAR_TOTAL_W-1]) ? disp_char_color : disp_back_color;

 endmodule