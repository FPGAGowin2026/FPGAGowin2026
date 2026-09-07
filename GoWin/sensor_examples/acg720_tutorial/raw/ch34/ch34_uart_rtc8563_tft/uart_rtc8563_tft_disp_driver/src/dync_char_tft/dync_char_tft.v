/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: dync_char_tft
// Module Name: dync_char_tft
// Project Name: dync_char_tft
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

module dync_char_tft
#(
  parameter TFT_WIDTH  = 800,
  parameter TFT_HEIGHT = 480 
)
(
  clk,
  reset_n,
  tft_ctrl_clk,

  char_disp_hbegin,
  char_disp_vbegin,
  disp_back_color,
  disp_char_color,  
  scan_en_pulse,
  disp_data,

	TFT_rgb,
	TFT_hs,
	TFT_vs,
	TFT_clk,
	TFT_de,
	TFT_pwm
);
  input         clk;         //模块工作时钟输入
	input         reset_n;     //复位信号输入，低有效
  input         tft_ctrl_clk;//TFT屏控制时钟

  input  [15:0] char_disp_hbegin; //待显示图片左上角第一个像素点在TFT屏的行向坐标
  input  [15:0] char_disp_vbegin; //待显示图片左上角第一个像素点在TFT屏的场向坐标
  input  [15:0] disp_back_color;  //显示的背景颜色
  input  [15:0] disp_char_color;  //显示字符的颜色  
  input         scan_en_pulse;
  input  [39:0] disp_data; 
  
	output [15:0] TFT_rgb;  //TFT数据输出
	output        TFT_hs;   //TFT行同步信号
	output        TFT_vs;   //TFT场同步信号
	output        TFT_clk;  //TFT像素时钟
	output        TFT_de;   //TFT数据使能
	output        TFT_pwm;  //TFT背光控制

	wire [15:0]  tft_disp_data;
  wire         tft_disp_data_req;
	wire [11:0]  visible_hcount;
	wire [11:0]  visible_vcount;
  
  wire [8:0]   extr_data_addr;
  wire [15:0]  extr_data;
  wire [15:0]  update_data;
  wire [9:0]   update_data_wr;
  wire [4:0]   update_data_addr;
  wire [4:0]   ram_addrb;
  wire [159:0] ram_doutb;

//	assign sys_reset_n = pll_locked;
	
	wire tft_reset_p;
    wire [4:0]Disp_Red;
    wire [5:0]Disp_Green;
    wire [4:0]Disp_Blue;
    wire [15:0]TFT_rgb;
    wire Frame_Begin;  

    num_lib_rom num_lib_rom(
        .dout(extr_data), //output [15:0] dout
        .clk(clk), //input clk
        .oce(1'd1), //input oce
        .ce(1'd1), //input ce
        .reset(~reset_n), //input reset
        .ad(extr_data_addr) //input [8:0] ad
    );


assign update_data = extr_data;

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

genvar i;
generate
  for (i=0; i < 10; i=i+1)
  begin: num_dpram
    Gowin_SDPB Gowin_SDPB(
        .dout(ram_doutb[16*(i+1)-1:16*i]), //output [15:0] dout
        .clka(clk), //input clka
        .cea(update_data_wr[i] ), //input cea
        .clkb(tft_ctrl_clk), //input clkb
        .ceb(1'd1), //input ceb
        .oce(1'd1), //input oce
        .reset(~reset_n), //input reset
        .ada(update_data_addr), //input [4:0] ada
        .din(update_data), //input [15:0] din
        .adb(ram_addrb) //input [4:0] adb
    );
  end
endgenerate

  char_extract
  #(
    .H_Visible_area   (TFT_WIDTH ), //屏幕显示区域宽度
    .V_Visible_area   (TFT_HEIGHT), //屏幕显示区域高度
    .CHAR_WIDTH       (   16     ), //单个字符显示宽度
    .CHAR_HEIGHT      (   32     ), //单个字符显示高度
    .ROW_DISP_CHAR_NUM(   10     ), //一行显示字符个数
    .COL_DISP_CHAR_NUM(   1      ), //显示字符行数
    .CHAR_ROM_ADDR_W  (   5      ), //存储字符ROM地址位宽，log2(CHAR_HEIGHT * COL_DISP_CHAR_NUM)
    .DISP_DATA_W      (   16     )  //图片像素点数据位宽
  )char_extract
  (
    .clk_ctrl        (tft_ctrl_clk     ),
    .reset_n         (reset_n          ),
    .char_disp_hbegin(char_disp_hbegin ),
    .char_disp_vbegin(char_disp_vbegin ),
    .disp_back_color (disp_back_color  ),
    .disp_char_color (disp_char_color  ),
    .rom_addra       (ram_addrb        ),
    .rom_data        (ram_doutb        ),
    .Frame_Begin     (Frame_Begin      ),
    .disp_data_req   (tft_disp_data_req),
    .visible_hcount  (visible_hcount   ),
    .visible_vcount  (visible_vcount   ),
    .disp_data       (tft_disp_data    )
  );

	disp_driver disp_driver
	(
		.ClkDisp(tft_ctrl_clk),    //注意是用tft_ctrl_clk还是clk_ctrl,在例化中，上层括号文件名，本实验的时钟信号名称与前序章节的例化名称不同
		.Rst_p(tft_reset_p),

		.Data(tft_disp_data),//串信号预警，从彩条和ROM、字符静态复制过来的例化使用的是disp_data，与data_scan_ctrl有相同的变量名称，含义完全不同
		.DataReq(tft_disp_data_req),//串信号预警，从彩条和ROM、字符静态复制过来的例化使用的是disp_data_req，与data_scan_ctrl有相同的变量名称，含义完全不同

		.H_Addr(visible_hcount),
		.V_Addr(visible_vcount),
                            
		.Disp_HS(TFT_hs),
		.Disp_VS(TFT_vs),
		.Disp_Red(Disp_Red),
		.Disp_Green(Disp_Green),
		.Disp_Blue(Disp_Blue),
		.Frame_Begin(Frame_Begin),
                            
		.Disp_DE(TFT_de),
		.Disp_PCLK(TFT_clk)
	);

//	assign tft_reset_p = ~pll_locked; //锁相环提供的TFT屏复位信号进行取反，满足高电平复位
	assign tft_reset_p=~reset_n;//慎重使用本行代码！锁相环提供的TFT屏复位信号，传递到本层，经过了名称变化，如果锁相环在本层，请参考上一行代码
	assign TFT_rgb={Disp_Red,Disp_Green,Disp_Blue};
//	assign TFT_clk=Disp_PCLK;信号转换，按各上层文件的变量声明情况决定变量名称和取舍
	assign TFT_pwm=1'b1;
	
	

endmodule
