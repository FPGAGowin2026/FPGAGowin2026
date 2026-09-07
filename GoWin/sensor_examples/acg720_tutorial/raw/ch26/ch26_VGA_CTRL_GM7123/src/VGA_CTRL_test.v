module VGA_CTRL_test(
	Clk, 	//50MHZ时钟
	Rst_n,
	VGA_RGB,//TFT数据输出
	VGA_HS,	//TFT行同步信号
	VGA_VS,	//TFT场同步信号
	VGA_BLK,		//VGA 场消隐信号
	VGA_CLK		//VGA DAC输出时钟
);

	input Clk;
	input Rst_n;
	output [23:0]VGA_RGB;
	output VGA_HS;
	output VGA_VS;
	output VGA_BLK;		//VGA 场消隐信号
	output VGA_CLK;		//VGA DAC输出时钟
	
	reg [23:0]disp_data;
	wire [9:0]hcount;
	wire [9:0]vcount;
	wire Clk25M;
	
	Gowin_PLL Gowin_PLL(
        .clkout0(Clk25M), //output clkout0
        .clkin(Clk) //input clkin
    );

	VGA_CTRL VGA_CTRL(
		.Clk25M(Clk25M),	//系统输入时钟25MHZ
		.Rst_n(Rst_n),
		.data_in(disp_data),	//待显示数据
		.hcount(hcount),		//VGA行扫描计数器
		.vcount(vcount),		//VGA场扫描计数器
		.VGA_RGB(VGA_RGB),	//VGA数据输出
		.VGA_HS(VGA_HS),		//VGA行同步信号
		.VGA_VS(VGA_VS),		//VGA场同步信号
		.VGA_BLK(VGA_BLK),		//VGA 场消隐信号
		.VGA_CLK(VGA_CLK)	//VGA DAC输出时钟
	);
	
	
//定义颜色编码
localparam 
	BLACK		= 24'h000000, //黑色
	BLUE		= 24'h0000FF, //蓝色
	RED		= 24'hFF0000, //红色
	PURPPLE	= 24'hFF00FF, //紫色
	GREEN		= 24'h00FF00, //绿色
	CYAN		= 24'h00FFFF, //青色
	YELLOW	= 24'hFFFF00, //黄色
	WHITE		= 24'hFFFFFF; //白色
	
//定义每个像素块的默认显示颜色值
localparam 
	R0_C0 = BLACK,	//第0行0列像素块
	R0_C1 = BLUE,	//第0行1列像素块
	R1_C0 = RED,	//第1行0列像素块
	R1_C1 = PURPPLE,//第1行1列像素块
	R2_C0 = GREEN,	//第2行0列像素块
	R2_C1 = CYAN,	//第2行1列像素块
	R3_C0 = YELLOW,	//第3行0列像素块
	R3_C1 = WHITE;	//第3行1列像素块

	wire R0_act = vcount >= 0 && vcount < 120;	//正在扫描第0行
	wire R1_act = vcount >= 120 && vcount < 240;//正在扫描第1行
	wire R2_act = vcount >= 240 && vcount < 360;//正在扫描第2行
	wire R3_act = vcount >= 360 && vcount < 480;//正在扫描第3行
	
	wire C0_act = hcount >= 0 && hcount < 320; //正在扫描第0列
	wire C1_act = hcount >= 320 && hcount < 640;//正在扫描第1列 
	
	wire R0_C0_act = R0_act & C0_act;	//第0行0列像素块处于被扫描中标志信号
	wire R0_C1_act = R0_act & C1_act;	//第0行1列像素块处于被扫描中标志信号
	wire R1_C0_act = R1_act & C0_act;	//第1行0列像素块处于被扫描中标志信号
	wire R1_C1_act = R1_act & C1_act;	//第1行1列像素块处于被扫描中标志信号
	wire R2_C0_act = R2_act & C0_act;	//第2行0列像素块处于被扫描中标志信号
	wire R2_C1_act = R2_act & C1_act;	//第2行1列像素块处于被扫描中标志信号
	wire R3_C0_act = R3_act & C0_act;	//第3行0列像素块处于被扫描中标志信号
	wire R3_C1_act = R3_act & C1_act;	//第3行1列像素块处于被扫描中标志信号
	
	always@(*)
		case({R3_C1_act,R3_C0_act,R2_C1_act,R2_C0_act,
				R1_C1_act,R1_C0_act,R0_C1_act,R0_C0_act})
			8'b0000_0001:disp_data = R0_C0;
			8'b0000_0010:disp_data = R0_C1;
			8'b0000_0100:disp_data = R1_C0;
			8'b0000_1000:disp_data = R1_C1;
			8'b0001_0000:disp_data = R2_C0;
			8'b0010_0000:disp_data = R2_C1;
			8'b0100_0000:disp_data = R3_C0;
			8'b1000_0000:disp_data = R3_C1;
			default:disp_data = R0_C0;
		endcase
		
endmodule
