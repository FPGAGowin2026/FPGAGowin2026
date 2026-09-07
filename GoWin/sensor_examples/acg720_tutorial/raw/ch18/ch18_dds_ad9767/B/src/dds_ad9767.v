module dds_ad9767(
    input clk_50M,
    //按键功能说明
    input key_ch,    //控制通道切换
    input key_fre,   //频率切换
    input key_wave,  //波形切换

    output [1:0] led,

    output sh_cp,
	output st_cp,
	output ds,
    
    output reg [13:0]Data0,
    output Clk0,
    output WRT0,
    output reg [13:0]Data1,
    output Clk1,
    output WRT1
);
    
    wire CLK125M;
    wire pll_locked;

    reg [31:0]Fword_A;
    reg [31:0]Fword_B;

    reg [11:0]Pword_A;
    reg [11:0]Pword_B;

    wire [13:0] Data_A;
    wire [13:0] Data_B;

    wire  [1:0] Mode_Sel_A;
    wire  [1:0] Mode_Sel_B;

    wire [2:0] channel;

    wire [2:0] CH_Fword_Sel_A;
    wire [2:0] CH_Fword_Sel_B;

    wire [2:0] CH_Pword_Sel_A;
    wire [2:0] CH_Pword_Sel_B;
    
    reg [31:0]disp_data;
	wire [7:0] sel;//数码管位选（选择当前要显示的数码管）
	wire [6:0] seg;//数码管段选（当前要显示的内容)

    assign CH_Pword_Sel_A = 3'd0;
    assign CH_Pword_Sel_B = 3'd0;

    assign Reset_n = pll_locked;

    Gowin_PLL Gowin_PLL(
        .clkout0(CLK125M), //output clkout0
        .clkin(clk_50M), //input clkin
        .lock(pll_locked)
    );

    assign Clk0 = CLK125M;
    assign Clk1 = CLK125M;
    assign WRT0 = CLK125M;
    assign WRT1 = CLK125M;  

   DDS_Module DDS_Module_A(
        .Clk(CLK125M),
        .Reset_n(Reset_n),  
        .Mode_Sel(Mode_Sel_A),
        .Fword(Fword_A),
        .Pword(Pword_A),
        .Data(Data_A)
    );

   DDS_Module DDS_Module_B(
        .Clk(CLK125M),
        .Reset_n(Reset_n),  
        .Mode_Sel(Mode_Sel_B),
        .Fword(Fword_B),
        .Pword(Pword_B),
        .Data(Data_B)
    );

    key_control key_control
    (
        .CLK125M(CLK125M),
        .Reset_n(Reset_n),
        .key_ch(key_ch),
        .key_fre(key_fre),
        .key_wave(key_wave),
//        .channel(channel),   
//        .CH_Fword_Sel_A(CH_Fword_Sel_A),     //仿真时注释
//        .CH_Fword_Sel_B(CH_Fword_Sel_B),
//        .Mode_Sel_A(Mode_Sel_A),
//        .Mode_Sel_B(Mode_Sel_B),
        .led(led)
    );
        //仿真时使用
    assign CH_Fword_Sel_A = 3'd4;
    assign Mode_Sel_A = 2'd0;
    assign CH_Fword_Sel_B = 3'd4;
    assign Mode_Sel_B = 2'd1;
    assign channel = 3'd0;


    reg [13:0] Data0_r0;
    reg [13:0] Data0_r1;
    reg [13:0] Data0_r2;

    reg [13:0] Data1_r0;
    reg [13:0] Data1_r1;
    reg [13:0] Data1_r2;

    always@(posedge CLK125M)
    begin
        Data0_r0 <= Data_A;
        Data0_r1 <= Data0_r0;
        Data0_r2 <= Data0_r1;
        Data0 <= Data0_r2;
        Data1_r0 <= Data_B;
        Data1_r1 <= Data1_r0;
        Data1_r2 <= Data1_r1;
        Data1 <= Data1_r2;
    end

    //频率控制字
    //如果把周期完整的一个波形等分成2的32次方份，在时钟频率为125M次/秒的条件下,
    //如果希望1秒钟输出一个完成的周期，那么每一拍递进多少份？
    always@(posedge CLK125M)
        case(CH_Fword_Sel_A)   
            0:Fword_A = 34;//2**32 / 1250000000;     34.35  
            1:Fword_A = 344;//2**32 / 125000000;
            2:Fword_A = 3436;//2**32 / 12500000;
            3:Fword_A = 34360;//2**32 / 1250000;
            4:Fword_A = 343597;//2**32 / 125000;
            5:Fword_A = 3435974;//2**32 / 12500;
            6:Fword_A = 34359738;//2**32 / 1250;
            7:Fword_A = 343597384;//2**32 / 125;
        endcase
        
    always@(posedge CLK125M)
        case(CH_Fword_Sel_B)   
            0:Fword_B = 34;//2**32 / 1250000000;     34.35
            1:Fword_B = 344;//2**32 / 125000000;
            2:Fword_B = 3436;//2**32 / 12500000;
            3:Fword_B = 34360;//2**32 / 1250000;
            4:Fword_B = 343597;//2**32 / 125000;
            5:Fword_B = 3435974;//2**32 / 12500;
            6:Fword_B = 34359738;//2**32 / 1250;
            7:Fword_B = 343597384;//2**32 / 125;
        endcase  
        
    always@(posedge CLK125M)
        case(CH_Pword_Sel_A)
            0:Pword_A = 0;   //0
            1:Pword_A = 341; //30
            2:Pword_A = 683; //60
            3:Pword_A = 1024;    //90
            4:Pword_A = 1707;    //150
            5:Pword_A = 2048;    //180
            6:Pword_A = 3072;    //270
            7:Pword_A = 3641;    //320
        endcase 

    always@(posedge CLK125M)
        case(CH_Pword_Sel_B)
            0:Pword_B = 0;   //0
            1:Pword_B = 341; //30
            2:Pword_B = 683; //60
            3:Pword_B = 1024;    //90
            4:Pword_B = 1707;    //150
            5:Pword_B = 2048;    //180
            6:Pword_B = 3072;    //270
            7:Pword_B = 3641;    //320
        endcase 
        
    //数码管显示频率

    always@(posedge CLK125M)
        case(channel)
            0: begin
                    case(CH_Fword_Sel_A)
                        0: disp_data = 32'h0000_0001;
                        1: disp_data = 32'h0000_0010;
                        2: disp_data = 32'h0000_0100;
                        3: disp_data = 32'h0000_1000;
                        4: disp_data = 32'h0001_0000;
                        5: disp_data = 32'h0010_0000;
                        6: disp_data = 32'h0100_0000;
                        7: disp_data = 32'h1000_0000;
                    endcase
               end
            1: begin
                   case(CH_Fword_Sel_B)
                        0: disp_data = 32'h0000_0001;
                        1: disp_data = 32'h0000_0010;
                        2: disp_data = 32'h0000_0100;
                        3: disp_data = 32'h0000_1000;
                        4: disp_data = 32'h0001_0000;
                        5: disp_data = 32'h0010_0000;
                        6: disp_data = 32'h0100_0000;
                        7: disp_data = 32'h1000_0000;
                    endcase
               end
        endcase

	hc595_driver hc595_driver(
		.clk(clk_50M),
		.reset_n(Reset_n),
		.data({1'd1,seg,sel}),
		.s_en(1'b1),
		.sh_cp(sh_cp),
		.st_cp(st_cp),
		.ds(ds)
	);
    

	hex8 hex8(
		.clk(clk_50M),
		.reset_n(Reset_n),
		.en(1'b1),
		.disp_data(disp_data),
		.sel(sel),
		.seg(seg)
	);

endmodule