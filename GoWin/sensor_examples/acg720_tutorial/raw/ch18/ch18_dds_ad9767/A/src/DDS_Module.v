module DDS_Module(
    input Clk,
    input Reset_n,
    input [1:0]Mode_Sel,
    input [31:0]Fword,
    input [11:0]Pword,
    output reg[13:0]Data
);
    
    //频率控制字同步寄存器
    reg [31:0]Fword_r;
    always@(posedge Clk)
        Fword_r <= Fword;
    
    //相位控制字同步寄存器
    reg [11:0]Pword_r;
    always@(posedge Clk)
        Pword_r <= Pword; 
    
    //相位累加器    
    reg [31:0]Freq_ACC;
    always@(posedge Clk or negedge Reset_n)
    if(!Reset_n)
        Freq_ACC <= 0;
    else
        Freq_ACC <= Fword_r + Freq_ACC;

    //波形数据表地址
    reg [11:0]Rom_Addr;      
    always@(posedge Clk)
        Rom_Addr <= Freq_ACC[31:20] + Pword_r;

    wire [13:0]Data_sine,Data_square,Data_triangular;

    rom_sine rom_sine(
        .dout(Data_sine), //output [13:0] dout
        .clk(Clk), //input clk
        .oce(1'd1), //input oce
        .ce(1'd1), //input ce
        .reset(~Reset_n), //input reset
        .ad(Rom_Addr) //input [11:0] ad
    );

    rom_square rom_square(
        .dout(Data_square), //output [13:0] dout
        .clk(Clk), //input clk
        .oce(1'd1), //input oce
        .ce(1'd1), //input ce
        .reset(~Reset_n), //input reset
        .ad(Rom_Addr) //input [11:0] ad
    );

    rom_triangular rom_triangular(
        .dout(Data_triangular), //output [13:0] dout
        .clk(Clk), //input clk
        .oce(1'd1), //input oce
        .ce(1'd1), //input ce
        .reset(~Reset_n), //input reset
        .ad(Rom_Addr) //input [11:0] ad
    );

    always@(posedge Clk)
        case(Mode_Sel)
            0:Data = Data_sine;
            1:Data = Data_square;
            2:Data = Data_triangular;
            3:Data = 8192;
        endcase

    

endmodule