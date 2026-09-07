`timescale 1ns / 1ps

module DDS_Module_tb;
    
    reg Clk;
    reg  Reset_n;
    reg [31:0]FwordA,FwordB;
    reg [11:0]PwordA,PwordB;
	reg [1:0]Mode_SelA,Mode_SelB;
    wire [13:0]DataA,DataB;   
    
    DDS_Module DDS_ModuleA(
        .Clk(Clk),
        .Reset_n(Reset_n),
		.Mode_Sel(Mode_SelA),
        .Fword(FwordA),
        .Pword(PwordA),
        .Data(DataA)
    );
    
    DDS_Module DDS_ModuleB(
        .Clk(Clk),
        .Reset_n(Reset_n),
		.Mode_Sel(Mode_SelB),
        .Fword(FwordB),
        .Pword(PwordB),
        .Data(DataB)
    );
    
    initial Clk = 1;
    always #10 Clk = ~Clk;
    
    initial begin
        Reset_n = 0;
        FwordA = 65536;
        PwordA=0;
        FwordB = 65536;
        PwordB=1024;
		Mode_SelA=2'b00;
		Mode_SelB=2'b00;
        #201;
        Reset_n = 1;
        #5000000;
        FwordA = 65536 *1024;  
        FwordB = 65536 *1024; 
        PwordA=0;
        PwordB=2048;
        #1000000;
        $stop;        
    end
    
    
endmodule
