`timescale 1ns / 1ps

module DDS_AD9767_tb;
    
    reg Clk;
    reg  Reset_n;
    reg  key_ch;
    reg  key_fre;
    reg  key_wave;
	
	wire [1:0] led;
	wire sh_cp;
	wire st_cp;
	wire ds;
    wire  [13:0]Data0;
	wire  [13:0]Data1;
	wire  Clk0;
	wire  WRT0;
	wire  Clk1;
	wire  WRT1;
    		   
	GSR GSR(.GSRI(1'b1));
	
    dds_ad9767 dds_ad9767(
        .clk_50M(Clk),
        .key_ch(key_ch),
        .key_fre(key_fre),
		.key_wave(key_wave),
		.led(led),
		.sh_cp(sh_cp),
		.st_cp(st_cp),
		.ds(ds),
        .Data0(Data0),
        .Clk0(Clk0),
        .WRT0(WRT0),
        .Data1(Data1),
        .WRT1(WRT1),
        .Clk1(Clk1)
    );   
    
    initial Clk = 1;
    always #10 Clk = ~Clk;
		   
    initial begin
        key_ch = 0;
		key_fre = 0;
		key_wave = 0;
        #1000000;
        $stop;    
    end
    
    
endmodule
