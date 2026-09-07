`timescale 1ns/1ns
`define CLK_PERIOD 20

module ram_tb();
    reg reset;
    reg clk;
    reg wea;
    reg [7:0]ad;
    reg [7:0]dina;
    wire[7:0]doutb;
    integer i;

    GSR GSR(.GSRI(1'b1));

    Gowin_SP your_instance_name(
        .dout(doutb), //output [7:0] dout
        .clk(clk), //input clk
        .oce(), //input oce
        .ce(1), //input ce
        .reset(reset), //input reset
        .wre(wea), //input wre
        .ad(ad), //input [7:0] ad
        .din(dina) //input [7:0] din
    );


    initial clk = 1'b1;
    always #(`CLK_PERIOD/2) clk = ~clk;

initial begin
    wea=0;
    ad=0;
    dina=0;
    
    #(`CLK_PERIOD*10 +1 );
    wea=1;
    for (i=0;i<=15;i=i+1)begin    
        dina=255-i;
        ad = i;
    #`CLK_PERIOD;
    end
        wea=0;
        ad=0;
    #1;
    for (i=0;i<=15;i=i+1)begin
        ad=i;
        #`CLK_PERIOD;
    end
    #200;
    $stop;  
end

endmodule
