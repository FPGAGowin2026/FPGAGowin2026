module rom_top(
    clk,
    reset_n,
    dout
);
    input clk;
    input reset_n;
    output wire [7:0]dout;

    reg [9:0]addr;
    always@(posedge clk or negedge reset_n)
    if(!reset_n)
        addr<=0;
    else
        addr<=addr+1'b1;

    Gowin_pROM Gowin_pROM(
        .dout(dout), //output [7:0] dout
        .clk(clk), //input clk
        .oce(1'b1), //input oce
        .ce(1'b1), //input ce
        .reset(~reset_n), //input reset
        .ad(addr) //input [7:0] ad
    );

endmodule