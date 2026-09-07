`timescale 1ns/1ps

module controller_top_tb;

    reg clk;
    reg rst_n;
    wire wrclk1;
    reg wrreq1;
    reg [7:0]wrdata1;

    reg [7:0]wrdata2;
    reg wrreq2;
    reg fifo1_aclr;
    reg fifo2_aclr;
    wire wrclk2;

    wire wrclk;
    wire fifo_aclr;
    wire fifo_wrreq;
    wire [7:0] fifo_wrdata;
    
    assign wrclk1 = clk;
    assign wrclk2 = clk;
    
    initial clk = 1;
    always#10 clk = ~clk;   
    
    controller_top controller_top(
        .clk(clk),
        .rst_n(rst_n),
        
        .wrclk1(clk),
        .fifo1_aclr(fifo1_aclr),
        .wrdata1(wrdata1),
        .wrreq1(wrreq1),
        
        .wrclk2(clk),
        .fifo2_aclr(fifo2_aclr),
        .wrdata2(wrdata2),
        .wrreq2(wrreq2),
        
        .wrclk(wrclk),
        .fifo_aclr(fifo_aclr),
        .fifo_wrreq(fifo_wrreq),
        .fifo_wrdata(fifo_wrdata)
    );
    
    initial begin
        rst_n = 0;
        wrreq1 = 0;
        wrreq2 = 0;
        fifo1_aclr = 1;
        fifo2_aclr = 1;
        #201;
        rst_n = 1;
        fifo1_aclr = 0;
        #3;
        fifo2_aclr = 0;
        #201;
        wrreq1 = 1;
        wrreq2 = 1;
        #30000;
        wrreq1 = 0;
        wrreq2 = 1;
        #30000;
        wrreq1 = 1;
        wrreq2 = 0;
        #30000;
        $stop;
    end
    
    always@(posedge clk)
    if(!rst_n)
        wrdata1 <=  0;
    else if(wrreq1)
        wrdata1 <= wrdata1 + 1'b1;
    else
        wrdata1 <= 0;
    
    always@(posedge clk)
    if(!rst_n)
        wrdata2 <=  0;
    else if(wrreq2)
        wrdata2 <= wrdata2 + 1'b1;
    else
        wrdata2 <= 0;
        
endmodule 
