`timescale 1ns / 1ns
`define WR_CLK_PERIOD 10
`define RD_CLK_PERIOD 30

module fifo_ip_tb();
    reg [7:0]Data;
    reg Reset;
    reg WrClk;
    reg RdClk;
    reg WrEn;
    reg RdEn;
    wire [8:0] Wnum;
    wire [8:0] Rnum;
    wire Almost_Empty;
    wire Almost_Full;
    wire [7:0] Q;
    wire Empty;
    wire Full;

    initial WrClk = 1;
    always #(`WR_CLK_PERIOD/2) WrClk = ~WrClk;

    initial RdClk = 1;
    always #(`RD_CLK_PERIOD/2) RdClk = ~RdClk;

    initial begin
        Reset   = 1'b1;
        WrEn = 1'b0;
        RdEn = 1'b0;
        Data   = 8'hFF;
        #(`WR_CLK_PERIOD*8+1);
        Reset = 1'b0;
      
        //write data
        while(Full == 1'b0)
        begin
            @(posedge WrClk);
            #1;      
            WrEn = 1'b1;
            Data   = Data + 1'b1;
        end
        WrEn = 1'b0;
    
        while(Empty == 1'b0)
        begin      
            @(posedge RdClk);
            #1;
            RdEn = 1'b1;
        end
        RdEn = 1'b0; 
    
        //reset
        #200;
        Reset   = 1'b1;
        #(`WR_CLK_PERIOD*3+1);
        Reset = 1'b0;
    
        #2000;
        $stop;  
    end

	fifo_top fifo_top(
		.Data(Data), //input [7:0] Data
		.Reset(Reset), //input Reset
		.WrClk(WrClk), //input WrClk
		.RdClk(RdClk), //input RdClk
		.WrEn(WrEn), //input WrEn
		.RdEn(RdEn), //input RdEn
		.Wnum(Wnum), //output [8:0] Wnum
		.Rnum(Rnum), //output [8:0] Rnum
		.Almost_Empty(Almost_Empty), //output Almost_Empty
		.Almost_Full(Almost_Full), //output Almost_Full
		.Q(Q), //output [7:0] Q
		.Empty(Empty), //output Empty
		.Full(Full) //output Full
	);

endmodule