`timescale 1ns / 1ps
`define CLK_PERIOD 8

module gmii_to_rgmii_tb();
    reg  reset_n;
    reg gmii_tx_clk;
    reg [3:0] tx_byte_cnt;
    reg [7:0] gmii_txd;
    reg gmii_txen;
    reg gmii_txer;
    wire rgmii_tx_clk;
    wire [3:0] rgmii_txd;
    wire rgmii_txen;
	
	GSR GSR(.GSRI(1'b1));
    
    gmii_to_rgmii gmii_to_rgmii(
      .reset_n(reset_n),
      .gmii_tx_clk(gmii_tx_clk),
      .gmii_txd(gmii_txd),
      .gmii_txen(gmii_txen),
      .gmii_txer(gmii_txer),
    
      .rgmii_tx_clk(rgmii_tx_clk),
      .rgmii_txd(rgmii_txd),
      .rgmii_txen(rgmii_txen)
    );

        //clock generate
    initial gmii_tx_clk = 1'b1;
    always #(`CLK_PERIOD/2)gmii_tx_clk = ~gmii_tx_clk;
    
    always@(posedge gmii_tx_clk or negedge reset_n)
    if(!reset_n)
        tx_byte_cnt <= 4'd0;
    else if(gmii_txen)
        tx_byte_cnt <= tx_byte_cnt + 1'b1;
    else
        tx_byte_cnt <= 4'd0;

    always@(*)
    begin
        case(tx_byte_cnt)
        16'd0  : gmii_txd = 25;
        16'd1  : gmii_txd = 34;
        16'd2  : gmii_txd = 18;
        16'd3  : gmii_txd = 96;
        16'd4  : gmii_txd = 78;
        16'd5  : gmii_txd = 29;
        16'd6  : gmii_txd = 63;
        16'd7  : gmii_txd = 42;
        16'd8  : gmii_txd = 51;
        16'd9  : gmii_txd = 74;
        16'd10 : gmii_txd = 39;
        16'd11 : gmii_txd = 60;
        16'd12 : gmii_txd = 27;
        16'd13 : gmii_txd = 36;
        16'd14 : gmii_txd = 145;
        16'd15 : gmii_txd = 231;
        endcase
    end

  initial
    begin
    reset_n = 0;
    gmii_txen = 0;
    gmii_txer = 0;
    #201;
    reset_n = 1;
    gmii_txen = 1;
    #2000; 
    $stop;
  end

endmodule
