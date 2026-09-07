`timescale 1ns / 1ps
`define CLK_PERIOD 8

module rgmii_to_gmii_tb();
    reg  reset_n;
    wire gmii_rx_clk;
    reg [3:0] rx_byte_cnt;
    wire [7:0] gmii_rxd;
    wire gmii_rxdv;
    wire gmii_rxerr;
    wire rgmii_rx_clk;
    reg [3:0] rgmii_rxd;
    reg rgmii_rxdv;
    reg rx_clk;
    wire locked;
    
    GSR GSR(.GSRI(1'b1));
    rgmii_to_gmii rgmii_to_gmii(
        .reset(!reset_n),
        .gmii_rx_clk(gmii_rx_clk),  
        .gmii_rxdv(gmii_rxdv),
        .gmii_rxd(gmii_rxd),
        .gmii_rxer(gmii_rxerr),
    
        .rgmii_rx_clk(rgmii_rx_clk),
        .rgmii_rxd(rgmii_rxd),
        .rgmii_rxdv(rgmii_rxdv)
    );

    Gowin_PLL Gowin_PLL(
        .lock(locked), //output lock
        .clkout0(rgmii_rx_clk), //output clkout0
        .clkin(rx_clk), //input clkin
        .reset(!reset_n) //input reset
    );

    //clock generate
    initial rx_clk = 1'b1;
    always #(`CLK_PERIOD/2)rx_clk = ~rx_clk;
    
    always@(rx_clk or negedge reset_n)
    if(!reset_n)
        rx_byte_cnt <= 4'd0;
    else if(rgmii_rxdv && locked)
        rx_byte_cnt <= rx_byte_cnt + 1'b1;
    else
        rx_byte_cnt <= 4'd0;

    always@(*)
    begin
        case(rx_byte_cnt)
        16'd0  : rgmii_rxd = 12;
        16'd1  : rgmii_rxd = 7;
        16'd2  : rgmii_rxd = 9;
        16'd3  : rgmii_rxd = 6;
        16'd4  : rgmii_rxd = 11;
        16'd5  : rgmii_rxd = 15;
        16'd6  : rgmii_rxd = 0;
        16'd7  : rgmii_rxd = 8;
        16'd8  : rgmii_rxd = 4;
        16'd9  : rgmii_rxd = 2;
        16'd10 : rgmii_rxd = 5;
        16'd11 : rgmii_rxd = 1;
        16'd12 : rgmii_rxd = 3;
        16'd13 : rgmii_rxd = 10;
        16'd14 : rgmii_rxd = 14;
        16'd15 : rgmii_rxd = 13;
        endcase
    end

  initial
    begin
    reset_n = 0;
    rgmii_rxdv = 0;
    #201;
    reset_n = 1;
    rgmii_rxdv = 1;
    @(posedge locked)
    #2000; 
    $stop;
  end

endmodule
