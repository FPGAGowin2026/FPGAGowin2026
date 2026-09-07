`timescale 1ns/1ps

// 仅检查 Gowin 原语、官方 PLL 与板级顶层能够共同展开；不替代功能或时序验证。
module tb_acg720_udp_loopback_project;
    reg reset_n = 1'b0;
    reg rgmii_rx_clk_i = 1'b0;
    reg [3:0] rgmii_rxd = 4'd0;
    reg rgmii_rxdv = 1'b0;
    wire eth_rst_n;
    wire rgmii_tx_clk;
    wire [3:0] rgmii_txd;
    wire rgmii_txen;

    GSR GSR(.GSRI(1'b1));

    eth_udp_loopback_rgmii dut (
        .reset_n(reset_n),
        .rgmii_rx_clk_i(rgmii_rx_clk_i),
        .rgmii_rxd(rgmii_rxd),
        .rgmii_rxdv(rgmii_rxdv),
        .eth_rst_n(eth_rst_n),
        .rgmii_tx_clk(rgmii_tx_clk),
        .rgmii_txd(rgmii_txd),
        .rgmii_txen(rgmii_txen)
    );

    always #4 rgmii_rx_clk_i = ~rgmii_rx_clk_i;

    initial begin
        #32;
        $display("PASS: ACG720 Gowin PLL/RGMII 顶层展开通过");
        $finish;
    end
endmodule
