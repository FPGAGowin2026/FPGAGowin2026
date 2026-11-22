// 1000BASE-T: low nibble on the rising edge, high nibble on the falling edge.
// RTL8211F TXID supplies the receive-clock delay; forward the same 125 MHz clock.
module gmii_to_rgmii (
    input wire gmii_tx_clk,
    input wire [7:0] gmii_txd,
    input wire gmii_txen, gmii_txer,
    output wire rgmii_tx_clk,
    output wire [3:0] rgmii_txd,
    output wire rgmii_txen
);
    genvar lane;
    generate
        for (lane = 0; lane < 4; lane = lane + 1) begin : data_lane
            ODDR data_ddr (
                .Q0(rgmii_txd[lane]), .Q1(), .TX(1'b1), .CLK(gmii_tx_clk),
                .D0(gmii_txd[lane]), .D1(gmii_txd[lane+4])
            );
        end
    endgenerate

    ODDR control_ddr (
        .Q0(rgmii_txen), .Q1(), .TX(1'b1), .CLK(gmii_tx_clk),
        .D0(gmii_txen), .D1(gmii_txen ^ gmii_txer)
    );
    ODDR clock_ddr (
        .Q0(rgmii_tx_clk), .Q1(), .TX(1'b1), .CLK(gmii_tx_clk),
        .D0(1'b1), .D1(1'b0)
    );
endmodule
