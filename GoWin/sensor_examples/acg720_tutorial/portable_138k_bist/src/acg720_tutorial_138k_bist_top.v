module acg720_tutorial_138k_bist_top (
    input  wire clk_i,
    input  wire reset_n_i,
    input  wire uart_rx_i,
    output wire uart_tx_o
);
    // The ACG720 tutorial uses an active-low reset on F15, while the proven
    // Tang Primer harness exposes an active-high reset input.
    acg720_tutorial_25k_bist_top u_portable_bist (
        .clk_i(clk_i),
        .reset_i(~reset_n_i),
        .uart_rx_i(uart_rx_i),
        .uart_tx_o(uart_tx_o)
    );
endmodule
