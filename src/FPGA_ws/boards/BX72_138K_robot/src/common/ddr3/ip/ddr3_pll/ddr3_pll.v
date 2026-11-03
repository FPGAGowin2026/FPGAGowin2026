// BX72 GW5AT138C PLL with reference-example analog initialization.
module ddr3_pll(input clkin, reset, enclk0, enclk1, enclk2,
    output lock, clkout0, clkout1, clkout2);
    wire pll_lock, pll_reset;
    wire [5:0] icpsel;
    wire [2:0] lpfres;
    assign clkout2 = 1'b0; // unused by DDR PHY and application
    ddr3_pll_mod u_pll(.lock(pll_lock), .clkout0(clkout0),
        .clkout1(clkout1), .clkin(clkin), .reset(pll_reset),
        .icpsel(icpsel), .lpfres(lpfres), .lpfcap(2'b00), .enclk1(enclk1));
    PLL_INIT #(.CLK_PERIOD(20), .MULTI_FAC(16)) u_pll_init(
        .CLKIN(clkin), .I_RST(reset), .O_RST(pll_reset),
        .PLLLOCK(pll_lock), .O_LOCK(lock), .ICPSEL(icpsel), .LPFRES(lpfres));
endmodule
