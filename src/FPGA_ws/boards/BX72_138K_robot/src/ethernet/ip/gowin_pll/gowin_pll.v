module Gowin_PLL(output lock, clkout0, clkout1, clkout2, clkout3,
    input clkin, reset);
    wire pll_lock, pll_reset;
    wire [5:0] icpsel;
    wire [2:0] lpfres;
    bx72_video_pll_mod u_pll(.lock(pll_lock), .clkout0(clkout0),
        .clkout1(clkout1), .clkout2(clkout2), .clkout3(clkout3),
        .clkin(clkin), .reset(pll_reset), .icpsel(icpsel),
        .lpfres(lpfres), .lpfcap(2'b00));
    PLL_INIT #(.CLK_PERIOD(20), .MULTI_FAC(20)) u_pll_init(
        .CLKIN(clkin), .I_RST(reset), .O_RST(pll_reset),
        .PLLLOCK(pll_lock), .O_LOCK(lock), .ICPSEL(icpsel), .LPFRES(lpfres));
endmodule
