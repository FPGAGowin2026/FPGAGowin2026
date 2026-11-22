module ddr3_pll (
    input wire clkin,
    input wire reset,
    input wire enclk0,
    input wire enclk1,
    input wire enclk2,
    output wire lock,
    output wire clkout0,
    output wire clkout1,
    output wire clkout2
);
    wire [7:0] mdrdo;
    wire [1:0] mdopc;
    wire mdainc;
    wire [7:0] mdwdi;
    wire pll_lock;
    wire pll_reset;

    ddr3_pll_mod u_pll (
        .clkout0(clkout0),
        .clkout1(clkout1),
        .clkout2(clkout2),
        .lock(pll_lock),
        .mdrdo(mdrdo),
        .clkin(clkin),
        .reset(pll_reset),
        .mdclk(clkin),
        .mdopc(mdopc),
        .mdainc(mdainc),
        .mdwdi(mdwdi)
    );

    PLL_INIT #(.CLK_PERIOD(20), .MULTI_FAC(16)) u_pll_init (
        .I_RST(reset),
        .O_RST(pll_reset),
        .I_LOCK(pll_lock),
        .O_LOCK(lock),
        .I_MD_CLK(clkin),
        .O_MD_INC(mdainc),
        .O_MD_OPC(mdopc),
        .O_MD_WR_DATA(mdwdi),
        .I_MD_RD_DATA(mdrdo),
        .PLL_INIT_BYPASS(1'b0),
        .MDRDO(),
        .MDOPC(2'b00),
        .MDAINC(1'b0),
        .MDWDI(8'h00)
    );
endmodule
