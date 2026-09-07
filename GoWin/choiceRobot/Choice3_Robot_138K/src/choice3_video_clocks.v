`timescale 1ns/1ps

// GW5AT-138B uses the Arora-V PLL primitive (not the 25K PLLA primitive).
// VCO = 50 MHz * 20 = 1000 MHz; outputs are 125 MHz, 25 MHz and 25 MHz.
// The 125:25 exact 5:1 ratio feeds OSER10 DVI; 25 MHz is a valid OV5640 XCLK.
module choice3_video_clocks (
    input wire clk_50m_i,input wire rst_n_i,
    output wire serial_125m_o,output wire pixel_25m_o,
    output wire camera_25m_o,output wire pll_locked_o
);
    wire gnd=1'b0,vcc=1'b1;
    wire c3,c4,c5,c6,clkfb_unused;
    PLL u_pll(
      .LOCK(pll_locked_o),.CLKOUT0(serial_125m_o),.CLKOUT1(pixel_25m_o),
      .CLKOUT2(camera_25m_o),.CLKOUT3(c3),.CLKOUT4(c4),.CLKOUT5(c5),
      .CLKOUT6(c6),.CLKFBOUT(clkfb_unused),.CLKIN(clk_50m_i),.CLKFB(gnd),
      .RESET(~rst_n_i),.PLLPWD(gnd),.RESET_I(gnd),.RESET_O(gnd),
      .FBDSEL(6'b0),.IDSEL(6'b0),.MDSEL(7'b0),.MDSEL_FRAC(3'b0),
      .ODSEL0(7'b0),.ODSEL0_FRAC(3'b0),.ODSEL1(7'b0),.ODSEL2(7'b0),
      .ODSEL3(7'b0),.ODSEL4(7'b0),.ODSEL5(7'b0),.ODSEL6(7'b0),
      .DT0(4'b0),.DT1(4'b0),.DT2(4'b0),.DT3(4'b0),
      .ICPSEL(6'b0),.LPFRES(3'b0),.LPFCAP(2'b0),
      .PSSEL(3'b0),.PSDIR(gnd),.PSPULSE(gnd),
      .ENCLK0(vcc),.ENCLK1(vcc),.ENCLK2(vcc),.ENCLK3(vcc),
      .ENCLK4(vcc),.ENCLK5(vcc),.ENCLK6(vcc),
      .SSCPOL(gnd),.SSCON(gnd),.SSCMDSEL(7'b0),.SSCMDSEL_FRAC(3'b0));
    defparam u_pll.FCLKIN="50";
    defparam u_pll.IDIV_SEL=1;
    defparam u_pll.FBDIV_SEL=1;
    defparam u_pll.MDIV_SEL=20;
    defparam u_pll.MDIV_FRAC_SEL=0;
    defparam u_pll.ODIV0_SEL=8;
    defparam u_pll.ODIV0_FRAC_SEL=0;
    defparam u_pll.ODIV1_SEL=40;
    defparam u_pll.ODIV2_SEL=40;
    defparam u_pll.ODIV3_SEL=8;
    defparam u_pll.ODIV4_SEL=8;
    defparam u_pll.ODIV5_SEL=8;
    defparam u_pll.ODIV6_SEL=8;
    defparam u_pll.CLKOUT0_EN="TRUE";
    defparam u_pll.CLKOUT1_EN="TRUE";
    defparam u_pll.CLKOUT2_EN="TRUE";
    defparam u_pll.CLKOUT3_EN="FALSE";
    defparam u_pll.CLKOUT4_EN="FALSE";
    defparam u_pll.CLKOUT5_EN="FALSE";
    defparam u_pll.CLKOUT6_EN="FALSE";
    defparam u_pll.CLKFB_SEL="INTERNAL";
    defparam u_pll.CLKOUT0_DT_DIR=1'b1;
    defparam u_pll.CLKOUT1_DT_DIR=1'b1;
    defparam u_pll.CLKOUT2_DT_DIR=1'b1;
    defparam u_pll.CLKOUT3_DT_DIR=1'b1;
    defparam u_pll.CLKOUT0_DT_STEP=0;
    defparam u_pll.CLKOUT1_DT_STEP=0;
    defparam u_pll.CLKOUT2_DT_STEP=0;
    defparam u_pll.CLKOUT3_DT_STEP=0;
    defparam u_pll.CLK0_IN_SEL=1'b0;
    defparam u_pll.CLK0_OUT_SEL=1'b0;
    defparam u_pll.CLK1_IN_SEL=1'b0;
    defparam u_pll.CLK1_OUT_SEL=1'b0;
    defparam u_pll.CLK2_IN_SEL=1'b0;
    defparam u_pll.CLK2_OUT_SEL=1'b0;
    defparam u_pll.CLK3_IN_SEL=1'b0;
    defparam u_pll.CLK3_OUT_SEL=1'b0;
    defparam u_pll.CLK4_IN_SEL=2'b00;
    defparam u_pll.CLK4_OUT_SEL=1'b0;
    defparam u_pll.CLK5_IN_SEL=1'b0;
    defparam u_pll.CLK5_OUT_SEL=1'b0;
    defparam u_pll.CLK6_IN_SEL=1'b0;
    defparam u_pll.CLK6_OUT_SEL=1'b0;
    defparam u_pll.DYN_DPA_EN="FALSE";
    defparam u_pll.CLKOUT0_PE_COARSE=0;
    defparam u_pll.CLKOUT0_PE_FINE=0;
    defparam u_pll.CLKOUT1_PE_COARSE=0;
    defparam u_pll.CLKOUT1_PE_FINE=0;
    defparam u_pll.CLKOUT2_PE_COARSE=0;
    defparam u_pll.CLKOUT2_PE_FINE=0;
    defparam u_pll.CLKOUT3_PE_COARSE=0;
    defparam u_pll.CLKOUT3_PE_FINE=0;
    defparam u_pll.CLKOUT4_PE_COARSE=0;
    defparam u_pll.CLKOUT4_PE_FINE=0;
    defparam u_pll.CLKOUT5_PE_COARSE=0;
    defparam u_pll.CLKOUT5_PE_FINE=0;
    defparam u_pll.CLKOUT6_PE_COARSE=0;
    defparam u_pll.CLKOUT6_PE_FINE=0;
    defparam u_pll.DYN_PE0_SEL="FALSE";
    defparam u_pll.DYN_PE1_SEL="FALSE";
    defparam u_pll.DYN_PE2_SEL="FALSE";
    defparam u_pll.DYN_PE3_SEL="FALSE";
    defparam u_pll.DYN_PE4_SEL="FALSE";
    defparam u_pll.DYN_PE5_SEL="FALSE";
    defparam u_pll.DYN_PE6_SEL="FALSE";
    defparam u_pll.DE0_EN="FALSE";
    defparam u_pll.DE1_EN="FALSE";
    defparam u_pll.DE2_EN="FALSE";
    defparam u_pll.DE3_EN="FALSE";
    defparam u_pll.DE4_EN="FALSE";
    defparam u_pll.DE5_EN="FALSE";
    defparam u_pll.DE6_EN="FALSE";
    defparam u_pll.RESET_I_EN="FALSE";
    defparam u_pll.RESET_O_EN="FALSE";
    defparam u_pll.ICP_SEL=6'bXXXXXX;
    defparam u_pll.LPF_RES=3'bXXX;
    defparam u_pll.LPF_CAP=2'b00;
    defparam u_pll.SSC_EN="FALSE";
    defparam u_pll.DYN_IDIV_SEL="FALSE";
    defparam u_pll.DYN_FBDIV_SEL="FALSE";
    defparam u_pll.DYN_MDIV_SEL="FALSE";
    defparam u_pll.DYN_ODIV0_SEL="FALSE";
    defparam u_pll.DYN_ODIV1_SEL="FALSE";
    defparam u_pll.DYN_ODIV2_SEL="FALSE";
    defparam u_pll.DYN_ODIV3_SEL="FALSE";
    defparam u_pll.DYN_ODIV4_SEL="FALSE";
    defparam u_pll.DYN_ODIV5_SEL="FALSE";
    defparam u_pll.DYN_ODIV6_SEL="FALSE";
    defparam u_pll.DYN_DT0_SEL="FALSE";
    defparam u_pll.DYN_DT1_SEL="FALSE";
    defparam u_pll.DYN_DT2_SEL="FALSE";
    defparam u_pll.DYN_DT3_SEL="FALSE";
    defparam u_pll.DYN_ICP_SEL="FALSE";
    defparam u_pll.DYN_LPF_SEL="FALSE";
endmodule
