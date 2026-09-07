`timescale 1ns/1ps

// 同一 PLL VCO 约1006.25 MHz：CLKOUT0 /8≈125.781 MHz，再 /5 得
// 25.156 MHz 像素时钟；CLKOUT1 /42≈23.958 MHz 直接作为 OV5640 XCLK。
module choice3_video_clocks (
    input wire clk_50m_i,input wire rst_n_i,
    output wire serial_125m_o,output wire pixel_25m_o,
    output wire camera_24m_o,output wire pll_locked_o
);
    wire gnd=1'b0;
    wire [7:0] mdrdo_unused;
    wire clkfb_unused;
    wire c2,c3,c4,c5,c6;
    PLLA u_pll(
      .LOCK(pll_locked_o),.CLKOUT0(serial_125m_o),.CLKOUT1(camera_24m_o),.CLKOUT2(c2),
      .CLKOUT3(c3),.CLKOUT4(c4),.CLKOUT5(c5),.CLKOUT6(c6),
      .CLKFBOUT(clkfb_unused),.MDRDO(mdrdo_unused),.CLKIN(clk_50m_i),.CLKFB(gnd),
      .RESET(~rst_n_i),.PLLPWD(gnd),.RESET_I(gnd),.RESET_O(gnd),
      .PSSEL(3'b000),.PSDIR(gnd),.PSPULSE(gnd),.SSCPOL(gnd),.SSCON(gnd),
      .SSCMDSEL(7'b0),.SSCMDSEL_FRAC(3'b0),.MDCLK(gnd),.MDOPC(2'b0),
      .MDAINC(gnd),.MDWDI(8'b0));
    defparam u_pll.FCLKIN="50";
    defparam u_pll.IDIV_SEL=1;
    defparam u_pll.FBDIV_SEL=1;
    defparam u_pll.CLKFB_SEL="INTERNAL";
    defparam u_pll.ODIV0_SEL=8;
    defparam u_pll.ODIV0_FRAC_SEL=0;
    defparam u_pll.ODIV1_SEL=42;
    defparam u_pll.MDIV_SEL=20;
    defparam u_pll.MDIV_FRAC_SEL=1;
    defparam u_pll.CLKOUT0_EN="TRUE";
    defparam u_pll.CLKOUT1_EN="TRUE";
    defparam u_pll.CLKOUT2_EN="FALSE";
    defparam u_pll.CLKOUT3_EN="FALSE";
    defparam u_pll.CLKOUT4_EN="FALSE";
    defparam u_pll.CLKOUT5_EN="FALSE";
    defparam u_pll.CLKOUT6_EN="FALSE";
    defparam u_pll.RESET_I_EN="FALSE";
    defparam u_pll.RESET_O_EN="FALSE";
    defparam u_pll.SSC_EN="FALSE";

    CLKDIV u_pixel_div(.CLKOUT(pixel_25m_o),.HCLKIN(serial_125m_o),
      .RESETN(pll_locked_o && rst_n_i),.CALIB(gnd));
    defparam u_pixel_div.DIV_MODE="5";
endmodule
