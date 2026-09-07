module hdmi_color(
	input clk,
	input rst_n,
	
    output led,
	//hdmi output
	output tmds_clk_p,
	output tmds_clk_n,
	output [2:0] tmds_data_p, //rgb
	output [2:0] tmds_data_n //rgb
);

	wire clk_vga;//74.25m
	wire clk_vgax5;//371.25m
    wire pll_lock;
    wire disp_data_req;
    wire hdmi4_rst_n;

    //Resolution_800*480   1 //时钟为MHz
    parameter DISP_WIDTH  = 800;
    parameter DISP_HEIGHT = 480;

    Gowin_PLL Gowin_PLL(
        .lock(pll_lock), //output lock
        .clkout0(clk_vga), //output clkout0
        .clkout1(clk_vgax5), //output clkout1
        .clkin(clk) //input clkin
    );


    assign led = pll_lock;

	wire [11:0] hcount;
	wire [11:0] vcount;
	wire [23:0] disp_data;
	color_bar #(
    .DISP_WIDTH  ( DISP_WIDTH  ),
    .DISP_HEIGHT ( DISP_HEIGHT )
  )colour_bar_dat_gen(
    .disp_h_addr   (hcount  ),
    .disp_v_addr   (vcount  ),

    .disp_data_req (disp_data_req),
    .disp_data     (disp_data    )
  );
 
	wire video_hs;
	wire video_vs;
	wire video_de;
	wire [7:0] video_r;
	wire [7:0] video_g;
	wire [7:0] video_b;
 
	disp_driver disp_driver(
		.ClkDisp(clk_vga),
		.Rst_n(rst_n),
		.Data(disp_data),
		.DataReq(disp_data_req),
		.H_Addr(hcount),
		.V_Addr(vcount),
		.Disp_HS(video_hs),
		.Disp_VS(video_vs),
		.Disp_Red(video_r),
		.Disp_Green(video_g),
		.Disp_Blue(video_b),
		.Disp_DE(video_de),
		.Disp_PCLK()
	);
	
	dvi_encoder u_dvi_encoder(
		.pixelclk(clk_vga),// system clock
		.pixelclk5x(clk_vgax5),// system clock x5
		.rst_n(rst_n),// reset
		.blue_din(video_b),// Blue data in
		.green_din(video_g),// Green data in
		.red_din(video_r),// Red data in
		.hsync(video_hs),// hsync data
		.vsync(video_vs),// vsync data
		.de(video_de),// data enable
		.tmds_clk_p(tmds_clk_p),
		.tmds_clk_n(tmds_clk_n),
		.tmds_data_p(tmds_data_p),//rgb
		.tmds_data_n(tmds_data_n) //rgb
	);
 
endmodule
