`timescale 1ns/1ns

module mdio_tb;

	reg      clk;
	reg      rst_n;
	wire     eth_rst_n;
	wire     eth_mdc;
	wire     eth_mdio;
	wire     phy_init;
	
	pullup PUP(eth_mdio);
	
	phy_reg_config phy_reg_config_inst(
		.Clk       		   (clk           		),
		.Rst_n     		   (rst_n  	    		),
		.Phy_rst_n 		   (eth_rst_n    	    ),
		.mdc       		   (eth_mdc    			),
		.mdio      		   (eth_mdio   			),
		.Phy_init_done     (phy_init      		)
	);
	
	always #10 clk = ~clk;
	
	initial begin
		clk = 1;
		rst_n = 0;
		#201;
		rst_n = 1;
		
		@(posedge phy_init);
		
		#2000;
		$stop;
	end
	
endmodule
