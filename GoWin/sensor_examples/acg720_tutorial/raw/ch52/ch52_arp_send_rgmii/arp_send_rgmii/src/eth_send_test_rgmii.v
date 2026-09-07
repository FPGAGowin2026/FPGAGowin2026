/*
	RGMII 接口的ARP发送
*/

module eth_send_test_rgmii(
	clk,
	rst_n,
	rgmii_gtx_clk,
	rgmii_tx_data,
	rgmii_tx_en,
	phy_rst_n
);
	input clk;
	input rst_n;
	
	output rgmii_gtx_clk;
	output [3:0]rgmii_tx_data;
	output rgmii_tx_en;
	
	wire gmii_tx_en;	//GMII接口发送数据使能信号，高电平有效
	wire gmii_tx_er;	//发送错误，用以破坏数据包发送
	wire [7:0]gmii_tx_data; //GMII接口发送数据线，FPGA通过该数据线将需要发送的数据依次送给PHY芯片
	
	output phy_rst_n;
	
	wire gmii_gtx_clk;

    Gowin_PLL Gowin_PLL(
        .clkout0(gmii_gtx_clk), //output clkout0
        .clkin(clk) //input clkin
    );
	
	eth_send_test eth_send_test(
		.rst_n(rst_n),
		.gmii_tx_clk(gmii_gtx_clk),
		.gmii_tx_en(gmii_tx_en),
		.gmii_tx_er(gmii_tx_er),
		.gmii_tx_data(gmii_tx_data),
		.phy_rst_n(phy_rst_n)
	);
	
	gmii_to_rgmii gmii_to_rgmii(
	    .reset_n(rst_n),
		.gmii_tx_clk(gmii_gtx_clk),
		.gmii_txd(gmii_tx_data),
		.gmii_txen(gmii_tx_en),
		.gmii_txer(),
		.rgmii_tx_clk(rgmii_gtx_clk),
		.rgmii_txd(rgmii_tx_data),
		.rgmii_txen(rgmii_tx_en)
	);

endmodule
