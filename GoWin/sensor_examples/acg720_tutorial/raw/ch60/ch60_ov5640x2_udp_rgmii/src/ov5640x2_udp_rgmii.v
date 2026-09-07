/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : ov5640x2_udp_gmii
// Description   : 摄像头采集，以太网UDP协议发送
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module ov5640x2_udp_rgmii(
	clk,
	rst_n,
	
	eth_gtxc,
	eth_txd,
	eth_txen,

	eth_mdc,
	eth_mdio,
	eth_rst_n,

    led,
	
	camera1_sclk,
	camera1_sdat,
	camera1_vsync,
	camera1_href,
	camera1_pclk,
	camera1_xclk,
	camera1_data,
	camera1_rst_n,
	camera1_pwdn,
	
	
	camera2_sclk,
	camera2_sdat,
	camera2_vsync,
	camera2_href,
	camera2_pclk,
	camera2_xclk,
	camera2_data,
	camera2_rst_n,
	camera2_pwdn
);
	input clk;
	input rst_n;
	
	output eth_gtxc;
	output [3:0] eth_txd;
	output eth_txen;
	inout eth_mdc;
	inout eth_mdio;
	output eth_rst_n;
    output led;
	assign led = Init_Done;
	//camera1 interface
	output		camera1_sclk;
	inout		camera1_sdat;
	input		camera1_vsync;
	input		camera1_href;
	input		camera1_pclk;
	output		camera1_xclk;
	input [7:0]	camera1_data;
	output		camera1_rst_n;
	output		camera1_pwdn;
	
	//camera2 interface
	output		camera2_sclk;
	inout		camera2_sdat;
	input		camera2_vsync;
	input		camera2_href;
	input		camera2_pclk;
	output		camera2_xclk;
	input [7:0]	camera2_data;
	output		camera2_rst_n;
	output		camera2_pwdn;
	
	wire clk_125m;
	wire clk_50m;
	wire clk_24m;
	wire locked;
	wire global_rst_n = locked;
	assign camera1_xclk = clk_24m;
	assign camera2_xclk = clk_24m;

    Gowin_PLL Gowin_PLL(
        .lock(locked), //output lock
        .clkout0(clk_125m), //output clkout0
        .clkout1(clk_50m), //output clkout1
        .clkout2(clk_24m), //output clkout2
        .clkin(clk), //input clkin
        .reset(~rst_n) //input reset
    );
	
	wire phy_init;	//phy initial done
	wire Rd_Data;
	phy_reg_config phy_reg_config(
		.Clk       			(clk_50m       	),
		.Rst_n     			(global_rst_n  	),
		.Phy_rst_n 			(eth_rst_n    	),
		.mdc       			(eth_mdc    	), 
		.mdio      			(eth_mdio   	),
		.Rd_Data  			(Rd_Data      	),
		.Phy_init_done		(phy_init      	)
	);

	assign camera1_pwdn = 1'b0;
	assign camera2_pwdn = 1'b0;
	
	localparam RGB = 0;
	localparam JPEG = 1;
	
	parameter IMAGE_WIDTH  = 640; //图片宽度
	parameter IMAGE_HEIGHT = 720; //图片高度(≤720)
	parameter IMAGE_FLIP   = 0;   //0：不翻转，1：上下翻转
	parameter IMAGE_MIRROR = 0;   //0：不镜像，1：左右镜像
	//-----------------------------------------------
	//摄像头1,摄像头2初始化配置
	wire Init_Done1;
	wire Init_Done2;
	wire Init_Done = (Init_Done1 && Init_Done2);
	camera_init
	#(
		.CAMERA_TYPE		("ov5640"		),
		.IMAGE_TYPE         (0              ),
		.IMAGE_WIDTH		(IMAGE_WIDTH	),
		.IMAGE_HEIGHT		(IMAGE_HEIGHT	),
		.IMAGE_FLIP_EN		(1	            ),
		.IMAGE_MIRROR_EN	(0             	)
	)
	camera_init1(
		.Clk				(clk_50m		),
		.Rst_n				(global_rst_n	),
		.Init_Done			(Init_Done1		),
		.camera_rst_n 		(camera1_rst_n  ),
		.i2c_sclk			(camera1_sclk	),
		.i2c_sdat			(camera1_sdat	)
	),
	camera_init2(
		.Clk				(clk_50m		),
		.Rst_n				(global_rst_n	),
		.Init_Done			(Init_Done2		),
		.camera_rst_n   	(camera2_rst_n  ),
		.i2c_sclk			(camera2_sclk	),
		.i2c_sdat			(camera2_sdat	)
	);
	
	//-----------------------------------------------
	//摄像头1,摄像头2图像输出
	wire fifo1_aclr;
	wire fifo1_wrreq;
	wire [7:0] fifo1_wrdata;
	wire fifo2_aclr;
	wire fifo2_wrreq;
	wire [7:0] fifo2_wrdata;
	Camera_ETH_Formator Camera_ETH_Formator1(
		.PCLK		    	(camera1_pclk   ),
		.Rst_n				(locked     	),
		.Init_Done			(Init_Done		),
		.HREF	     		(camera1_href   ),
		.VSYNC		    	(camera1_vsync	),
		.DATA			    (camera1_data	),
		.fifo_aclr			(fifo1_aclr		),
		.wrdata	            (fifo1_wrdata	),
		.wrreq		     	(fifo1_wrreq	)
	),
	Camera_ETH_Formator2(
		.PCLK			    (camera2_pclk   ),
		.Rst_n				(locked     	),
		.Init_Done			(Init_Done		),
		.HREF       		(camera2_href	),
		.VSYNC      		(camera2_vsync	),
		.DATA       		(camera2_data	),
		.fifo_aclr			(fifo2_aclr		),
		.wrdata	        	(fifo2_wrdata	),
		.wrreq	    		(fifo2_wrreq	)
	);
	


	wire fifo_aclr;
	wire fifo_wrreq;
	wire [7:0] fifo_wrdata;

	wire wrclk;//ethernet fifo wrclk
	wire GMII_GTXC;
	assign GMII_GTXC = clk_125m;

	
	controller_top controller_top(
        .clk(GMII_GTXC),
        .rst_n(rst_n),
        
        .wrclk1(camera1_pclk),
        .fifo1_aclr(fifo1_aclr),
        .wrdata1(fifo1_wrdata),
        .wrreq1(fifo1_wrreq),
        
        .wrclk2(camera2_pclk),
        .fifo2_aclr(fifo2_aclr),
        .wrdata2(fifo2_wrdata),
        .wrreq2(fifo2_wrreq),
        
        .wrclk(wrclk),
        .fifo_aclr(fifo_aclr),
        .fifo_wrreq(fifo_wrreq),
        .fifo_wrdata(fifo_wrdata)
    );
	
	
	wire [7:0] GMII_TXD;
	wire GMII_TXEN;
	UDP_Send UDP_Send(
		.Clk				(				),
		.GMII_GTXC			(GMII_GTXC		),
		.GMII_TXD			(GMII_TXD		),
		.GMII_TXEN			(GMII_TXEN		),
		.Rst_n				(Init_Done		),
		.Tx_Done			(				),
		.data_length		(640*2*2+2		),
		.des_ip				(32'hc0_a8_00_03),//hc0_a8_00_03; 192.168.0.3
		.des_mac			(48'hFF_FF_FF_FF_FF_FF),
		.des_port			(16'd6102		),
		.src_ip				(32'hc0_a8_00_02),//hc0_a8_65_02; 192.168.0.2
		.src_mac			(48'h00_0a_35_01_fe_c0),
		.src_port			(16'd5000		),
		//.wrclk			(camera_pclk		),
		.wrclk				(wrclk			),
		.wrdata				(fifo_wrdata	),
		.wrreq				(fifo_wrreq		),
		.aclr				(fifo_aclr		),
		.wrusedw			( )
	);
	
	gmii_to_rgmii gmii_to_rgmii(
	    .reset_n            (global_rst_n   ),
		.gmii_tx_clk		(GMII_GTXC		),
		.gmii_txd 			(GMII_TXD		),
		.gmii_txen			(GMII_TXEN		),
		.gmii_txer          (     0         ),
		
		.rgmii_tx_clk		(eth_gtxc   	),
		.rgmii_txd 			(eth_txd		),
		.rgmii_txen			(eth_txen		)
	);
	
endmodule