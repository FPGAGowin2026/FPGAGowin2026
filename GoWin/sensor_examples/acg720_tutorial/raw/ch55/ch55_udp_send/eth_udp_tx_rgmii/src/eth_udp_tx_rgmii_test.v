/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: Max
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: eth_udp_tx_gmii_test
// Module Name: eth_udp_tx_gmii_test
// Project Name: eth_udp_tx_gmii_test
// Target Devices: XC7A35T-2FGG484I
// Tool Versions: Vivado 2018.3
// Description: eth_udp_tx_gmii模块顶层测试文件
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
`define CLK_PERIOD 8

module eth_udp_tx_rgmii_test(
	clk50M,
	reset_n,
	led,
	eth_reset_n,
	rgmii_tx_clk,	
	rgmii_txd,
	rgmii_txen
);
    input         clk50M;
    input         reset_n;
    output        led;
    output        eth_reset_n;
    output        rgmii_tx_clk;
    output  [3:0] rgmii_txd;
    output        rgmii_txen;
    
    wire gmii_tx_clk;
    wire [7:0] gmii_txd;
    wire gmii_txen;

    wire       clk125M;
    wire       udp_gmii_rst_n;
    wire       pll_locked;
    reg [27:0] cnt_dly_time;
    wire       tx_en_pulse;  
    wire       payload_req;
    reg [7:0]  payload_dat;
    reg [15:0] tx_byte_cnt;

    assign led            = pll_locked;
    assign eth_reset_n    = pll_locked;
    assign udp_gmii_rst_n = pll_locked;

    Gowin_PLL Gowin_PLL(
        .lock(pll_locked), //output lock
        .clkout0(clk125M), //output clkout0
        .clkin(clk50M), //input clkin
        .reset(~reset_n) //input reset
    );

    eth_udp_tx_gmii eth_udp_tx_gmii
    (
        .clk125m       (clk125M               ),
        .reset_p       (~udp_gmii_rst_n        ),
                       
        .tx_en_pulse   (tx_en_pulse           ),
        .tx_done       (tx_done               ),
                       
        .dst_mac       (48'hff_ff_ff_ff_ff_ff ),
        .src_mac       (48'h00_0a_35_01_fe_c0 ),  
        .dst_ip        (32'hc0_a8_00_03       ),
        .src_ip        (32'hc0_a8_00_02       ),	
        .dst_port      (16'd6102              ),
        .src_port      (16'd5000              ),
                       
        .data_length   (23                    ),

        .payload_req_o (payload_req           ),
        .payload_dat_i (payload_dat           ),

        .gmii_tx_clk   (gmii_tx_clk           ),	
        .gmii_txen     (gmii_txen             ),
        .gmii_txd      (gmii_txd              )
    );

     gmii_to_rgmii gmii_to_rgmii(
      .reset_n(udp_gmii_rst_n),

      .gmii_tx_clk(gmii_tx_clk),
      .gmii_txd(gmii_txd),
      .gmii_txen(gmii_txen),
      .gmii_txer(1'b0),

      .rgmii_tx_clk(rgmii_tx_clk),
      .rgmii_txd(rgmii_txd),
      .rgmii_txen(rgmii_txen)
    );

    always@(posedge clk125M or negedge udp_gmii_rst_n)
    if(!udp_gmii_rst_n)
        cnt_dly_time <= 16'd0;
    else
        cnt_dly_time <= cnt_dly_time + 1'b1;

    assign tx_en_pulse = &cnt_dly_time;

    always@(posedge clk125M or negedge reset_n)
    if(!reset_n)
        tx_byte_cnt <= 16'd0;
    else if(payload_req)
        tx_byte_cnt <= tx_byte_cnt + 1'b1;
    else
        tx_byte_cnt <= 16'd0;

    always@(*)
    begin
    case(tx_byte_cnt)
      16'd0  : payload_dat = "H";
      16'd1  : payload_dat = "e";
      16'd2  : payload_dat = "l";
      16'd3  : payload_dat = "l";
      16'd4  : payload_dat = "o";
      16'd5  : payload_dat = ",";
      16'd6  : payload_dat = " ";
      16'd7  : payload_dat = "w";
      16'd8  : payload_dat = "e";
      16'd9  : payload_dat = "l";
      16'd10 : payload_dat = "c";
      16'd11 : payload_dat = "o";
      16'd12 : payload_dat = "m";
      16'd13 : payload_dat = "e";
      16'd14 : payload_dat = " ";
      16'd15 : payload_dat = "t";
      16'd16 : payload_dat = "o";
      16'd17 : payload_dat = " ";
      16'd18 : payload_dat = "F";
      16'd19 : payload_dat = "P";
      16'd20 : payload_dat = "G";
      16'd21 : payload_dat = "A";
      16'd22 : payload_dat = "!";
      default: payload_dat = 8'd0;

    endcase
    end

endmodule
