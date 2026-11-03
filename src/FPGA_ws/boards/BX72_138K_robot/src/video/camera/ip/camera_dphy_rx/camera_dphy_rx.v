//Copyright (C)2014-2026 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Post-PnR Verilog Simulation Model file
//Tool Version: V1.9.12.04 (64-bit)
//Created Time: Fri Oct 16 16:38:53 2026

`timescale 100 ps/100 ps
module Camera_DPHY_RX(
	reset_n,
	HS_CLK_P,
	HS_CLK_N,
	HS_DATA1_P,
	HS_DATA1_N,
	HS_DATA0_P,
	HS_DATA0_N,
	clk_hs_en,
	data_hs_en,
	delay_taps_lane0,
	delay_load_pulse_lane0,
	delay_taps_lane1,
	delay_load_pulse_lane1,
	clk_term_en,
	data_term_en,
	sclk_out,
	data_out1,
	data_out0,
	ready,
	sync_lane0,
	sync_lane1
);
input reset_n;
input HS_CLK_P;
input HS_CLK_N;
input HS_DATA1_P;
input HS_DATA1_N;
input HS_DATA0_P;
input HS_DATA0_N;
input clk_hs_en;
input data_hs_en;
input [7:0] delay_taps_lane0;
input delay_load_pulse_lane0;
input [7:0] delay_taps_lane1;
input delay_load_pulse_lane1;
input clk_term_en;
input data_term_en;
output sclk_out;
output [7:0] data_out1;
output [7:0] data_out0;
output [3:0] ready;
output sync_lane0;
output sync_lane1;
wire GND;
wire HS_CLK_N;
wire HS_CLK_P;
wire HS_DATA0_N;
wire HS_DATA0_P;
wire HS_DATA1_N;
wire HS_DATA1_P;
wire VCC;
wire clk_hs_en;
wire clk_term_en;
wire data_hs_en;
wire [7:0] delay_taps_lane0;
wire delay_load_pulse_lane0;
wire [7:0] delay_taps_lane1;
wire delay_load_pulse_lane1;
wire [7:0] data_out0;
wire [7:0] data_out1;
wire data_term_en;
wire [3:0] ready;
wire reset_n;
wire sclk_out;
wire \DPHY_RX_INST/HS_CLK ;
wire \DPHY_RX_INST/n52_6 ;
wire dphy_reset_release_meta;
wire dphy_reset_release_sync;
wire dphy_internal_reset;
wire dphy_ides_reset_release_meta;
(* syn_keep = 1 *) wire dphy_ides_reset_release_lane0;
(* syn_keep = 1 *) wire dphy_ides_reset_release_lane1;
(* syn_keep = 1 *) wire dphy_tap_reset_release_lane0;
(* syn_keep = 1 *) wire dphy_tap_reset_release_lane1;
wire dphy_byte_reset_release_meta;
wire dphy_byte_reset_release_sync;
wire dphy_byte_internal_reset;
wire dphy_hs_clk_n;
wire dphy_hs_clk_static32;
wire [7:2] dphy_tap_meta_lane0;
wire [7:2] dphy_tap_meta_lane1;
wire [7:0] dphy_tap_shadow_lane0;
wire [7:0] dphy_tap_shadow_lane1;
wire [1:0] \DPHY_RX_INST/HS_DATA ;
wire [7:0] \DPHY_RX_INST/data0 ;
wire [7:0] \DPHY_RX_INST/data1 ;
wire [7:0] \DPHY_RX_INST/data1_ides ;
wire [7:0] \DPHY_RX_INST/data0_ides ;
wire \DPHY_RX_INST/u_idesx8/opensync_cken ;
wire \DPHY_RX_INST/u_idesx8/xstop ;
wire \DPHY_RX_INST/u_idesx8/udel_dataini1_1_DF ;
wire \DPHY_RX_INST/u_idesx8/dataini_t1 ;
wire \DPHY_RX_INST/u_idesx8/udel_dataini0_1_DF ;
wire \DPHY_RX_INST/u_idesx8/dataini_t0 ;
wire \DPHY_RX_INST/u_idesx8/eclko ;
wire [3:0] \DPHY_RX_INST/u_idesx8/opensync ;
wire \DPHY_RX_INST/u_Aligner/lane_ready_val_w ;
wire \DPHY_RX_INST/u_Aligner/n116_5 ;
wire \DPHY_RX_INST/u_Aligner/lane_ready_val_w_5 ;
wire \DPHY_RX_INST/u_Aligner/lane_ready_val_w_6 ;
wire \DPHY_RX_INST/u_Aligner/lane_ready_val ;
wire \DPHY_RX_INST/u_Aligner/ln_ready ;
wire [7:0] \DPHY_RX_INST/u_Aligner/lane0 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/lane1 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n428_3 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n429_3 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n430_3 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n431_3 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n432_3 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n433_3 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n434_3 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n427_3 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n330_3 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n331_3 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n332_3 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n333_3 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n334_3 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n335_3 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n336_3 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n337_3 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n428_4 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n428_5 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n428_6 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n428_7 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n429_4 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n430_4 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n431_4 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n432_4 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n433_4 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n434_4 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n427_4 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n330_4 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n330_5 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n331_4 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n331_5 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n332_4 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n332_5 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n332_6 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n333_4 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n333_5 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n334_4 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n334_5 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n335_4 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n335_5 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n336_4 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n336_5 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n337_4 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n337_5 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n337_6 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n428_8 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n428_9 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n428_10 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n428_11 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n428_12 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n428_13 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n428_14 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n428_15 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n428_16 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n428_17 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n428_18 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n429_5 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n431_5 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n433_5 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n330_6 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n330_7 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n330_8 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n330_9 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n331_6 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n331_7 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n331_8 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n332_7 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n332_8 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n332_9 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n332_10 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n333_6 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n333_7 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n333_8 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n334_6 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n334_7 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n334_8 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n335_6 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n335_7 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n335_8 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n336_6 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n336_7 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n336_8 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n337_7 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n337_8 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n337_9 ;
wire \DPHY_RX_INST/u_Aligner/wd0/n337_10 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/wd0/reg0 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/wd0/reg1 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/wd0/reg2 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/wd0/reg3 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/wd0/reg4 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/wd0/reg5 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/wd0/shftamt ;
wire \DPHY_RX_INST/u_Aligner/wd1/n428_3 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n429_3 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n430_3 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n431_3 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n432_3 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n433_3 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n434_3 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n427_3 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n330_3 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n331_3 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n332_3 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n333_3 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n334_3 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n335_3 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n336_3 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n337_3 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n428_4 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n428_5 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n428_6 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n428_7 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n429_4 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n430_4 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n431_4 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n432_4 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n433_4 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n434_4 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n427_4 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n330_4 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n330_5 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n331_4 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n331_5 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n332_4 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n332_5 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n333_4 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n333_5 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n333_6 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n334_4 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n334_5 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n335_4 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n335_5 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n336_4 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n336_5 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n337_4 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n337_5 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n337_6 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n428_8 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n428_9 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n428_10 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n428_11 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n428_12 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n428_13 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n428_14 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n428_15 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n428_16 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n428_17 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n428_18 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n429_5 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n431_5 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n433_5 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n330_6 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n330_7 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n330_8 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n330_9 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n331_6 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n331_7 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n331_8 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n332_6 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n332_7 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n332_8 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n333_7 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n333_8 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n333_9 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n333_10 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n334_6 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n334_7 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n334_8 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n335_6 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n335_7 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n335_8 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n336_6 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n336_7 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n336_8 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n337_7 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n337_8 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n337_9 ;
wire \DPHY_RX_INST/u_Aligner/wd1/n337_10 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/wd1/reg0 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/wd1/reg1 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/wd1/reg2 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/wd1/reg3 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/wd1/reg4 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/wd1/reg5 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/wd1/shftamt ;
wire \DPHY_RX_INST/u_Aligner/ln0/n17_3 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n91_3 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n160_3 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n260_6 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n259_6 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n258_6 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n257_6 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n256_6 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n255_6 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n254_6 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n253_6 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n211_6 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n210_6 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n209_6 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n208_6 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n207_6 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n206_6 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n205_6 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n204_6 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n21_4 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n20_4 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n95_4 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n94_4 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n17_4 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n17_5 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n91_4 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n91_5 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n260_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n260_8 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n259_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n258_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n257_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n256_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n255_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n254_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n253_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n211_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n211_8 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n210_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n209_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n208_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n207_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n206_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n205_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n204_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n17_6 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n17_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n17_8 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n17_9 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n17_10 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n91_6 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n91_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n91_8 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n91_9 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n91_10 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n260_9 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n211_9 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n22_10 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n390_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n23_8 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n96_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n396_7 ;
wire \DPHY_RX_INST/u_Aligner/ln0/n97_8 ;
wire dphy_sync0_internal;
wire dphy_sync1_internal;
assign sync_lane0 = dphy_sync0_internal;
assign sync_lane1 = dphy_sync1_internal;
wire [7:0] \DPHY_RX_INST/u_Aligner/ln0/q_in0_0 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/ln0/q_in0_1 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/ln0/q_in0_2 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/ln0/q_in0_3 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/ln0/q_in0_4 ;
wire [3:0] \DPHY_RX_INST/u_Aligner/ln0/lane0_cnt ;
wire [7:0] \DPHY_RX_INST/u_Aligner/ln0/q_in1_0 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/ln0/q_in1_1 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/ln0/q_in1_2 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/ln0/q_in1_3 ;
wire [7:0] \DPHY_RX_INST/u_Aligner/ln0/q_in1_4 ;
wire [3:0] \DPHY_RX_INST/u_Aligner/ln0/lane1_cnt ;
VCC VCC_cZ (
  .V(VCC)
);
GND GND_cZ (
  .G(GND)
);
GSR GSR (
	.GSRI(VCC)
);
OBUF gowin_buf_0 (
	.I(ready[0]),
	.O(ready[1])
);
OBUF gowin_buf_1 (
	.I(GND),
	.O(ready[2])
);
OBUF gowin_buf_2 (
	.I(GND),
	.O(ready[3])
);
TLVDS_IBUF \DPHY_RX_INST/U0_IB  (
	.I(HS_CLK_P),
	.IB(HS_CLK_N),
	.O(\DPHY_RX_INST/HS_CLK )
);
TLVDS_IBUF \DPHY_RX_INST/U1_IB1  (
	.I(HS_DATA1_P),
	.IB(HS_DATA1_N),
	.O(\DPHY_RX_INST/HS_DATA [1])
);
TLVDS_IBUF \DPHY_RX_INST/U1_IB0  (
	.I(HS_DATA0_P),
	.IB(HS_DATA0_N),
	.O(\DPHY_RX_INST/HS_DATA [0])
);
DFFCE \DPHY_RX_INST/data0_6_s0  (
	.D(\DPHY_RX_INST/data0_ides [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(dphy_byte_internal_reset),
	.Q(\DPHY_RX_INST/data0 [6])
);
defparam \DPHY_RX_INST/data0_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/data0_5_s0  (
	.D(\DPHY_RX_INST/data0_ides [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(dphy_byte_internal_reset),
	.Q(\DPHY_RX_INST/data0 [5])
);
defparam \DPHY_RX_INST/data0_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/data0_4_s0  (
	.D(\DPHY_RX_INST/data0_ides [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(dphy_byte_internal_reset),
	.Q(\DPHY_RX_INST/data0 [4])
);
defparam \DPHY_RX_INST/data0_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/data0_3_s0  (
	.D(\DPHY_RX_INST/data0_ides [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(dphy_byte_internal_reset),
	.Q(\DPHY_RX_INST/data0 [3])
);
defparam \DPHY_RX_INST/data0_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/data0_2_s0  (
	.D(\DPHY_RX_INST/data0_ides [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(dphy_byte_internal_reset),
	.Q(\DPHY_RX_INST/data0 [2])
);
defparam \DPHY_RX_INST/data0_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/data0_1_s0  (
	.D(\DPHY_RX_INST/data0_ides [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(dphy_byte_internal_reset),
	.Q(\DPHY_RX_INST/data0 [1])
);
defparam \DPHY_RX_INST/data0_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/data0_0_s0  (
	.D(\DPHY_RX_INST/data0_ides [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(dphy_byte_internal_reset),
	.Q(\DPHY_RX_INST/data0 [0])
);
defparam \DPHY_RX_INST/data0_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/data1_7_s0  (
	.D(\DPHY_RX_INST/data1_ides [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(dphy_byte_internal_reset),
	.Q(\DPHY_RX_INST/data1 [7])
);
defparam \DPHY_RX_INST/data1_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/data1_6_s0  (
	.D(\DPHY_RX_INST/data1_ides [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(dphy_byte_internal_reset),
	.Q(\DPHY_RX_INST/data1 [6])
);
defparam \DPHY_RX_INST/data1_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/data1_5_s0  (
	.D(\DPHY_RX_INST/data1_ides [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(dphy_byte_internal_reset),
	.Q(\DPHY_RX_INST/data1 [5])
);
defparam \DPHY_RX_INST/data1_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/data1_4_s0  (
	.D(\DPHY_RX_INST/data1_ides [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(dphy_byte_internal_reset),
	.Q(\DPHY_RX_INST/data1 [4])
);
defparam \DPHY_RX_INST/data1_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/data1_3_s0  (
	.D(\DPHY_RX_INST/data1_ides [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(dphy_byte_internal_reset),
	.Q(\DPHY_RX_INST/data1 [3])
);
defparam \DPHY_RX_INST/data1_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/data1_2_s0  (
	.D(\DPHY_RX_INST/data1_ides [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(dphy_byte_internal_reset),
	.Q(\DPHY_RX_INST/data1 [2])
);
defparam \DPHY_RX_INST/data1_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/data1_1_s0  (
	.D(\DPHY_RX_INST/data1_ides [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(dphy_byte_internal_reset),
	.Q(\DPHY_RX_INST/data1 [1])
);
defparam \DPHY_RX_INST/data1_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/data1_0_s0  (
	.D(\DPHY_RX_INST/data1_ides [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(dphy_byte_internal_reset),
	.Q(\DPHY_RX_INST/data1 [0])
);
defparam \DPHY_RX_INST/data1_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/data0_7_s0  (
	.D(\DPHY_RX_INST/data0_ides [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(dphy_byte_internal_reset),
	.Q(\DPHY_RX_INST/data0 [7])
);
defparam \DPHY_RX_INST/data0_7_s0 .INIT=1'b0;
INV \DPHY_RX_INST/n52_s2  (
	.I(reset_n),
	.O(\DPHY_RX_INST/n52_6 )
);
// Hold D-PHY logic in reset until raw HS clock has been observed twice.
DFFCE dphy_reset_release_meta_ff (
	.D(VCC),
	.CLK(\DPHY_RX_INST/HS_CLK ),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/n52_6 ),
	.Q(dphy_reset_release_meta)
);
defparam dphy_reset_release_meta_ff.INIT=1'b0;
DFFCE dphy_reset_release_sync_ff (
	.D(dphy_reset_release_meta),
	.CLK(\DPHY_RX_INST/HS_CLK ),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/n52_6 ),
	.Q(dphy_reset_release_sync)
);
defparam dphy_reset_release_sync_ff.INIT=1'b0;
INV dphy_internal_reset_inv (
	.I(dphy_reset_release_sync),
	.O(dphy_internal_reset)
);
// Release the two IDES resets only after two raw HS falling edges.
DFFPE dphy_ides_reset_release_meta_ff (
	.D(GND),
	.CLK(dphy_hs_clk_n),
	.CE(VCC),
	.PRESET(\DPHY_RX_INST/n52_6 ),
	.Q(dphy_ides_reset_release_meta)
);
defparam dphy_ides_reset_release_meta_ff.INIT=1'b1;
(* syn_keep = 1, syn_preserve = 1 *) DFFPE dphy_ides_reset_release_lane0_sync_ff (
	.D(dphy_ides_reset_release_meta),
	.CLK(dphy_hs_clk_n),
	.CE(VCC),
	.PRESET(\DPHY_RX_INST/n52_6 ),
	.Q(dphy_ides_reset_release_lane0)
);
defparam dphy_ides_reset_release_lane0_sync_ff.INIT=1'b1;
(* syn_keep = 1, syn_preserve = 1 *) DFFPE dphy_ides_reset_release_lane1_sync_ff (
	.D(dphy_ides_reset_release_meta),
	.CLK(dphy_hs_clk_n),
	.CE(VCC),
	.PRESET(\DPHY_RX_INST/n52_6 ),
	.Q(dphy_ides_reset_release_lane1)
);
defparam dphy_ides_reset_release_lane1_sync_ff.INIT=1'b1;
// Give each lane's tap flops a local falling-edge reset release from the shared meta stage.
(* syn_keep = 1, syn_preserve = 1 *) DFFPE dphy_tap_reset_release_lane0_sync_ff (
	.D(dphy_ides_reset_release_meta),
	.CLK(dphy_hs_clk_n),
	.CE(VCC),
	.PRESET(\DPHY_RX_INST/n52_6 ),
	.Q(dphy_tap_reset_release_lane0)
);
defparam dphy_tap_reset_release_lane0_sync_ff.INIT=1'b1;
(* syn_keep = 1, syn_preserve = 1 *) DFFPE dphy_tap_reset_release_lane1_sync_ff (
	.D(dphy_ides_reset_release_meta),
	.CLK(dphy_hs_clk_n),
	.CE(VCC),
	.PRESET(\DPHY_RX_INST/n52_6 ),
	.Q(dphy_tap_reset_release_lane1)
);
defparam dphy_tap_reset_release_lane1_sync_ff.INIT=1'b1;
// Keep byte-clocked data and aligner state in reset until sclk_out is stable.
DFFCE dphy_byte_reset_release_meta_ff (
	.D(VCC),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(dphy_internal_reset),
	.Q(dphy_byte_reset_release_meta)
);
defparam dphy_byte_reset_release_meta_ff.INIT=1'b0;
DFFCE dphy_byte_reset_release_sync_ff (
	.D(dphy_byte_reset_release_meta),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(dphy_internal_reset),
	.Q(dphy_byte_reset_release_sync)
);
defparam dphy_byte_reset_release_sync_ff.INIT=1'b0;
INV dphy_byte_internal_reset_inv (
	.I(dphy_byte_reset_release_sync),
	.O(dphy_byte_internal_reset)
);
INV dphy_hs_clk_inv (
	.I(\DPHY_RX_INST/HS_CLK ),
	.O(dphy_hs_clk_n)
);
// Add a static delay only on the dedicated DPHY clock path into DHCE.
// Raw HS remains the clock for fabric synchronizers and tap staging flops.
IODELAY #(.C_STATIC_DLY(32), .DYN_DLY_EN("FALSE"), .ADAPT_EN("FALSE")) dphy_hs_clk_static32_delay (
	.DI(\DPHY_RX_INST/HS_CLK ),
	.DO(dphy_hs_clk_static32),
	.DF(),
	.DLYSTEP(8'b0),
	.SDTAP(1'b0),
	.VALUE(1'b0)
);
// The board autocal emits multiples of four; retain only the six variable tap bits.
assign dphy_tap_shadow_lane0[1:0] = 2'b00;
assign dphy_tap_shadow_lane1[1:0] = 2'b00;
DFFCE dphy_tap_meta_lane0_ff_2 (
	.D(delay_taps_lane0[2]),
	.CLK(\DPHY_RX_INST/HS_CLK ),
	.CE(VCC),
	.CLEAR(dphy_internal_reset),
	.Q(dphy_tap_meta_lane0[2])
);
defparam dphy_tap_meta_lane0_ff_2.INIT=1'b0;
DFFCE dphy_tap_meta_lane0_ff_3 (
	.D(delay_taps_lane0[3]),
	.CLK(\DPHY_RX_INST/HS_CLK ),
	.CE(VCC),
	.CLEAR(dphy_internal_reset),
	.Q(dphy_tap_meta_lane0[3])
);
defparam dphy_tap_meta_lane0_ff_3.INIT=1'b0;
DFFCE dphy_tap_meta_lane0_ff_4 (
	.D(delay_taps_lane0[4]),
	.CLK(\DPHY_RX_INST/HS_CLK ),
	.CE(VCC),
	.CLEAR(dphy_internal_reset),
	.Q(dphy_tap_meta_lane0[4])
);
defparam dphy_tap_meta_lane0_ff_4.INIT=1'b0;
DFFCE dphy_tap_meta_lane0_ff_5 (
	.D(delay_taps_lane0[5]),
	.CLK(\DPHY_RX_INST/HS_CLK ),
	.CE(VCC),
	.CLEAR(dphy_internal_reset),
	.Q(dphy_tap_meta_lane0[5])
);
defparam dphy_tap_meta_lane0_ff_5.INIT=1'b0;
DFFCE dphy_tap_meta_lane0_ff_6 (
	.D(delay_taps_lane0[6]),
	.CLK(\DPHY_RX_INST/HS_CLK ),
	.CE(VCC),
	.CLEAR(dphy_internal_reset),
	.Q(dphy_tap_meta_lane0[6])
);
defparam dphy_tap_meta_lane0_ff_6.INIT=1'b0;
DFFCE dphy_tap_meta_lane0_ff_7 (
	.D(delay_taps_lane0[7]),
	.CLK(\DPHY_RX_INST/HS_CLK ),
	.CE(VCC),
	.CLEAR(dphy_internal_reset),
	.Q(dphy_tap_meta_lane0[7])
);
defparam dphy_tap_meta_lane0_ff_7.INIT=1'b0;
DFFCE dphy_tap_meta_lane1_ff_2 (
	.D(delay_taps_lane1[2]),
	.CLK(\DPHY_RX_INST/HS_CLK ),
	.CE(VCC),
	.CLEAR(dphy_internal_reset),
	.Q(dphy_tap_meta_lane1[2])
);
defparam dphy_tap_meta_lane1_ff_2.INIT=1'b0;
DFFCE dphy_tap_meta_lane1_ff_3 (
	.D(delay_taps_lane1[3]),
	.CLK(\DPHY_RX_INST/HS_CLK ),
	.CE(VCC),
	.CLEAR(dphy_internal_reset),
	.Q(dphy_tap_meta_lane1[3])
);
defparam dphy_tap_meta_lane1_ff_3.INIT=1'b0;
DFFCE dphy_tap_meta_lane1_ff_4 (
	.D(delay_taps_lane1[4]),
	.CLK(\DPHY_RX_INST/HS_CLK ),
	.CE(VCC),
	.CLEAR(dphy_internal_reset),
	.Q(dphy_tap_meta_lane1[4])
);
defparam dphy_tap_meta_lane1_ff_4.INIT=1'b0;
DFFCE dphy_tap_meta_lane1_ff_5 (
	.D(delay_taps_lane1[5]),
	.CLK(\DPHY_RX_INST/HS_CLK ),
	.CE(VCC),
	.CLEAR(dphy_internal_reset),
	.Q(dphy_tap_meta_lane1[5])
);
defparam dphy_tap_meta_lane1_ff_5.INIT=1'b0;
DFFCE dphy_tap_meta_lane1_ff_6 (
	.D(delay_taps_lane1[6]),
	.CLK(\DPHY_RX_INST/HS_CLK ),
	.CE(VCC),
	.CLEAR(dphy_internal_reset),
	.Q(dphy_tap_meta_lane1[6])
);
defparam dphy_tap_meta_lane1_ff_6.INIT=1'b0;
DFFCE dphy_tap_meta_lane1_ff_7 (
	.D(delay_taps_lane1[7]),
	.CLK(\DPHY_RX_INST/HS_CLK ),
	.CE(VCC),
	.CLEAR(dphy_internal_reset),
	.Q(dphy_tap_meta_lane1[7])
);
defparam dphy_tap_meta_lane1_ff_7.INIT=1'b0;
DFFCE dphy_tap_shadow_lane0_2_ff (
	.D(dphy_tap_meta_lane0[2]),
	.CLK(dphy_hs_clk_n),
	.CE(VCC),
	.CLEAR(dphy_tap_reset_release_lane0),
	.Q(dphy_tap_shadow_lane0[2])
);
defparam dphy_tap_shadow_lane0_2_ff.INIT=1'b0;
DFFCE dphy_tap_shadow_lane0_3_ff (
	.D(dphy_tap_meta_lane0[3]),
	.CLK(dphy_hs_clk_n),
	.CE(VCC),
	.CLEAR(dphy_tap_reset_release_lane0),
	.Q(dphy_tap_shadow_lane0[3])
);
defparam dphy_tap_shadow_lane0_3_ff.INIT=1'b0;
DFFCE dphy_tap_shadow_lane0_4_ff (
	.D(dphy_tap_meta_lane0[4]),
	.CLK(dphy_hs_clk_n),
	.CE(VCC),
	.CLEAR(dphy_tap_reset_release_lane0),
	.Q(dphy_tap_shadow_lane0[4])
);
defparam dphy_tap_shadow_lane0_4_ff.INIT=1'b0;
DFFCE dphy_tap_shadow_lane0_5_ff (
	.D(dphy_tap_meta_lane0[5]),
	.CLK(dphy_hs_clk_n),
	.CE(VCC),
	.CLEAR(dphy_tap_reset_release_lane0),
	.Q(dphy_tap_shadow_lane0[5])
);
defparam dphy_tap_shadow_lane0_5_ff.INIT=1'b0;
DFFCE dphy_tap_shadow_lane0_6_ff (
	.D(dphy_tap_meta_lane0[6]),
	.CLK(dphy_hs_clk_n),
	.CE(VCC),
	.CLEAR(dphy_tap_reset_release_lane0),
	.Q(dphy_tap_shadow_lane0[6])
);
defparam dphy_tap_shadow_lane0_6_ff.INIT=1'b0;
DFFCE dphy_tap_shadow_lane0_7_ff (
	.D(dphy_tap_meta_lane0[7]),
	.CLK(dphy_hs_clk_n),
	.CE(VCC),
	.CLEAR(dphy_tap_reset_release_lane0),
	.Q(dphy_tap_shadow_lane0[7])
);
defparam dphy_tap_shadow_lane0_7_ff.INIT=1'b0;
DFFCE dphy_tap_shadow_lane1_2_ff (
	.D(dphy_tap_meta_lane1[2]),
	.CLK(dphy_hs_clk_n),
	.CE(VCC),
	.CLEAR(dphy_tap_reset_release_lane1),
	.Q(dphy_tap_shadow_lane1[2])
);
defparam dphy_tap_shadow_lane1_2_ff.INIT=1'b0;
DFFCE dphy_tap_shadow_lane1_3_ff (
	.D(dphy_tap_meta_lane1[3]),
	.CLK(dphy_hs_clk_n),
	.CE(VCC),
	.CLEAR(dphy_tap_reset_release_lane1),
	.Q(dphy_tap_shadow_lane1[3])
);
defparam dphy_tap_shadow_lane1_3_ff.INIT=1'b0;
DFFCE dphy_tap_shadow_lane1_4_ff (
	.D(dphy_tap_meta_lane1[4]),
	.CLK(dphy_hs_clk_n),
	.CE(VCC),
	.CLEAR(dphy_tap_reset_release_lane1),
	.Q(dphy_tap_shadow_lane1[4])
);
defparam dphy_tap_shadow_lane1_4_ff.INIT=1'b0;
DFFCE dphy_tap_shadow_lane1_5_ff (
	.D(dphy_tap_meta_lane1[5]),
	.CLK(dphy_hs_clk_n),
	.CE(VCC),
	.CLEAR(dphy_tap_reset_release_lane1),
	.Q(dphy_tap_shadow_lane1[5])
);
defparam dphy_tap_shadow_lane1_5_ff.INIT=1'b0;
DFFCE dphy_tap_shadow_lane1_6_ff (
	.D(dphy_tap_meta_lane1[6]),
	.CLK(dphy_hs_clk_n),
	.CE(VCC),
	.CLEAR(dphy_tap_reset_release_lane1),
	.Q(dphy_tap_shadow_lane1[6])
);
defparam dphy_tap_shadow_lane1_6_ff.INIT=1'b0;
DFFCE dphy_tap_shadow_lane1_7_ff (
	.D(dphy_tap_meta_lane1[7]),
	.CLK(dphy_hs_clk_n),
	.CE(VCC),
	.CLEAR(dphy_tap_reset_release_lane1),
	.Q(dphy_tap_shadow_lane1[7])
);
defparam dphy_tap_shadow_lane1_7_ff.INIT=1'b0;
LUT4 \DPHY_RX_INST/u_idesx8/LUT4_1  (
	.I0(VCC),
	.I1(VCC),
	.I2(\DPHY_RX_INST/u_idesx8/opensync [3]),
	.I3(\DPHY_RX_INST/u_idesx8/opensync [0]),
	.F(\DPHY_RX_INST/u_idesx8/opensync_cken )
);
defparam \DPHY_RX_INST/u_idesx8/LUT4_1 .INIT=16'h0A78;
LUT4 \DPHY_RX_INST/u_idesx8/LUT4_0  (
	.I0(GND),
	.I1(\DPHY_RX_INST/u_idesx8/opensync [2]),
	.I2(\DPHY_RX_INST/u_idesx8/opensync [1]),
	.I3(\DPHY_RX_INST/u_idesx8/opensync [0]),
	.F(\DPHY_RX_INST/u_idesx8/xstop )
);
defparam \DPHY_RX_INST/u_idesx8/LUT4_0 .INIT=16'hFFFE;
DFFCE \DPHY_RX_INST/u_idesx8/opensync_2_s0  (
	.D(\DPHY_RX_INST/u_idesx8/opensync [1]),
	.CLK(dphy_hs_clk_n),
	.CE(\DPHY_RX_INST/u_idesx8/opensync_cken ),
	.CLEAR(dphy_internal_reset),
	.Q(\DPHY_RX_INST/u_idesx8/opensync [2])
);
defparam \DPHY_RX_INST/u_idesx8/opensync_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_idesx8/opensync_1_s0  (
	.D(\DPHY_RX_INST/u_idesx8/opensync [0]),
	.CLK(dphy_hs_clk_n),
	.CE(\DPHY_RX_INST/u_idesx8/opensync_cken ),
	.CLEAR(dphy_internal_reset),
	.Q(\DPHY_RX_INST/u_idesx8/opensync [1])
);
defparam \DPHY_RX_INST/u_idesx8/opensync_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_idesx8/opensync_3_s0  (
	.D(\DPHY_RX_INST/u_idesx8/opensync [2]),
	.CLK(dphy_hs_clk_n),
	.CE(\DPHY_RX_INST/u_idesx8/opensync_cken ),
	.CLEAR(dphy_internal_reset),
	.Q(\DPHY_RX_INST/u_idesx8/opensync [3])
);
defparam \DPHY_RX_INST/u_idesx8/opensync_3_s0 .INIT=1'b0;
DFFPE \DPHY_RX_INST/u_idesx8/opensync_0_s0  (
	.D(\DPHY_RX_INST/u_idesx8/opensync [3]),
	.CLK(dphy_hs_clk_n),
	.CE(\DPHY_RX_INST/u_idesx8/opensync_cken ),
	.PRESET(dphy_internal_reset),
	.Q(\DPHY_RX_INST/u_idesx8/opensync [0])
);
defparam \DPHY_RX_INST/u_idesx8/opensync_0_s0 .INIT=1'b1;
IODELAY \DPHY_RX_INST/u_idesx8/udel_dataini1  (
	.DI(\DPHY_RX_INST/HS_DATA [1]),
	.SDTAP(GND),
	.VALUE(delay_load_pulse_lane1),
	.DLYSTEP(dphy_tap_shadow_lane1),
	.DO(\DPHY_RX_INST/u_idesx8/dataini_t1 ),
	.DF(\DPHY_RX_INST/u_idesx8/udel_dataini1_1_DF )
);
defparam \DPHY_RX_INST/u_idesx8/udel_dataini1 .C_STATIC_DLY=0;
defparam \DPHY_RX_INST/u_idesx8/udel_dataini1 .DYN_DLY_EN="TRUE";
defparam \DPHY_RX_INST/u_idesx8/udel_dataini1 .ADAPT_EN="FALSE";
IDES8 \DPHY_RX_INST/u_idesx8/Inst4_IDES81  (
	.D(\DPHY_RX_INST/u_idesx8/dataini_t1 ),
	.PCLK(sclk_out),
	.FCLK(\DPHY_RX_INST/u_idesx8/eclko ),
	.RESET(dphy_ides_reset_release_lane1),
	.CALIB(GND),
	.Q0(\DPHY_RX_INST/data1_ides [0]),
	.Q1(\DPHY_RX_INST/data1_ides [1]),
	.Q2(\DPHY_RX_INST/data1_ides [2]),
	.Q3(\DPHY_RX_INST/data1_ides [3]),
	.Q4(\DPHY_RX_INST/data1_ides [4]),
	.Q5(\DPHY_RX_INST/data1_ides [5]),
	.Q6(\DPHY_RX_INST/data1_ides [6]),
	.Q7(\DPHY_RX_INST/data1_ides [7])
);
IODELAY \DPHY_RX_INST/u_idesx8/udel_dataini0  (
	.DI(\DPHY_RX_INST/HS_DATA [0]),
	.SDTAP(GND),
	.VALUE(delay_load_pulse_lane0),
	.DLYSTEP(dphy_tap_shadow_lane0),
	.DO(\DPHY_RX_INST/u_idesx8/dataini_t0 ),
	.DF(\DPHY_RX_INST/u_idesx8/udel_dataini0_1_DF )
);
defparam \DPHY_RX_INST/u_idesx8/udel_dataini0 .C_STATIC_DLY=0;
defparam \DPHY_RX_INST/u_idesx8/udel_dataini0 .DYN_DLY_EN="TRUE";
defparam \DPHY_RX_INST/u_idesx8/udel_dataini0 .ADAPT_EN="FALSE";
IDES8 \DPHY_RX_INST/u_idesx8/Inst4_IDES80  (
	.D(\DPHY_RX_INST/u_idesx8/dataini_t0 ),
	.PCLK(sclk_out),
	.FCLK(\DPHY_RX_INST/u_idesx8/eclko ),
	.RESET(dphy_ides_reset_release_lane0),
	.CALIB(GND),
	.Q0(\DPHY_RX_INST/data0_ides [0]),
	.Q1(\DPHY_RX_INST/data0_ides [1]),
	.Q2(\DPHY_RX_INST/data0_ides [2]),
	.Q3(\DPHY_RX_INST/data0_ides [3]),
	.Q4(\DPHY_RX_INST/data0_ides [4]),
	.Q5(\DPHY_RX_INST/data0_ides [5]),
	.Q6(\DPHY_RX_INST/data0_ides [6]),
	.Q7(\DPHY_RX_INST/data0_ides [7])
);
DHCE \DPHY_RX_INST/u_idesx8/u_DHCEN  (
	.CLKIN(dphy_hs_clk_static32),
	.CEN(\DPHY_RX_INST/u_idesx8/xstop ),
	.CLKOUT(\DPHY_RX_INST/u_idesx8/eclko )
);
CLKDIV \DPHY_RX_INST/u_idesx8/Inst3_CLKDIV  (
	.HCLKIN(\DPHY_RX_INST/u_idesx8/eclko ),
	.RESETN(dphy_reset_release_sync),
	.CALIB(GND),
	.CLKOUT(sclk_out)
);
defparam \DPHY_RX_INST/u_idesx8/Inst3_CLKDIV .DIV_MODE="4";
LUT4 \DPHY_RX_INST/u_Aligner/lane_ready_val_w_s0  (
	.I0(data_out0[0]),
	.I1(data_out0[1]),
	.I2(\DPHY_RX_INST/u_Aligner/lane_ready_val_w_5 ),
	.I3(\DPHY_RX_INST/u_Aligner/lane_ready_val_w_6 ),
	.F(\DPHY_RX_INST/u_Aligner/lane_ready_val_w )
);
defparam \DPHY_RX_INST/u_Aligner/lane_ready_val_w_s0 .INIT=16'h1000;
LUT3 \DPHY_RX_INST/u_Aligner/ready_d_1_s  (
	.I0(\DPHY_RX_INST/u_Aligner/lane_ready_val_w ),
	.I1(\DPHY_RX_INST/u_Aligner/lane_ready_val ),
	.I2(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.F(ready[0])
);
defparam \DPHY_RX_INST/u_Aligner/ready_d_1_s .INIT=8'hE0;
LUT2 \DPHY_RX_INST/u_Aligner/n116_s1  (
	.I0(dphy_byte_reset_release_sync),
	.I1(data_hs_en),
	.F(\DPHY_RX_INST/u_Aligner/n116_5 )
);
defparam \DPHY_RX_INST/u_Aligner/n116_s1 .INIT=4'h7;
LUT3 \DPHY_RX_INST/u_Aligner/lane_ready_val_w_s1  (
	.I0(data_out0[6]),
	.I1(data_out0[7]),
	.I2(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.F(\DPHY_RX_INST/u_Aligner/lane_ready_val_w_5 )
);
defparam \DPHY_RX_INST/u_Aligner/lane_ready_val_w_s1 .INIT=8'h40;
LUT4 \DPHY_RX_INST/u_Aligner/lane_ready_val_w_s2  (
	.I0(data_out0[2]),
	.I1(data_out0[3]),
	.I2(data_out0[4]),
	.I3(data_out0[5]),
	.F(\DPHY_RX_INST/u_Aligner/lane_ready_val_w_6 )
);
defparam \DPHY_RX_INST/u_Aligner/lane_ready_val_w_s2 .INIT=16'h4000;
DFFCE \DPHY_RX_INST/u_Aligner/lane_ready_val_s0  (
	.D(VCC),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/lane_ready_val_w ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/lane_ready_val )
);
defparam \DPHY_RX_INST/u_Aligner/lane_ready_val_s0 .INIT=1'b0;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n428_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/n428_4 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n428_5 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n428_6 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n428_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n428_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n428_s0 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n429_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/n429_4 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n428_5 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n428_6 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n428_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n429_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n429_s0 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n430_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/n430_4 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n428_5 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n428_6 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n428_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n430_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n430_s0 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n431_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/n431_4 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n428_5 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n428_6 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n428_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n431_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n431_s0 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n432_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/n432_4 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n428_5 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n428_6 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n428_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n432_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n432_s0 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n433_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/n433_4 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n428_5 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n428_6 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n428_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n433_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n433_s0 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n434_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/n434_4 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n428_5 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n428_6 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n428_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n434_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n434_s0 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n427_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/n427_4 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n428_5 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n428_6 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n428_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n427_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n427_s0 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n330_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/shftamt [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg4 [2]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n330_4 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n330_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n330_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n330_s0 .INIT=16'hFFF8;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n331_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/shftamt [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg4 [1]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n331_4 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n331_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n331_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n331_s0 .INIT=16'hFFF8;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n332_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/shftamt [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n332_4 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n332_5 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n332_6 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n332_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n332_s0 .INIT=16'hEF00;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n333_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/shftamt [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg3 [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n333_4 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n333_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n333_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n333_s0 .INIT=16'hFFF8;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n334_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/shftamt [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg3 [6]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n334_4 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n334_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n334_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n334_s0 .INIT=16'hFFF8;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n335_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/shftamt [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg3 [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n335_4 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n335_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n335_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n335_s0 .INIT=16'hFFF8;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n336_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/shftamt [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg3 [4]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n336_4 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n336_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n336_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n336_s0 .INIT=16'hFFF8;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n337_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/shftamt [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n337_4 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n337_5 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n337_6 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n337_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n337_s0 .INIT=16'hEF00;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n428_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg1 [3]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg1 [4]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg1 [2]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n428_8 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n428_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n428_s1 .INIT=16'h4000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n428_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/n428_9 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n428_10 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n428_11 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n428_12 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n428_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n428_s2 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n428_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/n428_13 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n428_14 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n428_15 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n428_16 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n428_6 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n428_s3 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n428_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg5 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg5 [1]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n428_17 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n428_18 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n428_7 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n428_s4 .INIT=16'h1000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n429_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg1 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg1 [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg1 [1]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n429_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n429_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n429_s1 .INIT=16'h4000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n430_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg1 [1]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg1 [2]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg1 [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n429_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n430_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n430_s1 .INIT=16'h4000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n431_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg1 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg1 [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg0 [7]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n431_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n431_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n431_s1 .INIT=16'h4000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n432_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg0 [7]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg1 [0]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg0 [6]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n431_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n432_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n432_s1 .INIT=16'h4000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n433_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg0 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg1 [1]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg0 [5]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n433_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n433_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n433_s1 .INIT=16'h4000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n434_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg0 [5]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg0 [6]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg0 [4]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n433_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n434_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n434_s1 .INIT=16'h4000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n427_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg1 [4]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg1 [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg1 [7]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n428_8 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n427_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n427_s1 .INIT=16'h4000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n330_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/n330_6 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n330_7 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n428_18 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n330_8 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n330_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n330_s1 .INIT=16'h7000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n330_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg4 [1]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n330_9 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/shftamt [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n330_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n330_s2 .INIT=16'h0A03;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n331_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/n330_6 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n331_6 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n428_18 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n331_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n331_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n331_s1 .INIT=16'h7000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n331_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg4 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n331_8 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/shftamt [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n331_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n331_s2 .INIT=16'h0A03;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n332_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/n332_7 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n332_8 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/shftamt [3]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n332_9 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n332_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n332_s1 .INIT=16'h0D00;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n332_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/shftamt [3]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg3 [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/shftamt [1]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [0]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n332_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n332_s2 .INIT=16'h0007;
LUT3 \DPHY_RX_INST/u_Aligner/wd0/n332_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg4 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n332_10 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/shftamt [0]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n332_6 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n332_s3 .INIT=8'hAC;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n333_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/n330_6 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n333_6 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n428_18 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n333_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n333_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n333_s1 .INIT=16'h7000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n333_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n333_8 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/shftamt [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n333_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n333_s2 .INIT=16'h0A03;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n334_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/n330_6 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n334_6 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n428_18 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n334_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n334_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n334_s1 .INIT=16'h7000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n334_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [5]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n334_8 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/shftamt [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n334_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n334_s2 .INIT=16'h0A03;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n335_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/n330_6 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n335_6 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n428_18 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n335_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n335_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n335_s1 .INIT=16'h7000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n335_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [4]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n335_8 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/shftamt [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n335_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n335_s2 .INIT=16'h0A03;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n336_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/n330_6 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n336_6 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/n428_18 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n336_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n336_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n336_s1 .INIT=16'h7000;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n336_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [3]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n336_8 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/shftamt [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n336_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n336_s2 .INIT=16'h0A03;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n337_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/n337_7 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n337_8 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/shftamt [3]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/n337_9 ),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n337_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n337_s1 .INIT=16'h0D00;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n337_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/shftamt [3]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg3 [0]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/shftamt [1]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [0]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n337_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n337_s2 .INIT=16'h0007;
LUT3 \DPHY_RX_INST/u_Aligner/wd0/n337_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [3]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/n337_10 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/shftamt [0]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n337_6 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n337_s3 .INIT=8'hAC;
LUT2 \DPHY_RX_INST/u_Aligner/wd0/n428_s5  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg1 [5]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg1 [6]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n428_8 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n428_s5 .INIT=4'h8;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n428_s6  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg4 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg4 [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg4 [4]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/reg4 [5]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n428_9 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n428_s6 .INIT=16'h0001;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n428_s7  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg5 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg5 [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg4 [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/reg4 [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n428_10 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n428_s7 .INIT=16'h0001;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n428_s8  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg4 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg4 [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg3 [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/reg3 [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n428_11 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n428_s8 .INIT=16'h0001;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n428_s9  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg5 [4]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg5 [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/shftamt [5]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n428_12 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n428_s9 .INIT=16'h0001;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n428_s10  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg2 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg2 [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg2 [4]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/reg2 [5]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n428_13 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n428_s10 .INIT=16'h0001;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n428_s11  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg3 [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg3 [4]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/reg3 [5]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n428_14 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n428_s11 .INIT=16'h0001;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n428_s12  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg2 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg2 [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/shftamt [6]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [7]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n428_15 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n428_s12 .INIT=16'h0001;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n428_s13  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg3 [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg2 [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/reg2 [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n428_16 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n428_s13 .INIT=16'h0001;
LUT2 \DPHY_RX_INST/u_Aligner/wd0/n428_s14  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg5 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg5 [3]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n428_17 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n428_s14 .INIT=4'h1;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n428_s15  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/shftamt [3]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [2]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/shftamt [1]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [0]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n428_18 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n428_s15 .INIT=16'h0001;
LUT2 \DPHY_RX_INST/u_Aligner/wd0/n429_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg1 [3]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg1 [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n429_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n429_s2 .INIT=4'h8;
LUT2 \DPHY_RX_INST/u_Aligner/wd0/n431_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg1 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg1 [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n431_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n431_s2 .INIT=4'h8;
LUT2 \DPHY_RX_INST/u_Aligner/wd0/n433_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg1 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg0 [7]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n433_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n433_s2 .INIT=4'h8;
LUT2 \DPHY_RX_INST/u_Aligner/wd0/n330_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/shftamt [5]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n330_6 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n330_s3 .INIT=4'h1;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n330_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [3]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg3 [4]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [6]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n330_7 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n330_s4 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n330_s5  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [5]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg3 [6]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n330_8 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n330_s5 .INIT=16'hF0BB;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n330_s6  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [7]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg4 [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [2]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n330_9 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n330_s6 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n331_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg3 [3]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [6]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n331_6 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n331_s3 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n331_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [4]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg3 [5]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n331_7 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n331_s4 .INIT=16'hF0BB;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n331_s5  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg3 [7]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [2]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n331_8 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n331_s5 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n332_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/shftamt [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg3 [2]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/shftamt [5]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n332_7 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n332_s4 .INIT=16'h0007;
LUT3 \DPHY_RX_INST/u_Aligner/wd0/n332_s5  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/shftamt [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg3 [1]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/shftamt [7]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n332_8 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n332_s5 .INIT=8'h40;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n332_s6  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [3]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg3 [4]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n332_9 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n332_s6 .INIT=16'hF0BB;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n332_s7  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [2]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg3 [7]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n332_10 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n332_s7 .INIT=16'hF0BB;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n333_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg3 [1]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [6]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n333_6 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n333_s3 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n333_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg3 [3]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n333_7 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n333_s4 .INIT=16'hF0BB;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n333_s5  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [4]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg3 [5]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [2]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n333_8 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n333_s5 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n334_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg2 [7]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg3 [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [6]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n334_6 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n334_s3 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n334_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [1]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg3 [2]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n334_7 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n334_s4 .INIT=16'hF0BB;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n334_s5  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [3]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg3 [4]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [2]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n334_8 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n334_s5 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n335_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg2 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg2 [7]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [6]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n335_6 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n335_s3 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n335_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg3 [1]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n335_7 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n335_s4 .INIT=16'hF0BB;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n335_s5  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg3 [3]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [2]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n335_8 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n335_s5 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n336_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg2 [5]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg2 [6]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [6]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n336_6 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n336_s3 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n336_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg2 [7]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg3 [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n336_7 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n336_s4 .INIT=16'hF0BB;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n336_s5  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [1]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg3 [2]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [2]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n336_8 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n336_s5 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n337_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/shftamt [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg2 [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/shftamt [5]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n337_7 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n337_s4 .INIT=16'h0007;
LUT3 \DPHY_RX_INST/u_Aligner/wd0/n337_s5  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/shftamt [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/reg2 [4]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/shftamt [7]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n337_8 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n337_s5 .INIT=8'h40;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n337_s6  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg2 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg2 [7]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n337_9 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n337_s6 .INIT=16'hF0BB;
LUT4 \DPHY_RX_INST/u_Aligner/wd0/n337_s7  (
	.I0(\DPHY_RX_INST/u_Aligner/wd0/reg3 [1]),
	.I1(\DPHY_RX_INST/u_Aligner/wd0/shftamt [2]),
	.I2(\DPHY_RX_INST/u_Aligner/wd0/reg3 [2]),
	.I3(\DPHY_RX_INST/u_Aligner/wd0/shftamt [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd0/n337_10 )
);
defparam \DPHY_RX_INST/u_Aligner/wd0/n337_s7 .INIT=16'hF0BB;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg0_6_s0  (
	.D(\DPHY_RX_INST/data0 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg0 [6])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg0_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg0_5_s0  (
	.D(\DPHY_RX_INST/data0 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg0 [5])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg0_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg0_4_s0  (
	.D(\DPHY_RX_INST/data0 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg0 [4])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg0_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg0_3_s0  (
	.D(\DPHY_RX_INST/data0 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg0 [3])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg0_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg0_2_s0  (
	.D(\DPHY_RX_INST/data0 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg0 [2])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg0_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg0_1_s0  (
	.D(\DPHY_RX_INST/data0 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg0 [1])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg0_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg0_0_s0  (
	.D(\DPHY_RX_INST/data0 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg0 [0])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg0_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg1_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg0 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg1 [7])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg1_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg1_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg0 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg1 [6])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg1_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg1_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg0 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg1 [5])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg1_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg1_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg0 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg1 [4])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg1_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg1_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg0 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg1 [3])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg1_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg1_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg0 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg1 [2])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg1_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg1_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg0 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg1 [1])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg1_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg1_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg0 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg1 [0])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg1_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg2_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg1 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg2 [7])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg2_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg2_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg1 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg2 [6])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg2_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg2_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg1 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg2 [5])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg2_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg2_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg1 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg2 [4])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg2_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg2_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg1 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg2 [3])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg2_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg2_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg1 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg2 [2])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg2_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg2_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg1 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg2 [1])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg2_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg2_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg1 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg2 [0])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg2_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg3_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg2 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg3 [7])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg3_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg3_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg2 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg3 [6])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg3_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg3_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg2 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg3 [5])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg3_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg3_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg2 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg3 [4])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg3_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg3_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg2 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg3 [3])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg3_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg3_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg2 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg3 [2])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg3_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg3_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg2 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg3 [1])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg3_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg3_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg2 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg3 [0])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg3_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg4_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg3 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg4 [7])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg4_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg4_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg3 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg4 [6])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg4_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg4_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg3 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg4 [5])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg4_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg4_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg3 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg4 [4])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg4_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg4_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg3 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg4 [3])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg4_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg4_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg3 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg4 [2])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg4_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg4_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg3 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg4 [1])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg4_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg4_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg3 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg4 [0])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg4_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg5_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg4 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg5 [7])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg5_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg5_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg4 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg5 [6])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg5_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg5_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg4 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg5 [5])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg5_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg5_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg4 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg5 [4])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg5_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg5_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg4 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg5 [3])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg5_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg5_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg4 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg5 [2])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg5_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg5_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg4 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg5 [1])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg5_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg5_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/reg4 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg5 [0])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg5_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/shftamt_0_s0  (
	.D(VCC),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/wd0/n427_3 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/shftamt [0])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/shftamt_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/shftamt_1_s0  (
	.D(VCC),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/wd0/n428_3 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/shftamt [1])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/shftamt_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/shftamt_2_s0  (
	.D(VCC),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/wd0/n429_3 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/shftamt [2])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/shftamt_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/shftamt_3_s0  (
	.D(VCC),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/wd0/n430_3 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/shftamt [3])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/shftamt_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/shftamt_4_s0  (
	.D(VCC),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/wd0/n431_3 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/shftamt [4])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/shftamt_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/shftamt_5_s0  (
	.D(VCC),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/wd0/n432_3 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/shftamt [5])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/shftamt_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/shftamt_6_s0  (
	.D(VCC),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/wd0/n433_3 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/shftamt [6])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/shftamt_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/shftamt_7_s0  (
	.D(VCC),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/wd0/n434_3 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/shftamt [7])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/shftamt_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/dout_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/n330_3 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/lane0 [0])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/dout_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/dout_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/n331_3 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/lane0 [1])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/dout_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/dout_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/n332_3 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/lane0 [2])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/dout_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/dout_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/n333_3 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/lane0 [3])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/dout_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/dout_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/n334_3 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/lane0 [4])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/dout_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/dout_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/n335_3 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/lane0 [5])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/dout_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/dout_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/n336_3 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/lane0 [6])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/dout_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/dout_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd0/n337_3 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/lane0 [7])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/dout_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd0/reg0_7_s0  (
	.D(\DPHY_RX_INST/data0 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd0/reg0 [7])
);
defparam \DPHY_RX_INST/u_Aligner/wd0/reg0_7_s0 .INIT=1'b0;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n428_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/n428_4 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n428_5 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n428_6 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n428_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n428_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n428_s0 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n429_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/n429_4 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n428_5 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n428_6 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n428_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n429_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n429_s0 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n430_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/n430_4 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n428_5 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n428_6 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n428_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n430_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n430_s0 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n431_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/n431_4 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n428_5 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n428_6 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n428_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n431_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n431_s0 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n432_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/n432_4 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n428_5 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n428_6 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n428_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n432_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n432_s0 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n433_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/n433_4 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n428_5 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n428_6 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n428_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n433_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n433_s0 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n434_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/n434_4 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n428_5 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n428_6 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n428_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n434_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n434_s0 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n427_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/n427_4 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n428_5 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n428_6 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n428_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n427_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n427_s0 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n330_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/shftamt [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg4 [2]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n330_4 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n330_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n330_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n330_s0 .INIT=16'hFFF8;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n331_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/shftamt [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg4 [1]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n331_4 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n331_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n331_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n331_s0 .INIT=16'hFFF8;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n332_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/shftamt [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg4 [0]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n332_4 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n332_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n332_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n332_s0 .INIT=16'hFFF8;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n333_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/shftamt [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n333_4 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n333_5 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n333_6 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n333_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n333_s0 .INIT=16'hEF00;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n334_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/shftamt [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg3 [6]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n334_4 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n334_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n334_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n334_s0 .INIT=16'hFFF8;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n335_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/shftamt [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg3 [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n335_4 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n335_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n335_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n335_s0 .INIT=16'hFFF8;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n336_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/shftamt [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg3 [4]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n336_4 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n336_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n336_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n336_s0 .INIT=16'hFFF8;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n337_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/shftamt [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n337_4 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n337_5 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n337_6 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n337_3 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n337_s0 .INIT=16'hEF00;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n428_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg1 [3]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg1 [4]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg1 [2]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n428_8 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n428_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n428_s1 .INIT=16'h4000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n428_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/n428_9 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n428_10 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n428_11 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n428_12 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n428_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n428_s2 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n428_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/n428_13 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n428_14 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n428_15 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n428_16 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n428_6 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n428_s3 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n428_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg5 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg5 [1]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n428_17 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n428_18 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n428_7 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n428_s4 .INIT=16'h1000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n429_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg1 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg1 [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg1 [1]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n429_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n429_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n429_s1 .INIT=16'h4000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n430_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg1 [1]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg1 [2]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg1 [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n429_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n430_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n430_s1 .INIT=16'h4000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n431_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg1 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg1 [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg0 [7]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n431_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n431_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n431_s1 .INIT=16'h4000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n432_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg0 [7]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg1 [0]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg0 [6]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n431_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n432_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n432_s1 .INIT=16'h4000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n433_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg0 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg1 [1]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg0 [5]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n433_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n433_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n433_s1 .INIT=16'h4000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n434_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg0 [5]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg0 [6]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg0 [4]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n433_5 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n434_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n434_s1 .INIT=16'h4000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n427_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg1 [4]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg1 [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg1 [7]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n428_8 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n427_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n427_s1 .INIT=16'h4000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n330_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/n330_6 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n330_7 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n428_18 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n330_8 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n330_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n330_s1 .INIT=16'h7000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n330_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg4 [1]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n330_9 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/shftamt [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n330_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n330_s2 .INIT=16'h0A03;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n331_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/n330_6 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n331_6 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n428_18 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n331_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n331_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n331_s1 .INIT=16'h7000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n331_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg4 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n331_8 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/shftamt [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n331_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n331_s2 .INIT=16'h0A03;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n332_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/n330_6 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n332_6 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n428_18 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n332_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n332_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n332_s1 .INIT=16'h7000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n332_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [7]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n332_8 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/shftamt [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n332_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n332_s2 .INIT=16'h0A03;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n333_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/n333_7 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n333_8 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/shftamt [3]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n333_9 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n333_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n333_s1 .INIT=16'h0D00;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n333_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/shftamt [3]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg3 [4]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/shftamt [1]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [0]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n333_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n333_s2 .INIT=16'h0007;
LUT3 \DPHY_RX_INST/u_Aligner/wd1/n333_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [7]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n333_10 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/shftamt [0]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n333_6 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n333_s3 .INIT=8'hAC;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n334_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/n330_6 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n334_6 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n428_18 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n334_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n334_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n334_s1 .INIT=16'h7000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n334_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [5]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n334_8 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/shftamt [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n334_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n334_s2 .INIT=16'h0A03;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n335_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/n330_6 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n335_6 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n428_18 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n335_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n335_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n335_s1 .INIT=16'h7000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n335_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [4]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n335_8 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/shftamt [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n335_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n335_s2 .INIT=16'h0A03;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n336_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/n330_6 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n336_6 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/n428_18 ),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n336_7 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n336_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n336_s1 .INIT=16'h7000;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n336_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [3]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n336_8 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/shftamt [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n336_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n336_s2 .INIT=16'h0A03;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n337_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/n337_7 ),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n337_8 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/shftamt [3]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/n337_9 ),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n337_4 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n337_s1 .INIT=16'h0D00;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n337_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/shftamt [3]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg3 [0]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/shftamt [1]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [0]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n337_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n337_s2 .INIT=16'h0007;
LUT3 \DPHY_RX_INST/u_Aligner/wd1/n337_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [3]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/n337_10 ),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/shftamt [0]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n337_6 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n337_s3 .INIT=8'hAC;
LUT2 \DPHY_RX_INST/u_Aligner/wd1/n428_s5  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg1 [5]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg1 [6]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n428_8 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n428_s5 .INIT=4'h8;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n428_s6  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg2 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg2 [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg2 [4]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/reg2 [5]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n428_9 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n428_s6 .INIT=16'h0001;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n428_s7  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg3 [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg3 [4]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/reg3 [5]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n428_10 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n428_s7 .INIT=16'h0001;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n428_s8  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg2 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg2 [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/shftamt [6]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [7]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n428_11 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n428_s8 .INIT=16'h0001;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n428_s9  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg3 [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg2 [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/reg2 [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n428_12 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n428_s9 .INIT=16'h0001;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n428_s10  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg4 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg4 [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg4 [4]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/reg4 [5]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n428_13 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n428_s10 .INIT=16'h0001;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n428_s11  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg5 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg5 [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg4 [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/reg4 [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n428_14 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n428_s11 .INIT=16'h0001;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n428_s12  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg4 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg4 [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg3 [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/reg3 [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n428_15 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n428_s12 .INIT=16'h0001;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n428_s13  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg5 [4]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg5 [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/shftamt [5]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n428_16 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n428_s13 .INIT=16'h0001;
LUT2 \DPHY_RX_INST/u_Aligner/wd1/n428_s14  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg5 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg5 [3]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n428_17 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n428_s14 .INIT=4'h1;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n428_s15  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/shftamt [3]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [2]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/shftamt [1]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [0]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n428_18 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n428_s15 .INIT=16'h0001;
LUT2 \DPHY_RX_INST/u_Aligner/wd1/n429_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg1 [3]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg1 [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n429_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n429_s2 .INIT=4'h8;
LUT2 \DPHY_RX_INST/u_Aligner/wd1/n431_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg1 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg1 [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n431_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n431_s2 .INIT=4'h8;
LUT2 \DPHY_RX_INST/u_Aligner/wd1/n433_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg1 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg0 [7]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n433_5 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n433_s2 .INIT=4'h8;
LUT2 \DPHY_RX_INST/u_Aligner/wd1/n330_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/shftamt [5]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n330_6 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n330_s3 .INIT=4'h1;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n330_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [3]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg3 [4]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [6]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n330_7 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n330_s4 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n330_s5  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [5]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg3 [6]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n330_8 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n330_s5 .INIT=16'hF0BB;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n330_s6  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [7]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg4 [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [2]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n330_9 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n330_s6 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n331_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg3 [3]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [6]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n331_6 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n331_s3 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n331_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [4]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg3 [5]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n331_7 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n331_s4 .INIT=16'hF0BB;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n331_s5  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg3 [7]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [2]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n331_8 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n331_s5 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n332_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [1]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg3 [2]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [6]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n332_6 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n332_s3 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n332_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [3]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg3 [4]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n332_7 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n332_s4 .INIT=16'hF0BB;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n332_s5  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [5]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg3 [6]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [2]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n332_8 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n332_s5 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n333_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/shftamt [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg3 [1]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/shftamt [5]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n333_7 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n333_s4 .INIT=16'h0007;
LUT3 \DPHY_RX_INST/u_Aligner/wd1/n333_s5  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/shftamt [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg3 [0]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/shftamt [7]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n333_8 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n333_s5 .INIT=8'h40;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n333_s6  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg3 [3]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n333_9 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n333_s6 .INIT=16'hF0BB;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n333_s7  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [5]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [2]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg3 [6]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n333_10 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n333_s7 .INIT=16'hF0BB;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n334_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg2 [7]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg3 [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [6]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n334_6 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n334_s3 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n334_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [1]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg3 [2]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n334_7 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n334_s4 .INIT=16'hF0BB;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n334_s5  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [3]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg3 [4]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [2]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n334_8 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n334_s5 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n335_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg2 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg2 [7]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [6]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n335_6 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n335_s3 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n335_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg3 [1]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n335_7 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n335_s4 .INIT=16'hF0BB;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n335_s5  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg3 [3]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [2]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n335_8 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n335_s5 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n336_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg2 [5]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [7]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg2 [6]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [6]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n336_6 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n336_s3 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n336_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg2 [7]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg3 [0]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n336_7 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n336_s4 .INIT=16'hF0BB;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n336_s5  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [1]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [3]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg3 [2]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [2]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n336_8 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n336_s5 .INIT=16'h0F77;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n337_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/shftamt [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg2 [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/shftamt [5]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n337_7 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n337_s4 .INIT=16'h0007;
LUT3 \DPHY_RX_INST/u_Aligner/wd1/n337_s5  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/shftamt [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/reg2 [4]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/shftamt [7]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n337_8 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n337_s5 .INIT=8'h40;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n337_s6  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg2 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [5]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg2 [7]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [4]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n337_9 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n337_s6 .INIT=16'hF0BB;
LUT4 \DPHY_RX_INST/u_Aligner/wd1/n337_s7  (
	.I0(\DPHY_RX_INST/u_Aligner/wd1/reg3 [1]),
	.I1(\DPHY_RX_INST/u_Aligner/wd1/shftamt [2]),
	.I2(\DPHY_RX_INST/u_Aligner/wd1/reg3 [2]),
	.I3(\DPHY_RX_INST/u_Aligner/wd1/shftamt [1]),
	.F(\DPHY_RX_INST/u_Aligner/wd1/n337_10 )
);
defparam \DPHY_RX_INST/u_Aligner/wd1/n337_s7 .INIT=16'hF0BB;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg0_6_s0  (
	.D(\DPHY_RX_INST/data1 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg0 [6])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg0_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg0_5_s0  (
	.D(\DPHY_RX_INST/data1 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg0 [5])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg0_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg0_4_s0  (
	.D(\DPHY_RX_INST/data1 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg0 [4])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg0_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg0_3_s0  (
	.D(\DPHY_RX_INST/data1 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg0 [3])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg0_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg0_2_s0  (
	.D(\DPHY_RX_INST/data1 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg0 [2])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg0_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg0_1_s0  (
	.D(\DPHY_RX_INST/data1 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg0 [1])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg0_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg0_0_s0  (
	.D(\DPHY_RX_INST/data1 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg0 [0])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg0_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg1_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg0 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg1 [7])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg1_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg1_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg0 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg1 [6])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg1_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg1_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg0 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg1 [5])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg1_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg1_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg0 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg1 [4])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg1_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg1_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg0 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg1 [3])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg1_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg1_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg0 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg1 [2])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg1_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg1_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg0 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg1 [1])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg1_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg1_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg0 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg1 [0])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg1_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg2_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg1 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg2 [7])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg2_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg2_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg1 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg2 [6])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg2_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg2_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg1 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg2 [5])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg2_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg2_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg1 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg2 [4])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg2_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg2_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg1 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg2 [3])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg2_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg2_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg1 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg2 [2])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg2_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg2_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg1 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg2 [1])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg2_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg2_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg1 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg2 [0])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg2_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg3_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg2 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg3 [7])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg3_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg3_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg2 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg3 [6])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg3_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg3_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg2 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg3 [5])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg3_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg3_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg2 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg3 [4])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg3_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg3_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg2 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg3 [3])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg3_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg3_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg2 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg3 [2])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg3_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg3_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg2 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg3 [1])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg3_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg3_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg2 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg3 [0])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg3_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg4_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg3 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg4 [7])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg4_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg4_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg3 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg4 [6])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg4_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg4_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg3 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg4 [5])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg4_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg4_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg3 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg4 [4])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg4_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg4_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg3 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg4 [3])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg4_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg4_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg3 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg4 [2])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg4_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg4_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg3 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg4 [1])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg4_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg4_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg3 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg4 [0])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg4_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg5_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg4 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg5 [7])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg5_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg5_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg4 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg5 [6])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg5_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg5_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg4 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg5 [5])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg5_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg5_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg4 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg5 [4])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg5_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg5_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg4 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg5 [3])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg5_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg5_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg4 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg5 [2])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg5_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg5_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg4 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg5 [1])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg5_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg5_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/reg4 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg5 [0])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg5_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/shftamt_0_s0  (
	.D(VCC),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/wd1/n427_3 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/shftamt [0])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/shftamt_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/shftamt_1_s0  (
	.D(VCC),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/wd1/n428_3 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/shftamt [1])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/shftamt_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/shftamt_2_s0  (
	.D(VCC),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/wd1/n429_3 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/shftamt [2])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/shftamt_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/shftamt_3_s0  (
	.D(VCC),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/wd1/n430_3 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/shftamt [3])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/shftamt_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/shftamt_4_s0  (
	.D(VCC),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/wd1/n431_3 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/shftamt [4])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/shftamt_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/shftamt_5_s0  (
	.D(VCC),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/wd1/n432_3 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/shftamt [5])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/shftamt_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/shftamt_6_s0  (
	.D(VCC),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/wd1/n433_3 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/shftamt [6])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/shftamt_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/shftamt_7_s0  (
	.D(VCC),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/wd1/n434_3 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/shftamt [7])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/shftamt_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/dout_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/n330_3 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/lane1 [0])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/dout_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/dout_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/n331_3 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/lane1 [1])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/dout_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/dout_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/n332_3 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/lane1 [2])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/dout_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/dout_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/n333_3 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/lane1 [3])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/dout_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/dout_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/n334_3 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/lane1 [4])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/dout_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/dout_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/n335_3 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/lane1 [5])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/dout_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/dout_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/n336_3 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/lane1 [6])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/dout_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/dout_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/wd1/n337_3 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/lane1 [7])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/dout_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/wd1/reg0_7_s0  (
	.D(\DPHY_RX_INST/data1 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/wd1/reg0 [7])
);
defparam \DPHY_RX_INST/u_Aligner/wd1/reg0_7_s0 .INIT=1'b0;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n17_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [4]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [5]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n17_4 ),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/n17_5 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n17_3 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n17_s0 .INIT=16'h1000;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n91_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [4]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [5]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n91_4 ),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/n91_5 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n91_3 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n91_s0 .INIT=16'h1000;
LUT2 \DPHY_RX_INST/u_Aligner/ln0/n160_s0  (
	.I0(dphy_sync1_internal),
	.I1(dphy_sync0_internal),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n160_3 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n160_s0 .INIT=4'h8;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n260_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/n260_7 ),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n260_8 ),
	.I3(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n260_6 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n260_s1 .INIT=16'hA300;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n259_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [1]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/n259_7 ),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n260_8 ),
	.I3(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n259_6 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n259_s1 .INIT=16'hA300;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n258_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/n258_7 ),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n260_8 ),
	.I3(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n258_6 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n258_s1 .INIT=16'hA300;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n257_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [3]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/n257_7 ),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n260_8 ),
	.I3(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n257_6 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n257_s1 .INIT=16'hA300;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n256_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [4]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/n256_7 ),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n260_8 ),
	.I3(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n256_6 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n256_s1 .INIT=16'hA300;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n255_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [5]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/n255_7 ),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n260_8 ),
	.I3(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n255_6 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n255_s1 .INIT=16'hA300;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n254_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/n254_7 ),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n260_8 ),
	.I3(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n254_6 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n254_s1 .INIT=16'hA300;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n253_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [7]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/n253_7 ),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n260_8 ),
	.I3(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n253_6 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n253_s1 .INIT=16'hA300;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n211_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_4 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/n211_7 ),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n211_8 ),
	.I3(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n211_6 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n211_s1 .INIT=16'hA300;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n210_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_4 [1]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/n210_7 ),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n211_8 ),
	.I3(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n210_6 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n210_s1 .INIT=16'hA300;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n209_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_4 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/n209_7 ),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n211_8 ),
	.I3(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n209_6 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n209_s1 .INIT=16'hA300;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n208_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_4 [3]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/n208_7 ),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n211_8 ),
	.I3(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n208_6 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n208_s1 .INIT=16'hA300;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n207_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_4 [4]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/n207_7 ),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n211_8 ),
	.I3(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n207_6 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n207_s1 .INIT=16'hA300;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n206_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_4 [5]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/n206_7 ),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n211_8 ),
	.I3(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n206_6 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n206_s1 .INIT=16'hA300;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n205_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_4 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/n205_7 ),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n211_8 ),
	.I3(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n205_6 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n205_s1 .INIT=16'hA300;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n204_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_4 [7]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/n204_7 ),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n211_8 ),
	.I3(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n204_6 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n204_s1 .INIT=16'hA300;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n21_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [0]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [1]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [2]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n21_4 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n21_s0 .INIT=8'h78;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n20_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [0]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [1]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [2]),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [3]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n20_4 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n20_s0 .INIT=16'h7F80;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n95_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [0]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [1]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [2]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n95_4 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n95_s0 .INIT=8'h78;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n94_s0  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [0]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [1]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [2]),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [3]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n94_4 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n94_s0 .INIT=16'h7F80;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n17_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [1]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [2]),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/n17_6 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n17_4 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n17_s1 .INIT=16'h0100;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n17_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/n17_7 ),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/n17_8 ),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n17_9 ),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/n17_10 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n17_5 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n17_s2 .INIT=16'h8000;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n91_s1  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [1]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [2]),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/n91_6 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n91_4 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n91_s1 .INIT=16'h0100;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n91_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/n91_7 ),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/n91_8 ),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n91_9 ),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/n91_10 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n91_5 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n91_s2 .INIT=16'h8000;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n260_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_4 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [0]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n260_9 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n260_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n260_s2 .INIT=8'h53;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n260_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [1]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [2]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [3]),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [0]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n260_8 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n260_s3 .INIT=16'h0100;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n259_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_4 [1]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [1]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n260_9 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n259_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n259_s2 .INIT=8'h53;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n258_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_4 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [2]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n260_9 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n258_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n258_s2 .INIT=8'h53;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n257_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_4 [3]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [3]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n260_9 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n257_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n257_s2 .INIT=8'h53;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n256_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_4 [4]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [4]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n260_9 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n256_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n256_s2 .INIT=8'h53;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n255_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_4 [5]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [5]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n260_9 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n255_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n255_s2 .INIT=8'h53;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n254_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_4 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [6]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n260_9 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n254_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n254_s2 .INIT=8'h53;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n253_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_4 [7]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [7]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n260_9 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n253_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n253_s2 .INIT=8'h53;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n211_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [0]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n211_9 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n211_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n211_s2 .INIT=8'h53;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n211_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [0]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [2]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [3]),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [1]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n211_8 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n211_s3 .INIT=16'h0100;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n210_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [1]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [1]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n211_9 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n210_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n210_s2 .INIT=8'h53;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n209_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [2]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [2]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n211_9 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n209_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n209_s2 .INIT=8'h53;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n208_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [3]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [3]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n211_9 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n208_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n208_s2 .INIT=8'h53;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n207_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [4]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [4]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n211_9 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n207_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n207_s2 .INIT=8'h53;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n206_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [5]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [5]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n211_9 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n206_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n206_s2 .INIT=8'h53;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n205_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [6]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n211_9 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n205_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n205_s2 .INIT=8'h53;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n204_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [7]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [7]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/n211_9 ),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n204_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n204_s2 .INIT=8'h53;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n17_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [3]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [6]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [7]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n17_6 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n17_s3 .INIT=8'h01;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n17_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [1]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [2]),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [3]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n17_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n17_s4 .INIT=16'h0100;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n17_s5  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [1]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [2]),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [3]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n17_8 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n17_s5 .INIT=16'h0001;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n17_s6  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [5]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [4]),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [7]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n17_9 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n17_s6 .INIT=16'h4000;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n17_s7  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [4]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [5]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [6]),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [7]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n17_10 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n17_s7 .INIT=16'h0001;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n91_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [3]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [6]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [7]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n91_6 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n91_s3 .INIT=8'h01;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n91_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [1]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [2]),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [3]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n91_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n91_s4 .INIT=16'h0100;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n91_s5  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [0]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [1]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [2]),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [3]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n91_8 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n91_s5 .INIT=16'h0001;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n91_s6  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [6]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [5]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [4]),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [7]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n91_9 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n91_s6 .INIT=16'h4000;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n91_s7  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [4]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [5]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [6]),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [7]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n91_10 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n91_s7 .INIT=16'h0001;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n260_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [0]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [2]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [3]),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [1]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n260_9 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n260_s4 .INIT=16'h0100;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n211_s4  (
	.I0(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [1]),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [2]),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [3]),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [0]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n211_9 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n211_s4 .INIT=16'h0100;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n22_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [0]),
	.I2(dphy_sync0_internal),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [1]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n22_10 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n22_s2 .INIT=16'hBF40;
LUT2 \DPHY_RX_INST/u_Aligner/ln0/n390_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.I1(dphy_sync0_internal),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n390_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n390_s2 .INIT=4'h4;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n23_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.I1(dphy_sync0_internal),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [0]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n23_8 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n23_s3 .INIT=8'hB4;
LUT4 \DPHY_RX_INST/u_Aligner/ln0/n96_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.I1(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [0]),
	.I2(dphy_sync1_internal),
	.I3(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [1]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n96_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n96_s2 .INIT=16'hBF40;
LUT2 \DPHY_RX_INST/u_Aligner/ln0/n396_s2  (
	.I0(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.I1(dphy_sync1_internal),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n396_7 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n396_s2 .INIT=4'h4;
LUT3 \DPHY_RX_INST/u_Aligner/ln0/n97_s3  (
	.I0(\DPHY_RX_INST/u_Aligner/ln_ready ),
	.I1(dphy_sync1_internal),
	.I2(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [0]),
	.F(\DPHY_RX_INST/u_Aligner/ln0/n97_8 )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/n97_s3 .INIT=8'hB4;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_0_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/lane0 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [6])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_0_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_0_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/lane0 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [5])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_0_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_0_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/lane0 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [4])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_0_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_0_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/lane0 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [3])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_0_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_0_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/lane0 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [2])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_0_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_0_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/lane0 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [1])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_0_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_0_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/lane0 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [0])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_0_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_1_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [7])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_1_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_1_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [6])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_1_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_1_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [5])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_1_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_1_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [4])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_1_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_1_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [3])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_1_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_1_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [2])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_1_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_1_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [1])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_1_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_1_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [0])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_1_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_2_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [7])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_2_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_2_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [6])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_2_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_2_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [5])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_2_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_2_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [4])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_2_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_2_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [3])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_2_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_2_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [2])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_2_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_2_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [1])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_2_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_2_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_1 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [0])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_2_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_3_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [7])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_3_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_3_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [6])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_3_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_3_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [5])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_3_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_3_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [4])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_3_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_3_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [3])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_3_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_3_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [2])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_3_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_3_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [1])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_3_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_3_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_2 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [0])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_3_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_4_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_4 [7])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_4_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_4_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_4 [6])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_4_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_4_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_4 [5])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_4_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_4_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_4 [4])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_4_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_4_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_4 [3])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_4_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_4_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_4 [2])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_4_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_4_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_4 [1])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_4_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_4_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in0_3 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_4 [0])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_4_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/sync0_s0  (
	.D(VCC),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/ln0/n17_3 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(dphy_sync0_internal)
);
defparam \DPHY_RX_INST/u_Aligner/ln0/sync0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane0_cnt_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n20_4 ),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/ln0/n390_7 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [3])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane0_cnt_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane0_cnt_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n21_4 ),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/ln0/n390_7 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [2])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane0_cnt_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_0_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/lane1 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [7])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_0_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_0_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/lane1 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [6])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_0_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_0_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/lane1 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [5])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_0_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_0_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/lane1 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [4])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_0_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_0_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/lane1 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [3])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_0_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_0_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/lane1 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [2])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_0_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_0_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/lane1 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [1])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_0_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_0_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/lane1 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [0])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_0_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_1_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [7])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_1_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_1_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [6])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_1_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_1_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [5])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_1_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_1_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [4])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_1_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_1_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [3])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_1_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_1_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [2])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_1_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_1_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [1])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_1_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_1_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_0 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [0])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_1_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_2_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [7])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_2_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_2_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [6])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_2_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_2_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [5])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_2_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_2_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [4])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_2_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_2_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [3])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_2_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_2_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [2])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_2_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_2_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [1])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_2_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_2_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_1 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [0])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_2_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_3_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [7])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_3_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_3_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [6])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_3_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_3_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [5])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_3_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_3_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [4])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_3_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_3_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [3])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_3_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_3_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [2])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_3_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_3_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [1])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_3_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_3_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_2 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [0])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_3_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_4_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_4 [7])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_4_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_4_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [6]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_4 [6])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_4_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_4_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [5]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_4 [5])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_4_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_4_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [4]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_4 [4])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_4_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_4_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [3]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_4 [3])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_4_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_4_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [2]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_4 [2])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_4_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_4_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [1]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_4 [1])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_4_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in1_4_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/q_in1_3 [0]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in1_4 [0])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in1_4_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/sync1_s0  (
	.D(VCC),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/ln0/n91_3 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(dphy_sync1_internal)
);
defparam \DPHY_RX_INST/u_Aligner/ln0/sync1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane1_cnt_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n94_4 ),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/ln0/n396_7 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [3])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane1_cnt_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane1_cnt_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n95_4 ),
	.CLK(sclk_out),
	.CE(\DPHY_RX_INST/u_Aligner/ln0/n396_7 ),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [2])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane1_cnt_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/syncs_active_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n160_3 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln_ready )
);
defparam \DPHY_RX_INST/u_Aligner/ln0/syncs_active_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane0_align_data_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n204_6 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(data_out0[7])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane0_align_data_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane0_align_data_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n205_6 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(data_out0[6])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane0_align_data_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane0_align_data_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n206_6 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(data_out0[5])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane0_align_data_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane0_align_data_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n207_6 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(data_out0[4])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane0_align_data_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane0_align_data_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n208_6 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(data_out0[3])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane0_align_data_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane0_align_data_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n209_6 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(data_out0[2])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane0_align_data_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane0_align_data_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n210_6 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(data_out0[1])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane0_align_data_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane0_align_data_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n211_6 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(data_out0[0])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane0_align_data_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane1_align_data_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n253_6 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(data_out1[7])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane1_align_data_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane1_align_data_6_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n254_6 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(data_out1[6])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane1_align_data_6_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane1_align_data_5_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n255_6 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(data_out1[5])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane1_align_data_5_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane1_align_data_4_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n256_6 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(data_out1[4])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane1_align_data_4_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane1_align_data_3_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n257_6 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(data_out1[3])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane1_align_data_3_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane1_align_data_2_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n258_6 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(data_out1[2])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane1_align_data_2_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane1_align_data_1_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n259_6 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(data_out1[1])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane1_align_data_1_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane1_align_data_0_s0  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n260_6 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(data_out1[0])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane1_align_data_0_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/q_in0_0_7_s0  (
	.D(\DPHY_RX_INST/u_Aligner/lane0 [7]),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/q_in0_0 [7])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/q_in0_0_7_s0 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane0_cnt_1_s1  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n22_10 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [1])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane0_cnt_1_s1 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane0_cnt_0_s1  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n23_8 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/lane0_cnt [0])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane0_cnt_0_s1 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane1_cnt_1_s1  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n96_7 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [1])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane1_cnt_1_s1 .INIT=1'b0;
DFFCE \DPHY_RX_INST/u_Aligner/ln0/lane1_cnt_0_s1  (
	.D(\DPHY_RX_INST/u_Aligner/ln0/n97_8 ),
	.CLK(sclk_out),
	.CE(VCC),
	.CLEAR(\DPHY_RX_INST/u_Aligner/n116_5 ),
	.Q(\DPHY_RX_INST/u_Aligner/ln0/lane1_cnt [0])
);
defparam \DPHY_RX_INST/u_Aligner/ln0/lane1_cnt_0_s1 .INIT=1'b0;
endmodule
