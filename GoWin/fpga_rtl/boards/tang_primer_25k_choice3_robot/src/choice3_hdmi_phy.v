/*
 * OSER10/ELVDS 连接方式派生自 Sipeed Tang Primer 25K 官方 SVO 示例。
 * SVO 原许可：Copyright (C) 2014 Clifford Wolf，ISC License。
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES.
 */
`timescale 1ns/1ps

module choice3_hdmi_phy (
    input wire pixel_clk_i,input wire serial_5x_clk_i,
    input wire pixel_rst_n_i,input wire serial_rst_n_i,
    input wire [7:0] red_i,input wire [7:0] green_i,input wire [7:0] blue_i,
    input wire hsync_i,input wire vsync_i,input wire de_i,
    output wire tmds_clk_p_o,output wire tmds_clk_n_o,
    output wire [2:0] tmds_data_p_o,output wire [2:0] tmds_data_n_o
);
    wire [9:0] code_r,code_g,code_b;
    wire [2:0] serial_data;
    choice3_tmds_encoder u_blue(.pixel_clk_i(pixel_clk_i),.rst_n_i(pixel_rst_n_i),
      .de_i(de_i),.control_i({vsync_i,hsync_i}),.data_i(blue_i),.code_o(code_b));
    choice3_tmds_encoder u_green(.pixel_clk_i(pixel_clk_i),.rst_n_i(pixel_rst_n_i),
      .de_i(de_i),.control_i(2'b00),.data_i(green_i),.code_o(code_g));
    choice3_tmds_encoder u_red(.pixel_clk_i(pixel_clk_i),.rst_n_i(pixel_rst_n_i),
      .de_i(de_i),.control_i(2'b00),.data_i(red_i),.code_o(code_r));

    OSER10 u_ser_b(.Q(serial_data[0]),.D0(code_b[0]),.D1(code_b[1]),
      .D2(code_b[2]),.D3(code_b[3]),.D4(code_b[4]),.D5(code_b[5]),
      .D6(code_b[6]),.D7(code_b[7]),.D8(code_b[8]),.D9(code_b[9]),
      .PCLK(pixel_clk_i),.FCLK(serial_5x_clk_i),.RESET(~serial_rst_n_i));
    OSER10 u_ser_g(.Q(serial_data[1]),.D0(code_g[0]),.D1(code_g[1]),
      .D2(code_g[2]),.D3(code_g[3]),.D4(code_g[4]),.D5(code_g[5]),
      .D6(code_g[6]),.D7(code_g[7]),.D8(code_g[8]),.D9(code_g[9]),
      .PCLK(pixel_clk_i),.FCLK(serial_5x_clk_i),.RESET(~serial_rst_n_i));
    OSER10 u_ser_r(.Q(serial_data[2]),.D0(code_r[0]),.D1(code_r[1]),
      .D2(code_r[2]),.D3(code_r[3]),.D4(code_r[4]),.D5(code_r[5]),
      .D6(code_r[6]),.D7(code_r[7]),.D8(code_r[8]),.D9(code_r[9]),
      .PCLK(pixel_clk_i),.FCLK(serial_5x_clk_i),.RESET(~serial_rst_n_i));
    ELVDS_OBUF u_clk_buf(.I(pixel_clk_i),.O(tmds_clk_p_o),.OB(tmds_clk_n_o));
    ELVDS_OBUF u_b_buf(.I(serial_data[0]),.O(tmds_data_p_o[0]),.OB(tmds_data_n_o[0]));
    ELVDS_OBUF u_g_buf(.I(serial_data[1]),.O(tmds_data_p_o[1]),.OB(tmds_data_n_o[1]));
    ELVDS_OBUF u_r_buf(.I(serial_data[2]),.O(tmds_data_p_o[2]),.OB(tmds_data_n_o[2]));
endmodule
