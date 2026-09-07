`timescale 1ns/1ps

// OV5640 DVP RGB565 像素采集前端。
// VSYNC 高电平表示帧同步区，HREF 高电平表示行有效。
// OV5640 的 0x4300=0x6F 配置在 DVP 上按低字节、再高字节发送。
// 参数保留高字节优先模式，便于复用到使用其他格式寄存器配置的模块。
module ov5640_dvp_capture #(
    parameter VSYNC_ACTIVE_LEVEL = 1'b1,
    parameter DVP_LITTLE_ENDIAN = 1'b1
) (
    input  wire        pclk_i,
    input  wire        rst_n_i,
    input  wire        vsync_i,
    input  wire        href_i,
    input  wire [7:0]  data_i,
    output reg         frame_valid_o,
    output reg         line_valid_o,
    output reg         pixel_valid_o,
    output reg  [15:0] pixel_rgb565_o
);
    reg       byte_phase_q;
    reg       frame_armed_q;
    reg [7:0] first_byte_q;
    wire frame_sync_active = (vsync_i == VSYNC_ACTIVE_LEVEL);

    always @(posedge pclk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            frame_valid_o  <= 1'b0;
            line_valid_o   <= 1'b0;
            pixel_valid_o  <= 1'b0;
            pixel_rgb565_o <= 16'd0;
            byte_phase_q   <= 1'b0;
            frame_armed_q   <= 1'b0;
            first_byte_q   <= 8'd0;
        end else begin
            pixel_valid_o <= 1'b0;
            if (frame_sync_active) begin
                // 只有观察到完整帧同步后才允许采集，避免复位在半帧中
                // 释放时把未知的高/低字节相位误当成新图像。
                frame_armed_q  <= 1'b1;
                frame_valid_o <= 1'b0;
                line_valid_o  <= 1'b0;
                byte_phase_q  <= 1'b0;
            end else if (!frame_armed_q) begin
                frame_valid_o <= 1'b0;
                line_valid_o  <= 1'b0;
                byte_phase_q  <= 1'b0;
            end else if (!href_i) begin
                frame_valid_o <= 1'b1;
                line_valid_o  <= 1'b0;
                byte_phase_q <= 1'b0;
            end else if (!byte_phase_q) begin
                frame_valid_o <= 1'b1;
                line_valid_o  <= 1'b1;
                first_byte_q <= data_i;
                byte_phase_q <= 1'b1;
            end else begin
                frame_valid_o  <= 1'b1;
                line_valid_o   <= 1'b1;
                pixel_rgb565_o <= DVP_LITTLE_ENDIAN ? {data_i, first_byte_q}
                                                       : {first_byte_q, data_i};
                pixel_valid_o  <= 1'b1;
                byte_phase_q   <= 1'b0;
            end
        end
    end
endmodule
