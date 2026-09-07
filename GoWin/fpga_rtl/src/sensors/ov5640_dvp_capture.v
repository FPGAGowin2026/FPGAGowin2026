`timescale 1ns/1ps

// OV5640 DVP 像素采集最小前端。
// 本模块只在 pclk_i 时钟域按高字节在前的 RGB565 格式组帧；寄存器配置、
// 跨时钟 FIFO、DDR3 和显示链路均属于后续实板工程。
module ov5640_dvp_capture #(
    parameter VSYNC_ACTIVE_LEVEL = 1'b1
) (
    input  wire        pclk_i,
    input  wire        rst_n_i,
    input  wire        vsync_i,
    input  wire        href_i,
    input  wire [7:0]  data_i,

    // valid 输出均在 pclk_i 上升沿更新；pixel_valid_o 仅保持一个周期。
    output reg         frame_valid_o,
    output reg         line_valid_o,
    output reg         pixel_valid_o,
    output reg  [15:0] pixel_rgb565_o
);

    reg       byte_phase_q;
    reg [7:0] high_byte_q;

    wire frame_sync_active;
    assign frame_sync_active = (vsync_i == VSYNC_ACTIVE_LEVEL);

    always @(posedge pclk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            frame_valid_o   <= 1'b0;
            line_valid_o    <= 1'b0;
            pixel_valid_o   <= 1'b0;
            pixel_rgb565_o  <= 16'd0;
            byte_phase_q    <= 1'b0;
            high_byte_q     <= 8'd0;
        end else begin
            frame_valid_o <= !frame_sync_active;
            line_valid_o  <= !frame_sync_active && href_i;
            pixel_valid_o <= 1'b0;

            // 帧同步或行无效会丢弃未配对的半个像素，禁止跨行/跨帧拼接。
            if (frame_sync_active || !href_i) begin
                byte_phase_q <= 1'b0;
            end else if (!byte_phase_q) begin
                high_byte_q  <= data_i;
                byte_phase_q <= 1'b1;
            end else begin
                pixel_rgb565_o <= {high_byte_q, data_i};
                pixel_valid_o  <= 1'b1;
                byte_phase_q   <= 1'b0;
            end
        end
    end

endmodule
