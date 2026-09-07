`timescale 1ns/1ps

module sensor_example_top #(
    parameter integer SCCB_CLK_DIV = 32,
    parameter integer POWERDOWN_HOLD_CYCLES = 240000,
    parameter integer RESET_HOLD_CYCLES = 240000,
    parameter integer STARTUP_DELAY_CYCLES = 480000,
    parameter integer SOFT_RESET_DELAY_CYCLES = 120000
) (
    input  wire        clk_i,
    input  wire        rst_n_i,
    output wire        camera_xclk_o,
    output wire        camera_reset_n_o,
    output wire        camera_pwdn_o,
    inout  wire        camera_scl_io,
    inout  wire        camera_sda_io,
    output wire        camera_configured_o,
    output wire        camera_identity_error_o,
    output wire        camera_bus_error_o,
    output wire [15:0] camera_chip_id_o,
    input  wire        camera_pclk_i,
    input  wire        camera_vsync_i,
    input  wire        camera_href_i,
    input  wire [7:0]  camera_data_i,
    output wire        frame_valid_o,
    output wire        line_valid_o,
    output wire        pixel_valid_o,
    output wire [15:0] pixel_rgb565_o
);
    // configured 在配置/XCLK 域产生（本工程约束为 24 MHz）。先在 PCLK 域同步，再同步释放
    // 采集器复位；rst_n_i 仍可异步拉低，避免跨时钟域亚稳态直接进入复位树。
    reg configured_pclk_meta_q;
    reg configured_pclk_sync_q;
    always @(posedge camera_pclk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            configured_pclk_meta_q <= 1'b0;
            configured_pclk_sync_q <= 1'b0;
        end else begin
            configured_pclk_meta_q <= camera_configured_o;
            configured_pclk_sync_q <= configured_pclk_meta_q;
        end
    end
    wire capture_reset_n = rst_n_i & configured_pclk_sync_q;

    ov5640_sccb_init #(
        .SCCB_CLK_DIV(SCCB_CLK_DIV),
        .POWERDOWN_HOLD_CYCLES(POWERDOWN_HOLD_CYCLES),
        .RESET_HOLD_CYCLES(RESET_HOLD_CYCLES),
        .STARTUP_DELAY_CYCLES(STARTUP_DELAY_CYCLES),
        .SOFT_RESET_DELAY_CYCLES(SOFT_RESET_DELAY_CYCLES)
    ) init_u (
        .clk(clk_i),.reset_n(rst_n_i),.camera_xclk(camera_xclk_o),
        .camera_reset_n(camera_reset_n_o),.camera_pwdn(camera_pwdn_o),
        .camera_scl(camera_scl_io),.camera_sda(camera_sda_io),
        .configured(camera_configured_o),.identity_error(camera_identity_error_o),
        .bus_error(camera_bus_error_o),.chip_id(camera_chip_id_o)
    );

    ov5640_dvp_capture #(
        .DVP_LITTLE_ENDIAN(1'b1)
    ) capture_u (
        .pclk_i(camera_pclk_i),
        .rst_n_i(capture_reset_n),
        .vsync_i(camera_vsync_i),
        .href_i(camera_href_i),
        .data_i(camera_data_i),
        .frame_valid_o(frame_valid_o),
        .line_valid_o(line_valid_o),
        .pixel_valid_o(pixel_valid_o),
        .pixel_rgb565_o(pixel_rgb565_o)
    );
endmodule
