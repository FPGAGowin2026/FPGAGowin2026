`timescale 1ns/1ps
module sensor_example_top #(
    parameter integer CLK_DIV = 2,
    parameter integer STARTUP_DELAY_CYCLES = 432000,
    parameter integer SAMPLE_INTERVAL_CYCLES = 27000
) (
    input wire clk, input wire reset_n, input wire encoder_miso,
    output wire encoder_cs_n, output wire encoder_sclk, output wire encoder_mosi,
    output wire initialized, output wire sensor_ok, output wire [13:0] angle_raw,
    output wire no_magnet, output wire over_speed,
    output wire sample_valid, output wire protocol_error
);
    mt6816_spi #(.CLK_DIV(CLK_DIV),
        .STARTUP_DELAY_CYCLES(STARTUP_DELAY_CYCLES),
        .SAMPLE_INTERVAL_CYCLES(SAMPLE_INTERVAL_CYCLES)) u_sensor (
        .clk(clk), .reset_n(reset_n), .spi_miso(encoder_miso),
        .spi_cs_n(encoder_cs_n), .spi_sclk(encoder_sclk), .spi_mosi(encoder_mosi),
        .initialized(initialized), .sensor_ok(sensor_ok), .angle_raw(angle_raw),
        .no_magnet(no_magnet), .over_speed(over_speed),
        .sample_valid(sample_valid), .protocol_error(protocol_error));
endmodule
