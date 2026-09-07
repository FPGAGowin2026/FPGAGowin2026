`timescale 1ns/1ps

module sensor_example_top #(
    parameter integer CLK_DIV = 8,
    parameter integer STARTUP_DELAY_CYCLES = 270000,
    parameter integer SAMPLE_INTERVAL_CYCLES = 270000
) (
    input  wire               clk,
    input  wire               reset_n,
    input  wire               mag_miso,
    output wire               mag_cs_n,
    output wire               mag_sclk,
    output wire               mag_mosi,
    output wire               sensor_ok,
    output wire               initialized,
    output wire signed [15:0] mag_x,
    output wire signed [15:0] mag_y,
    output wire signed [15:0] mag_z,
    output wire               sample_valid
);

    lis3mdl_spi #(
        .CLK_DIV(CLK_DIV),
        .STARTUP_DELAY_CYCLES(STARTUP_DELAY_CYCLES),
        .SAMPLE_INTERVAL_CYCLES(SAMPLE_INTERVAL_CYCLES)
    ) u_sensor (
        .clk(clk), .reset_n(reset_n),
        .spi_miso(mag_miso), .spi_cs_n(mag_cs_n),
        .spi_sclk(mag_sclk), .spi_mosi(mag_mosi),
        .sensor_ok(sensor_ok), .initialized(initialized),
        .mag_x(mag_x), .mag_y(mag_y), .mag_z(mag_z),
        .sample_valid(sample_valid)
    );

endmodule
