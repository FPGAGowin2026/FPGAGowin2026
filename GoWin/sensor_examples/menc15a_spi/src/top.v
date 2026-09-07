`timescale 1ns/1ps
module sensor_example_top #(
    parameter integer CLK_DIV=1, parameter integer STARTUP_DELAY_CYCLES=27000,
    parameter integer SAMPLE_INTERVAL_CYCLES=27000
) (
    input wire clk,input wire reset_n,input wire encoder_miso,
    output wire encoder_cs_n,output wire encoder_sclk,output wire encoder_mosi,
    output wire initialized,output wire sensor_ok,output wire [14:0] position_raw,
    output wire signed [15:0] speed_raw,output wire position_status,
    output wire speed_status,output wire sample_valid,output wire protocol_error
);
    menc15a_spi #(.CLK_DIV(CLK_DIV),.STARTUP_DELAY_CYCLES(STARTUP_DELAY_CYCLES),
        .SAMPLE_INTERVAL_CYCLES(SAMPLE_INTERVAL_CYCLES)) u_sensor(
        .clk(clk),.reset_n(reset_n),.spi_miso(encoder_miso),.spi_cs_n(encoder_cs_n),
        .spi_sclk(encoder_sclk),.spi_mosi(encoder_mosi),.initialized(initialized),
        .sensor_ok(sensor_ok),.position_raw(position_raw),.speed_raw(speed_raw),
        .position_status(position_status),.speed_status(speed_status),
        .sample_valid(sample_valid),.protocol_error(protocol_error));
endmodule
