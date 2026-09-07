`timescale 1ns/1ps

module sensor_example_top #(
    parameter integer ACTIVE_LOW    = 1,
    parameter integer FILTER_CYCLES = 16
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tcrt5000_digital,
    output wire        line_detected,
    output wire        line_rise,
    output wire        line_fall,
    output wire [31:0] filter_level
);

    tcrt5000_filter #(
        .ACTIVE_LOW(ACTIVE_LOW),
        .FILTER_CYCLES(FILTER_CYCLES)
    ) u_tcrt5000_filter (
        .clk(clk),
        .rst_n(rst_n),
        .sensor_in(tcrt5000_digital),
        .line_detected(line_detected),
        .line_rise(line_rise),
        .line_fall(line_fall),
        .filter_level(filter_level)
    );

endmodule
