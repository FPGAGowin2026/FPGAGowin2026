`timescale 1ns/1ps

module sensor_example_top #(
    parameter integer CLK_FREQ_HZ       = 27000000,
    parameter integer TRIGGER_US        = 10,
    parameter integer TRIGGER_PERIOD_US = 60000,
    parameter integer ECHO_TIMEOUT_US   = 30000
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        echo_in,
    output wire        trigger_out,
    output wire [15:0] distance_mm,
    output wire [15:0] pulse_width_us,
    output wire        sample_valid,
    output wire        timeout_error,
    output wire        busy
);

    hc_sr04_controller #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .TRIGGER_US(TRIGGER_US),
        .TRIGGER_PERIOD_US(TRIGGER_PERIOD_US),
        .ECHO_TIMEOUT_US(ECHO_TIMEOUT_US)
    ) u_hc_sr04_controller (
        .clk(clk),
        .rst_n(rst_n),
        .echo_in(echo_in),
        .trigger_out(trigger_out),
        .distance_mm(distance_mm),
        .pulse_width_us(pulse_width_us),
        .sample_valid(sample_valid),
        .timeout_error(timeout_error),
        .busy(busy)
    );

endmodule
