`timescale 1ns/1ps

module sensor_example_top #(
    parameter integer CLK_FREQ_HZ = 27000000,
    parameter integer BAUD_RATE   = 115200
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tf_luna_rx,
    output wire [15:0] distance_cm,
    output wire [15:0] strength,
    output wire [15:0] temperature_raw,
    output wire        frame_valid,
    output wire        checksum_error
);

    tf_luna_uart #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUD_RATE(BAUD_RATE)
    ) u_tf_luna_uart (
        .clk(clk),
        .rst_n(rst_n),
        .uart_rx(tf_luna_rx),
        .distance_cm(distance_cm),
        .strength(strength),
        .temperature_raw(temperature_raw),
        .frame_valid(frame_valid),
        .checksum_error(checksum_error)
    );

endmodule
