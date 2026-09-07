`timescale 1ns/1ps

module sensor_example_top #(
    parameter integer ASSERT_SAMPLES = 4,
    parameter integer RELEASE_SAMPLES = 4
) (
    input  wire clk_i,
    input  wire rst_n_i,
    input  wire sensor_n_i,
    output wire object_near_o,
    output wire changed_o
);
    digital_ir_proximity #(
        .ASSERT_SAMPLES(ASSERT_SAMPLES), .RELEASE_SAMPLES(RELEASE_SAMPLES)
    ) filter_u (
        .clk_i(clk_i), .rst_n_i(rst_n_i), .sensor_n_i(sensor_n_i),
        .object_near_o(object_near_o), .changed_o(changed_o)
    );
endmodule
