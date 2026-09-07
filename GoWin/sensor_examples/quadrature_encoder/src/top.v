`timescale 1ns/1ps

module sensor_example_top (
    input  wire              clk_i,
    input  wire              rst_n_i,
    input  wire              encoder_a_i,
    input  wire              encoder_b_i,
    output wire signed [31:0] position_ticks_o,
    output wire              step_valid_o,
    output wire              direction_o,
    output wire              invalid_transition_o
);
    quadrature_decoder decoder_u (
        .clk_i(clk_i), .rst_n_i(rst_n_i),
        .encoder_a_i(encoder_a_i), .encoder_b_i(encoder_b_i),
        .position_ticks_o(position_ticks_o), .step_valid_o(step_valid_o),
        .direction_o(direction_o), .invalid_transition_o(invalid_transition_o)
    );
endmodule
