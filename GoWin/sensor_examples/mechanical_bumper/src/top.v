`timescale 1ns/1ps

module sensor_example_top #(
    parameter integer ACTIVE_LOW      = 1,
    parameter integer DEBOUNCE_CYCLES = 270000
) (
    input  wire clk,
    input  wire rst_n,
    input  wire bumper_left_n,
    input  wire bumper_right_n,
    input  wire clear_latched,
    output wire left_pressed,
    output wire right_pressed,
    output wire collision,
    output wire collision_event,
    output wire collision_latched
);

    mechanical_bumper #(
        .ACTIVE_LOW(ACTIVE_LOW),
        .DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)
    ) u_mechanical_bumper (
        .clk(clk),
        .rst_n(rst_n),
        .bumper_left_in(bumper_left_n),
        .bumper_right_in(bumper_right_n),
        .clear_latched(clear_latched),
        .left_pressed(left_pressed),
        .right_pressed(right_pressed),
        .collision(collision),
        .collision_event(collision_event),
        .collision_latched(collision_latched)
    );

endmodule
