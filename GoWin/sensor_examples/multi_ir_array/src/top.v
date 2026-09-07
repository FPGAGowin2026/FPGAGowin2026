`timescale 1ns/1ps
module sensor_example_top #(parameter integer DEBOUNCE_CYCLES=27000)(
    input wire clk,input wire rst_n,input wire [7:0] ir_sensor_n,
    output wire [7:0] obstacle_mask,output wire change_valid);
    multi_ir_array #(.CHANNELS(8),.DEBOUNCE_CYCLES(DEBOUNCE_CYCLES),.ACTIVE_LOW(1'b1)) u_core(
        .clk(clk),.rst_n(rst_n),.sensor_i(ir_sensor_n),.active_mask_o(obstacle_mask),.change_valid_o(change_valid));
endmodule
