`timescale 1ns/1ps
module sensor_example_top #(parameter integer WINDOW_CYCLES=2700000,parameter integer MIN_EDGE_GAP_CYCLES=270)(
    input wire clk,input wire rst_n,input wire itr9606_pulse,input wire debug_select,
    output wire[31:0]debug_data,output wire debug_valid,output wire count_overflow);
    wire[31:0]pulse_count,period_cycles;
    wire sample_valid,period_valid;
    assign debug_data=debug_select?period_cycles:pulse_count;
    assign debug_valid=debug_select?period_valid:sample_valid;
    itr9606_pulse_speed #(.WINDOW_CYCLES(WINDOW_CYCLES),.MIN_EDGE_GAP_CYCLES(MIN_EDGE_GAP_CYCLES)) u_core(
        .clk(clk),.rst_n(rst_n),.pulse_i(itr9606_pulse),.pulse_count_o(pulse_count),
        .period_cycles_o(period_cycles),.sample_valid_o(sample_valid),.period_valid_o(period_valid),.count_overflow_o(count_overflow));
endmodule
