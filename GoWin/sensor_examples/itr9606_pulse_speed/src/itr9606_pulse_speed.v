`timescale 1ns/1ps

// ITR9606 optointerrupter pulse counter. A rising edge is accepted only after
// MIN_EDGE_GAP_CYCLES, suppressing narrow chatter at the digital comparator.
// pulse_count_o is the number of accepted slots in each measurement window;
// period_cycles_o is the interval between the two most recent accepted edges.
module itr9606_pulse_speed #(
    parameter integer WINDOW_CYCLES=2700000,
    parameter integer MIN_EDGE_GAP_CYCLES=270
) (
    input wire clk,input wire rst_n,input wire pulse_i,
    output reg [31:0] pulse_count_o,output reg [31:0] period_cycles_o,
    output reg sample_valid_o,output reg period_valid_o,output reg count_overflow_o
);
    localparam integer WIN=(WINDOW_CYCLES<1)?1:WINDOW_CYCLES;
    localparam integer GAP=(MIN_EDGE_GAP_CYCLES<1)?1:MIN_EDGE_GAP_CYCLES;
    reg sync1_q,sync2_q,sync2_d_q;
    reg [31:0] window_q,accepted_q,gap_q;
    reg have_edge_q;
    wire rising=sync2_q & ~sync2_d_q;
    wire accepted=rising && (gap_q>=GAP);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin sync1_q<=0;sync2_q<=0;sync2_d_q<=0;window_q<=0;accepted_q<=0;gap_q<=GAP;have_edge_q<=0;pulse_count_o<=0;period_cycles_o<=0;sample_valid_o<=0;period_valid_o<=0;count_overflow_o<=0;end
        else begin
            sync1_q<=pulse_i;sync2_q<=sync1_q;sync2_d_q<=sync2_q;
            sample_valid_o<=0;period_valid_o<=0;
            if(gap_q!=32'hFFFFFFFF)gap_q<=gap_q+1'b1;
            if(accepted) begin
                if(accepted_q!=32'hFFFFFFFF) accepted_q<=accepted_q+1'b1; else count_overflow_o<=1;
                if(have_edge_q)begin period_cycles_o<=gap_q;period_valid_o<=1;end
                have_edge_q<=1;gap_q<=0;
            end
            if(window_q>=WIN-1) begin
                pulse_count_o<=accepted_q+(accepted?1'b1:1'b0);sample_valid_o<=1;window_q<=0;accepted_q<=0;count_overflow_o<=0;
            end else window_q<=window_q+1'b1;
        end
    end
endmodule
