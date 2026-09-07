`timescale 1ns/1ps

// 最小航点状态机：直行一段、原地转向、再直行一段。
// 遇障时先后退再转向，并从当前航段重新建立里程基线。
module choice3_navigation_fsm #(
    parameter signed [31:0] CRUISE_MM_S = 32'sd260,
    parameter signed [31:0] SLOW_MM_S = 32'sd120,
    parameter signed [31:0] TURN_MM_S = 32'sd180,
    parameter integer LEG1_TICKS = 400,
    parameter integer TURN_TICKS = 180,
    parameter integer LEG2_TICKS = 400,
    parameter integer AVOID_BACK_SAMPLES = 20,
    parameter integer AVOID_TURN_SAMPLES = 35
) (
    input  wire               clk_i,
    input  wire               rst_n_i,
    input  wire               sample_tick_i,
    input  wire               enable_i,
    input  wire               start_pulse_i,
    input  wire               stop_pulse_i,
    input  wire               front_blocked_i,
    input  wire               front_slow_i,
    input  wire               left_blocked_i,
    input  wire               right_blocked_i,
    input  wire signed [31:0] wheel0_ticks_i,
    input  wire signed [31:0] wheel1_ticks_i,
    input  wire signed [31:0] wheel2_ticks_i,
    input  wire signed [31:0] wheel3_ticks_i,
    output reg signed [31:0]  target0_mm_s_o,
    output reg signed [31:0]  target1_mm_s_o,
    output reg signed [31:0]  target2_mm_s_o,
    output reg signed [31:0]  target3_mm_s_o,
    output reg [3:0]          state_o,
    output reg                route_done_o
);
    localparam [3:0] ST_IDLE=0, ST_LEG1=1, ST_TURN=2, ST_LEG2=3,
                     ST_BACK=4, ST_AVOID_TURN=5, ST_DONE=6;
    reg [3:0] resume_state_q;
    reg [15:0] timed_samples_q;
    reg avoid_turn_left_q;
    reg signed [31:0] base0_q, base1_q, base2_q, base3_q;
    reg signed [33:0] progress_sum;

    function [31:0] abs32;
        input signed [31:0] value;
        begin abs32 = value[31] ? (~value + 1'b1) : value; end
    endfunction

    always @* begin
        progress_sum = abs32(wheel0_ticks_i-base0_q) +
                       abs32(wheel1_ticks_i-base1_q) +
                       abs32(wheel2_ticks_i-base2_q) +
                       abs32(wheel3_ticks_i-base3_q);
    end

    task set_baseline;
        begin
            base0_q <= wheel0_ticks_i;
            base1_q <= wheel1_ticks_i;
            base2_q <= wheel2_ticks_i;
            base3_q <= wheel3_ticks_i;
        end
    endtask

    always @* begin
        target0_mm_s_o = 0; target1_mm_s_o = 0;
        target2_mm_s_o = 0; target3_mm_s_o = 0;
        if (enable_i) begin
            case (state_o)
                ST_LEG1, ST_LEG2: begin
                    target0_mm_s_o = front_slow_i ? SLOW_MM_S : CRUISE_MM_S;
                    target1_mm_s_o = front_slow_i ? SLOW_MM_S : CRUISE_MM_S;
                    target2_mm_s_o = front_slow_i ? SLOW_MM_S : CRUISE_MM_S;
                    target3_mm_s_o = front_slow_i ? SLOW_MM_S : CRUISE_MM_S;
                end
                ST_TURN: begin
                    target0_mm_s_o = -TURN_MM_S; target1_mm_s_o = -TURN_MM_S;
                    target2_mm_s_o =  TURN_MM_S; target3_mm_s_o =  TURN_MM_S;
                end
                ST_BACK: begin
                    target0_mm_s_o = -SLOW_MM_S; target1_mm_s_o = -SLOW_MM_S;
                    target2_mm_s_o = -SLOW_MM_S; target3_mm_s_o = -SLOW_MM_S;
                end
                ST_AVOID_TURN: begin
                    target0_mm_s_o = avoid_turn_left_q ? -TURN_MM_S : TURN_MM_S;
                    target1_mm_s_o = avoid_turn_left_q ? -TURN_MM_S : TURN_MM_S;
                    target2_mm_s_o = avoid_turn_left_q ?  TURN_MM_S : -TURN_MM_S;
                    target3_mm_s_o = avoid_turn_left_q ?  TURN_MM_S : -TURN_MM_S;
                end
                default: begin end
            endcase
        end
    end

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            state_o <= ST_IDLE;
            resume_state_q <= ST_IDLE;
            timed_samples_q <= 0;
            avoid_turn_left_q <= 1'b1;
            route_done_o <= 1'b0;
            base0_q <= 0; base1_q <= 0; base2_q <= 0; base3_q <= 0;
        end else begin
            if (!enable_i || stop_pulse_i) begin
                state_o <= ST_IDLE;
                route_done_o <= 1'b0;
                timed_samples_q <= 0;
                set_baseline();
            end else if (start_pulse_i && state_o == ST_IDLE) begin
                state_o <= ST_LEG1;
                route_done_o <= 1'b0;
                set_baseline();
            end else if (sample_tick_i) begin
                case (state_o)
                    ST_LEG1: begin
                        if (front_blocked_i) begin
                            resume_state_q <= ST_LEG1; state_o <= ST_BACK;
                            timed_samples_q <= 0;
                            avoid_turn_left_q <= right_blocked_i && !left_blocked_i;
                        end else if ((progress_sum >> 2) >= LEG1_TICKS) begin
                            state_o <= ST_TURN; set_baseline();
                        end
                    end
                    ST_TURN: begin
                        if ((progress_sum >> 2) >= TURN_TICKS) begin
                            state_o <= ST_LEG2; set_baseline();
                        end
                    end
                    ST_LEG2: begin
                        if (front_blocked_i) begin
                            resume_state_q <= ST_LEG2; state_o <= ST_BACK;
                            timed_samples_q <= 0;
                            avoid_turn_left_q <= right_blocked_i && !left_blocked_i;
                        end else if ((progress_sum >> 2) >= LEG2_TICKS) begin
                            state_o <= ST_DONE; route_done_o <= 1'b1;
                        end
                    end
                    ST_BACK: begin
                        if (timed_samples_q >= AVOID_BACK_SAMPLES-1) begin
                            state_o <= ST_AVOID_TURN; timed_samples_q <= 0;
                        end else timed_samples_q <= timed_samples_q + 1'b1;
                    end
                    ST_AVOID_TURN: begin
                        if (timed_samples_q >= AVOID_TURN_SAMPLES-1) begin
                            state_o <= resume_state_q; timed_samples_q <= 0;
                            set_baseline();
                        end else timed_samples_q <= timed_samples_q + 1'b1;
                    end
                    default: begin end
                endcase
            end
        end
    end
endmodule
