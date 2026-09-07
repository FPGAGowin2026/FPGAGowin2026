`timescale 1ns/1ps

// Fail-safe high-level command arbiter.  CPU and external commands are merged
// before the safety/control core.  Simultaneous non-disarm requests from both
// domains are treated as an ownership conflict and become a disarm request.
module choice3_command_arbiter (
    input  wire       clk_i,
    input  wire       rst_n_i,
    input  wire       cpu_arm_i,
    input  wire       cpu_disarm_i,
    input  wire       cpu_clear_i,
    input  wire       cpu_route_i,
    input  wire       cpu_manual_valid_i,
    input  wire [2:0] cpu_manual_motion_i,
    input  wire       ext_arm_i,
    input  wire       ext_disarm_i,
    input  wire       ext_clear_i,
    input  wire       ext_route_i,
    input  wire       ext_manual_valid_i,
    input  wire [2:0] ext_manual_motion_i,
    output reg        arm_o,
    output reg        disarm_o,
    output reg        clear_o,
    output reg        route_o,
    output reg        manual_valid_o,
    output reg  [2:0] manual_motion_o,
    output reg        conflict_pulse_o
);
    wire cpu_action_w = cpu_arm_i || cpu_clear_i || cpu_route_i ||
                        cpu_manual_valid_i;
    wire ext_action_w = ext_arm_i || ext_clear_i || ext_route_i ||
                        ext_manual_valid_i;

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            arm_o <= 1'b0;
            disarm_o <= 1'b0;
            clear_o <= 1'b0;
            route_o <= 1'b0;
            manual_valid_o <= 1'b0;
            manual_motion_o <= 3'd0;
            conflict_pulse_o <= 1'b0;
        end else begin
            arm_o <= 1'b0;
            disarm_o <= 1'b0;
            clear_o <= 1'b0;
            route_o <= 1'b0;
            manual_valid_o <= 1'b0;
            conflict_pulse_o <= 1'b0;
            if (cpu_disarm_i || ext_disarm_i) begin
                disarm_o <= 1'b1;
            end else if (cpu_action_w && ext_action_w) begin
                disarm_o <= 1'b1;
                conflict_pulse_o <= 1'b1;
            end else if (cpu_action_w) begin
                arm_o <= cpu_arm_i;
                clear_o <= cpu_clear_i;
                route_o <= cpu_route_i;
                manual_valid_o <= cpu_manual_valid_i;
                manual_motion_o <= cpu_manual_motion_i;
            end else if (ext_action_w) begin
                arm_o <= ext_arm_i;
                clear_o <= ext_clear_i;
                route_o <= ext_route_i;
                manual_valid_o <= ext_manual_valid_i;
                manual_motion_o <= ext_manual_motion_i;
            end
        end
    end
endmodule
