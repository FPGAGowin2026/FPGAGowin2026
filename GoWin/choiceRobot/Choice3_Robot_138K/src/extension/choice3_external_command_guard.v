`timescale 1ns/1ps

// Protocol-neutral AI/host command boundary.  The external controller can only
// request high-level actions; it never receives a path to PWM, direction or
// motor standby.  A strictly increasing 32-bit sequence and a renewable lease
// make duplicates, delayed packets and loss of the external controller safe.
module choice3_external_command_guard #(
    parameter integer LEASE_TIMEOUT_CYCLES = 10_000_000
) (
    input  wire        clk_i,
    input  wire        rst_n_i,
    input  wire        safety_block_i,

    input  wire        command_valid_i,
    output wire        command_ready_o,
    input  wire [31:0] command_sequence_i,
    input  wire [3:0]  command_opcode_i,
    input  wire [2:0]  command_motion_i,

    output reg         arm_pulse_o,
    output reg         disarm_pulse_o,
    output reg         clear_fault_pulse_o,
    output reg         route_start_pulse_o,
    output reg         manual_command_valid_o,
    output reg  [2:0]  manual_motion_o,

    output reg         accepted_pulse_o,
    output reg         rejected_pulse_o,
    output reg  [2:0]  reject_reason_o,
    output reg         lease_active_o,
    output reg         lease_timeout_pulse_o,
    output reg         sequence_seen_o,
    output reg  [31:0] last_sequence_o,
    output reg  [31:0] accepted_count_o,
    output reg  [31:0] rejected_count_o,
    output reg  [31:0] timeout_count_o
);
    localparam [3:0] OP_KEEPALIVE  = 4'h0;
    localparam [3:0] OP_ARM        = 4'h1;
    localparam [3:0] OP_DISARM     = 4'h2;
    localparam [3:0] OP_CLEAR      = 4'h3;
    localparam [3:0] OP_ROUTE      = 4'h4;
    localparam [3:0] OP_MANUAL     = 4'h5;
    localparam [3:0] OP_STOP       = 4'h6;

    localparam [2:0] REJECT_NONE     = 3'd0;
    localparam [2:0] REJECT_SEQUENCE = 3'd1;
    localparam [2:0] REJECT_OPCODE   = 3'd2;
    localparam [2:0] REJECT_PAYLOAD  = 3'd3;
    localparam [2:0] REJECT_SAFETY   = 3'd4;
    localparam [2:0] REJECT_NO_LEASE = 3'd5;

    localparam integer LEASE_LIMIT =
        (LEASE_TIMEOUT_CYCLES < 1) ? 1 : LEASE_TIMEOUT_CYCLES;

    reg [31:0] lease_age_q;
    wire [31:0] sequence_delta_w = command_sequence_i - last_sequence_o;
    wire sequence_newer_w = !sequence_seen_o ||
        ((sequence_delta_w != 32'd0) && !sequence_delta_w[31]);
    wire opcode_known_w = command_opcode_i <= OP_STOP;
    wire payload_valid_w = (command_opcode_i == OP_MANUAL) ?
                           (command_motion_i <= 3'd6) :
                           (command_motion_i == 3'd0);

    assign command_ready_o = rst_n_i;

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            arm_pulse_o <= 1'b0;
            disarm_pulse_o <= 1'b0;
            clear_fault_pulse_o <= 1'b0;
            route_start_pulse_o <= 1'b0;
            manual_command_valid_o <= 1'b0;
            manual_motion_o <= 3'd0;
            accepted_pulse_o <= 1'b0;
            rejected_pulse_o <= 1'b0;
            reject_reason_o <= REJECT_NONE;
            lease_active_o <= 1'b0;
            lease_timeout_pulse_o <= 1'b0;
            sequence_seen_o <= 1'b0;
            last_sequence_o <= 32'd0;
            accepted_count_o <= 32'd0;
            rejected_count_o <= 32'd0;
            timeout_count_o <= 32'd0;
            lease_age_q <= 32'd0;
        end else begin
            arm_pulse_o <= 1'b0;
            disarm_pulse_o <= 1'b0;
            clear_fault_pulse_o <= 1'b0;
            route_start_pulse_o <= 1'b0;
            manual_command_valid_o <= 1'b0;
            accepted_pulse_o <= 1'b0;
            rejected_pulse_o <= 1'b0;
            lease_timeout_pulse_o <= 1'b0;

            // The physical E-stop and CPU watchdog dominate every external
            // request.  A safety trip revokes the lease without consuming a
            // sequence number, so an operator may deliberately retry later.
            if (safety_block_i) begin
                lease_age_q <= 32'd0;
                if (lease_active_o) begin
                    lease_active_o <= 1'b0;
                    disarm_pulse_o <= 1'b1;
                end
                if (command_valid_i) begin
                    rejected_pulse_o <= 1'b1;
                    reject_reason_o <= REJECT_SAFETY;
                    rejected_count_o <= rejected_count_o + 1'b1;
                end
            end else if (command_valid_i) begin
                if (!sequence_newer_w) begin
                    rejected_pulse_o <= 1'b1;
                    reject_reason_o <= REJECT_SEQUENCE;
                    rejected_count_o <= rejected_count_o + 1'b1;
                end else if (!opcode_known_w) begin
                    rejected_pulse_o <= 1'b1;
                    reject_reason_o <= REJECT_OPCODE;
                    rejected_count_o <= rejected_count_o + 1'b1;
                end else if (!payload_valid_w) begin
                    rejected_pulse_o <= 1'b1;
                    reject_reason_o <= REJECT_PAYLOAD;
                    rejected_count_o <= rejected_count_o + 1'b1;
                end else if ((command_opcode_i == OP_KEEPALIVE) &&
                             !lease_active_o) begin
                    rejected_pulse_o <= 1'b1;
                    reject_reason_o <= REJECT_NO_LEASE;
                    rejected_count_o <= rejected_count_o + 1'b1;
                end else begin
                    accepted_pulse_o <= 1'b1;
                    reject_reason_o <= REJECT_NONE;
                    accepted_count_o <= accepted_count_o + 1'b1;
                    sequence_seen_o <= 1'b1;
                    last_sequence_o <= command_sequence_i;
                    lease_age_q <= 32'd0;
                    case (command_opcode_i)
                        OP_KEEPALIVE: lease_active_o <= 1'b1;
                        OP_ARM: begin
                            lease_active_o <= 1'b1;
                            arm_pulse_o <= 1'b1;
                        end
                        OP_DISARM, OP_STOP: begin
                            lease_active_o <= 1'b0;
                            disarm_pulse_o <= 1'b1;
                        end
                        OP_CLEAR: clear_fault_pulse_o <= 1'b1;
                        OP_ROUTE: begin
                            lease_active_o <= 1'b1;
                            route_start_pulse_o <= 1'b1;
                        end
                        OP_MANUAL: begin
                            lease_active_o <= 1'b1;
                            manual_command_valid_o <= 1'b1;
                            manual_motion_o <= command_motion_i;
                        end
                        default: begin end
                    endcase
                end
            end else if (lease_active_o) begin
                if (lease_age_q >= LEASE_LIMIT-1) begin
                    lease_age_q <= 32'd0;
                    lease_active_o <= 1'b0;
                    lease_timeout_pulse_o <= 1'b1;
                    disarm_pulse_o <= 1'b1;
                    timeout_count_o <= timeout_count_o + 1'b1;
                end else begin
                    lease_age_q <= lease_age_q + 1'b1;
                end
            end else begin
                lease_age_q <= 32'd0;
            end
        end
    end
endmodule
