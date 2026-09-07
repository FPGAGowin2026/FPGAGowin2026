`timescale 1ns/1ps

// 执行外部 ROM/BRAM 顺序提供的 MOVE、ROTATE、STOP 段，不依赖软核。
// segment_progress_i 必须是当前段的单调进度；progress_enable_o 为低时外部累积器应冻结。
module path_segment_executor #(
    parameter COMMAND_WIDTH = 32,
    parameter PROGRESS_WIDTH = 32
) (
    input  wire                             clk_i,
    input  wire                             rst_n_i,
    input  wire                             start_i,
    input  wire                             abort_i,
    input  wire                             pause_i,
    input  wire                             motion_stopped_i,

    output wire                             segment_ready_o,
    input  wire                             segment_valid_i,
    input  wire        [1:0]                segment_opcode_i,
    input  wire        [PROGRESS_WIDTH-1:0] segment_target_i,
    input  wire signed [COMMAND_WIDTH-1:0]  segment_command_i,
    input  wire                             segment_last_i,
    input  wire        [PROGRESS_WIDTH-1:0] segment_progress_i,

    output wire                             path_active_o,
    output wire                             path_finished_o,
    output wire                             progress_enable_o,
    output reg         [7:0]                segment_index_o,
    output reg                              command_valid_o,
    output reg signed [COMMAND_WIDTH-1:0]   linear_o,
    output reg signed [COMMAND_WIDTH-1:0]   angular_o
);

    localparam [1:0] OP_MOVE   = 2'd0;
    localparam [1:0] OP_ROTATE = 2'd1;
    localparam [1:0] OP_STOP   = 2'd2;

    localparam [1:0] STATE_IDLE     = 2'd0;
    localparam [1:0] STATE_LOAD     = 2'd1;
    localparam [1:0] STATE_EXECUTE  = 2'd2;
    localparam [1:0] STATE_FINISHED = 2'd3;

    reg [1:0] state_q;
    reg [1:0] opcode_q;
    reg [PROGRESS_WIDTH-1:0] target_q;
    reg signed [COMMAND_WIDTH-1:0] command_q;
    reg last_q;

    wire segment_complete;

    assign segment_ready_o = state_q == STATE_LOAD;
    assign path_active_o = state_q == STATE_LOAD || state_q == STATE_EXECUTE;
    assign path_finished_o = state_q == STATE_FINISHED;
    assign progress_enable_o =
        state_q == STATE_EXECUTE && !pause_i && opcode_q != OP_STOP;
    assign segment_complete =
        (opcode_q == OP_STOP) ? motion_stopped_i : (segment_progress_i >= target_q);

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            state_q <= STATE_IDLE;
            opcode_q <= OP_STOP;
            target_q <= {PROGRESS_WIDTH{1'b0}};
            command_q <= {COMMAND_WIDTH{1'b0}};
            last_q <= 1'b0;
            segment_index_o <= 8'd0;
        end else if (abort_i) begin
            state_q <= STATE_IDLE;
            segment_index_o <= 8'd0;
        end else begin
            case (state_q)
                STATE_IDLE: begin
                    if (start_i) begin
                        state_q <= STATE_LOAD;
                        segment_index_o <= 8'd0;
                    end
                end

                STATE_LOAD: begin
                    if (segment_valid_i) begin
                        opcode_q <= segment_opcode_i;
                        target_q <= segment_target_i;
                        command_q <= segment_command_i;
                        last_q <= segment_last_i;
                        state_q <= STATE_EXECUTE;
                    end
                end

                STATE_EXECUTE: begin
                    if (!pause_i && segment_complete) begin
                        if (last_q) begin
                            state_q <= STATE_FINISHED;
                        end else begin
                            state_q <= STATE_LOAD;
                            segment_index_o <= segment_index_o + 1'b1;
                        end
                    end
                end

                STATE_FINISHED: begin
                    if (start_i) begin
                        state_q <= STATE_LOAD;
                        segment_index_o <= 8'd0;
                    end
                end

                default: state_q <= STATE_IDLE;
            endcase
        end
    end

    // pause_i 只关闭有效标志和进度许可；保留当前段目标以便绕障结束后继续。
    always @(*) begin
        command_valid_o = state_q == STATE_EXECUTE && !pause_i;
        linear_o = {COMMAND_WIDTH{1'b0}};
        angular_o = {COMMAND_WIDTH{1'b0}};

        if (state_q == STATE_EXECUTE) begin
            case (opcode_q)
                OP_MOVE:   linear_o = command_q;
                OP_ROTATE: angular_o = command_q;
                default: begin
                    linear_o = {COMMAND_WIDTH{1'b0}};
                    angular_o = {COMMAND_WIDTH{1'b0}};
                end
            endcase
        end
    end

endmodule
