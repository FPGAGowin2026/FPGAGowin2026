`timescale 1ns/1ps

// FPGA 独立模式的确定性局部绕障序列。
// phase_tick_i 的真实周期以及各阶段参数必须在实车上标定；仿真值不是距离或角度证明。
module local_avoidance_fsm #(
    parameter COMMAND_WIDTH = 32,
    parameter RANGE_WIDTH = 16,
    parameter STOP_TICKS = 2,
    parameter BACKUP_TICKS = 20,
    parameter TURN_TICKS = 20,
    parameter BYPASS_TICKS = 40,
    parameter RECOVER_TICKS = 20,
    parameter integer BACKUP_COMMAND = 100,
    parameter integer TURN_COMMAND = 300,
    parameter integer BYPASS_COMMAND = 150
) (
    input  wire                           clk_i,
    input  wire                           rst_n_i,
    input  wire                           start_i,
    input  wire                           abort_i,
    input  wire                           phase_tick_i,
    input  wire                           forward_allowed_i,
    input  wire        [RANGE_WIDTH-1:0]  left_range_i,
    input  wire        [RANGE_WIDTH-1:0]  right_range_i,

    output wire                           active_o,
    output reg                            done_o,
    output reg                            avoid_left_o,
    output reg         [2:0]              state_o,
    output reg                            command_valid_o,
    output reg signed [COMMAND_WIDTH-1:0] linear_o,
    output reg signed [COMMAND_WIDTH-1:0] angular_o
);

    localparam [2:0] STATE_IDLE    = 3'd0;
    localparam [2:0] STATE_STOP    = 3'd1;
    localparam [2:0] STATE_SELECT  = 3'd2;
    localparam [2:0] STATE_BACKUP  = 3'd3;
    localparam [2:0] STATE_TURN    = 3'd4;
    localparam [2:0] STATE_BYPASS  = 3'd5;
    localparam [2:0] STATE_RECOVER = 3'd6;

    reg [31:0] phase_count_q;

    assign active_o = state_o != STATE_IDLE;

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            state_o <= STATE_IDLE;
            phase_count_q <= 32'd0;
            avoid_left_o <= 1'b0;
            done_o <= 1'b0;
        end else begin
            done_o <= 1'b0;

            if (abort_i) begin
                state_o <= STATE_IDLE;
                phase_count_q <= 32'd0;
            end else begin
                case (state_o)
                    STATE_IDLE: begin
                        phase_count_q <= 32'd0;
                        if (start_i)
                            state_o <= STATE_STOP;
                    end

                    STATE_STOP: begin
                        if (phase_tick_i) begin
                            if (phase_count_q + 1'b1 >= STOP_TICKS) begin
                                state_o <= STATE_SELECT;
                                phase_count_q <= 32'd0;
                            end else begin
                                phase_count_q <= phase_count_q + 1'b1;
                            end
                        end
                    end

                    STATE_SELECT: begin
                        // 空间相等时固定选择左侧，保证相同输入得到相同结果。
                        avoid_left_o <= left_range_i >= right_range_i;
                        state_o <= STATE_BACKUP;
                        phase_count_q <= 32'd0;
                    end

                    STATE_BACKUP: begin
                        if (phase_tick_i) begin
                            if (phase_count_q + 1'b1 >= BACKUP_TICKS) begin
                                state_o <= STATE_TURN;
                                phase_count_q <= 32'd0;
                            end else begin
                                phase_count_q <= phase_count_q + 1'b1;
                            end
                        end
                    end

                    STATE_TURN: begin
                        if (phase_tick_i) begin
                            if (phase_count_q + 1'b1 >= TURN_TICKS) begin
                                state_o <= STATE_BYPASS;
                                phase_count_q <= 32'd0;
                            end else begin
                                phase_count_q <= phase_count_q + 1'b1;
                            end
                        end
                    end

                    STATE_BYPASS: begin
                        // 新障碍仍在停车区时，前进命令由安全层置零且本阶段不计时。
                        if (phase_tick_i && forward_allowed_i) begin
                            if (phase_count_q + 1'b1 >= BYPASS_TICKS) begin
                                state_o <= STATE_RECOVER;
                                phase_count_q <= 32'd0;
                            end else begin
                                phase_count_q <= phase_count_q + 1'b1;
                            end
                        end
                    end

                    STATE_RECOVER: begin
                        if (phase_tick_i) begin
                            if (phase_count_q + 1'b1 >= RECOVER_TICKS) begin
                                state_o <= STATE_IDLE;
                                phase_count_q <= 32'd0;
                                done_o <= 1'b1;
                            end else begin
                                phase_count_q <= phase_count_q + 1'b1;
                            end
                        end
                    end

                    default: begin
                        state_o <= STATE_IDLE;
                        phase_count_q <= 32'd0;
                    end
                endcase
            end
        end
    end

    always @(*) begin
        command_valid_o = active_o;
        linear_o = {COMMAND_WIDTH{1'b0}};
        angular_o = {COMMAND_WIDTH{1'b0}};

        case (state_o)
            STATE_BACKUP:
                linear_o = -BACKUP_COMMAND;

            STATE_TURN:
                angular_o = avoid_left_o ? TURN_COMMAND : -TURN_COMMAND;

            STATE_BYPASS:
                linear_o = BYPASS_COMMAND;

            STATE_RECOVER:
                angular_o = avoid_left_o ? -TURN_COMMAND : TURN_COMMAND;

            default: begin
                linear_o = {COMMAND_WIDTH{1'b0}};
                angular_o = {COMMAND_WIDTH{1'b0}};
            end
        endcase
    end

endmodule
