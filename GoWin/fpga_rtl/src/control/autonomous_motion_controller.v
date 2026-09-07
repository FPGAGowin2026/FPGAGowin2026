`timescale 1ns/1ps

// 组合独立路径、局部绕障与三路测距安全闭环。
// 本模块输出仍是抽象运动目标，需经现有模式仲裁器和后续电机闭环后才能驱动车轮。
module autonomous_motion_controller #(
    parameter COMMAND_WIDTH = 32,
    parameter RANGE_WIDTH = 16,
    parameter PROGRESS_WIDTH = 32,
    parameter [RANGE_WIDTH-1:0] STOP_DISTANCE = 16'd200,
    parameter [RANGE_WIDTH-1:0] SLOW_DISTANCE = 16'd500,
    parameter STOP_TICKS = 2,
    parameter BACKUP_TICKS = 20,
    parameter TURN_TICKS = 20,
    parameter BYPASS_TICKS = 40,
    parameter RECOVER_TICKS = 20,
    parameter integer BACKUP_COMMAND = 100,
    parameter integer TURN_COMMAND = 300,
    parameter integer BYPASS_COMMAND = 150
) (
    input  wire                             clk_i,
    input  wire                             rst_n_i,
    input  wire                             start_i,
    input  wire                             abort_i,
    input  wire                             path_pause_i,
    input  wire                             motion_stopped_i,
    input  wire                             avoidance_tick_i,

    input  wire        [RANGE_WIDTH-1:0]    front_range_i,
    input  wire        [RANGE_WIDTH-1:0]    left_range_i,
    input  wire        [RANGE_WIDTH-1:0]    right_range_i,
    input  wire                             front_range_valid_i,
    input  wire                             left_range_valid_i,
    input  wire                             right_range_valid_i,

    output wire                             segment_ready_o,
    input  wire                             segment_valid_i,
    input  wire        [1:0]                segment_opcode_i,
    input  wire        [PROGRESS_WIDTH-1:0] segment_target_i,
    input  wire signed [COMMAND_WIDTH-1:0]  segment_command_i,
    input  wire                             segment_last_i,
    input  wire        [PROGRESS_WIDTH-1:0] segment_progress_i,

    output wire                             path_active_o,
    output wire                             path_finished_o,
    output wire                             path_progress_enable_o,
    output wire        [7:0]                segment_index_o,
    output wire                             path_paused_o,
    output wire                             avoidance_active_o,
    output wire                             avoidance_done_o,
    output wire                             avoid_left_o,
    output wire        [2:0]                avoidance_state_o,
    output wire        [1:0]                safety_level_o,
    output wire                             command_valid_o,
    output wire signed [COMMAND_WIDTH-1:0]  linear_o,
    output wire signed [COMMAND_WIDTH-1:0]  angular_o
);

    wire path_command_valid;
    wire signed [COMMAND_WIDTH-1:0] path_linear;
    wire signed [COMMAND_WIDTH-1:0] path_angular;
    wire avoidance_command_valid;
    wire signed [COMMAND_WIDTH-1:0] avoidance_linear;
    wire signed [COMMAND_WIDTH-1:0] avoidance_angular;
    wire selected_command_valid;
    wire signed [COMMAND_WIDTH-1:0] selected_linear;
    wire signed [COMMAND_WIDTH-1:0] selected_angular;
    wire forward_blocked;
    wire path_forward_blocked;
    wire avoidance_request;
    wire internal_path_pause;
    wire limited_command_valid;

    // 只有正在执行正向 MOVE 时，前方停车条件才启动主动绕障。
    assign path_forward_blocked =
        path_active_o && path_linear > 0 && forward_blocked;
    assign avoidance_request =
        path_forward_blocked && !path_pause_i && !avoidance_active_o &&
        front_range_valid_i && left_range_valid_i && right_range_valid_i;
    assign internal_path_pause =
        path_pause_i || avoidance_active_o || path_forward_blocked;
    assign path_paused_o = internal_path_pause;

    assign selected_command_valid =
        avoidance_active_o ? avoidance_command_valid : path_command_valid;
    assign selected_linear =
        avoidance_active_o ? avoidance_linear : path_linear;
    assign selected_angular =
        avoidance_active_o ? avoidance_angular : path_angular;

    path_segment_executor #(
        .COMMAND_WIDTH(COMMAND_WIDTH),
        .PROGRESS_WIDTH(PROGRESS_WIDTH)
    ) path_executor (
        .clk_i(clk_i),
        .rst_n_i(rst_n_i),
        .start_i(start_i),
        .abort_i(abort_i),
        .pause_i(internal_path_pause),
        .motion_stopped_i(motion_stopped_i),
        .segment_ready_o(segment_ready_o),
        .segment_valid_i(segment_valid_i),
        .segment_opcode_i(segment_opcode_i),
        .segment_target_i(segment_target_i),
        .segment_command_i(segment_command_i),
        .segment_last_i(segment_last_i),
        .segment_progress_i(segment_progress_i),
        .path_active_o(path_active_o),
        .path_finished_o(path_finished_o),
        .progress_enable_o(path_progress_enable_o),
        .segment_index_o(segment_index_o),
        .command_valid_o(path_command_valid),
        .linear_o(path_linear),
        .angular_o(path_angular)
    );

    local_avoidance_fsm #(
        .COMMAND_WIDTH(COMMAND_WIDTH),
        .RANGE_WIDTH(RANGE_WIDTH),
        .STOP_TICKS(STOP_TICKS),
        .BACKUP_TICKS(BACKUP_TICKS),
        .TURN_TICKS(TURN_TICKS),
        .BYPASS_TICKS(BYPASS_TICKS),
        .RECOVER_TICKS(RECOVER_TICKS),
        .BACKUP_COMMAND(BACKUP_COMMAND),
        .TURN_COMMAND(TURN_COMMAND),
        .BYPASS_COMMAND(BYPASS_COMMAND)
    ) avoidance (
        .clk_i(clk_i),
        .rst_n_i(rst_n_i),
        .start_i(avoidance_request),
        .abort_i(abort_i),
        .phase_tick_i(avoidance_tick_i),
        .forward_allowed_i(!forward_blocked),
        .left_range_i(left_range_i),
        .right_range_i(right_range_i),
        .active_o(avoidance_active_o),
        .done_o(avoidance_done_o),
        .avoid_left_o(avoid_left_o),
        .state_o(avoidance_state_o),
        .command_valid_o(avoidance_command_valid),
        .linear_o(avoidance_linear),
        .angular_o(avoidance_angular)
    );

    range_safety_limiter #(
        .COMMAND_WIDTH(COMMAND_WIDTH),
        .RANGE_WIDTH(RANGE_WIDTH),
        .STOP_DISTANCE(STOP_DISTANCE),
        .SLOW_DISTANCE(SLOW_DISTANCE)
    ) safety_limiter (
        .command_valid_i(selected_command_valid && !abort_i),
        .linear_i(selected_linear),
        .angular_i(selected_angular),
        .front_valid_i(front_range_valid_i),
        .front_range_i(front_range_i),
        .command_valid_o(limited_command_valid),
        .linear_o(linear_o),
        .angular_o(angular_o),
        .safety_level_o(safety_level_o),
        .forward_blocked_o(forward_blocked)
    );

    assign command_valid_o = limited_command_valid && !abort_i;

endmodule
