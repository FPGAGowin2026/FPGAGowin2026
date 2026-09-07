`timescale 1ns/1ps

// 无实板阶段的系统级 RTL 骨架：协议、传感器、独立路径、绕障、模式仲裁和电机 PI。
// 板级 PHY、I2C/SPI、DDR3/HDMI、引脚约束、CDC 与物理急停断电必须在 Gowin 顶层补齐。
module robot_rtl_core #(
    parameter integer CONTROL_TIMEOUT_CYCLES = 25000000,
    parameter integer RANGE_TIMEOUT_CYCLES = 10000000,
    parameter [15:0] STOP_DISTANCE_MM = 16'd200,
    parameter [15:0] SLOW_DISTANCE_MM = 16'd500,
    parameter integer AVOID_STOP_TICKS = 2,
    parameter integer AVOID_BACKUP_TICKS = 20,
    parameter integer AVOID_TURN_TICKS = 20,
    parameter integer AVOID_BYPASS_TICKS = 40,
    parameter integer AVOID_RECOVER_TICKS = 20
) (
    input  wire               clk_i,
    input  wire               rst_n_i,
    input  wire               requested_mode_i,
    input  wire               motion_stopped_i,
    input  wire               physical_estop_i,
    input  wire               critical_fault_i,

    input  wire               udp_payload_valid_i,
    input  wire [7:0]         udp_payload_byte_i,
    input  wire               udp_packet_done_i,
    input  wire               udp_packet_error_i,

    input  wire               front_sample_valid_i,
    input  wire [15:0]        front_distance_mm_i,
    input  wire               left_sample_valid_i,
    input  wire [15:0]        left_distance_mm_i,
    input  wire               right_sample_valid_i,
    input  wire [15:0]        right_distance_mm_i,

    input  wire               imu_sample_start_i,
    input  wire               imu_byte_valid_i,
    input  wire [7:0]         imu_byte_data_i,
    output wire               imu_byte_ready_o,
    output wire [95:0]        imu_sample_data_o,
    output wire               imu_sample_valid_o,

    input  wire               camera_pclk_i,
    input  wire               camera_vsync_i,
    input  wire               camera_href_i,
    input  wire [7:0]         camera_data_i,
    output wire               camera_frame_valid_o,
    output wire               camera_line_valid_o,
    output wire               camera_pixel_valid_o,
    output wire [15:0]        camera_pixel_rgb565_o,

    input  wire               path_start_i,
    input  wire               path_abort_i,
    input  wire               avoidance_tick_i,
    input  wire [31:0]        segment_progress_i,
    input  wire               motor_sample_tick_i,
    input  wire signed [31:0] left_velocity_mm_s_i,
    input  wire signed [31:0] right_velocity_mm_s_i,

    output wire               active_mode_o,
    output wire               switch_pending_o,
    output wire               motion_enabled_o,
    output wire signed [31:0] final_linear_mm_s_o,
    output wire signed [31:0] final_angular_mrad_s_o,
    output wire signed [31:0] left_target_mm_s_o,
    output wire signed [31:0] right_target_mm_s_o,
    output wire [15:0]        left_pwm_duty_o,
    output wire               left_direction_o,
    output wire [15:0]        right_pwm_duty_o,
    output wire               right_direction_o,

    output wire [15:0]        front_range_mm_o,
    output wire [15:0]        left_range_mm_o,
    output wire [15:0]        right_range_mm_o,
    output wire               ranges_valid_o,
    output wire [1:0]         safety_level_o,
    output wire               avoidance_active_o,
    output wire               path_active_o,
    output wire               path_finished_o,
    output wire               path_progress_enable_o,
    output wire               path_paused_o,
    output wire [1:0]         path_segment_opcode_o,
    output wire [7:0]         segment_index_o,
    output wire               software_estop_o,
    output wire               control_command_active_o,
    output wire               control_timeout_pulse_o,
    output wire [31:0]        last_control_sequence_o,
    output wire               imu_sample_error_o,
    output wire               control_packet_accepted_o,
    output wire               control_packet_rejected_o
);
    wire decoded_estop;
    wire signed [31:0] decoded_linear;
    wire signed [31:0] decoded_angular;
    wire jetson_command_valid;
    wire signed [31:0] jetson_linear;
    wire signed [31:0] jetson_angular;
    wire jetson_limited_valid;
    wire signed [31:0] jetson_limited_linear;
    wire signed [31:0] jetson_limited_angular;
    wire [1:0] jetson_safety_level;

    wire front_valid;
    wire left_valid;
    wire right_valid;

    wire segment_ready;
    wire segment_valid;
    wire [1:0] segment_opcode;
    wire [31:0] segment_target;
    wire signed [31:0] segment_command;
    wire segment_last;
    wire avoidance_done;
    wire avoid_left;
    wire [2:0] avoidance_state;
    wire autonomous_command_valid;
    wire signed [31:0] autonomous_linear;
    wire signed [31:0] autonomous_angular;
    wire [1:0] autonomous_safety_level;
    wire autonomy_mode_allowed;
    wire autonomy_abort;
    wire command_invalidate;

    fpga_control_packet_rx control_decoder (
        .clk_i(clk_i), .rst_n_i(rst_n_i),
        .payload_valid_i(udp_payload_valid_i), .payload_byte_i(udp_payload_byte_i),
        .packet_done_i(udp_packet_done_i), .packet_error_i(udp_packet_error_i),
        .control_valid_o(control_packet_accepted_o),
        .packet_rejected_o(control_packet_rejected_o), .duplicate_sequence_o(),
        .sequence_o(last_control_sequence_o), .timestamp_us_o(),
        .linear_mm_s_o(decoded_linear), .angular_mrad_s_o(decoded_angular),
        .estop_o(decoded_estop)
    );

    control_command_watchdog #(
        .TIMEOUT_CYCLES(CONTROL_TIMEOUT_CYCLES)
    ) command_watchdog (
        .clk_i(clk_i), .rst_n_i(rst_n_i),
        .command_update_i(control_packet_accepted_o),
        .command_invalidate_i(command_invalidate),
        .linear_mm_s_i(decoded_linear), .angular_mrad_s_i(decoded_angular),
        .estop_i(decoded_estop), .command_valid_o(jetson_command_valid),
        .linear_mm_s_o(jetson_linear), .angular_mrad_s_o(jetson_angular),
        .software_estop_o(software_estop_o),
        .timeout_pulse_o(control_timeout_pulse_o)
    );

    // 离开协同模式、物理急停或严重故障后，旧速度不得自动恢复。
    assign command_invalidate = !requested_mode_i || physical_estop_i || critical_fault_i;

    ov5640_dvp_capture camera_capture (
        .pclk_i(camera_pclk_i), .rst_n_i(rst_n_i),
        .vsync_i(camera_vsync_i), .href_i(camera_href_i), .data_i(camera_data_i),
        .frame_valid_o(camera_frame_valid_o), .line_valid_o(camera_line_valid_o),
        .pixel_valid_o(camera_pixel_valid_o), .pixel_rgb565_o(camera_pixel_rgb565_o)
    );

    imu_sample_commit imu_capture (
        .clk_i(clk_i), .rst_n_i(rst_n_i), .sample_start_i(imu_sample_start_i),
        .byte_valid_i(imu_byte_valid_i), .byte_data_i(imu_byte_data_i),
        .byte_ready_o(imu_byte_ready_o), .sample_data_o(imu_sample_data_o),
        .sample_valid_o(imu_sample_valid_o), .sample_error_o(imu_sample_error_o)
    );

    range_validity_monitor #(
        .TIMEOUT_CYCLES(RANGE_TIMEOUT_CYCLES)
    ) range_monitor (
        .clk_i(clk_i), .rst_n_i(rst_n_i),
        .front_sample_valid_i(front_sample_valid_i),
        .front_distance_mm_i(front_distance_mm_i),
        .left_sample_valid_i(left_sample_valid_i), .left_distance_mm_i(left_distance_mm_i),
        .right_sample_valid_i(right_sample_valid_i), .right_distance_mm_i(right_distance_mm_i),
        .front_distance_mm_o(front_range_mm_o), .front_valid_o(front_valid),
        .front_timeout_o(), .front_invalid_sample_o(),
        .left_distance_mm_o(left_range_mm_o), .left_valid_o(left_valid),
        .left_timeout_o(), .left_invalid_sample_o(),
        .right_distance_mm_o(right_range_mm_o), .right_valid_o(right_valid),
        .right_timeout_o(), .right_invalid_sample_o()
    );
    assign ranges_valid_o = front_valid && left_valid && right_valid;
    assign control_command_active_o = jetson_command_valid;

    demo_path_rom path_rom (
        .segment_ready_i(segment_ready), .segment_index_i(segment_index_o),
        .segment_valid_o(segment_valid), .segment_opcode_o(segment_opcode),
        .segment_target_o(segment_target), .segment_command_o(segment_command),
        .segment_last_o(segment_last)
    );
    assign path_segment_opcode_o = segment_opcode;

    // 独立路径只属于稳定生效的 FPGA 模式；任何离开请求或安全故障都中止状态机。
    assign autonomy_mode_allowed =
        rst_n_i && !requested_mode_i && !active_mode_o && !switch_pending_o &&
        !physical_estop_i && !critical_fault_i && !software_estop_o;
    assign autonomy_abort = path_abort_i || !autonomy_mode_allowed;

    autonomous_motion_controller #(
        .STOP_DISTANCE(STOP_DISTANCE_MM), .SLOW_DISTANCE(SLOW_DISTANCE_MM),
        .STOP_TICKS(AVOID_STOP_TICKS), .BACKUP_TICKS(AVOID_BACKUP_TICKS),
        .TURN_TICKS(AVOID_TURN_TICKS), .BYPASS_TICKS(AVOID_BYPASS_TICKS),
        .RECOVER_TICKS(AVOID_RECOVER_TICKS)
    ) autonomous_controller (
        .clk_i(clk_i), .rst_n_i(rst_n_i),
        .start_i(path_start_i && autonomy_mode_allowed),
        .abort_i(autonomy_abort), .path_pause_i(1'b0),
        .motion_stopped_i(motion_stopped_i), .avoidance_tick_i(avoidance_tick_i),
        .front_range_i(front_range_mm_o), .left_range_i(left_range_mm_o),
        .right_range_i(right_range_mm_o), .front_range_valid_i(front_valid),
        .left_range_valid_i(left_valid), .right_range_valid_i(right_valid),
        .segment_ready_o(segment_ready), .segment_valid_i(segment_valid),
        .segment_opcode_i(segment_opcode), .segment_target_i(segment_target),
        .segment_command_i(segment_command), .segment_last_i(segment_last),
        .segment_progress_i(segment_progress_i), .path_active_o(path_active_o),
        .path_finished_o(path_finished_o),
        .path_progress_enable_o(path_progress_enable_o),
        .segment_index_o(segment_index_o), .path_paused_o(path_paused_o),
        .avoidance_active_o(avoidance_active_o), .avoidance_done_o(avoidance_done),
        .avoid_left_o(avoid_left), .avoidance_state_o(avoidance_state),
        .safety_level_o(autonomous_safety_level),
        .command_valid_o(autonomous_command_valid),
        .linear_o(autonomous_linear), .angular_o(autonomous_angular)
    );

    // Jetson 模式沿用同一前向测距安全约束，但不由 FPGA 主动选择绕行方向。
    range_safety_limiter #(
        .STOP_DISTANCE(STOP_DISTANCE_MM), .SLOW_DISTANCE(SLOW_DISTANCE_MM)
    ) jetson_safety_limiter (
        .command_valid_i(jetson_command_valid), .linear_i(jetson_linear),
        .angular_i(jetson_angular), .front_valid_i(front_valid),
        .front_range_i(front_range_mm_o), .command_valid_o(jetson_limited_valid),
        .linear_o(jetson_limited_linear), .angular_o(jetson_limited_angular),
        .safety_level_o(jetson_safety_level), .forward_blocked_o()
    );

    assign safety_level_o = active_mode_o ? jetson_safety_level : autonomous_safety_level;

    motion_mode_arbiter mode_arbiter (
        .clk_i(clk_i), .rst_n_i(rst_n_i), .requested_mode_i(requested_mode_i),
        .motion_stopped_i(motion_stopped_i),
        .estop_i(physical_estop_i || software_estop_o),
        .critical_fault_i(critical_fault_i), .safety_stop_i(1'b0),
        .fpga_command_valid_i(autonomous_command_valid),
        .fpga_linear_i(autonomous_linear), .fpga_angular_i(autonomous_angular),
        .jetson_command_valid_i(jetson_limited_valid),
        .jetson_linear_i(jetson_limited_linear),
        .jetson_angular_i(jetson_limited_angular),
        .active_mode_o(active_mode_o), .switch_pending_o(switch_pending_o),
        .motion_enabled_o(motion_enabled_o),
        .linear_command_o(final_linear_mm_s_o),
        .angular_command_o(final_angular_mrad_s_o)
    );

    differential_drive_control motor_control (
        .clk_i(clk_i), .rst_n_i(rst_n_i), .enable_i(motion_enabled_o),
        .sample_tick_i(motor_sample_tick_i),
        .linear_mm_s_i(final_linear_mm_s_o),
        .angular_mrad_s_i(final_angular_mrad_s_o),
        .left_velocity_mm_s_i(left_velocity_mm_s_i),
        .right_velocity_mm_s_i(right_velocity_mm_s_i),
        .left_target_mm_s_o(left_target_mm_s_o),
        .right_target_mm_s_o(right_target_mm_s_o),
        .left_pwm_duty_o(left_pwm_duty_o), .left_direction_o(left_direction_o),
        .right_pwm_duty_o(right_pwm_duty_o), .right_direction_o(right_direction_o)
    );

endmodule
