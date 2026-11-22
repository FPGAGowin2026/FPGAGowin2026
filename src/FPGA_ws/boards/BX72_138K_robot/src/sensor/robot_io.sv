`timescale 1ns/1ps

// Common-clock I/O boundary. Navigation commands must already be registered
// in clk_i; a separate front-lidar guard and command watchdog are required.
module robot_io #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer SAMPLE_HZ = 50,
    parameter integer MAX_WHEEL_MM_S = motor_closed_loop_config::MAX_WHEEL_MM_S,
    parameter integer WHEEL_STEP_MM_S = 10,
    parameter [7:0] MOTOR_CHANNEL_MAP = 8'hd2,
    parameter [7:0] ENCODER_CHANNEL_MAP = 8'hd2,
    parameter [3:0] MOTOR_REVERSE = 4'b1011,
    parameter [3:0] ENCODER_REVERSE = 4'b1010
) (
    input wire clk_i,
    input wire reset_n_i,
    input wire lidar_rx_i,
    input wire imu_miso_i,
    output wire imu_cs_n_o,
    output wire imu_sck_o,
    output wire imu_mosi_o,
    input wire [3:0] encoder_a_i,
    input wire [3:0] encoder_b_i,
    input wire motor_enable_i,
    input wire signed [31:0] vx_mm_s_i,
    input wire signed [31:0] vy_mm_s_i,
    input wire signed [31:0] wz_mrad_s_i,
    output wire [3:0] motor_in1_o,
    output wire [3:0] motor_in2_o,
    output wire [31:0] time_us_o,
    output wire lidar_sample_o,
    output wire lidar_hit_o,
    output wire lidar_in_front_o,
    output wire [15:0] lidar_angle_cdeg_o,
    output wire [15:0] lidar_distance_mm_o,
    output wire [7:0] lidar_strength_o,
    output wire [31:0] lidar_packet_received_us_o,
    output wire [31:0] lidar_packets_o,
    output wire [31:0] lidar_checksum_errors_o,
    output wire [31:0] lidar_length_errors_o,
    output wire [31:0] lidar_angle_errors_o,
    output wire lidar_uart_error_o,
    output wire imu_present_o,
    output wire imu_configured_o,
    output wire imu_identity_checked_o,
    output wire [31:0] imu_identity_errors_o,
    output wire [7:0] imu_who_am_i_o,
    output reg imu_sample_o,
    output wire [15:0] imu_sequence_o,
    output reg [31:0] imu_received_us_o,
    output wire signed [15:0] accel_x_o, accel_y_o, accel_z_o,
    output wire signed [15:0] gyro_x_o, gyro_y_o, gyro_z_o,
    output wire [127:0] wheel_position_ticks_o,
    output wire [127:0] wheel_speed_mm_s_o,
    output wire [127:0] wheel_target_mm_s_o,
    output wire [3:0] encoder_invalid_o,
    output reg wheel_sample_o,
    output reg [31:0] wheel_received_us_o
);
    localparam integer CYCLES_PER_US = CLK_HZ / 1_000_000;
    localparam integer CLOCK_COUNTER_WIDTH = CYCLES_PER_US <= 1 ? 1 : $clog2(CYCLES_PER_US);
    reg [CLOCK_COUNTER_WIDTH-1:0] microsecond_counter;
    reg [31:0] time_us;
    wire [3:0] encoder_a_logical, encoder_b_logical;
    wire [3:0] motor_in1_logical, motor_in2_logical;
    wire imu_sample_raw, wheel_sample_raw;

    assign time_us_o = time_us;
    always @(posedge clk_i or negedge reset_n_i) begin
        if (!reset_n_i) begin
            microsecond_counter <= 0;
            time_us <= 0;
            imu_received_us_o <= 0;
            wheel_received_us_o <= 0;
            imu_sample_o <= 0;
            wheel_sample_o <= 0;
        end else begin
            if (microsecond_counter == CYCLES_PER_US - 1) begin
                microsecond_counter <= 0;
                time_us <= time_us + 1'b1;
            end else microsecond_counter <= microsecond_counter + 1'b1;
            // These are readout/publish times, not guaranteed sensor sampling times.
            imu_sample_o <= imu_sample_raw;
            wheel_sample_o <= wheel_sample_raw;
            if (imu_sample_raw) imu_received_us_o <= time_us;
            if (wheel_sample_raw) wheel_received_us_o <= time_us;
        end
    end

    n10_receiver #(.CLK_HZ(CLK_HZ), .BAUD(230400)) lidar (
        .clk_i(clk_i), .reset_n_i(reset_n_i), .rx_i(lidar_rx_i), .now_us_i(time_us),
        .sample_valid_o(lidar_sample_o), .hit_valid_o(lidar_hit_o), .in_front_o(lidar_in_front_o),
        .angle_cdeg_o(lidar_angle_cdeg_o), .distance_mm_o(lidar_distance_mm_o),
        .strength_o(lidar_strength_o), .packet_received_us_o(lidar_packet_received_us_o),
        .packet_count_o(lidar_packets_o), .checksum_errors_o(lidar_checksum_errors_o),
        .length_errors_o(lidar_length_errors_o), .angle_errors_o(lidar_angle_errors_o),
        .uart_error_o(lidar_uart_error_o)
    );
    imu660rc_spi_reader #(.CLK_HZ(CLK_HZ)) imu (
        .clk(clk_i), .rst(!reset_n_i), .spi_miso_i(imu_miso_i),
        .spi_cs_n_o(imu_cs_n_o), .spi_sck_o(imu_sck_o), .spi_mosi_o(imu_mosi_o),
        .imu_present_o(imu_present_o), .identity_checked_o(imu_identity_checked_o),
        .identity_error_count_o(imu_identity_errors_o), .who_am_i_o(imu_who_am_i_o),
        .imu_configured_o(imu_configured_o), .sample_valid_o(imu_sample_raw),
        .accel_x_o(accel_x_o), .accel_y_o(accel_y_o), .accel_z_o(accel_z_o),
        .gyro_x_o(gyro_x_o), .gyro_y_o(gyro_y_o), .gyro_z_o(gyro_z_o),
        .sample_sequence_o(imu_sequence_o)
    );

    genvar wheel;
    generate for (wheel = 0; wheel < 4; wheel = wheel + 1) begin: g_wheel_mapping
        localparam [1:0] ENC_CHANNEL = (ENCODER_CHANNEL_MAP >> (wheel * 2)) & 2'b11;
        localparam [1:0] MOTOR_CHANNEL = (MOTOR_CHANNEL_MAP >> (wheel * 2)) & 2'b11;
        assign encoder_a_logical[wheel] = encoder_a_i[ENC_CHANNEL];
        assign encoder_b_logical[wheel] = encoder_b_i[ENC_CHANNEL];
        assign motor_in1_o[MOTOR_CHANNEL] = motor_in1_logical[wheel];
        assign motor_in2_o[MOTOR_CHANNEL] = motor_in2_logical[wheel];
    end endgenerate
    mecanum_motor_control #(
        .CLK_HZ(CLK_HZ), .SAMPLE_HZ(SAMPLE_HZ), .GEOMETRY_MM(178),
        .MAX_WHEEL_MM_S(MAX_WHEEL_MM_S), .STEP_MM_S(WHEEL_STEP_MM_S),
        .MM_PER_TICK_Q16(32'sd164), .PWM_PERIOD(2500),
        .ENCODER_REVERSE(ENCODER_REVERSE), .MOTOR_REVERSE(MOTOR_REVERSE)
    ) motor (
        .clk_i(clk_i), .rst_n_i(reset_n_i), .enable_i(motor_enable_i),
        .vx_mm_s_i(vx_mm_s_i), .vy_mm_s_i(vy_mm_s_i), .wz_mrad_s_i(wz_mrad_s_i),
        .encoder_a_i(encoder_a_logical), .encoder_b_i(encoder_b_logical),
        .in1_o(motor_in1_logical), .in2_o(motor_in2_logical),
        .position_ticks_o(wheel_position_ticks_o), .measured_mm_s_o(wheel_speed_mm_s_o),
        .target_mm_s_o(wheel_target_mm_s_o), .encoder_invalid_o(encoder_invalid_o),
        .sample_valid_o(wheel_sample_raw)
    );
endmodule
