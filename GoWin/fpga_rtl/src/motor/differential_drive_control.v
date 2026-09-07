`timescale 1ns/1ps

// 将车体线/角速度换算为左右轮目标，并运行两个独立 PI 速度环。
// angular_mrad_s * TRACK_WIDTH_MM / 2000 得到单侧切向速度 mm/s。
module differential_drive_control #(
    parameter integer TRACK_WIDTH_MM = 300,
    parameter integer MAX_WHEEL_SPEED_MM_S = 1200,
    parameter signed [31:0] KP = 32'sd8,
    parameter signed [31:0] KI = 32'sd1,
    parameter integer GAIN_SHIFT = 4,
    parameter integer PWM_MAX = 1000
) (
    input  wire               clk_i,
    input  wire               rst_n_i,
    input  wire               enable_i,
    input  wire               sample_tick_i,
    input  wire signed [31:0] linear_mm_s_i,
    input  wire signed [31:0] angular_mrad_s_i,
    input  wire signed [31:0] left_velocity_mm_s_i,
    input  wire signed [31:0] right_velocity_mm_s_i,
    output reg signed [31:0]  left_target_mm_s_o,
    output reg signed [31:0]  right_target_mm_s_o,
    output wire [15:0]        left_pwm_duty_o,
    output wire               left_direction_o,
    output wire [15:0]        right_pwm_duty_o,
    output wire               right_direction_o
);
    reg signed [63:0] turn_velocity;
    reg signed [63:0] angular_wide;
    reg signed [63:0] left_unclamped;
    reg signed [63:0] right_unclamped;
    wire [15:0] left_pwm_raw;
    wire [15:0] right_pwm_raw;

    // 最终 PWM 再做组合门控，安全零目标不等待下一个速度采样拍。
    assign left_pwm_duty_o =
        (enable_i && left_target_mm_s_o != 0) ? left_pwm_raw : 16'd0;
    assign right_pwm_duty_o =
        (enable_i && right_target_mm_s_o != 0) ? right_pwm_raw : 16'd0;

    function signed [31:0] clamp_velocity;
        input signed [63:0] value;
        begin
            if (value > MAX_WHEEL_SPEED_MM_S)
                clamp_velocity = MAX_WHEEL_SPEED_MM_S;
            else if (value < -MAX_WHEEL_SPEED_MM_S)
                clamp_velocity = -MAX_WHEEL_SPEED_MM_S;
            else
                clamp_velocity = value[31:0];
        end
    endfunction

    always @(*) begin
        angular_wide = angular_mrad_s_i;
        turn_velocity = (angular_wide * TRACK_WIDTH_MM) / 2000;
        left_unclamped = linear_mm_s_i - turn_velocity;
        right_unclamped = linear_mm_s_i + turn_velocity;
        if (enable_i) begin
            left_target_mm_s_o = clamp_velocity(left_unclamped);
            right_target_mm_s_o = clamp_velocity(right_unclamped);
        end else begin
            left_target_mm_s_o = 32'sd0;
            right_target_mm_s_o = 32'sd0;
        end
    end

    wheel_speed_controller #(
        .KP(KP), .KI(KI), .GAIN_SHIFT(GAIN_SHIFT), .PWM_MAX(PWM_MAX)
    ) left_controller (
        .clk_i(clk_i), .rst_n_i(rst_n_i), .enable_i(enable_i),
        .sample_tick_i(sample_tick_i),
        .target_velocity_mm_s_i(left_target_mm_s_o),
        .measured_velocity_mm_s_i(left_velocity_mm_s_i),
        .pwm_duty_o(left_pwm_raw), .direction_o(left_direction_o)
    );

    wheel_speed_controller #(
        .KP(KP), .KI(KI), .GAIN_SHIFT(GAIN_SHIFT), .PWM_MAX(PWM_MAX)
    ) right_controller (
        .clk_i(clk_i), .rst_n_i(rst_n_i), .enable_i(enable_i),
        .sample_tick_i(sample_tick_i),
        .target_velocity_mm_s_i(right_target_mm_s_o),
        .measured_velocity_mm_s_i(right_velocity_mm_s_i),
        .pwm_duty_o(right_pwm_raw), .direction_o(right_direction_o)
    );
endmodule
