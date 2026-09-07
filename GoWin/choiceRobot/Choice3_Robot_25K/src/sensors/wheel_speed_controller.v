`timescale 1ns/1ps

// 每个轮子的离散 PI 速度环。增益和 PWM 上限均为实车标定入口。
module wheel_speed_controller #(
    parameter signed [31:0] KP = 32'sd8,
    parameter signed [31:0] KI = 32'sd1,
    parameter integer GAIN_SHIFT = 4,
    parameter integer PWM_MAX = 1000
) (
    input  wire               clk_i,
    input  wire               rst_n_i,
    input  wire               enable_i,
    input  wire               sample_tick_i,
    input  wire signed [31:0] target_velocity_mm_s_i,
    input  wire signed [31:0] measured_velocity_mm_s_i,
    output reg  [15:0]        pwm_duty_o,
    output reg                direction_o
);
    localparam signed [63:0] INTEGRAL_LIMIT = PWM_MAX <<< GAIN_SHIFT;
    localparam signed [63:0] PWM_MAX_SIGNED = PWM_MAX;

    reg signed [63:0] integral_q;
    reg signed [63:0] error_q;
    reg signed [63:0] proportional_q;
    reg signed [63:0] integral_stage_q;
    reg signed [63:0] control_q;
    reg stage1_valid_q,stage2_valid_q,stage3_valid_q;
    reg target_negative_q;
    reg signed [63:0] integral_candidate;

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            integral_q <= 64'sd0;
            pwm_duty_o <= 16'd0;
            direction_o <= 1'b1;
            error_q <= 64'sd0;
            proportional_q <= 64'sd0;
            integral_stage_q <= 64'sd0;
            control_q <= 64'sd0;
            stage1_valid_q <= 1'b0;
            stage2_valid_q <= 1'b0;
            stage3_valid_q <= 1'b0;
            target_negative_q <= 1'b0;
        end else if (!enable_i || target_velocity_mm_s_i == 0) begin
            integral_q <= 64'sd0;
            pwm_duty_o <= 16'd0;
            direction_o <= 1'b1;
            stage1_valid_q <= 1'b0;
            stage2_valid_q <= 1'b0;
            stage3_valid_q <= 1'b0;
        end else begin
            // 100Hz 计算拆成四个 50MHz 流水阶段，控制延迟约 80ns，远小于
            // 10ms 采样周期；这样不会把健康/导航组合路径串进 64 位 PI 算术。
            stage1_valid_q <= sample_tick_i;
            stage2_valid_q <= stage1_valid_q;
            stage3_valid_q <= stage2_valid_q;

            // 第1拍：锁存误差与目标符号。
            if (sample_tick_i) begin
                error_q <= {{32{target_velocity_mm_s_i[31]}}, target_velocity_mm_s_i} -
                           {{32{measured_velocity_mm_s_i[31]}}, measured_velocity_mm_s_i};
                target_negative_q <= target_velocity_mm_s_i < 0;
            end

            // 第2拍：比例项和积分项分别计算并限幅。
            if (stage1_valid_q) begin
                proportional_q <= error_q * KP;
                if ((target_negative_q && integral_q > 0) ||
                    (!target_negative_q && integral_q < 0))
                    integral_candidate = error_q * KI;
                else
                    integral_candidate = integral_q + error_q * KI;
                if (integral_candidate > INTEGRAL_LIMIT)
                    integral_stage_q <= INTEGRAL_LIMIT;
                else if (integral_candidate < -INTEGRAL_LIMIT)
                    integral_stage_q <= -INTEGRAL_LIMIT;
                else
                    integral_stage_q <= integral_candidate;
            end

            // 第3拍：合成控制量并提交积分状态。
            if (stage2_valid_q) begin
                control_q <= (proportional_q + integral_stage_q) >>> GAIN_SHIFT;
                integral_q <= integral_stage_q;
            end

            // 第4拍：方向与 PWM 绝对值/饱和。
            if (stage3_valid_q) begin
                direction_o <= control_q >= 0;
                if (control_q > PWM_MAX_SIGNED || control_q < -PWM_MAX_SIGNED)
                    pwm_duty_o <= PWM_MAX;
                else if (control_q < 0)
                    pwm_duty_o <= (~control_q[15:0]) + 16'd1;
                else
                    pwm_duty_o <= control_q[15:0];
            end
        end
    end
endmodule
