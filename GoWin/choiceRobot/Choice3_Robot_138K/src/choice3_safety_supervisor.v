`timescale 1ns/1ps

// 赛题三统一安全监控器。
// 约束：健康恢复后绝不自动重新武装，必须重新收到 arm 脉冲。
module choice3_safety_supervisor #(
    parameter [7:0] REQUIRED_HEALTH_MASK = 8'h1f
) (
    input  wire       clk_i,
    input  wire       rst_n_i,
    input  wire       estop_n_i,
    input  wire [7:0] health_i,
    input  wire       arm_pulse_i,
    input  wire       disarm_pulse_i,
    input  wire       clear_fault_pulse_i,
    output reg        armed_o,
    output wire       motor_enable_o,
    output wire [7:0] fault_now_o,
    output reg  [7:0] fault_latched_o,
    output reg        arm_rejected_pulse_o
);
    wire [7:0] missing_health = REQUIRED_HEALTH_MASK & ~health_i;

    // bit7 专用于物理急停；其余位与健康掩码逐位对应。
    assign fault_now_o = missing_health | (estop_n_i ? 8'h00 : 8'h80);
    assign motor_enable_o = armed_o && estop_n_i && (missing_health == 8'h00);

    reg estop_seen_q;

    // 急停只对 armed 触发异步清零；诊断锁存器不使用异步自反馈结构。
    always @(posedge clk_i or negedge rst_n_i or negedge estop_n_i) begin
        if (!rst_n_i) begin
            armed_o <= 1'b0;
        end else if (!estop_n_i) begin
            armed_o <= 1'b0;
        end else begin
            if (disarm_pulse_i || missing_health != 8'h00)
                armed_o <= 1'b0;
            else if (arm_pulse_i && missing_health == 8'h00)
                    armed_o <= 1'b1;
        end
    end

    // 窄急停脉冲也会异步置位事件位。这里只保留一个异步控制（preset），
    // 避免同一触发器同时异步 set/reset；系统复位与清故障均在时钟沿同步清除。
    // 功率引脚的即时关闭不依赖此诊断位，而由 armed_o 与顶层 raw 门控完成。
    always @(posedge clk_i or negedge estop_n_i) begin
        if (!estop_n_i) estop_seen_q <= 1'b1;
        else if (!rst_n_i) estop_seen_q <= 1'b0;
        else if (clear_fault_pulse_i) estop_seen_q <= 1'b0;
    end

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            fault_latched_o <= 8'h00;
            arm_rejected_pulse_o <= 1'b0;
        end else begin
            arm_rejected_pulse_o <= 1'b0;
            fault_latched_o <= fault_latched_o | missing_health |
                               (estop_seen_q ? 8'h80 : 8'h00);
            if (clear_fault_pulse_i && missing_health == 8'h00 && estop_n_i)
                fault_latched_o <= 8'h00;
            if (arm_pulse_i && (missing_health != 8'h00 || !estop_n_i))
                arm_rejected_pulse_o <= 1'b1;
        end
    end
endmodule
