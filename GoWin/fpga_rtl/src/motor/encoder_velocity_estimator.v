`timescale 1ns/1ps

// 在固定 sample_tick_i 周期内用编码器增量估计轮速。
// MM_PER_TICK_Q16 和 SAMPLE_HZ 是实车必须重新标定的物理参数。
module encoder_velocity_estimator #(
    parameter signed [31:0] MM_PER_TICK_Q16 = 32'sd6554,
    parameter integer SAMPLE_HZ = 100
) (
    input  wire               clk_i,
    input  wire               rst_n_i,
    input  wire               sample_tick_i,
    input  wire signed [31:0] position_ticks_i,
    output reg signed [31:0]  velocity_mm_s_o,
    output reg                velocity_valid_o
);
    // 位置与差分均按 32 位二补码自然回绕；正常采样间隔内增量远小于半量程。
    reg signed [31:0] previous_ticks_q;
    reg signed [31:0] delta_q;
    reg signed [63:0] scaled_hz_q;
    reg stage1_valid_q,stage2_valid_q;
    // 两个常量比例在 elaboration 合并，避免每轮级联三颗 MULT27X36。
    // 实车标定必须保证合并后的 Q16 比例落在 signed 32 位范围内。
    localparam signed [31:0] VELOCITY_SCALE_Q16 = MM_PER_TICK_Q16 * SAMPLE_HZ;
    localparam signed [63:0] MAX_VELOCITY_Q16 = 64'sh0000_7fff_ffff_0000;
    localparam signed [63:0] MIN_VELOCITY_Q16 = 64'shffff_8000_0000_0000;

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            previous_ticks_q <= 32'sd0;
            velocity_mm_s_o <= 32'sd0;
            velocity_valid_o <= 1'b0;
            delta_q <= 32'sd0;
            scaled_hz_q <= 64'sd0;
            stage1_valid_q <= 1'b0;
            stage2_valid_q <= 1'b0;
        end else begin
            // 100Hz 估计拆为三拍：差分、合并比例乘法、饱和提交。
            // 约 60ns 的固定延迟远小于 10ms 采样周期。
            stage1_valid_q <= sample_tick_i;
            stage2_valid_q <= stage1_valid_q;
            velocity_valid_o <= stage2_valid_q;
            if (sample_tick_i) begin
                delta_q <= position_ticks_i - previous_ticks_q;
                previous_ticks_q <= position_ticks_i;
            end
            if(stage1_valid_q)
                scaled_hz_q <= delta_q * VELOCITY_SCALE_Q16;
            if(stage2_valid_q) begin
                // 正常范围显式取 Q16 的 [47:16]；极端计数跳变饱和，避免
                // 隐式截断翻转符号并消除综合器的 48->32 位告警。
                if (scaled_hz_q > MAX_VELOCITY_Q16)
                    velocity_mm_s_o <= 32'sh7fff_ffff;
                else if (scaled_hz_q < MIN_VELOCITY_Q16)
                    velocity_mm_s_o <= -32'sh8000_0000;
                else
                    velocity_mm_s_o <= $signed(scaled_hz_q[47:16]);
            end
        end
    end
endmodule
