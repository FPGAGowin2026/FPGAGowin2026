`timescale 1ns/1ps

// 主/备毫米测距源与四路数字红外的保守融合；任一新鲜测距源即可工作，双源取较小值。
// IR 只有开关量语义，绝不伪装成毫米距离。
module choice3_range_fusion #(
    parameter integer STOP_MM = 220,
    parameter integer SLOW_MM = 420,
    parameter integer MAX_AGE_TICKS = 50
) (
    input  wire        clk_i,
    input  wire        rst_n_i,
    input  wire        sample_tick_i,
    input  wire [15:0] vl53_distance_mm_i,
    input  wire        vl53_valid_i,
    input  wire        vl53_sensor_ok_i,
    input  wire [15:0] ultrasonic_distance_mm_i,
    input  wire        ultrasonic_valid_i,
    input  wire        ultrasonic_timeout_i,
    // 约定 [0]=左前，[1]=右前，[2]=左侧，[3]=右侧；1 表示障碍。
    input  wire [3:0]  ir_active_i,
    output reg  [15:0] fused_front_mm_o,
    output wire        fused_front_valid_o,
    output wire        front_blocked_o,
    output wire        front_slow_o,
    output wire        left_blocked_o,
    output wire        right_blocked_o,
    output wire        vl53_fresh_o,
    output wire        ultrasonic_fresh_o
);
    localparam integer AGE_W = (MAX_AGE_TICKS < 2) ? 1 : $clog2(MAX_AGE_TICKS + 1);
    reg [AGE_W-1:0] vl53_age_q;
    reg [AGE_W-1:0] ultrasonic_age_q;
    reg [15:0] vl53_mm_q;
    reg [15:0] ultrasonic_mm_q;

    assign vl53_fresh_o = vl53_sensor_ok_i && (vl53_age_q < MAX_AGE_TICKS);
    assign ultrasonic_fresh_o = !ultrasonic_timeout_i &&
                                (ultrasonic_age_q < MAX_AGE_TICKS);
    assign fused_front_valid_o = vl53_fresh_o || ultrasonic_fresh_o;
    assign left_blocked_o = ir_active_i[0] || ir_active_i[2];
    assign right_blocked_o = ir_active_i[1] || ir_active_i[3];
    assign front_blocked_o = ir_active_i[0] || ir_active_i[1] ||
                             (fused_front_valid_o && fused_front_mm_o <= STOP_MM);
    assign front_slow_o = front_blocked_o ||
                          (fused_front_valid_o && fused_front_mm_o <= SLOW_MM);

    always @* begin
        if (vl53_fresh_o && ultrasonic_fresh_o && vl53_mm_q <= ultrasonic_mm_q)
            fused_front_mm_o = vl53_mm_q;
        else if (vl53_fresh_o && ultrasonic_fresh_o)
            fused_front_mm_o = ultrasonic_mm_q;
        else if (vl53_fresh_o)
            fused_front_mm_o = vl53_mm_q;
        else if (ultrasonic_fresh_o)
            fused_front_mm_o = ultrasonic_mm_q;
        else
            fused_front_mm_o = 16'hffff;
    end

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            vl53_age_q <= MAX_AGE_TICKS[AGE_W-1:0];
            ultrasonic_age_q <= MAX_AGE_TICKS[AGE_W-1:0];
            vl53_mm_q <= 16'hffff;
            ultrasonic_mm_q <= 16'hffff;
        end else begin
            if (vl53_valid_i) begin
                vl53_mm_q <= vl53_distance_mm_i;
                vl53_age_q <= {AGE_W{1'b0}};
            end else if (sample_tick_i && vl53_age_q < MAX_AGE_TICKS)
                vl53_age_q <= vl53_age_q + 1'b1;

            if (ultrasonic_valid_i) begin
                ultrasonic_mm_q <= ultrasonic_distance_mm_i;
                ultrasonic_age_q <= {AGE_W{1'b0}};
            end else if (sample_tick_i && ultrasonic_age_q < MAX_AGE_TICKS)
                ultrasonic_age_q <= ultrasonic_age_q + 1'b1;

            if (ultrasonic_timeout_i)
                ultrasonic_age_q <= MAX_AGE_TICKS[AGE_W-1:0];
        end
    end
endmodule

