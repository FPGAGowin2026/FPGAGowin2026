`timescale 1ns/1ps

// 前、左、右三路测距样本的统一有效性与超时监视器。
// 具体 TOF/超声波协议驱动只需提交毫米值和单周期 sample_valid；型号未定前
// 不在本模块猜测 I2C、UART 或脉宽接口。TIMEOUT_CYCLES 必须大于零。
module range_validity_monitor #(
    parameter integer TIMEOUT_CYCLES = 10000000,
    parameter [15:0] MIN_DISTANCE_MM = 16'd20,
    parameter [15:0] MAX_DISTANCE_MM = 16'd4000,
    // 安装误差以毫米为单位，实板标定后覆盖。
    parameter signed [15:0] FRONT_OFFSET_MM = 16'sd0,
    parameter signed [15:0] LEFT_OFFSET_MM  = 16'sd0,
    parameter signed [15:0] RIGHT_OFFSET_MM = 16'sd0
) (
    input  wire        clk_i,
    input  wire        rst_n_i,

    input  wire        front_sample_valid_i,
    input  wire [15:0] front_distance_mm_i,
    input  wire        left_sample_valid_i,
    input  wire [15:0] left_distance_mm_i,
    input  wire        right_sample_valid_i,
    input  wire [15:0] right_distance_mm_i,

    output reg  [15:0] front_distance_mm_o,
    output reg         front_valid_o,
    output reg         front_timeout_o,
    output reg         front_invalid_sample_o,
    output reg  [15:0] left_distance_mm_o,
    output reg         left_valid_o,
    output reg         left_timeout_o,
    output reg         left_invalid_sample_o,
    output reg  [15:0] right_distance_mm_o,
    output reg         right_valid_o,
    output reg         right_timeout_o,
    output reg         right_invalid_sample_o
);

    localparam integer TIMEOUT_LIMIT =
        (TIMEOUT_CYCLES > 0) ? (TIMEOUT_CYCLES - 1) : 0;

    reg [31:0] front_age_q;
    reg [31:0] left_age_q;
    reg [31:0] right_age_q;

    wire [15:0] calibrated_front;
    wire [15:0] calibrated_left;
    wire [15:0] calibrated_right;
    wire front_in_range;
    wire left_in_range;
    wire right_in_range;

    // 校准加法饱和到无符号毫米范围，避免负偏置回绕成极大距离。
    function [15:0] apply_offset;
        input [15:0] raw_distance;
        input [15:0] signed_offset;
        reg signed [17:0] adjusted_distance;
        begin
            adjusted_distance = $signed({2'b00, raw_distance}) +
                                $signed({{2{signed_offset[15]}}, signed_offset});
            if (adjusted_distance < 0)
                apply_offset = 16'd0;
            else if (adjusted_distance > 18'sd65535)
                apply_offset = 16'hffff;
            else
                apply_offset = adjusted_distance[15:0];
        end
    endfunction

    assign calibrated_front = apply_offset(front_distance_mm_i, FRONT_OFFSET_MM);
    assign calibrated_left  = apply_offset(left_distance_mm_i, LEFT_OFFSET_MM);
    assign calibrated_right = apply_offset(right_distance_mm_i, RIGHT_OFFSET_MM);

    assign front_in_range = calibrated_front >= MIN_DISTANCE_MM &&
                            calibrated_front <= MAX_DISTANCE_MM;
    assign left_in_range  = calibrated_left >= MIN_DISTANCE_MM &&
                            calibrated_left <= MAX_DISTANCE_MM;
    assign right_in_range = calibrated_right >= MIN_DISTANCE_MM &&
                            calibrated_right <= MAX_DISTANCE_MM;

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            front_distance_mm_o    <= 16'd0;
            front_valid_o          <= 1'b0;
            front_timeout_o        <= 1'b0;
            front_invalid_sample_o <= 1'b0;
            front_age_q            <= 32'd0;
        end else begin
            front_timeout_o        <= 1'b0;
            front_invalid_sample_o <= 1'b0;

            if (front_sample_valid_i) begin
                front_age_q <= 32'd0;
                if (front_in_range) begin
                    front_distance_mm_o <= calibrated_front;
                    front_valid_o       <= 1'b1;
                end else begin
                    front_valid_o          <= 1'b0;
                    front_invalid_sample_o <= 1'b1;
                end
            end else if (front_valid_o) begin
                if (front_age_q >= TIMEOUT_LIMIT) begin
                    front_valid_o   <= 1'b0;
                    front_timeout_o <= 1'b1;
                    front_age_q     <= 32'd0;
                end else begin
                    front_age_q <= front_age_q + 1'b1;
                end
            end
        end
    end

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            left_distance_mm_o    <= 16'd0;
            left_valid_o          <= 1'b0;
            left_timeout_o        <= 1'b0;
            left_invalid_sample_o <= 1'b0;
            left_age_q            <= 32'd0;
        end else begin
            left_timeout_o        <= 1'b0;
            left_invalid_sample_o <= 1'b0;

            if (left_sample_valid_i) begin
                left_age_q <= 32'd0;
                if (left_in_range) begin
                    left_distance_mm_o <= calibrated_left;
                    left_valid_o       <= 1'b1;
                end else begin
                    left_valid_o          <= 1'b0;
                    left_invalid_sample_o <= 1'b1;
                end
            end else if (left_valid_o) begin
                if (left_age_q >= TIMEOUT_LIMIT) begin
                    left_valid_o   <= 1'b0;
                    left_timeout_o <= 1'b1;
                    left_age_q     <= 32'd0;
                end else begin
                    left_age_q <= left_age_q + 1'b1;
                end
            end
        end
    end

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            right_distance_mm_o    <= 16'd0;
            right_valid_o          <= 1'b0;
            right_timeout_o        <= 1'b0;
            right_invalid_sample_o <= 1'b0;
            right_age_q            <= 32'd0;
        end else begin
            right_timeout_o        <= 1'b0;
            right_invalid_sample_o <= 1'b0;

            if (right_sample_valid_i) begin
                right_age_q <= 32'd0;
                if (right_in_range) begin
                    right_distance_mm_o <= calibrated_right;
                    right_valid_o       <= 1'b1;
                end else begin
                    right_valid_o          <= 1'b0;
                    right_invalid_sample_o <= 1'b1;
                end
            end else if (right_valid_o) begin
                if (right_age_q >= TIMEOUT_LIMIT) begin
                    right_valid_o   <= 1'b0;
                    right_timeout_o <= 1'b1;
                    right_age_q     <= 32'd0;
                end else begin
                    right_age_q <= right_age_q + 1'b1;
                end
            end
        end
    end

endmodule
