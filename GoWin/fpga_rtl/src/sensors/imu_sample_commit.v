`timescale 1ns/1ps

// 与具体 SPI/I2C 型号无关的 IMU 字节流提交前端。
// sample_start_i 开始一帧，随后握手接收 12 个高字节在前的有符号 16 位量：
// accel_x/y/z、gyro_x/y/z。只有第 12 字节到达后才原子更新 sample_data_o。
module imu_sample_commit #(
    // 偏置以传感器原始 LSB 为单位；实板静态标定后覆盖，减法按 16 位回绕。
    parameter signed [15:0] ACCEL_X_BIAS = 16'sd0,
    parameter signed [15:0] ACCEL_Y_BIAS = 16'sd0,
    parameter signed [15:0] ACCEL_Z_BIAS = 16'sd0,
    parameter signed [15:0] GYRO_X_BIAS  = 16'sd0,
    parameter signed [15:0] GYRO_Y_BIAS  = 16'sd0,
    parameter signed [15:0] GYRO_Z_BIAS  = 16'sd0
) (
    input  wire        clk_i,
    input  wire        rst_n_i,
    input  wire        sample_start_i,
    input  wire        byte_valid_i,
    input  wire [7:0]  byte_data_i,
    output wire        byte_ready_o,

    output reg  [95:0] sample_data_o,
    output reg         sample_valid_o,
    // 未完成时再次开始一帧会丢弃旧字节，并给出单周期错误脉冲。
    output reg         sample_error_o
);

    reg        collecting_q;
    reg [3:0]  byte_count_q;
    reg [95:0] sample_shift_q;

    wire [95:0] completed_sample;
    assign completed_sample = {sample_shift_q[87:0], byte_data_i};
    assign byte_ready_o = collecting_q;

    function [15:0] remove_bias;
        input [15:0] raw_value;
        input [15:0] bias_value;
        begin
            remove_bias = $signed(raw_value) - $signed(bias_value);
        end
    endfunction

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            collecting_q   <= 1'b0;
            byte_count_q   <= 4'd0;
            sample_shift_q <= 96'd0;
            sample_data_o  <= 96'd0;
            sample_valid_o <= 1'b0;
            sample_error_o <= 1'b0;
        end else begin
            sample_valid_o <= 1'b0;
            sample_error_o <= 1'b0;

            if (sample_start_i) begin
                if (collecting_q && byte_count_q != 4'd0)
                    sample_error_o <= 1'b1;
                collecting_q   <= 1'b1;
                byte_count_q   <= 4'd0;
                sample_shift_q <= 96'd0;
            end else if (byte_valid_i && byte_ready_o) begin
                sample_shift_q <= completed_sample;

                if (byte_count_q == 4'd11) begin
                    sample_data_o <= {
                        remove_bias(completed_sample[95:80], ACCEL_X_BIAS),
                        remove_bias(completed_sample[79:64], ACCEL_Y_BIAS),
                        remove_bias(completed_sample[63:48], ACCEL_Z_BIAS),
                        remove_bias(completed_sample[47:32], GYRO_X_BIAS),
                        remove_bias(completed_sample[31:16], GYRO_Y_BIAS),
                        remove_bias(completed_sample[15:0],  GYRO_Z_BIAS)
                    };
                    sample_valid_o <= 1'b1;
                    collecting_q   <= 1'b0;
                    byte_count_q   <= 4'd0;
                end else begin
                    byte_count_q <= byte_count_q + 1'b1;
                end
            end
        end
    end

endmodule
