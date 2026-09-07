`timescale 1ns/1ps

// ICM-42688-P 运行期样本哨兵过滤。
// SPI 失联/短路常表现为连续全 1 或全 0；这些样本不得刷新健康计时器。
// 这不是运行期 WHO_AM_I 重读，实车仍需做断线与总线故障注入验证。
module choice3_imu_sample_guard (
    input  wire               sample_valid_i,
    input  wire signed [15:0] accel_x_i,
    input  wire signed [15:0] accel_y_i,
    input  wire signed [15:0] accel_z_i,
    input  wire signed [15:0] gyro_x_i,
    input  wire signed [15:0] gyro_y_i,
    input  wire signed [15:0] gyro_z_i,
    output wire               good_sample_o,
    output wire               sentinel_rejected_o
);
    wire all_zero=(accel_x_i==16'h0000)&&(accel_y_i==16'h0000)&&
                  (accel_z_i==16'h0000)&&(gyro_x_i==16'h0000)&&
                  (gyro_y_i==16'h0000)&&(gyro_z_i==16'h0000);
    wire all_ones=(accel_x_i==16'hffff)&&(accel_y_i==16'hffff)&&
                  (accel_z_i==16'hffff)&&(gyro_x_i==16'hffff)&&
                  (gyro_y_i==16'hffff)&&(gyro_z_i==16'hffff);
    assign sentinel_rejected_o=sample_valid_i&&(all_zero||all_ones);
    assign good_sample_o=sample_valid_i&&!all_zero&&!all_ones;
endmodule
