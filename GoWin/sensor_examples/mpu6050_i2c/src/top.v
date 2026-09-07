`timescale 1ns/1ps

module sensor_example_top #(
    parameter integer I2C_CLK_DIV = 32,
    parameter integer I2C_STRETCH_TIMEOUT_TICKS = 8192,
    parameter integer STARTUP_DELAY_CYCLES = 270000,
    parameter integer WAKE_STABILIZE_CYCLES = 810000,
    parameter integer SAMPLE_INTERVAL_CYCLES = 27000,
    parameter [6:0] DEVICE_ADDR = 7'h68
) (
    input wire clk, input wire reset_n,
    inout wire imu_scl, inout wire imu_sda,
    input wire [2:0] sample_select,
    output wire sensor_ok, output wire initialized,
    output wire identity_error, output wire bus_error,
    output reg [15:0] sample_word,
    output wire sample_valid
);
    wire signed [15:0] accel_x, accel_y, accel_z;
    wire signed [15:0] temperature_raw;
    wire signed [15:0] gyro_x, gyro_y, gyro_z;

    always @* begin
        case (sample_select)
            3'd0: sample_word = accel_x;
            3'd1: sample_word = accel_y;
            3'd2: sample_word = accel_z;
            3'd3: sample_word = temperature_raw;
            3'd4: sample_word = gyro_x;
            3'd5: sample_word = gyro_y;
            default: sample_word = gyro_z;
        endcase
    end
    mpu6050_i2c #(
        .I2C_CLK_DIV(I2C_CLK_DIV),
        .I2C_STRETCH_TIMEOUT_TICKS(I2C_STRETCH_TIMEOUT_TICKS),
        .STARTUP_DELAY_CYCLES(STARTUP_DELAY_CYCLES),
        .WAKE_STABILIZE_CYCLES(WAKE_STABILIZE_CYCLES),
        .SAMPLE_INTERVAL_CYCLES(SAMPLE_INTERVAL_CYCLES),
        .DEVICE_ADDR(DEVICE_ADDR)
    ) u_sensor (
        .clk(clk), .reset_n(reset_n), .i2c_scl(imu_scl), .i2c_sda(imu_sda),
        .sensor_ok(sensor_ok), .initialized(initialized),
        .identity_error(identity_error), .bus_error(bus_error),
        .accel_x(accel_x), .accel_y(accel_y), .accel_z(accel_z),
        .temperature_raw(temperature_raw),
        .gyro_x(gyro_x), .gyro_y(gyro_y), .gyro_z(gyro_z),
        .sample_valid(sample_valid)
    );
endmodule
