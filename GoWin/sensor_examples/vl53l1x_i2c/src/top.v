`timescale 1ns/1ps

module sensor_example_top #(
    parameter integer I2C_CLK_DIV = 32,
    parameter integer STARTUP_DELAY_CYCLES = 270000,
    parameter integer RESET_DELAY_CYCLES = 27000,
    parameter integer BOOT_RETRY_LIMIT = 100,
    parameter integer CALIBRATION_READY_RETRY_LIMIT = 255,
    parameter IOVDD_2V8 = 1'b0,
    parameter integer SAMPLE_INTERVAL_CYCLES = 270000
) (
    input  wire        clk,
    input  wire        reset_n,
    inout  wire        tof_scl,
    inout  wire        tof_sda,
    output wire        initialized,
    output wire        bus_error,
    output wire [4:0]  range_status,
    output wire [15:0] distance_mm,
    output wire        distance_valid
);

    vl53l1x_i2c #(
        .I2C_CLK_DIV(I2C_CLK_DIV),
        .STARTUP_DELAY_CYCLES(STARTUP_DELAY_CYCLES),
        .RESET_DELAY_CYCLES(RESET_DELAY_CYCLES),
        .BOOT_RETRY_LIMIT(BOOT_RETRY_LIMIT),
        .CALIBRATION_READY_RETRY_LIMIT(CALIBRATION_READY_RETRY_LIMIT),
        .IOVDD_2V8(IOVDD_2V8),
        .SAMPLE_INTERVAL_CYCLES(SAMPLE_INTERVAL_CYCLES)
    ) u_sensor (
        .clk(clk), .reset_n(reset_n),
        .i2c_scl(tof_scl), .i2c_sda(tof_sda),
        .initialized(initialized), .bus_error(bus_error),
        .range_status(range_status),
        .distance_mm(distance_mm), .distance_valid(distance_valid)
    );
endmodule
