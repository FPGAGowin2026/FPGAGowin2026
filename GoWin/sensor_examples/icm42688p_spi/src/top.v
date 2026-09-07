`timescale 1ns/1ps

module sensor_example_top #(
    parameter integer CLK_DIV = 8,
    parameter integer STARTUP_DELAY_CYCLES = 270000,
    parameter integer SENSOR_READY_DELAY_CYCLES = 1215000,
    parameter integer SAMPLE_INTERVAL_CYCLES = 270000
) (
    input  wire               clk,
    input  wire               reset_n,
    input  wire               imu_miso,
    output wire               imu_cs_n,
    output wire               imu_sclk,
    output wire               imu_mosi,
    output wire               sensor_ok,
    output wire               initialized,
    input  wire        [2:0]  axis_select,
    output reg  signed [15:0] axis_data,
    output wire               sample_valid
);

    wire signed [15:0] accel_x;
    wire signed [15:0] accel_y;
    wire signed [15:0] accel_z;
    wire signed [15:0] gyro_x;
    wire signed [15:0] gyro_y;
    wire signed [15:0] gyro_z;

    icm42688p_spi #(
        .CLK_DIV(CLK_DIV),
        .STARTUP_DELAY_CYCLES(STARTUP_DELAY_CYCLES),
        .SENSOR_READY_DELAY_CYCLES(SENSOR_READY_DELAY_CYCLES),
        .SAMPLE_INTERVAL_CYCLES(SAMPLE_INTERVAL_CYCLES)
    ) u_sensor (
        .clk(clk),
        .reset_n(reset_n),
        .spi_miso(imu_miso),
        .spi_cs_n(imu_cs_n),
        .spi_sclk(imu_sclk),
        .spi_mosi(imu_mosi),
        .sensor_ok(sensor_ok),
        .initialized(initialized),
        .accel_x(accel_x),
        .accel_y(accel_y),
        .accel_z(accel_z),
        .gyro_x(gyro_x),
        .gyro_y(gyro_y),
        .gyro_z(gyro_z),
        .sample_valid(sample_valid)
    );

    // MG121 封装无法同时引出六轴 96 位并行总线，使用三位选择器
    // 复用为一条 16 位观测口；核心内部仍在同一 sample_valid 更新六轴。
    always @(*) begin
        case (axis_select)
            3'd0: axis_data = accel_x;
            3'd1: axis_data = accel_y;
            3'd2: axis_data = accel_z;
            3'd3: axis_data = gyro_x;
            3'd4: axis_data = gyro_y;
            3'd5: axis_data = gyro_z;
            default: axis_data = 16'sd0;
        endcase
    end

endmodule
