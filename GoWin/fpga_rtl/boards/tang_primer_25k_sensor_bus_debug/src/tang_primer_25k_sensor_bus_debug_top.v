`timescale 1ns/1ps

// Tang Primer 25K 传感器总线“无设备”实板诊断顶层。
// 四组 I2C 各自连接一对带上拉的 Dock GPIO；未接传感器时必须真实收到地址 NACK。
// 八个 SPI 核的 MISO 固定为 1，模拟总线悬空/全 1 返回；其 SCLK/MOSI 留在芯片内部。
module tang_primer_25k_sensor_bus_debug_top #(
    parameter integer CLK_FREQ_HZ = 50000000,
    parameter integer BAUD_RATE = 115200,
    parameter integer POR_CYCLES = 65536,
    parameter integer I2C_CLK_DIV = 32,
    parameter integer I2C_STARTUP_CYCLES = 4096,
    parameter integer I2C_RESET_DELAY_CYCLES = 4096,
    parameter integer SPI_CLK_DIV = 4,
    parameter integer SPI_STARTUP_CYCLES = 4096,
    parameter integer SPI_RESET_DELAY_CYCLES = 4096,
    parameter integer SPI_RETRY_LIMIT = 2
) (
    input  wire clk_i,
    input  wire reset_i,
    input  wire uart_rx_i,
    output wire uart_tx_o,
    inout  wire ina219_scl_io,
    inout  wire ina219_sda_io,
    inout  wire mpu6050_scl_io,
    inout  wire mpu6050_sda_io,
    inout  wire vl53l0x_scl_io,
    inout  wire vl53l0x_sda_io,
    inout  wire vl53l1x_scl_io,
    inout  wire vl53l1x_sda_io
);
    localparam integer SAFE_POR_CYCLES = (POR_CYCLES < 2) ? 2 : POR_CYCLES;

    reg [31:0] por_count;
    reg por_done;
    reg reset_meta;
    reg reset_sync;
    wire rst_n = por_done && !reset_sync;

    initial begin
        por_count = 32'd0;
        por_done = 1'b0;
        reset_meta = 1'b0;
        reset_sync = 1'b0;
    end

    always @(posedge clk_i) begin
        reset_meta <= reset_i;
        reset_sync <= reset_meta;
        if (reset_sync) begin
            por_count <= 32'd0;
            por_done <= 1'b0;
        end else if (!por_done) begin
            if (por_count >= SAFE_POR_CYCLES - 1) begin
                por_count <= 32'd0;
                por_done <= 1'b1;
            end else begin
                por_count <= por_count + 1'b1;
            end
        end
    end

    // I2C 错误位顺序：0 INA219、1 MPU6050、2 VL53L0X、3 VL53L1X。
    wire ina219_bus_error;
    wire mpu6050_bus_error;
    wire vl53l0x_bus_error;
    wire vl53l1x_bus_error;
    wire [3:0] i2c_bus_error_raw = {
        vl53l1x_bus_error, vl53l0x_bus_error,
        mpu6050_bus_error, ina219_bus_error
    };
    reg [3:0] i2c_error_bitmap;

    ina219_i2c #(
        .I2C_CLK_DIV(I2C_CLK_DIV),
        .STARTUP_DELAY_CYCLES(I2C_STARTUP_CYCLES),
        .SAMPLE_INTERVAL_CYCLES(I2C_STARTUP_CYCLES)
    ) u_ina219 (
        .clk(clk_i), .reset_n(rst_n),
        .i2c_scl(ina219_scl_io), .i2c_sda(ina219_sda_io),
        .sensor_ok(), .initialized(), .bus_error(ina219_bus_error),
        .shunt_voltage_raw(), .bus_voltage_raw(), .bus_voltage_mv(),
        .sample_valid()
    );

    mpu6050_i2c #(
        .I2C_CLK_DIV(I2C_CLK_DIV),
        .I2C_STRETCH_TIMEOUT_TICKS(256),
        .STARTUP_DELAY_CYCLES(I2C_STARTUP_CYCLES),
        .SAMPLE_INTERVAL_CYCLES(I2C_STARTUP_CYCLES)
    ) u_mpu6050 (
        .clk(clk_i), .reset_n(rst_n),
        .i2c_scl(mpu6050_scl_io), .i2c_sda(mpu6050_sda_io),
        .sensor_ok(), .initialized(), .identity_error(),
        .bus_error(mpu6050_bus_error),
        .accel_x(), .accel_y(), .accel_z(), .temperature_raw(),
        .gyro_x(), .gyro_y(), .gyro_z(), .sample_valid()
    );

    vl53l0x_i2c #(
        .I2C_CLK_DIV(I2C_CLK_DIV),
        .I2C_STRETCH_TIMEOUT_TICKS(256),
        .STARTUP_DELAY_CYCLES(I2C_STARTUP_CYCLES),
        .SAMPLE_INTERVAL_CYCLES(I2C_STARTUP_CYCLES),
        .POLL_RETRY_LIMIT(2), .IOVDD_2V8(1'b1)
    ) u_vl53l0x (
        .clk(clk_i), .reset_n(rst_n),
        .i2c_scl(vl53l0x_scl_io), .i2c_sda(vl53l0x_sda_io),
        .sensor_ok(), .initialized(), .identity_error(),
        .bus_error(vl53l0x_bus_error), .timeout_error(),
        .model_id(), .interrupt_status(), .distance_mm(), .distance_valid()
    );

    vl53l1x_i2c #(
        .I2C_CLK_DIV(I2C_CLK_DIV),
        .STARTUP_DELAY_CYCLES(I2C_STARTUP_CYCLES),
        .RESET_DELAY_CYCLES(I2C_RESET_DELAY_CYCLES),
        .BOOT_RETRY_LIMIT(2), .CALIBRATION_READY_RETRY_LIMIT(2),
        .IOVDD_2V8(1'b1),
        .SAMPLE_INTERVAL_CYCLES(I2C_STARTUP_CYCLES)
    ) u_vl53l1x (
        .clk(clk_i), .reset_n(rst_n),
        .i2c_scl(vl53l1x_scl_io), .i2c_sda(vl53l1x_sda_io),
        .initialized(), .bus_error(vl53l1x_bus_error),
        .range_status(), .distance_mm(), .distance_valid()
    );

    // SPI 位顺序：0 ICM20602、1 ICM42688P、2 IMU660RA、3 IMU660RX、
    // 4 IMU963RA、5 LIS3MDL、6 MENC15A、7 MT6816。
    wire [7:0] spi_cs_n;
    wire [7:0] spi_sclk;
    wire [7:0] spi_mosi;
    wire [7:0] spi_sensor_ok;
    wire [7:0] spi_protocol_error_raw;
    reg [7:0] spi_low_seen;
    reg [7:0] spi_transaction_bitmap;
    reg [7:0] spi_protocol_error_seen;
    wire [7:0] spi_reject_bitmap = {
        spi_protocol_error_seen[7],
        spi_protocol_error_seen[6],
        spi_transaction_bitmap[5] && !spi_sensor_ok[5],
        spi_protocol_error_seen[4],
        spi_protocol_error_seen[3],
        spi_protocol_error_seen[2],
        spi_transaction_bitmap[1] && !spi_sensor_ok[1],
        spi_protocol_error_seen[0]
    };

    // MISO=1 是明确的故障注入，不代表接入真实传感器。
    icm20602_spi #(
        .CLK_DIV(SPI_CLK_DIV), .STARTUP_DELAY_CYCLES(SPI_STARTUP_CYCLES),
        .RESET_DELAY_CYCLES(SPI_RESET_DELAY_CYCLES),
        .SAMPLE_INTERVAL_CYCLES(SPI_STARTUP_CYCLES),
        .ID_RETRY_LIMIT(SPI_RETRY_LIMIT)
    ) u_icm20602 (
        .clk(clk_i), .reset_n(rst_n), .spi_miso(1'b1),
        .spi_cs_n(spi_cs_n[0]), .spi_sclk(spi_sclk[0]), .spi_mosi(spi_mosi[0]),
        .initialized(), .sensor_ok(spi_sensor_ok[0]),
        .accel_x(), .accel_y(), .accel_z(), .gyro_x(), .gyro_y(), .gyro_z(),
        .sample_valid(), .protocol_error(spi_protocol_error_raw[0])
    );

    icm42688p_spi #(
        .CLK_DIV(SPI_CLK_DIV), .STARTUP_DELAY_CYCLES(SPI_STARTUP_CYCLES),
        .SENSOR_READY_DELAY_CYCLES(SPI_RESET_DELAY_CYCLES),
        .SAMPLE_INTERVAL_CYCLES(SPI_STARTUP_CYCLES)
    ) u_icm42688p (
        .clk(clk_i), .reset_n(rst_n), .spi_miso(1'b1),
        .spi_cs_n(spi_cs_n[1]), .spi_sclk(spi_sclk[1]), .spi_mosi(spi_mosi[1]),
        .sensor_ok(spi_sensor_ok[1]), .initialized(),
        .accel_x(), .accel_y(), .accel_z(), .gyro_x(), .gyro_y(), .gyro_z(),
        .sample_valid()
    );
    assign spi_protocol_error_raw[1] = 1'b0;

    imu660ra_spi #(
        .CLK_DIV(SPI_CLK_DIV), .STARTUP_DELAY_CYCLES(SPI_STARTUP_CYCLES),
        .SAMPLE_INTERVAL_CYCLES(SPI_STARTUP_CYCLES),
        .ID_RETRY_LIMIT(SPI_RETRY_LIMIT)
    ) u_imu660ra (
        .clk(clk_i), .reset_n(rst_n), .spi_miso(1'b1),
        .spi_cs_n(spi_cs_n[2]), .spi_sclk(spi_sclk[2]), .spi_mosi(spi_mosi[2]),
        .initialized(), .sensor_ok(spi_sensor_ok[2]), .configuration_required(),
        .accel_x(), .accel_y(), .accel_z(), .gyro_x(), .gyro_y(), .gyro_z(),
        .sample_valid(), .protocol_error(spi_protocol_error_raw[2])
    );

    imu660rx_spi #(
        .CLK_DIV(SPI_CLK_DIV), .STARTUP_DELAY_CYCLES(SPI_STARTUP_CYCLES),
        .SAMPLE_INTERVAL_CYCLES(SPI_STARTUP_CYCLES),
        .ID_RETRY_LIMIT(SPI_RETRY_LIMIT)
    ) u_imu660rx (
        .clk(clk_i), .reset_n(rst_n), .spi_miso(1'b1),
        .spi_cs_n(spi_cs_n[3]), .spi_sclk(spi_sclk[3]), .spi_mosi(spi_mosi[3]),
        .initialized(), .sensor_ok(spi_sensor_ok[3]), .detected_variant(),
        .configuration_required(),
        .accel_x(), .accel_y(), .accel_z(), .gyro_x(), .gyro_y(), .gyro_z(),
        .sample_valid(), .protocol_error(spi_protocol_error_raw[3])
    );

    imu963ra_spi #(
        .CLK_DIV(SPI_CLK_DIV), .STARTUP_DELAY_CYCLES(SPI_STARTUP_CYCLES),
        .RESET_DELAY_CYCLES(SPI_RESET_DELAY_CYCLES),
        .SAMPLE_INTERVAL_CYCLES(SPI_STARTUP_CYCLES),
        .ID_RETRY_LIMIT(SPI_RETRY_LIMIT)
    ) u_imu963ra (
        .clk(clk_i), .reset_n(rst_n), .spi_miso(1'b1),
        .spi_cs_n(spi_cs_n[4]), .spi_sclk(spi_sclk[4]), .spi_mosi(spi_mosi[4]),
        .initialized(), .sensor_ok(spi_sensor_ok[4]),
        .mag_configuration_required(),
        .accel_x(), .accel_y(), .accel_z(), .gyro_x(), .gyro_y(), .gyro_z(),
        .sample_valid(), .protocol_error(spi_protocol_error_raw[4])
    );

    lis3mdl_spi #(
        .CLK_DIV(SPI_CLK_DIV), .STARTUP_DELAY_CYCLES(SPI_STARTUP_CYCLES),
        .SAMPLE_INTERVAL_CYCLES(SPI_STARTUP_CYCLES)
    ) u_lis3mdl (
        .clk(clk_i), .reset_n(rst_n), .spi_miso(1'b1),
        .spi_cs_n(spi_cs_n[5]), .spi_sclk(spi_sclk[5]), .spi_mosi(spi_mosi[5]),
        .sensor_ok(spi_sensor_ok[5]), .initialized(),
        .mag_x(), .mag_y(), .mag_z(), .sample_valid()
    );
    assign spi_protocol_error_raw[5] = 1'b0;

    menc15a_spi #(
        .CLK_DIV(SPI_CLK_DIV), .STARTUP_DELAY_CYCLES(SPI_STARTUP_CYCLES),
        .SAMPLE_INTERVAL_CYCLES(SPI_STARTUP_CYCLES)
    ) u_menc15a (
        .clk(clk_i), .reset_n(rst_n), .spi_miso(1'b1),
        .spi_cs_n(spi_cs_n[6]), .spi_sclk(spi_sclk[6]), .spi_mosi(spi_mosi[6]),
        .initialized(), .sensor_ok(spi_sensor_ok[6]),
        .position_raw(), .speed_raw(), .position_status(), .speed_status(),
        .sample_valid(), .protocol_error(spi_protocol_error_raw[6])
    );

    mt6816_spi #(
        .CLK_DIV(SPI_CLK_DIV), .STARTUP_DELAY_CYCLES(SPI_STARTUP_CYCLES),
        .SAMPLE_INTERVAL_CYCLES(SPI_STARTUP_CYCLES)
    ) u_mt6816 (
        .clk(clk_i), .reset_n(rst_n), .spi_miso(1'b1),
        .spi_cs_n(spi_cs_n[7]), .spi_sclk(spi_sclk[7]), .spi_mosi(spi_mosi[7]),
        .initialized(), .sensor_ok(spi_sensor_ok[7]),
        .angle_raw(), .no_magnet(), .over_speed(), .sample_valid(),
        .protocol_error(spi_protocol_error_raw[7])
    );

    always @(posedge clk_i or negedge rst_n) begin
        if (!rst_n) begin
            i2c_error_bitmap <= 4'd0;
            spi_low_seen <= 8'd0;
            spi_transaction_bitmap <= 8'd0;
            spi_protocol_error_seen <= 8'd0;
        end else begin
            i2c_error_bitmap <= i2c_error_bitmap | i2c_bus_error_raw;
            spi_low_seen <= spi_low_seen | ~spi_cs_n;
            // 先见到 CS 低、随后见到 CS 高，才算一次完整事务。
            spi_transaction_bitmap <= spi_transaction_bitmap |
                                      (spi_low_seen & spi_cs_n);
            spi_protocol_error_seen <= spi_protocol_error_seen |
                                       spi_protocol_error_raw;
        end
    end

    sensor_bus_debug_console #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(BAUD_RATE)
    ) u_console (
        .clk(clk_i), .rst_n(rst_n),
        .uart_rx_i(uart_rx_i), .uart_tx_o(uart_tx_o),
        .i2c_error_bitmap_i(i2c_error_bitmap),
        .spi_transaction_bitmap_i(spi_transaction_bitmap),
        .spi_reject_bitmap_i(spi_reject_bitmap)
    );
endmodule
