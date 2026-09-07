`timescale 1ns/1ps

// MPU-6050 基础采集例程。
// 上电后核对 WHO_AM_I，配置采样率/低通/量程/数据就绪中断，随后以一次
// 14 字节突发读取原子发布三轴加速度、温度和三轴角速度。
module mpu6050_i2c #(
    parameter integer I2C_CLK_DIV = 32,
    parameter integer I2C_STRETCH_TIMEOUT_TICKS = 8192,
    parameter integer STARTUP_DELAY_CYCLES = 270000,
    // 27 MHz 默认约 30 ms。退出睡眠并显式使能六轴后，先等待 PLL/模拟前端稳定。
    parameter integer WAKE_STABILIZE_CYCLES = 810000,
    parameter integer SAMPLE_INTERVAL_CYCLES = 27000,
    parameter [6:0] DEVICE_ADDR = 7'h68
) (
    input  wire clk,
    input  wire reset_n,
    inout  wire i2c_scl,
    inout  wire i2c_sda,
    output reg  sensor_ok,
    output reg  initialized,
    output reg  identity_error,
    output reg  bus_error,
    output reg signed [15:0] accel_x,
    output reg signed [15:0] accel_y,
    output reg signed [15:0] accel_z,
    output reg signed [15:0] temperature_raw,
    output reg signed [15:0] gyro_x,
    output reg signed [15:0] gyro_y,
    output reg signed [15:0] gyro_z,
    output reg  sample_valid
);
    localparam [7:0] REG_SMPLRT_DIV   = 8'h19;
    localparam [7:0] REG_CONFIG       = 8'h1A;
    localparam [7:0] REG_GYRO_CONFIG  = 8'h1B;
    localparam [7:0] REG_ACCEL_CONFIG = 8'h1C;
    localparam [7:0] REG_INT_ENABLE   = 8'h38;
    localparam [7:0] REG_INT_STATUS   = 8'h3A;
    localparam [7:0] REG_ACCEL_XOUT_H = 8'h3B;
    localparam [7:0] REG_PWR_MGMT_1   = 8'h6B;
    localparam [7:0] REG_PWR_MGMT_2   = 8'h6C;
    localparam [7:0] REG_WHO_AM_I     = 8'h75;
    localparam [7:0] EXPECTED_WHO_AM_I = 8'h68;

    localparam [3:0] ST_DELAY         = 4'd0;
    localparam [3:0] ST_WHO_START     = 4'd1;
    localparam [3:0] ST_WHO_WAIT      = 4'd2;
    localparam [3:0] ST_CONFIG_START  = 4'd3;
    localparam [3:0] ST_CONFIG_WAIT   = 4'd4;
    localparam [3:0] ST_SAMPLE_DELAY  = 4'd5;
    localparam [3:0] ST_STATUS_START  = 4'd6;
    localparam [3:0] ST_STATUS_WAIT   = 4'd7;
    localparam [3:0] ST_BURST_START   = 4'd8;
    localparam [3:0] ST_BURST_WAIT    = 4'd9;
    localparam [3:0] ST_PUBLISH       = 4'd10;
    localparam [3:0] ST_WAKE_DELAY    = 4'd11;

    localparam integer STARTUP_DELAY =
        (STARTUP_DELAY_CYCLES < 1) ? 1 : STARTUP_DELAY_CYCLES;
    localparam integer SAMPLE_INTERVAL =
        (SAMPLE_INTERVAL_CYCLES < 1) ? 1 : SAMPLE_INTERVAL_CYCLES;
    localparam integer WAKE_STABILIZE =
        (WAKE_STABILIZE_CYCLES < 1) ? 1 : WAKE_STABILIZE_CYCLES;

    reg [3:0] state;
    reg [2:0] config_step;
    reg [31:0] delay_count;
    reg txn_start;
    reg txn_read;
    reg [7:0] txn_reg;
    reg [31:0] txn_write_data;
    reg [2:0] txn_write_length;
    reg [4:0] txn_read_length;
    wire txn_busy;
    wire txn_done;
    wire txn_error;
    wire txn_rx_valid;
    wire [4:0] txn_rx_index;
    wire [7:0] txn_rx_data;
    reg [7:0] rx_bytes [0:13];
    integer i;

    function [7:0] config_register;
        input [2:0] index;
        begin
            case (index)
                3'd0: config_register = REG_PWR_MGMT_1;
                3'd1: config_register = REG_PWR_MGMT_2;
                3'd2: config_register = REG_SMPLRT_DIV;
                3'd3: config_register = REG_CONFIG;
                3'd4: config_register = REG_GYRO_CONFIG;
                3'd5: config_register = REG_ACCEL_CONFIG;
                default: config_register = REG_INT_ENABLE;
            endcase
        end
    endfunction

    function [7:0] config_value;
        input [2:0] index;
        begin
            case (index)
                3'd0: config_value = 8'h01; // PLL 参考 X 轴陀螺仪
                3'd1: config_value = 8'h00; // 六个轴均不置于待机
                3'd2: config_value = 8'h04; // 1 kHz/(1+4) = 200 Hz
                3'd3: config_value = 8'h03; // DLPF_CFG=3
                3'd4: config_value = 8'h00; // 陀螺仪 ±250 dps
                3'd5: config_value = 8'h00; // 加速度计 ±2 g
                default: config_value = 8'h01; // DATA_RDY_EN
            endcase
        end
    endfunction

    i2c_register8_master #(
        .CLK_DIV(I2C_CLK_DIV),
        .STRETCH_TIMEOUT_TICKS(I2C_STRETCH_TIMEOUT_TICKS)
    ) u_i2c (
        .clk(clk), .reset_n(reset_n), .start(txn_start),
        .device_addr(DEVICE_ADDR), .reg_addr(txn_reg), .is_read(txn_read),
        .write_data(txn_write_data), .write_length(txn_write_length),
        .read_length(txn_read_length), .busy(txn_busy), .done(txn_done),
        .error(txn_error), .rx_valid(txn_rx_valid), .rx_index(txn_rx_index),
        .rx_data(txn_rx_data), .i2c_scl(i2c_scl), .i2c_sda(i2c_sda)
    );

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= ST_DELAY;
            config_step <= 0;
            delay_count <= 0;
            txn_start <= 0;
            txn_read <= 0;
            txn_reg <= 0;
            txn_write_data <= 0;
            txn_write_length <= 0;
            txn_read_length <= 0;
            sensor_ok <= 0;
            initialized <= 0;
            identity_error <= 0;
            bus_error <= 0;
            accel_x <= 0; accel_y <= 0; accel_z <= 0;
            temperature_raw <= 0;
            gyro_x <= 0; gyro_y <= 0; gyro_z <= 0;
            sample_valid <= 0;
            for (i=0; i<14; i=i+1) rx_bytes[i] <= 0;
        end else begin
            txn_start <= 1'b0;
            sample_valid <= 1'b0;
            if (txn_rx_valid && txn_rx_index < 14)
                rx_bytes[txn_rx_index] <= txn_rx_data;
            case (state)
                ST_DELAY: begin
                    if (delay_count >= STARTUP_DELAY-1) begin
                        delay_count <= 0;
                        state <= ST_WHO_START;
                    end else delay_count <= delay_count + 1'b1;
                end
                ST_WHO_START: if (!txn_busy) begin
                    txn_read <= 1'b1;
                    txn_reg <= REG_WHO_AM_I;
                    txn_write_length <= 0;
                    txn_read_length <= 1;
                    txn_start <= 1'b1;
                    state <= ST_WHO_WAIT;
                end
                ST_WHO_WAIT: if (txn_done) begin
                    if (txn_error) begin
                        bus_error <= 1'b1;
                        sensor_ok <= 1'b0;
                        initialized <= 1'b0;
                        delay_count <= 0;
                        state <= ST_DELAY;
                    end else if (rx_bytes[0] != EXPECTED_WHO_AM_I) begin
                        identity_error <= 1'b1;
                        sensor_ok <= 1'b0;
                        initialized <= 1'b0;
                        delay_count <= 0;
                        state <= ST_DELAY;
                    end else begin
                        identity_error <= 1'b0;
                        sensor_ok <= 1'b1;
                        config_step <= 0;
                        state <= ST_CONFIG_START;
                    end
                end
                ST_CONFIG_START: if (!txn_busy) begin
                    txn_read <= 1'b0;
                    txn_reg <= config_register(config_step);
                    txn_write_data <= {24'd0, config_value(config_step)};
                    txn_write_length <= 1;
                    txn_read_length <= 0;
                    txn_start <= 1'b1;
                    state <= ST_CONFIG_WAIT;
                end
                ST_CONFIG_WAIT: if (txn_done) begin
                    if (txn_error) begin
                        bus_error <= 1'b1;
                        sensor_ok <= 1'b0;
                        initialized <= 1'b0;
                        delay_count <= 0;
                        state <= ST_DELAY;
                    end else if (config_step == 3'd1) begin
                        // PWR_MGMT_1 已退出睡眠，PWR_MGMT_2 已显式使能六轴；
                        // 等待稳定完成后再触碰采样链寄存器。
                        delay_count <= 0;
                        state <= ST_WAKE_DELAY;
                    end else if (config_step == 3'd6) begin
                        initialized <= 1'b1;
                        delay_count <= 0;
                        state <= ST_SAMPLE_DELAY;
                    end else begin
                        config_step <= config_step + 1'b1;
                        state <= ST_CONFIG_START;
                    end
                end
                ST_WAKE_DELAY: begin
                    if (delay_count >= WAKE_STABILIZE-1) begin
                        delay_count <= 0;
                        config_step <= 3'd2;
                        state <= ST_CONFIG_START;
                    end else delay_count <= delay_count + 1'b1;
                end
                ST_SAMPLE_DELAY: begin
                    if (delay_count >= SAMPLE_INTERVAL-1) begin
                        delay_count <= 0;
                        state <= ST_STATUS_START;
                    end else delay_count <= delay_count + 1'b1;
                end
                ST_STATUS_START: if (!txn_busy) begin
                    txn_read <= 1'b1;
                    txn_reg <= REG_INT_STATUS;
                    txn_write_length <= 0;
                    txn_read_length <= 1;
                    txn_start <= 1'b1;
                    state <= ST_STATUS_WAIT;
                end
                ST_STATUS_WAIT: if (txn_done) begin
                    if (txn_error) begin
                        bus_error <= 1'b1;
                        state <= ST_SAMPLE_DELAY;
                    end else if (rx_bytes[0][0]) state <= ST_BURST_START;
                    else state <= ST_SAMPLE_DELAY;
                end
                ST_BURST_START: if (!txn_busy) begin
                    txn_read <= 1'b1;
                    txn_reg <= REG_ACCEL_XOUT_H;
                    txn_write_length <= 0;
                    txn_read_length <= 5'd14;
                    txn_start <= 1'b1;
                    state <= ST_BURST_WAIT;
                end
                ST_BURST_WAIT: if (txn_done) begin
                    if (txn_error) begin
                        bus_error <= 1'b1;
                        state <= ST_SAMPLE_DELAY;
                    end else state <= ST_PUBLISH;
                end
                ST_PUBLISH: begin
                    accel_x <= {rx_bytes[0], rx_bytes[1]};
                    accel_y <= {rx_bytes[2], rx_bytes[3]};
                    accel_z <= {rx_bytes[4], rx_bytes[5]};
                    temperature_raw <= {rx_bytes[6], rx_bytes[7]};
                    gyro_x <= {rx_bytes[8], rx_bytes[9]};
                    gyro_y <= {rx_bytes[10], rx_bytes[11]};
                    gyro_z <= {rx_bytes[12], rx_bytes[13]};
                    sample_valid <= 1'b1;
                    state <= ST_SAMPLE_DELAY;
                end
                default: state <= ST_DELAY;
            endcase
        end
    end
endmodule
