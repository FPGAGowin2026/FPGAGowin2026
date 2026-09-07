`timescale 1ns/1ps

module sensor_example_top #(
    parameter integer I2C_CLK_DIV = 32,
    parameter integer STARTUP_DELAY_CYCLES = 270000,
    parameter integer SAMPLE_INTERVAL_CYCLES = 270000
) (
    input  wire               clk,
    input  wire               reset_n,
    inout  wire               ina_scl,
    inout  wire               ina_sda,
    output wire               sensor_ok,
    output wire               initialized,
    output wire               bus_error,
    output wire signed [15:0] shunt_voltage_raw,
    output wire [15:0]        bus_voltage_raw,
    output wire [15:0]        bus_voltage_mv,
    output wire               sample_valid
);
    ina219_i2c #(
        .I2C_CLK_DIV(I2C_CLK_DIV),
        .STARTUP_DELAY_CYCLES(STARTUP_DELAY_CYCLES),
        .SAMPLE_INTERVAL_CYCLES(SAMPLE_INTERVAL_CYCLES)
    ) u_sensor (
        .clk(clk),.reset_n(reset_n),.i2c_scl(ina_scl),.i2c_sda(ina_sda),
        .sensor_ok(sensor_ok),.initialized(initialized),.bus_error(bus_error),
        .shunt_voltage_raw(shunt_voltage_raw),.bus_voltage_raw(bus_voltage_raw),
        .bus_voltage_mv(bus_voltage_mv),.sample_valid(sample_valid)
    );
endmodule
