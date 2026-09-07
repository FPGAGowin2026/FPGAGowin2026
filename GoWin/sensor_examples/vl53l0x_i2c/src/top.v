`timescale 1ns/1ps
module sensor_example_top #(
    parameter integer I2C_CLK_DIV=32,parameter integer I2C_STRETCH_TIMEOUT_TICKS=8192,
    parameter integer STARTUP_DELAY_CYCLES=270000,parameter integer SAMPLE_INTERVAL_CYCLES=27000,
    parameter integer POLL_RETRY_LIMIT=255,parameter IOVDD_2V8=1'b1
)(input wire clk,input wire reset_n,inout wire tof_scl,inout wire tof_sda,
   output wire sensor_ok,output wire initialized,output wire identity_error,
   output wire bus_error,output wire timeout_error,output wire[7:0]model_id,
   output wire[2:0]interrupt_status,output wire[7:0]range_status_raw,
   output wire[15:0]distance_mm,output wire distance_valid);
  vl53l0x_i2c #(.I2C_CLK_DIV(I2C_CLK_DIV),.I2C_STRETCH_TIMEOUT_TICKS(I2C_STRETCH_TIMEOUT_TICKS),
    .STARTUP_DELAY_CYCLES(STARTUP_DELAY_CYCLES),.SAMPLE_INTERVAL_CYCLES(SAMPLE_INTERVAL_CYCLES),
    .POLL_RETRY_LIMIT(POLL_RETRY_LIMIT),.IOVDD_2V8(IOVDD_2V8))u_sensor(
    .clk(clk),.reset_n(reset_n),.i2c_scl(tof_scl),.i2c_sda(tof_sda),.sensor_ok(sensor_ok),
    .initialized(initialized),.identity_error(identity_error),.bus_error(bus_error),
    .timeout_error(timeout_error),.model_id(model_id),.interrupt_status(interrupt_status),
    .range_status_raw(range_status_raw),
    .distance_mm(distance_mm),.distance_valid(distance_valid));
endmodule
