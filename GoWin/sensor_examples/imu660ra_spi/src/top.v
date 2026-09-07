`timescale 1ns/1ps
module sensor_example_top#(parameter integer CLK_DIV=2,parameter integer STARTUP_DELAY_CYCLES=540000,
 parameter integer SAMPLE_INTERVAL_CYCLES=27000,parameter integer ID_RETRY_LIMIT=32)(input wire clk,input wire reset_n,input wire imu_miso,
 output wire imu_cs_n,output wire imu_sclk,output wire imu_mosi,output wire initialized,output wire sensor_ok,
 output wire configuration_required,input wire[2:0]axis_select,output reg signed[15:0]axis_data,output wire sample_valid,output wire protocol_error);
 wire signed[15:0]ax,ay,az,gx,gy,gz;imu660ra_spi#(.CLK_DIV(CLK_DIV),.STARTUP_DELAY_CYCLES(STARTUP_DELAY_CYCLES),.SAMPLE_INTERVAL_CYCLES(SAMPLE_INTERVAL_CYCLES),.ID_RETRY_LIMIT(ID_RETRY_LIMIT))u(
 .clk(clk),.reset_n(reset_n),.spi_miso(imu_miso),.spi_cs_n(imu_cs_n),.spi_sclk(imu_sclk),.spi_mosi(imu_mosi),.initialized(initialized),.sensor_ok(sensor_ok),.configuration_required(configuration_required),
 .accel_x(ax),.accel_y(ay),.accel_z(az),.gyro_x(gx),.gyro_y(gy),.gyro_z(gz),.sample_valid(sample_valid),.protocol_error(protocol_error));
 always@(*)case(axis_select)0:axis_data=ax;1:axis_data=ay;2:axis_data=az;3:axis_data=gx;4:axis_data=gy;5:axis_data=gz;default:axis_data=0;endcase
endmodule
