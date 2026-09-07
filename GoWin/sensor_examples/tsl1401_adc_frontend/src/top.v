`timescale 1ns/1ps

module sensor_example_top #(
    parameter integer ADC_WIDTH = 12,
    parameter integer CLK_HALF_PERIOD_CYCLES = 8,
    parameter integer PIXEL_SETTLE_CYCLES = 8,
    parameter integer ADC_TIMEOUT_CYCLES = 256,
    parameter integer INTEGRATION_CYCLES = 270000,
    parameter integer CHARGE_TRANSFER_CYCLES = 540
) (
    input wire clk, input wire reset_n,
    output wire tsl_clk, output wire tsl_si,
    output wire adc_start,
    input wire [ADC_WIDTH-1:0] adc_data,
    input wire adc_valid,
    output wire [6:0] pixel_index,
    output wire [ADC_WIDTH-1:0] pixel_data,
    output wire pixel_valid, output wire frame_start,
    output wire frame_done, output wire frame_active,
    output wire adc_timeout
);
    tsl1401_adc_frontend #(
        .ADC_WIDTH(ADC_WIDTH),
        .CLK_HALF_PERIOD_CYCLES(CLK_HALF_PERIOD_CYCLES),
        .PIXEL_SETTLE_CYCLES(PIXEL_SETTLE_CYCLES),
        .ADC_TIMEOUT_CYCLES(ADC_TIMEOUT_CYCLES),
        .INTEGRATION_CYCLES(INTEGRATION_CYCLES),
        .CHARGE_TRANSFER_CYCLES(CHARGE_TRANSFER_CYCLES)
    ) u_frontend (
        .clk(clk),.reset_n(reset_n),.sensor_clk(tsl_clk),.sensor_si(tsl_si),
        .adc_start(adc_start),.adc_data(adc_data),.adc_valid(adc_valid),
        .pixel_index(pixel_index),.pixel_data(pixel_data),.pixel_valid(pixel_valid),
        .frame_start(frame_start),.frame_done(frame_done),.frame_active(frame_active),
        .adc_timeout(adc_timeout)
    );
endmodule
