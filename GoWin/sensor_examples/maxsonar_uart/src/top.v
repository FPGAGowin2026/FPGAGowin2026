`timescale 1ns/1ps
module sensor_example_top #(parameter integer CLK_FREQ_HZ=27000000,parameter integer BAUD_RATE=9600)(input wire clk,input wire rst_n,input wire maxsonar_rx,output wire[15:0]range_raw,output wire range_valid,format_error,framing_error);
    maxsonar_uart #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.BAUD_RATE(BAUD_RATE))u_core(.clk(clk),.rst_n(rst_n),.uart_rx(maxsonar_rx),.range_raw(range_raw),.range_valid(range_valid),.format_error(format_error),.framing_error(framing_error));
endmodule
