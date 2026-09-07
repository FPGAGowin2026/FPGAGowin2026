`timescale 1ns/1ps
module sensor_example_top #(parameter integer CLK_FREQ_HZ=27000000,parameter integer BAUD_RATE=9600,parameter integer MAX_BODY_CHARS=96)(
    input wire clk,input wire rst_n,input wire gnss_rx,output wire sentence_valid,output wire[1:0]sentence_type,
    output wire[7:0]body_length,output wire[7:0]checksum_value,output wire checksum_error,format_error,overflow_error,framing_error);
    gnss_nmea_uart #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.BAUD_RATE(BAUD_RATE),.MAX_BODY_CHARS(MAX_BODY_CHARS))u_core(
        .clk(clk),.rst_n(rst_n),.uart_rx(gnss_rx),.sentence_valid(sentence_valid),.sentence_type(sentence_type),
        .body_length(body_length),.checksum_value(checksum_value),.checksum_error(checksum_error),.format_error(format_error),.overflow_error(overflow_error),.framing_error(framing_error));
endmodule
