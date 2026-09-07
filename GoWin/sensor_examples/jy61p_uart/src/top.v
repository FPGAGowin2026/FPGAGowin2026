`timescale 1ns/1ps
module sensor_example_top #(
    parameter integer CLK_FREQ_HZ=27000000, parameter integer BAUD_RATE=9600
) (
    input wire clk, input wire rst_n, input wire jy61p_rx,
    input wire [1:0] debug_select,
    output reg [15:0] debug_data,
    output wire frame_valid, protocol_error
);
    wire signed [15:0] roll_raw;
    wire signed [15:0] pitch_raw;
    wire signed [15:0] yaw_raw;
    wire [15:0] version_raw;
    wire checksum_error;
    wire framing_error;
    assign protocol_error = checksum_error | framing_error;
    always @* begin
        case (debug_select)
            2'd0: debug_data = roll_raw;
            2'd1: debug_data = pitch_raw;
            2'd2: debug_data = yaw_raw;
            default: debug_data = version_raw;
        endcase
    end
    jy61p_uart #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.BAUD_RATE(BAUD_RATE)) u_core(
        .clk(clk),.rst_n(rst_n),.uart_rx(jy61p_rx),.roll_raw(roll_raw),
        .pitch_raw(pitch_raw),.yaw_raw(yaw_raw),.version_raw(version_raw),
        .frame_valid(frame_valid),.checksum_error(checksum_error),.framing_error(framing_error));
endmodule
