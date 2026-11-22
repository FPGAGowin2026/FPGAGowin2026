`timescale 1ns/1ps

// The 16 emitted points share the packet-receipt timestamp; it is not their
// physical measurement time. Apply the mounting transform in the caller.
module n10_receiver #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD = 230400,
    parameter integer FRONT_HALF_ANGLE_CDEG = 9000
) (
    input wire clk_i,
    input wire reset_n_i,
    input wire rx_i,
    input wire [31:0] now_us_i,
    output wire sample_valid_o,
    output wire hit_valid_o,
    output wire in_front_o,
    output wire [15:0] angle_cdeg_o,
    output wire [15:0] distance_mm_o,
    output wire [7:0] strength_o,
    output reg [31:0] packet_received_us_o,
    output wire [31:0] packet_count_o,
    output wire [31:0] checksum_errors_o,
    output wire [31:0] length_errors_o,
    output wire [31:0] angle_errors_o,
    output wire uart_error_o
);
    wire [7:0] uart_byte;
    wire uart_valid, packet_valid, point_valid;

    n10_uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) receiver (
        .clk_i(clk_i), .reset_n_i(reset_n_i), .rx_i(rx_i),
        .data_o(uart_byte), .valid_o(uart_valid), .framing_error_o(uart_error_o)
    );
    n10_uart_packet_parser #(.CHECK_CRC(1)) parser (
        .clk(clk_i), .reset_n(reset_n_i), .byte_data_i(uart_byte), .byte_valid_i(uart_valid),
        .packet_valid_o(packet_valid), .packet_count_o(packet_count_o),
        .crc_error_count_o(checksum_errors_o), .length_error_count_o(length_errors_o),
        .angle_error_count_o(angle_errors_o), .point_sample_valid_o(sample_valid_o),
        .point_valid_o(point_valid), .point_angle_cdeg_o(angle_cdeg_o),
        .point_distance_mm_o(distance_mm_o), .point_strength_o(strength_o)
    );
    assign in_front_o = angle_cdeg_o <= FRONT_HALF_ANGLE_CDEG ||
                        angle_cdeg_o >= 36000 - FRONT_HALF_ANGLE_CDEG;
    assign hit_valid_o = sample_valid_o && point_valid && in_front_o;

    always @(posedge clk_i or negedge reset_n_i) begin
        if (!reset_n_i) packet_received_us_o <= 0;
        else if (packet_valid) packet_received_us_o <= now_us_i;
    end
endmodule

module n10_uart_rx #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD = 230400
) (
    input wire clk_i,
    input wire reset_n_i,
    input wire rx_i,
    output reg [7:0] data_o,
    output reg valid_o,
    output reg framing_error_o
);
    localparam integer BIT_CYCLES = (CLK_HZ + BAUD / 2) / BAUD;
    localparam integer TIMER_WIDTH = BIT_CYCLES <= 2 ? 1 : $clog2(BIT_CYCLES + 1);
    localparam [1:0] IDLE = 0, START = 1, DATA = 2, STOP = 3;
    (* ASYNC_REG = "TRUE" *) reg rx_meta, rx_sync;
    reg [1:0] state;
    reg [TIMER_WIDTH-1:0] timer;
    reg [2:0] bit_index;
    reg [7:0] byte_buffer;

    always @(posedge clk_i or negedge reset_n_i) begin
        if (!reset_n_i) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
            state <= IDLE;
            timer <= 0;
            bit_index <= 0;
            byte_buffer <= 0;
            data_o <= 0;
            valid_o <= 0;
            framing_error_o <= 0;
        end else begin
            rx_meta <= rx_i;
            rx_sync <= rx_meta;
            valid_o <= 0;
            framing_error_o <= 0;
            case (state)
                IDLE: if (!rx_sync) begin
                    timer <= BIT_CYCLES / 2 - 1;
                    state <= START;
                end
                START: if (timer != 0) timer <= timer - 1'b1;
                else if (!rx_sync) begin
                    timer <= BIT_CYCLES - 1;
                    bit_index <= 0;
                    state <= DATA;
                end else state <= IDLE;
                DATA: if (timer != 0) timer <= timer - 1'b1;
                else begin
                    byte_buffer[bit_index] <= rx_sync;
                    timer <= BIT_CYCLES - 1;
                    if (bit_index == 7) state <= STOP;
                    else bit_index <= bit_index + 1'b1;
                end
                STOP: if (timer != 0) timer <= timer - 1'b1;
                else begin
                    if (rx_sync) begin
                        data_o <= byte_buffer;
                        valid_o <= 1;
                    end else framing_error_o <= 1;
                    state <= IDLE;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
