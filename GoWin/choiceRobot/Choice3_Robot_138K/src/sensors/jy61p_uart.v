`timescale 1ns/1ps

// JY61P angle packet: 55 53 RollL RollH PitchL PitchH YawL YawH VerL VerH SUM.
// SUM is the low eight bits of bytes 0..9.  Raw signed values map to degrees
// with value * 180 / 32768; fixed-point conversion is intentionally left to
// the consuming control block.
module jy61p_uart #(
    parameter integer CLK_FREQ_HZ = 27000000,
    parameter integer BAUD_RATE   = 9600
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               uart_rx,
    output reg signed [15:0]  roll_raw,
    output reg signed [15:0]  pitch_raw,
    output reg signed [15:0]  yaw_raw,
    output reg        [15:0]  version_raw,
    output reg                frame_valid,
    output reg                checksum_error,
    output wire               framing_error
);
    wire [7:0] rx_byte;
    wire       rx_valid;
    reg  [3:0] index_q;
    reg  [7:0] sum_q;
    reg  [7:0] payload_q [0:7];

    uart_rx_8n1_jy61p #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(BAUD_RATE)) u_rx (
        .clk(clk), .rst_n(rst_n), .rx_i(uart_rx), .data_o(rx_byte),
        .valid_o(rx_valid), .framing_error_o(framing_error)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            index_q <= 0; sum_q <= 0; frame_valid <= 0; checksum_error <= 0;
            roll_raw <= 0; pitch_raw <= 0; yaw_raw <= 0; version_raw <= 0;
        end else begin
            frame_valid <= 0;
            checksum_error <= 0;
            if (framing_error)
                index_q <= 0;
            if (rx_valid) begin
                case (index_q)
                    0: if (rx_byte == 8'h55) begin index_q <= 1; sum_q <= 8'h55; end
                    1: begin
                        if (rx_byte == 8'h53) begin index_q <= 2; sum_q <= sum_q + rx_byte; end
                        else if (rx_byte == 8'h55) begin index_q <= 1; sum_q <= 8'h55; end
                        else index_q <= 0;
                    end
                    2,3,4,5,6,7,8,9: begin
                        payload_q[index_q-2] <= rx_byte;
                        sum_q <= sum_q + rx_byte;
                        index_q <= index_q + 1'b1;
                    end
                    10: begin
                        index_q <= 0;
                        if (rx_byte == sum_q) begin
                            roll_raw    <= {payload_q[1], payload_q[0]};
                            pitch_raw   <= {payload_q[3], payload_q[2]};
                            yaw_raw     <= {payload_q[5], payload_q[4]};
                            version_raw <= {payload_q[7], payload_q[6]};
                            frame_valid <= 1;
                        end else checksum_error <= 1;
                    end
                    default: index_q <= 0;
                endcase
            end
        end
    end
endmodule

module uart_rx_8n1_jy61p #(
    parameter integer CLK_FREQ_HZ = 27000000,
    parameter integer BAUD_RATE   = 9600
) (
    input wire clk, input wire rst_n, input wire rx_i,
    output reg [7:0] data_o, output reg valid_o, output reg framing_error_o
);
    localparam integer CPB0 = CLK_FREQ_HZ / BAUD_RATE;
    localparam integer CPB  = (CPB0 < 2) ? 2 : CPB0;
    localparam integer HALF = CPB / 2;
    localparam [1:0] IDLE=0, START=1, DATA=2, STOP=3;
    reg rx_meta, rx_sync;
    reg [1:0] state_q;
    reg [31:0] count_q;
    reg [2:0] bit_q;
    reg [7:0] shift_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin rx_meta<=1; rx_sync<=1; state_q<=IDLE; count_q<=0; bit_q<=0; shift_q<=0; data_o<=0; valid_o<=0; framing_error_o<=0; end
        else begin
            rx_meta <= rx_i; rx_sync <= rx_meta; valid_o <= 0; framing_error_o <= 0;
            case (state_q)
                IDLE: if (!rx_sync) begin state_q<=START; count_q<=0; end
                START: if (count_q == HALF-1) begin count_q<=0; if (!rx_sync) begin state_q<=DATA; bit_q<=0; end else state_q<=IDLE; end else count_q<=count_q+1'b1;
                DATA: if (count_q == CPB-1) begin count_q<=0; shift_q[bit_q]<=rx_sync; if (bit_q==7) state_q<=STOP; else bit_q<=bit_q+1'b1; end else count_q<=count_q+1'b1;
                STOP: if (count_q == CPB-1) begin count_q<=0; state_q<=IDLE; if (rx_sync) begin data_o<=shift_q; valid_o<=1; end else framing_error_o<=1; end else count_q<=count_q+1'b1;
                default: state_q<=IDLE;
            endcase
        end
    end
endmodule
