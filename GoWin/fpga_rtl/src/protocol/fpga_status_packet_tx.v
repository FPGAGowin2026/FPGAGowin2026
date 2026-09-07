`timescale 1ns/1ps

// 生成 udp_protocol.md 定义的固定 38 字节 Status 报文。
// start_i 原子锁存状态；busy_o 期间的状态变化只影响下一帧。
module fpga_status_packet_tx (
    input  wire        clk_i,
    input  wire        rst_n_i,
    input  wire        start_i,
    input  wire        ready_i,
    input  wire [31:0] sequence_i,
    input  wire [63:0] timestamp_us_i,
    input  wire [7:0]  mode_i,
    input  wire [7:0]  motion_enabled_i,
    input  wire [7:0]  estop_i,
    input  wire [7:0]  safety_stop_i,
    input  wire [31:0] fault_bits_i,
    input  wire [31:0] uptime_ms_i,
    input  wire [31:0] last_control_sequence_i,
    output wire        valid_o,
    output reg  [7:0]  data_o,
    output reg         busy_o,
    output reg         done_o
);
    reg [5:0] index_q;
    reg [15:0] crc_q;
    reg [31:0] sequence_q;
    reg [63:0] timestamp_q;
    reg [127:0] payload_q;

    assign valid_o = busy_o;

    function [15:0] crc16_ccitt_next;
        input [15:0] crc_in;
        input [7:0] data_in;
        integer bit_index;
        reg [15:0] crc;
        begin
            crc = crc_in ^ {data_in, 8'h00};
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1)
                crc = crc[15] ? ((crc << 1) ^ 16'h1021) : (crc << 1);
            crc16_ccitt_next = crc;
        end
    endfunction

    always @(*) begin
        data_o = 8'h00;
        case (index_q)
            6'd0: data_o = 8'h46;
            6'd1: data_o = 8'h50;
            6'd2: data_o = 8'h47;
            6'd3: data_o = 8'h41;
            6'd4: data_o = 8'h01;
            6'd5: data_o = 8'h03;
            6'd6: data_o = 8'h00;
            6'd7: data_o = 8'h10;
            6'd8: data_o = sequence_q[31:24];
            6'd9: data_o = sequence_q[23:16];
            6'd10: data_o = sequence_q[15:8];
            6'd11: data_o = sequence_q[7:0];
            6'd12: data_o = timestamp_q[63:56];
            6'd13: data_o = timestamp_q[55:48];
            6'd14: data_o = timestamp_q[47:40];
            6'd15: data_o = timestamp_q[39:32];
            6'd16: data_o = timestamp_q[31:24];
            6'd17: data_o = timestamp_q[23:16];
            6'd18: data_o = timestamp_q[15:8];
            6'd19: data_o = timestamp_q[7:0];
            6'd20: data_o = payload_q[127:120];
            6'd21: data_o = payload_q[119:112];
            6'd22: data_o = payload_q[111:104];
            6'd23: data_o = payload_q[103:96];
            6'd24: data_o = payload_q[95:88];
            6'd25: data_o = payload_q[87:80];
            6'd26: data_o = payload_q[79:72];
            6'd27: data_o = payload_q[71:64];
            6'd28: data_o = payload_q[63:56];
            6'd29: data_o = payload_q[55:48];
            6'd30: data_o = payload_q[47:40];
            6'd31: data_o = payload_q[39:32];
            6'd32: data_o = payload_q[31:24];
            6'd33: data_o = payload_q[23:16];
            6'd34: data_o = payload_q[15:8];
            6'd35: data_o = payload_q[7:0];
            6'd36: data_o = crc_q[15:8];
            6'd37: data_o = crc_q[7:0];
            default: data_o = 8'h00;
        endcase
    end

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            index_q <= 6'd0;
            crc_q <= 16'hffff;
            sequence_q <= 32'd0;
            timestamp_q <= 64'd0;
            payload_q <= 128'd0;
            busy_o <= 1'b0;
            done_o <= 1'b0;
        end else begin
            done_o <= 1'b0;
            if (start_i && !busy_o) begin
                index_q <= 6'd0;
                crc_q <= 16'hffff;
                sequence_q <= sequence_i;
                timestamp_q <= timestamp_us_i;
                payload_q <= {
                    mode_i, motion_enabled_i, estop_i, safety_stop_i,
                    fault_bits_i, uptime_ms_i, last_control_sequence_i
                };
                busy_o <= 1'b1;
            end else if (busy_o && ready_i) begin
                if (index_q < 6'd36)
                    crc_q <= crc16_ccitt_next(crc_q, data_o);
                if (index_q == 6'd37) begin
                    busy_o <= 1'b0;
                    done_o <= 1'b1;
                end else begin
                    index_q <= index_q + 1'b1;
                end
            end
        end
    end

endmodule
