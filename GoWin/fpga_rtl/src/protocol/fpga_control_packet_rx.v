`timescale 1ns/1ps

// 解析 udp_protocol.md 定义的固定 31 字节 Control 报文。
// UDP 负载可在以太网 FCS 判定前到达，因此只有 packet_done_i 且
// packet_error_i=0 后才原子提交命令；接收期间的字段不会直接驱动电机。
module fpga_control_packet_rx (
    input  wire               clk_i,
    input  wire               rst_n_i,
    input  wire               payload_valid_i,
    input  wire [7:0]         payload_byte_i,
    input  wire               packet_done_i,
    input  wire               packet_error_i,

    output reg                control_valid_o,
    output reg                packet_rejected_o,
    output reg                duplicate_sequence_o,
    output reg [31:0]         sequence_o,
    output reg [63:0]         timestamp_us_o,
    output reg signed [31:0]  linear_mm_s_o,
    output reg signed [31:0]  angular_mrad_s_o,
    output reg                estop_o
);

    localparam [31:0] MAGIC = 32'h46504741;
    localparam [7:0] VERSION = 8'd1;
    localparam [7:0] TYPE_CONTROL = 8'd1;
    localparam [15:0] CONTROL_PAYLOAD_LENGTH = 16'd9;
    localparam [5:0] CONTROL_PACKET_BYTES = 6'd31;

    reg receiving_q;
    reg [5:0] byte_count_q;
    reg [31:0] magic_q;
    reg [7:0] version_q;
    reg [7:0] message_type_q;
    reg [15:0] payload_length_q;
    reg [31:0] sequence_q;
    reg [63:0] timestamp_q;
    reg signed [31:0] linear_q;
    reg signed [31:0] angular_q;
    reg [7:0] flags_q;
    reg [15:0] received_crc_q;
    reg [15:0] calculated_crc_q;
    reg have_sequence_q;

    wire [31:0] sequence_delta = sequence_q - sequence_o;
    wire sequence_is_newer = !have_sequence_q ||
        ((sequence_delta != 32'd0) && (sequence_delta < 32'h80000000));
    wire header_ok = magic_q == MAGIC &&
        version_q == VERSION &&
        message_type_q == TYPE_CONTROL &&
        payload_length_q == CONTROL_PAYLOAD_LENGTH;
    wire packet_ok = receiving_q &&
        byte_count_q == CONTROL_PACKET_BYTES &&
        !packet_error_i && header_ok &&
        calculated_crc_q == received_crc_q;

    function [15:0] crc16_ccitt_next;
        input [15:0] crc_in;
        input [7:0] data_in;
        integer bit_index;
        reg [15:0] crc;
        begin
            crc = crc_in ^ {data_in, 8'h00};
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
                if (crc[15])
                    crc = (crc << 1) ^ 16'h1021;
                else
                    crc = crc << 1;
            end
            crc16_ccitt_next = crc;
        end
    endfunction

    // 固定格式逐字段接收。超过 31 字节时计数继续增长，最终长度检查会拒绝。
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            receiving_q <= 1'b0;
            byte_count_q <= 6'd0;
            magic_q <= 32'd0;
            version_q <= 8'd0;
            message_type_q <= 8'd0;
            payload_length_q <= 16'd0;
            sequence_q <= 32'd0;
            timestamp_q <= 64'd0;
            linear_q <= 32'sd0;
            angular_q <= 32'sd0;
            flags_q <= 8'd0;
            received_crc_q <= 16'd0;
            calculated_crc_q <= 16'hffff;
        end else begin
            if (packet_done_i) begin
                receiving_q <= 1'b0;
                byte_count_q <= 6'd0;
            end

            if (payload_valid_i) begin
                if (!receiving_q) begin
                    receiving_q <= 1'b1;
                    byte_count_q <= 6'd1;
                    magic_q <= {payload_byte_i, 24'd0};
                    version_q <= 8'd0;
                    message_type_q <= 8'd0;
                    payload_length_q <= 16'd0;
                    sequence_q <= 32'd0;
                    timestamp_q <= 64'd0;
                    linear_q <= 32'sd0;
                    angular_q <= 32'sd0;
                    flags_q <= 8'd0;
                    received_crc_q <= 16'd0;
                    calculated_crc_q <= crc16_ccitt_next(16'hffff, payload_byte_i);
                end else begin
                    if (byte_count_q != 6'h3f)
                        byte_count_q <= byte_count_q + 1'b1;
                    case (byte_count_q)
                        6'd1:  magic_q[23:16] <= payload_byte_i;
                        6'd2:  magic_q[15:8] <= payload_byte_i;
                        6'd3:  magic_q[7:0] <= payload_byte_i;
                        6'd4:  version_q <= payload_byte_i;
                        6'd5:  message_type_q <= payload_byte_i;
                        6'd6:  payload_length_q[15:8] <= payload_byte_i;
                        6'd7:  payload_length_q[7:0] <= payload_byte_i;
                        6'd8:  sequence_q[31:24] <= payload_byte_i;
                        6'd9:  sequence_q[23:16] <= payload_byte_i;
                        6'd10: sequence_q[15:8] <= payload_byte_i;
                        6'd11: sequence_q[7:0] <= payload_byte_i;
                        6'd12: timestamp_q[63:56] <= payload_byte_i;
                        6'd13: timestamp_q[55:48] <= payload_byte_i;
                        6'd14: timestamp_q[47:40] <= payload_byte_i;
                        6'd15: timestamp_q[39:32] <= payload_byte_i;
                        6'd16: timestamp_q[31:24] <= payload_byte_i;
                        6'd17: timestamp_q[23:16] <= payload_byte_i;
                        6'd18: timestamp_q[15:8] <= payload_byte_i;
                        6'd19: timestamp_q[7:0] <= payload_byte_i;
                        6'd20: linear_q[31:24] <= payload_byte_i;
                        6'd21: linear_q[23:16] <= payload_byte_i;
                        6'd22: linear_q[15:8] <= payload_byte_i;
                        6'd23: linear_q[7:0] <= payload_byte_i;
                        6'd24: angular_q[31:24] <= payload_byte_i;
                        6'd25: angular_q[23:16] <= payload_byte_i;
                        6'd26: angular_q[15:8] <= payload_byte_i;
                        6'd27: angular_q[7:0] <= payload_byte_i;
                        6'd28: flags_q <= payload_byte_i;
                        6'd29: received_crc_q[15:8] <= payload_byte_i;
                        6'd30: received_crc_q[7:0] <= payload_byte_i;
                        default: ;
                    endcase
                    if (byte_count_q < 6'd29)
                        calculated_crc_q <= crc16_ccitt_next(calculated_crc_q, payload_byte_i);
                end
            end
        end
    end

    // 只在完整 UDP/FCS 与应用层校验同时通过后提交；重复或倒序序号单独标记。
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            control_valid_o <= 1'b0;
            packet_rejected_o <= 1'b0;
            duplicate_sequence_o <= 1'b0;
            sequence_o <= 32'd0;
            timestamp_us_o <= 64'd0;
            linear_mm_s_o <= 32'sd0;
            angular_mrad_s_o <= 32'sd0;
            estop_o <= 1'b0;
            have_sequence_q <= 1'b0;
        end else begin
            control_valid_o <= 1'b0;
            packet_rejected_o <= 1'b0;
            duplicate_sequence_o <= 1'b0;

            if (packet_done_i) begin
                if (packet_ok && sequence_is_newer) begin
                    control_valid_o <= 1'b1;
                    sequence_o <= sequence_q;
                    timestamp_us_o <= timestamp_q;
                    linear_mm_s_o <= linear_q;
                    angular_mrad_s_o <= angular_q;
                    estop_o <= flags_q[0];
                    have_sequence_q <= 1'b1;
                end else begin
                    packet_rejected_o <= 1'b1;
                    duplicate_sequence_o <= packet_ok && !sequence_is_newer;
                end
            end
        end
    end

endmodule
