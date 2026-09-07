`timescale 1ns/1ps

// 生成 udp_protocol.md 定义的固定 72 字节 Telemetry 报文。
// start_i 时原子锁存一帧数据；busy_o 期间输入变化不会撕裂当前报文。
module fpga_telemetry_packet_tx (
    input  wire               clk_i,
    input  wire               rst_n_i,
    input  wire               start_i,
    input  wire               ready_i,
    input  wire [31:0]        sequence_i,
    input  wire [63:0]        timestamp_us_i,
    input  wire signed [31:0] left_ticks_i,
    input  wire signed [31:0] right_ticks_i,
    input  wire signed [31:0] left_velocity_mm_s_i,
    input  wire signed [31:0] right_velocity_mm_s_i,
    input  wire signed [31:0] accel_x_mm_s2_i,
    input  wire signed [31:0] accel_y_mm_s2_i,
    input  wire signed [31:0] accel_z_mm_s2_i,
    input  wire signed [31:0] gyro_x_mrad_s_i,
    input  wire signed [31:0] gyro_y_mrad_s_i,
    input  wire signed [31:0] gyro_z_mrad_s_i,
    input  wire [15:0]        range_front_mm_i,
    input  wire [15:0]        range_left_mm_i,
    input  wire [15:0]        range_right_mm_i,
    input  wire [31:0]        status_bits_i,
    output wire               valid_o,
    output reg  [7:0]         data_o,
    output reg                busy_o,
    output reg                done_o
);

    reg [6:0] index_q;
    reg [15:0] crc_q;
    reg [31:0] sequence_q;
    reg [63:0] timestamp_q;
    reg [319:0] fields_q;
    reg [47:0] ranges_q;
    reg [31:0] status_bits_q;

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

    // 索引 0..69 为头和 payload，70..71 为最终 CRC（网络字节序）。
    always @(*) begin
        data_o = 8'h00;
        case (index_q)
            7'd0: data_o = 8'h46;
            7'd1: data_o = 8'h50;
            7'd2: data_o = 8'h47;
            7'd3: data_o = 8'h41;
            7'd4: data_o = 8'h01;
            7'd5: data_o = 8'h02;
            7'd6: data_o = 8'h00;
            7'd7: data_o = 8'h32;
            7'd8: data_o = sequence_q[31:24];
            7'd9: data_o = sequence_q[23:16];
            7'd10: data_o = sequence_q[15:8];
            7'd11: data_o = sequence_q[7:0];
            7'd12: data_o = timestamp_q[63:56];
            7'd13: data_o = timestamp_q[55:48];
            7'd14: data_o = timestamp_q[47:40];
            7'd15: data_o = timestamp_q[39:32];
            7'd16: data_o = timestamp_q[31:24];
            7'd17: data_o = timestamp_q[23:16];
            7'd18: data_o = timestamp_q[15:8];
            7'd19: data_o = timestamp_q[7:0];
            7'd20: data_o = fields_q[319:312];
            7'd21: data_o = fields_q[311:304];
            7'd22: data_o = fields_q[303:296];
            7'd23: data_o = fields_q[295:288];
            7'd24: data_o = fields_q[287:280];
            7'd25: data_o = fields_q[279:272];
            7'd26: data_o = fields_q[271:264];
            7'd27: data_o = fields_q[263:256];
            7'd28: data_o = fields_q[255:248];
            7'd29: data_o = fields_q[247:240];
            7'd30: data_o = fields_q[239:232];
            7'd31: data_o = fields_q[231:224];
            7'd32: data_o = fields_q[223:216];
            7'd33: data_o = fields_q[215:208];
            7'd34: data_o = fields_q[207:200];
            7'd35: data_o = fields_q[199:192];
            7'd36: data_o = fields_q[191:184];
            7'd37: data_o = fields_q[183:176];
            7'd38: data_o = fields_q[175:168];
            7'd39: data_o = fields_q[167:160];
            7'd40: data_o = fields_q[159:152];
            7'd41: data_o = fields_q[151:144];
            7'd42: data_o = fields_q[143:136];
            7'd43: data_o = fields_q[135:128];
            7'd44: data_o = fields_q[127:120];
            7'd45: data_o = fields_q[119:112];
            7'd46: data_o = fields_q[111:104];
            7'd47: data_o = fields_q[103:96];
            7'd48: data_o = fields_q[95:88];
            7'd49: data_o = fields_q[87:80];
            7'd50: data_o = fields_q[79:72];
            7'd51: data_o = fields_q[71:64];
            7'd52: data_o = fields_q[63:56];
            7'd53: data_o = fields_q[55:48];
            7'd54: data_o = fields_q[47:40];
            7'd55: data_o = fields_q[39:32];
            7'd56: data_o = fields_q[31:24];
            7'd57: data_o = fields_q[23:16];
            7'd58: data_o = fields_q[15:8];
            7'd59: data_o = fields_q[7:0];
            7'd60: data_o = ranges_q[47:40];
            7'd61: data_o = ranges_q[39:32];
            7'd62: data_o = ranges_q[31:24];
            7'd63: data_o = ranges_q[23:16];
            7'd64: data_o = ranges_q[15:8];
            7'd65: data_o = ranges_q[7:0];
            7'd66: data_o = status_bits_q[31:24];
            7'd67: data_o = status_bits_q[23:16];
            7'd68: data_o = status_bits_q[15:8];
            7'd69: data_o = status_bits_q[7:0];
            7'd70: data_o = crc_q[15:8];
            7'd71: data_o = crc_q[7:0];
            default: data_o = 8'h00;
        endcase
    end

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            busy_o <= 1'b0;
            done_o <= 1'b0;
            index_q <= 7'd0;
            crc_q <= 16'hffff;
            sequence_q <= 32'd0;
            timestamp_q <= 64'd0;
            status_bits_q <= 32'd0;
            fields_q <= 320'd0;
            ranges_q <= 48'd0;
        end else begin
            done_o <= 1'b0;
            if (start_i && !busy_o) begin
                busy_o <= 1'b1;
                index_q <= 7'd0;
                crc_q <= 16'hffff;
                sequence_q <= sequence_i;
                timestamp_q <= timestamp_us_i;
                fields_q <= {
                    left_ticks_i, right_ticks_i,
                    left_velocity_mm_s_i, right_velocity_mm_s_i,
                    accel_x_mm_s2_i, accel_y_mm_s2_i, accel_z_mm_s2_i,
                    gyro_x_mrad_s_i, gyro_y_mrad_s_i, gyro_z_mrad_s_i
                };
                ranges_q <= {range_front_mm_i, range_left_mm_i, range_right_mm_i};
                status_bits_q <= status_bits_i;
            end else if (busy_o && ready_i) begin
                if (index_q < 7'd70)
                    crc_q <= crc16_ccitt_next(crc_q, data_o);
                if (index_q == 7'd71) begin
                    busy_o <= 1'b0;
                    done_o <= 1'b1;
                end else begin
                    index_q <= index_q + 1'b1;
                end
            end
        end
    end

endmodule
