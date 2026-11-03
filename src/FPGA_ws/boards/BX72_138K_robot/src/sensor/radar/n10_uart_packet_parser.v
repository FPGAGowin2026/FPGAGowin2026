`timescale 1ns/1ps

// LSLiDAR N10 serial packet decoder.
// Packet: A5 5A 3A [header/status bytes] start-angle(2) 16 x (range(2),strength)
// end-angle(2) checksum. The checksum is the low 8 bits of the sum of bytes
// 0..56, as used by LSLiDAR's N10_V1.0 driver. Angles are degrees x 100;
// ranges are millimetres.
module n10_uart_packet_parser #(
    parameter integer CHECK_CRC = 1
) (
    input  wire        clk,
    input  wire        reset_n,
    input  wire [7:0]  byte_data_i,
    input  wire        byte_valid_i,
    output reg         packet_valid_o,
    output reg [31:0]  packet_count_o,
    output reg [31:0]  crc_error_count_o,
    output reg [31:0]  length_error_count_o,
    output reg [31:0]  angle_error_count_o,
    output reg         point_sample_valid_o,
    output reg         point_valid_o,
    output reg [15:0]  point_angle_cdeg_o,
    output reg [15:0]  point_distance_mm_o,
    output reg [7:0]   point_strength_o
);
    localparam [1:0] ST_SYNC_A5 = 2'd0;
    localparam [1:0] ST_SYNC_5A = 2'd1;
    localparam [1:0] ST_LENGTH  = 2'd2;
    localparam [1:0] ST_DATA    = 2'd3;

    reg [1:0] state;
    reg [5:0] byte_index;
    reg [7:0] crc;
    reg [15:0] packet_start_angle;
    reg [15:0] packet_end_angle;
    reg [15:0] point_distance_buffer [0:15];
    reg [7:0] point_strength_buffer [0:15];
    reg [3:0] capture_point_index;
    reg [1:0] capture_point_phase;
    reg [4:0] point_index;
    reg emit_active;

    wire [16:0] packet_angle_delta = (packet_end_angle >= packet_start_angle)
        ? ({1'b0,packet_end_angle} - {1'b0,packet_start_angle})
        : ({1'b0,packet_end_angle} + 17'd36000 - {1'b0,packet_start_angle});
    // For 0..36000, floor(delta/15) is exactly (delta*34953)>>19. This
    // computes the packet increment once, instead of multiplying and dividing
    // again on every emitted point.
    wire [32:0] angle_reciprocal_product = {16'd0,packet_angle_delta} * 33'd34953;
    wire [12:0] packet_angle_increment = angle_reciprocal_product[31:19];
    wire [16:0] packet_angle_increment_wide = {4'd0,packet_angle_increment};
    wire [16:0] packet_angle_increment_times_15 =
        (packet_angle_increment_wide << 4) - packet_angle_increment_wide;
    wire [16:0] packet_angle_remainder_wide = packet_angle_delta - packet_angle_increment_times_15;
    wire [3:0] packet_angle_remainder = packet_angle_remainder_wide[3:0];

    reg [15:0] angle_current;
    reg [12:0] angle_increment;
    reg [3:0] angle_remainder;
    reg [3:0] angle_error;
    wire [4:0] angle_error_sum = {1'b0,angle_error} + {1'b0,angle_remainder};
    wire [17:0] angle_next_base = {2'b0,angle_current} + {5'b0,angle_increment};
    wire [17:0] angle_next_with_carry = angle_next_base + 18'd1;
    wire [17:0] angle_next_unwrapped = (angle_error_sum >= 5'd15)
                                      ? angle_next_with_carry : angle_next_base;
    wire [17:0] angle_next_wrapped = (angle_next_unwrapped >= 18'd36000)
                                    ? angle_next_unwrapped - 18'd36000
                                    : angle_next_unwrapped;
    wire [15:0] emitted_distance = point_distance_buffer[point_index[3:0]];
    wire [7:0] emitted_strength = point_strength_buffer[point_index[3:0]];

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= ST_SYNC_A5;
            byte_index <= 6'd0;
            crc <= 8'd0;
            packet_start_angle <= 16'd0;
            packet_end_angle <= 16'd0;
            capture_point_index <= 4'd0;
            capture_point_phase <= 2'd0;
            point_index <= 5'd0;
            emit_active <= 1'b0;
            angle_current <= 16'd0;
            angle_increment <= 13'd0;
            angle_remainder <= 4'd0;
            angle_error <= 4'd0;
            packet_valid_o <= 1'b0;
            packet_count_o <= 32'd0;
            crc_error_count_o <= 32'd0;
            length_error_count_o <= 32'd0;
            angle_error_count_o <= 32'd0;
            point_valid_o <= 1'b0;
            point_sample_valid_o <= 1'b0;
            point_angle_cdeg_o <= 16'd0;
            point_distance_mm_o <= 16'd0;
            point_strength_o <= 8'd0;
        end else begin
            packet_valid_o <= 1'b0;
            point_valid_o <= 1'b0;
            point_sample_valid_o <= 1'b0;

            if (byte_valid_i) begin
                case (state)
                    ST_SYNC_A5: begin
                        if (byte_data_i == 8'hA5) begin
                            crc <= byte_data_i;
                            state <= ST_SYNC_5A;
                        end
                    end

                    ST_SYNC_5A: begin
                        if (byte_data_i == 8'h5A) begin
                            crc <= crc + byte_data_i;
                            state <= ST_LENGTH;
                        end else if (byte_data_i == 8'hA5) begin
                            crc <= byte_data_i;
                        end else begin
                            state <= ST_SYNC_A5;
                        end
                    end

                    ST_LENGTH: begin
                        if (byte_data_i == 8'd58) begin
                            crc <= crc + byte_data_i;
                            byte_index <= 6'd3;
                            capture_point_index <= 4'd0;
                            capture_point_phase <= 2'd0;
                            state <= ST_DATA;
                        end else begin
                            length_error_count_o <= length_error_count_o + 1'b1;
                            if (byte_data_i == 8'hA5) begin
                                crc <= byte_data_i;
                                state <= ST_SYNC_5A;
                            end else begin
                                state <= ST_SYNC_A5;
                            end
                        end
                    end

                    ST_DATA: begin
                        case (byte_index)
                            6'd5: packet_start_angle[15:8] <= byte_data_i;
                            6'd6: packet_start_angle[7:0] <= byte_data_i;
                            6'd55: packet_end_angle[15:8] <= byte_data_i;
                            6'd56: packet_end_angle[7:0] <= byte_data_i;
                            default: begin end
                        endcase
                        if (byte_index >= 6'd7 && byte_index <= 6'd54) begin
                            case (capture_point_phase)
                                2'd0: point_distance_buffer[capture_point_index][15:8] <= byte_data_i;
                                2'd1: point_distance_buffer[capture_point_index][7:0] <= byte_data_i;
                                2'd2: begin
                                    point_strength_buffer[capture_point_index] <= byte_data_i;
                                    capture_point_index <= capture_point_index + 1'b1;
                                end
                                default: begin end
                            endcase
                            if (capture_point_phase == 2'd2)
                                capture_point_phase <= 2'd0;
                            else
                                capture_point_phase <= capture_point_phase + 1'b1;
                        end
                        if (byte_index < 6'd57) begin
                            crc <= crc + byte_data_i;
                            byte_index <= byte_index + 1'b1;
                        end else begin
                            state <= ST_SYNC_A5;
                            if (CHECK_CRC && byte_data_i != crc) begin
                                crc_error_count_o <= crc_error_count_o + 1'b1;
                            end else if (packet_start_angle >= 16'd36000 || packet_end_angle >= 16'd36000) begin
                                // 非法端点不能参加插值，也不能被后续当作角度覆盖。
                                angle_error_count_o <= angle_error_count_o + 1'b1;
                            end else begin
                                packet_count_o <= packet_count_o + 1'b1;
                                packet_valid_o <= 1'b1;
                                point_index <= 5'd0;
                                angle_current <= packet_start_angle;
                                angle_increment <= packet_angle_increment;
                                angle_remainder <= packet_angle_remainder;
                                angle_error <= 4'd0;
                                emit_active <= 1'b1;
                            end
                        end
                    end

                    default: state <= ST_SYNC_A5;
                endcase
            end

            if (emit_active) begin
                // 每个角度样本都通知规划器，包括 0xFFFF 无回波点；
                // 这样它可以识别雷达完整一圈的边界，而不把无回波当作净空。
                point_sample_valid_o <= 1'b1;
                point_angle_cdeg_o <= angle_current;
                point_distance_mm_o <= emitted_distance;
                point_strength_o <= emitted_strength;
                // LSLiDAR N10 用 0xFFFF 表示无效测距，不把它伪装成远处净空回波。
                point_valid_o <= emitted_distance != 16'hffff;
                if (point_index == 5'd15) begin
                    emit_active <= 1'b0;
                end else begin
                    if (angle_error_sum >= 5'd15)
                        angle_error <= 4'(angle_error_sum - 5'd15);
                    else
                        angle_error <= angle_error_sum[3:0];
                    angle_current <= angle_next_wrapped[15:0];
                    point_index <= point_index + 1'b1;
                end
            end
        end
    end
endmodule

`timescale 1ns/1ps

// Fixed-baud asynchronous 8-N-1 receiver. The integer divider is rounded to
// the nearest bit period; at 50 MHz / 230400 baud its error is below 0.01%.
// 雷达输入保留两级同步、异步低有效复位及停止位后提交数据。
module uart_8n1_rx #(
    parameter integer CLK_HZ=50000000,
    parameter integer BAUD=230400
)(
    input wire clk,reset_n,rx_i,
    output wire [7:0] data_o,
    output wire data_valid_o,framing_error_o
);
    localparam integer BIT_CYCLES=(CLK_HZ+BAUD/2)/BAUD;
    (* ASYNC_REG="TRUE" *) reg rx_meta,rx_sync;
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n)begin rx_meta<=1'b1;rx_sync<=1'b1;end
        else begin rx_meta<=rx_i;rx_sync<=rx_meta;end
    end
    bx72_uart_rx #(.BIT_CYCLES(BIT_CYCLES),.HALF_CYCLES(BIT_CYCLES/2),
        .ASYNC_RESET(1),.DATA_ON_SAMPLE(0)) receiver(
        .clk(clk),.reset_n(reset_n),.rx_i(rx_sync),.data_o(data_o),
        .valid_o(data_valid_o),.framing_error_o(framing_error_o));
endmodule
