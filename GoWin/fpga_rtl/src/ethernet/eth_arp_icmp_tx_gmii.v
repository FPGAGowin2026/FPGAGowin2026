`timescale 1ns/1ps

// 第 52 章 ARP 发送帧边界的最小派生实现，并复用现有 crc32_d8。
// 只发送固定 ARP Reply 或与请求等长的 ICMP Echo Reply。
module eth_arp_icmp_tx_gmii #(
    parameter [47:0] LOCAL_MAC = 48'h02_00_00_00_00_02,
    parameter [31:0] LOCAL_IP = 32'hc0_a8_32_02,
    parameter integer MAX_ICMP_BYTES = 64
) (
    input  wire                        clk_i,
    input  wire                        reset_p_i,
    input  wire                        request_valid_i,
    output wire                        request_ready_o,
    input  wire                        request_is_icmp_i,
    input  wire [47:0]                 peer_mac_i,
    input  wire [31:0]                 peer_ip_i,
    input  wire [15:0]                 ip_id_i,
    input  wire [15:0]                 icmp_length_i,
    input  wire [MAX_ICMP_BYTES*8-1:0] icmp_message_i,
    output wire                        busy_o,
    output reg                         done_o,
    output wire                        gmii_txen_o,
    output wire [7:0]                  gmii_txd_o
);
    localparam [3:0] IDLE = 4'd0;
    localparam [3:0] CHECKSUM_LENGTH = 4'd1;
    localparam [3:0] CHECKSUM_ID = 4'd2;
    localparam [3:0] CHECKSUM_DST_HIGH = 4'd3;
    localparam [3:0] CHECKSUM_DST_LOW = 4'd4;
    localparam [3:0] CHECKSUM_FOLD_ONCE = 4'd5;
    localparam [3:0] CHECKSUM_FOLD_TWICE = 4'd6;
    localparam [3:0] CHECKSUM_COMMIT = 4'd7;
    localparam [3:0] PREAMBLE = 4'd8;
    localparam [3:0] FRAME = 4'd9;
    localparam [3:0] CRC_BYTES = 4'd10;

    // IPv4 首部中的固定 16 位字预先求和；动态字每拍只累加一个，避免把
    // ICMP 长度、九项累加和两级折叠串成一个 125 MHz 组合关键路径。
    localparam [31:0] IPV4_STATIC_SUM =
        {16'd0, 16'h4500} + {16'd0, 16'h0000} + {16'd0, 16'h4001} +
        {16'd0, LOCAL_IP[31:16]} + {16'd0, LOCAL_IP[15:0]};

    reg [3:0] state_q;
    reg [3:0] preamble_index_q;
    reg [7:0] frame_index_q;
    reg [7:0] frame_lookup_index_q;
    reg [1:0] crc_index_q;

    reg is_icmp_q;
    reg [47:0] peer_mac_q;
    reg [31:0] peer_ip_q;
    reg [15:0] ip_id_q;
    reg [15:0] icmp_length_q;
    reg [MAX_ICMP_BYTES*8-1:0] icmp_message_shift_q;
    reg [15:0] ip_total_length_q;
    reg [15:0] payload_length_q;
    reg [15:0] ip_checksum_q;
    reg [15:0] icmp_checksum_q;
    reg [31:0] ip_checksum_sum_q;
    reg [16:0] ip_checksum_fold_q;

    reg [7:0] frame_byte_next;
    reg [7:0] frame_byte_q;
    reg [7:0] tx_byte;
    wire [15:0] frame_length = 16'd14 + payload_length_q;
    wire request_fire = request_valid_i && request_ready_o;
    wire crc_enable = state_q == FRAME;
    wire [31:0] crc_result;

    assign request_ready_o = state_q == IDLE;
    assign busy_o = state_q != IDLE;
    assign gmii_txen_o = state_q == PREAMBLE || state_q == FRAME ||
                         state_q == CRC_BYTES;
    assign gmii_txd_o = tx_byte;

    function [15:0] echo_reply_checksum;
        input [15:0] request_checksum;
        reg [16:0] adjusted;
        begin
            // type/code 由 0x0800 改成 0x0000，checksum 做一补码增量修正。
            adjusted = {1'b0, request_checksum} + 17'h00800;
            echo_reply_checksum = adjusted[15:0] + adjusted[16];
        end
    endfunction

    crc32_d8 frame_crc (
        .clk(clk_i), .reset_p(reset_p_i), .data(frame_byte_q),
        .crc_init(request_fire), .crc_en(crc_enable), .crc_result(crc_result)
    );

    always @(*) begin
        frame_byte_next = 8'h00;
        if (frame_lookup_index_q < 8'd6) begin
            case (frame_lookup_index_q)
                8'd0: frame_byte_next = peer_mac_q[47:40];
                8'd1: frame_byte_next = peer_mac_q[39:32];
                8'd2: frame_byte_next = peer_mac_q[31:24];
                8'd3: frame_byte_next = peer_mac_q[23:16];
                8'd4: frame_byte_next = peer_mac_q[15:8];
                default: frame_byte_next = peer_mac_q[7:0];
            endcase
        end else if (frame_lookup_index_q < 8'd12) begin
            case (frame_lookup_index_q)
                8'd6: frame_byte_next = LOCAL_MAC[47:40];
                8'd7: frame_byte_next = LOCAL_MAC[39:32];
                8'd8: frame_byte_next = LOCAL_MAC[31:24];
                8'd9: frame_byte_next = LOCAL_MAC[23:16];
                8'd10: frame_byte_next = LOCAL_MAC[15:8];
                default: frame_byte_next = LOCAL_MAC[7:0];
            endcase
        end else if (frame_lookup_index_q == 8'd12) begin
            frame_byte_next = 8'h08;
        end else if (frame_lookup_index_q == 8'd13) begin
            frame_byte_next = is_icmp_q ? 8'h00 : 8'h06;
        end else if (!is_icmp_q) begin
            case (frame_lookup_index_q - 8'd14)
                8'd0: frame_byte_next = 8'h00;
                8'd1: frame_byte_next = 8'h01;
                8'd2: frame_byte_next = 8'h08;
                8'd3: frame_byte_next = 8'h00;
                8'd4: frame_byte_next = 8'h06;
                8'd5: frame_byte_next = 8'h04;
                8'd6: frame_byte_next = 8'h00;
                8'd7: frame_byte_next = 8'h02;
                8'd8: frame_byte_next = LOCAL_MAC[47:40];
                8'd9: frame_byte_next = LOCAL_MAC[39:32];
                8'd10: frame_byte_next = LOCAL_MAC[31:24];
                8'd11: frame_byte_next = LOCAL_MAC[23:16];
                8'd12: frame_byte_next = LOCAL_MAC[15:8];
                8'd13: frame_byte_next = LOCAL_MAC[7:0];
                8'd14: frame_byte_next = LOCAL_IP[31:24];
                8'd15: frame_byte_next = LOCAL_IP[23:16];
                8'd16: frame_byte_next = LOCAL_IP[15:8];
                8'd17: frame_byte_next = LOCAL_IP[7:0];
                8'd18: frame_byte_next = peer_mac_q[47:40];
                8'd19: frame_byte_next = peer_mac_q[39:32];
                8'd20: frame_byte_next = peer_mac_q[31:24];
                8'd21: frame_byte_next = peer_mac_q[23:16];
                8'd22: frame_byte_next = peer_mac_q[15:8];
                8'd23: frame_byte_next = peer_mac_q[7:0];
                8'd24: frame_byte_next = peer_ip_q[31:24];
                8'd25: frame_byte_next = peer_ip_q[23:16];
                8'd26: frame_byte_next = peer_ip_q[15:8];
                8'd27: frame_byte_next = peer_ip_q[7:0];
                default: frame_byte_next = 8'h00;
            endcase
        end else if (frame_lookup_index_q < 8'd34) begin
            case (frame_lookup_index_q - 8'd14)
                8'd0: frame_byte_next = 8'h45;
                8'd1: frame_byte_next = 8'h00;
                8'd2: frame_byte_next = ip_total_length_q[15:8];
                8'd3: frame_byte_next = ip_total_length_q[7:0];
                8'd4: frame_byte_next = ip_id_q[15:8];
                8'd5: frame_byte_next = ip_id_q[7:0];
                8'd6: frame_byte_next = 8'h00;
                8'd7: frame_byte_next = 8'h00;
                8'd8: frame_byte_next = 8'h40;
                8'd9: frame_byte_next = 8'h01;
                8'd10: frame_byte_next = ip_checksum_q[15:8];
                8'd11: frame_byte_next = ip_checksum_q[7:0];
                8'd12: frame_byte_next = LOCAL_IP[31:24];
                8'd13: frame_byte_next = LOCAL_IP[23:16];
                8'd14: frame_byte_next = LOCAL_IP[15:8];
                8'd15: frame_byte_next = LOCAL_IP[7:0];
                8'd16: frame_byte_next = peer_ip_q[31:24];
                8'd17: frame_byte_next = peer_ip_q[23:16];
                8'd18: frame_byte_next = peer_ip_q[15:8];
                default: frame_byte_next = peer_ip_q[7:0];
            endcase
        end else if (frame_lookup_index_q < (8'd34 + icmp_length_q)) begin
            case (frame_lookup_index_q - 8'd34)
                8'd0: frame_byte_next = 8'h00;
                8'd1: frame_byte_next = 8'h00;
                8'd2: frame_byte_next = icmp_checksum_q[15:8];
                8'd3: frame_byte_next = icmp_checksum_q[7:0];
                default: frame_byte_next = frame_lookup_index_q >= 8'd39 ?
                    icmp_message_shift_q[MAX_ICMP_BYTES*8-9 -: 8] :
                    icmp_message_shift_q[MAX_ICMP_BYTES*8-1 -: 8];
            endcase
        end
    end

    always @(*) begin
        case (state_q)
            PREAMBLE: tx_byte = preamble_index_q == 4'd7 ? 8'hd5 : 8'h55;
            FRAME: tx_byte = frame_byte_q;
            CRC_BYTES: begin
                case (crc_index_q)
                    2'd0: tx_byte = crc_result[7:0];
                    2'd1: tx_byte = crc_result[15:8];
                    2'd2: tx_byte = crc_result[23:16];
                    default: tx_byte = crc_result[31:24];
                endcase
            end
            default: tx_byte = 8'h00;
        endcase
    end

    always @(posedge clk_i or posedge reset_p_i) begin
        if (reset_p_i) begin
            state_q <= IDLE;
            preamble_index_q <= 4'd0;
            frame_index_q <= 8'd0;
            frame_lookup_index_q <= 8'd0;
            crc_index_q <= 2'd0;
            is_icmp_q <= 1'b0;
            peer_mac_q <= 48'd0;
            peer_ip_q <= 32'd0;
            ip_id_q <= 16'd0;
            icmp_length_q <= 16'd0;
            icmp_message_shift_q <= {(MAX_ICMP_BYTES*8){1'b0}};
            frame_byte_q <= 8'd0;
            ip_total_length_q <= 16'd0;
            payload_length_q <= 16'd46;
            ip_checksum_q <= 16'd0;
            icmp_checksum_q <= 16'd0;
            ip_checksum_sum_q <= 32'd0;
            ip_checksum_fold_q <= 17'd0;
            done_o <= 1'b0;
        end else begin
            done_o <= 1'b0;
            case (state_q)
                IDLE: begin
                    if (request_fire) begin
                        is_icmp_q <= request_is_icmp_i;
                        peer_mac_q <= peer_mac_i;
                        peer_ip_q <= peer_ip_i;
                        ip_id_q <= ip_id_i;
                        icmp_length_q <= icmp_length_i;
                        // Reply 的前 4 字节单独生成；其余数据顺序移位，
                        // 避免 512 位变量索引直接进入 RGMII/CRC 关键路径。
                        icmp_message_shift_q <= {
                            icmp_message_i[MAX_ICMP_BYTES*8-33:0], 32'd0
                        };
                        ip_total_length_q <= 16'd20 + icmp_length_i;
                        payload_length_q <= request_is_icmp_i &&
                            ((16'd20 + icmp_length_i) > 16'd46) ?
                            (16'd20 + icmp_length_i) : 16'd46;
                        ip_checksum_sum_q <= IPV4_STATIC_SUM;
                        icmp_checksum_q <= echo_reply_checksum({
                            icmp_message_i[MAX_ICMP_BYTES*8-17 -: 8],
                            icmp_message_i[MAX_ICMP_BYTES*8-25 -: 8]
                        });
                        preamble_index_q <= 4'd0;
                        frame_lookup_index_q <= 8'd0;
                        state_q <= CHECKSUM_LENGTH;
                    end
                end
                CHECKSUM_LENGTH: begin
                    ip_checksum_sum_q <= ip_checksum_sum_q +
                                         {16'd0, ip_total_length_q};
                    state_q <= CHECKSUM_ID;
                end
                CHECKSUM_ID: begin
                    ip_checksum_sum_q <= ip_checksum_sum_q + {16'd0, ip_id_q};
                    state_q <= CHECKSUM_DST_HIGH;
                end
                CHECKSUM_DST_HIGH: begin
                    ip_checksum_sum_q <= ip_checksum_sum_q +
                                         {16'd0, peer_ip_q[31:16]};
                    state_q <= CHECKSUM_DST_LOW;
                end
                CHECKSUM_DST_LOW: begin
                    ip_checksum_sum_q <= ip_checksum_sum_q +
                                         {16'd0, peer_ip_q[15:0]};
                    state_q <= CHECKSUM_FOLD_ONCE;
                end
                CHECKSUM_FOLD_ONCE: begin
                    ip_checksum_fold_q <= ip_checksum_sum_q[31:16] +
                                          ip_checksum_sum_q[15:0];
                    state_q <= CHECKSUM_FOLD_TWICE;
                end
                CHECKSUM_FOLD_TWICE: begin
                    ip_checksum_fold_q <= ip_checksum_fold_q[15:0] +
                                          ip_checksum_fold_q[16];
                    state_q <= CHECKSUM_COMMIT;
                end
                CHECKSUM_COMMIT: begin
                    ip_checksum_q <= ~(ip_checksum_fold_q[15:0] +
                                       ip_checksum_fold_q[16]);
                    state_q <= PREAMBLE;
                end
                PREAMBLE: begin
                    if (preamble_index_q == 4'd7) begin
                        frame_index_q <= 8'd0;
                        frame_byte_q <= frame_byte_next;
                        frame_lookup_index_q <= 8'd1;
                        state_q <= FRAME;
                    end else begin
                        preamble_index_q <= preamble_index_q + 1'b1;
                    end
                end
                FRAME: begin
                    if (is_icmp_q && frame_index_q >= 8'd38 &&
                        frame_index_q < (8'd34 + icmp_length_q))
                        icmp_message_shift_q <= {
                            icmp_message_shift_q[MAX_ICMP_BYTES*8-9:0], 8'd0
                        };
                    if (frame_index_q == frame_length - 1'b1) begin
                        crc_index_q <= 2'd0;
                        state_q <= CRC_BYTES;
                    end
                    else begin
                        frame_index_q <= frame_index_q + 1'b1;
                        frame_byte_q <= frame_byte_next;
                        frame_lookup_index_q <= frame_lookup_index_q + 1'b1;
                    end
                end
                CRC_BYTES: begin
                    if (crc_index_q == 2'd3) begin
                        state_q <= IDLE;
                        done_o <= 1'b1;
                    end else begin
                        crc_index_q <= crc_index_q + 1'b1;
                    end
                end
                default: state_q <= IDLE;
            endcase
        end
    end

endmodule
