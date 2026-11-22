/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: Max
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: eth_udp_tx_gmii
// Module Name: eth_udp_tx_gmii
// Project Name: eth_udp_tx_gmii
// Target Devices: XC7A35T-2FGG484I
// Tool Versions: Vivado 2018.3
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

// 沿用武汉芯路恒 eth_udp_tx_gmii 的调用接口与 IPv4/UDP 包格式。
module eth_udp_tx_gmii #(
    parameter ETH_type = 16'h0800,
    parameter IP_ver = 4'h4, IP_hdr_len = 4'h5, IP_tos = 8'h00,
    parameter IP_id = 16'h0000, IP_rsv = 1'b0, IP_df = 1'b0, IP_mf = 1'b0,
    parameter IP_frag_offset = 13'h0000, IP_ttl = 8'h40, IP_protocol = 8'h11
) (
    input wire clk125m, reset_p,
    input wire tx_en_pulse,
    output reg tx_done,
    input wire [47:0] dst_mac, src_mac,
    input wire [31:0] dst_ip, src_ip,
    input wire [15:0] dst_port, src_port, data_length,
    output wire payload_req_o,
    input wire [7:0] payload_dat_i,
    output wire gmii_tx_clk,
    output reg gmii_txen,
    output reg [7:0] gmii_txd
);
    localparam [2:0] IDLE = 0, PREAMBLE = 1, HEADER = 2,
                     PAYLOAD = 3, PADDING = 4, FCS = 5;
    reg [2:0] state;
    reg [15:0] byte_index, payload_length;
    reg [335:0] header;
    reg [7:0] tx_byte;
    reg [7:0] byte_q;
    reg valid_q, crc_en_q, fcs_q;
    reg [1:0] fcs_index_q;
    wire accept = state == IDLE && !valid_q && tx_en_pulse;
    wire [15:0] udp_length = data_length + 16'd8;
    wire [15:0] ip_length = data_length + 16'd28;
    wire [15:0] checksum;
    wire [31:0] crc;

    assign gmii_tx_clk = clk125m;
    assign payload_req_o = state == PAYLOAD;

    ip_checksum header_checksum (
        .clk(clk125m), .reset_p(reset_p), .cal_en(accept),
        .IP_ver(IP_ver), .IP_hdr_len(IP_hdr_len), .IP_tos(IP_tos),
        .IP_total_len(ip_length), .IP_id(IP_id), .IP_rsv(IP_rsv),
        .IP_df(IP_df), .IP_mf(IP_mf), .IP_frag_offset(IP_frag_offset),
        .IP_ttl(IP_ttl), .IP_protocol(IP_protocol),
        .src_ip(src_ip), .dst_ip(dst_ip), .checksum(checksum)
    );
    crc32_d8 frame_crc (
        .clk(clk125m), .reset_p(reset_p), .data(byte_q), .crc_init(accept),
        .crc_en(crc_en_q),
        .crc_result(crc)
    );

    always @* begin
        case (state)
            PREAMBLE: tx_byte = byte_index == 7 ? 8'hD5 : 8'h55;
            HEADER: tx_byte = header[335 - {byte_index[5:0], 3'b000} -: 8];
            PAYLOAD: tx_byte = payload_dat_i;
            default: tx_byte = 0;
        endcase
    end

    always @(posedge clk125m or posedge reset_p) begin
        if (reset_p) begin
            state <= IDLE;
            byte_index <= 0;
            payload_length <= 0;
            header <= 0;
            tx_done <= 0;
            gmii_txen <= 0;
            gmii_txd <= 0;
            byte_q <= 0; valid_q <= 0; crc_en_q <= 0; fcs_q <= 0; fcs_index_q <= 0;
        end else begin
            // Register header/payload selection before the CRC network. FCS is
            // selected after the final registered data byte has entered CRC.
            byte_q <= tx_byte;
            valid_q <= state != IDLE;
            crc_en_q <= state == HEADER || state == PAYLOAD || state == PADDING;
            fcs_q <= state == FCS;
            fcs_index_q <= byte_index[1:0];
            gmii_txen <= valid_q;
            gmii_txd <= fcs_q ? crc[{fcs_index_q, 3'b000} +: 8] : byte_q;
            tx_done <= valid_q && fcs_q && fcs_index_q == 3;
            case (state)
                IDLE: if (accept) begin
                    payload_length <= data_length;
                    header <= {
                        dst_mac, src_mac, ETH_type,
                        IP_ver, IP_hdr_len, IP_tos, ip_length, IP_id,
                        IP_rsv, IP_df, IP_mf, IP_frag_offset, IP_ttl, IP_protocol,
                        16'd0, src_ip, dst_ip,
                        src_port, dst_port, udp_length, 16'd0
                    };
                    byte_index <= 0;
                    state <= PREAMBLE;
                end
                PREAMBLE: begin
                    // 校验器比包头寄存晚一拍；前导码期间补入结果。
                    if (byte_index == 0)
                        header[143:128] <= checksum;
                    if (byte_index == 7) begin
                        byte_index <= 0;
                        state <= HEADER;
                    end else
                        byte_index <= byte_index + 1'b1;
                end
                HEADER: begin
                    if (byte_index == 41) begin
                        byte_index <= 0;
                        state <= payload_length == 0 ? PADDING : PAYLOAD;
                    end else
                        byte_index <= byte_index + 1'b1;
                end
                PAYLOAD: begin
                    if (byte_index == payload_length - 1'b1) begin
                        if (payload_length < 18) begin
                            byte_index <= payload_length;
                            state <= PADDING;
                        end else begin
                            byte_index <= 0;
                            state <= FCS;
                        end
                    end else
                        byte_index <= byte_index + 1'b1;
                end
                PADDING: begin
                    if (byte_index == 17) begin
                        byte_index <= 0;
                        state <= FCS;
                    end else
                        byte_index <= byte_index + 1'b1;
                end
                FCS: begin
                    if (byte_index == 3) begin
                        state <= IDLE;
                    end else
                        byte_index <= byte_index + 1'b1;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
