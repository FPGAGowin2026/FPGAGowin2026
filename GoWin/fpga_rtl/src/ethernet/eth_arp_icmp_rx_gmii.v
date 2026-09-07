`timescale 1ns/1ps

// 网络域最小辅助协议接收器：只接受发往本机的 ARP Request 与
// IPv4/ICMP Echo Request。完整 FCS、IP checksum、ICMP checksum 通过后
// 才提交一条应答记录；不实现 ARP 缓存、IP options、分片或其他 ICMP。
module eth_arp_icmp_rx_gmii #(
    parameter [47:0] LOCAL_MAC = 48'h02_00_00_00_00_02,
    parameter [31:0] LOCAL_IP = 32'hc0_a8_32_02,
    parameter integer MAX_ICMP_BYTES = 64,
    parameter integer MAX_FRAME_BYTES = 160
) (
    input  wire                            clk_i,
    input  wire                            reset_p_i,
    input  wire                            gmii_rxdv_i,
    input  wire [7:0]                      gmii_rxd_i,

    output reg                             reply_valid_o,
    input  wire                            reply_ready_i,
    output reg                             reply_is_icmp_o,
    output reg  [47:0]                     peer_mac_o,
    output reg  [31:0]                     peer_ip_o,
    output reg  [15:0]                     ip_id_o,
    output reg  [15:0]                     icmp_length_o,
    output reg  [MAX_ICMP_BYTES*8-1:0]     icmp_message_o,
    output reg                             reply_overflow_o
);
    localparam [2:0] IDLE = 3'd0;
    localparam [2:0] PREAMBLE = 3'd1;
    localparam [2:0] FRAME = 3'd2;
    localparam [2:0] CHECKSUM_FOLD = 3'd3;
    localparam [2:0] CHECK_RESULT = 3'd4;
    localparam [2:0] COMMIT = 3'd5;

    reg [2:0] state_q;
    reg [3:0] preamble_count_q;
    reg [7:0] frame_count_q;
    reg frame_overflow_q;
    reg [7:0] frame_mem [0:MAX_FRAME_BYTES-1];

    reg [7:0] rxd_d1_q;
    reg [7:0] rxd_d2_q;
    reg rxdv_d1_q;
    reg rxdv_d2_q;

    reg [31:0] ip_checksum_sum_q;
    reg [7:0] ip_checksum_high_q;
    reg [31:0] icmp_checksum_sum_q;
    reg [7:0] icmp_checksum_high_q;
    reg icmp_checksum_odd_q;
    reg [16:0] ip_checksum_fold_q;
    reg [16:0] icmp_checksum_fold_q;
    reg pending_valid_q;
    reg pending_is_icmp_q;

    wire [15:0] eth_type = {frame_mem[12], frame_mem[13]};
    wire [15:0] ip_total_length = {frame_mem[16], frame_mem[17]};
    wire [15:0] icmp_length = ip_total_length - 16'd20;
    wire [31:0] icmp_checksum_sum_final = icmp_checksum_sum_q +
        (icmp_checksum_odd_q ? {icmp_checksum_high_q, 8'h00} : 16'd0);

    wire arp_request_ok =
        frame_count_q >= 8'd64 &&
        ({frame_mem[0], frame_mem[1], frame_mem[2], frame_mem[3],
          frame_mem[4], frame_mem[5]} == LOCAL_MAC ||
         {frame_mem[0], frame_mem[1], frame_mem[2], frame_mem[3],
          frame_mem[4], frame_mem[5]} == 48'hff_ff_ff_ff_ff_ff) &&
        eth_type == 16'h0806 &&
        {frame_mem[14], frame_mem[15]} == 16'h0001 &&
        {frame_mem[16], frame_mem[17]} == 16'h0800 &&
        frame_mem[18] == 8'd6 && frame_mem[19] == 8'd4 &&
        {frame_mem[20], frame_mem[21]} == 16'h0001 &&
        {frame_mem[28], frame_mem[29], frame_mem[30], frame_mem[31]} != LOCAL_IP &&
        {frame_mem[38], frame_mem[39], frame_mem[40], frame_mem[41]} == LOCAL_IP;

    wire icmp_header_ok =
        frame_count_q >= 8'd64 &&
        {frame_mem[0], frame_mem[1], frame_mem[2], frame_mem[3],
         frame_mem[4], frame_mem[5]} == LOCAL_MAC &&
        eth_type == 16'h0800 && frame_mem[14] == 8'h45 &&
        (frame_mem[20] & 8'hbf) == 8'h00 && frame_mem[21] == 8'h00 &&
        frame_mem[23] == 8'h01 &&
        {frame_mem[30], frame_mem[31], frame_mem[32], frame_mem[33]} == LOCAL_IP &&
        ip_total_length >= 16'd28 && icmp_length <= MAX_ICMP_BYTES &&
        frame_count_q >= (ip_total_length + 16'd18) &&
        frame_mem[34] == 8'h08 && frame_mem[35] == 8'h00;

    wire [16:0] ip_checksum_fold_final =
        ip_checksum_fold_q[15:0] + ip_checksum_fold_q[16];
    wire [16:0] icmp_checksum_fold_final =
        icmp_checksum_fold_q[15:0] + icmp_checksum_fold_q[16];
    wire ip_checksum_ok = ip_checksum_fold_final[15:0] == 16'hffff;
    wire icmp_checksum_ok = icmp_checksum_fold_final[15:0] == 16'hffff;

    wire crc_init = rxdv_d1_q && !rxdv_d2_q;
    wire crc_enable = state_q == FRAME && rxdv_d2_q;
    wire [31:0] crc_result;

    integer copy_index;

    crc32_d8 frame_crc (
        .clk(clk_i), .reset_p(reset_p_i), .data(rxd_d2_q),
        .crc_init(crc_init), .crc_en(crc_enable), .crc_result(crc_result)
    );

    always @(posedge clk_i or posedge reset_p_i) begin
        if (reset_p_i) begin
            rxd_d1_q <= 8'd0;
            rxd_d2_q <= 8'd0;
            rxdv_d1_q <= 1'b0;
            rxdv_d2_q <= 1'b0;
        end else begin
            rxd_d1_q <= gmii_rxd_i;
            rxd_d2_q <= rxd_d1_q;
            rxdv_d1_q <= gmii_rxdv_i;
            rxdv_d2_q <= rxdv_d1_q;
        end
    end

    always @(posedge clk_i or posedge reset_p_i) begin
        if (reset_p_i) begin
            state_q <= IDLE;
            preamble_count_q <= 4'd0;
        end else begin
            case (state_q)
                IDLE: begin
                    preamble_count_q <= 4'd0;
                    if (!rxdv_d2_q && rxdv_d1_q)
                        state_q <= PREAMBLE;
                end
                PREAMBLE: begin
                    if (!rxdv_d2_q) begin
                        state_q <= IDLE;
                    end else if (rxd_d2_q == 8'h55) begin
                        preamble_count_q <= preamble_count_q + 1'b1;
                    end else if (rxd_d2_q == 8'hd5 && preamble_count_q >= 4'd6) begin
                        state_q <= FRAME;
                    end else begin
                        state_q <= IDLE;
                    end
                end
                FRAME: begin
                    if (!rxdv_d2_q)
                        state_q <= CHECKSUM_FOLD;
                end
                CHECKSUM_FOLD: state_q <= CHECK_RESULT;
                CHECK_RESULT: state_q <= COMMIT;
                COMMIT: state_q <= IDLE;
                default: state_q <= IDLE;
            endcase
        end
    end

    always @(posedge clk_i or posedge reset_p_i) begin
        if (reset_p_i) begin
            frame_count_q <= 8'd0;
            frame_overflow_q <= 1'b0;
            ip_checksum_sum_q <= 32'd0;
            ip_checksum_high_q <= 8'd0;
            icmp_checksum_sum_q <= 32'd0;
            icmp_checksum_high_q <= 8'd0;
            icmp_checksum_odd_q <= 1'b0;
            ip_checksum_fold_q <= 17'd0;
            icmp_checksum_fold_q <= 17'd0;
        end else if (state_q == IDLE || state_q == PREAMBLE) begin
            frame_count_q <= 8'd0;
            frame_overflow_q <= 1'b0;
            ip_checksum_sum_q <= 32'd0;
            icmp_checksum_sum_q <= 32'd0;
            icmp_checksum_odd_q <= 1'b0;
        end else if (state_q == FRAME && rxdv_d2_q) begin
            if (frame_count_q < MAX_FRAME_BYTES) begin
                frame_mem[frame_count_q] <= rxd_d2_q;
                frame_count_q <= frame_count_q + 1'b1;
            end else begin
                frame_overflow_q <= 1'b1;
            end

            if (frame_count_q >= 8'd14 && frame_count_q < 8'd34) begin
                if (!frame_count_q[0])
                    ip_checksum_high_q <= rxd_d2_q;
                else
                    ip_checksum_sum_q <= ip_checksum_sum_q +
                                         {ip_checksum_high_q, rxd_d2_q};
            end

            if (frame_count_q == 8'd34) begin
                icmp_checksum_sum_q <= 32'd0;
                icmp_checksum_high_q <= rxd_d2_q;
                icmp_checksum_odd_q <= 1'b1;
            end else if (frame_count_q > 8'd34 &&
                         frame_count_q < (ip_total_length + 16'd14)) begin
                if (icmp_checksum_odd_q) begin
                    icmp_checksum_sum_q <= icmp_checksum_sum_q +
                                           {icmp_checksum_high_q, rxd_d2_q};
                    icmp_checksum_odd_q <= 1'b0;
                end else begin
                    icmp_checksum_high_q <= rxd_d2_q;
                    icmp_checksum_odd_q <= 1'b1;
                end
            end
        end else if (state_q == CHECKSUM_FOLD) begin
            // 奇数字节补齐后先做第一层折叠，下一拍再判定整帧合法性。
            ip_checksum_fold_q <= ip_checksum_sum_q[31:16] +
                                  ip_checksum_sum_q[15:0];
            icmp_checksum_fold_q <= icmp_checksum_sum_final[31:16] +
                                    icmp_checksum_sum_final[15:0];
        end
    end

    always @(posedge clk_i or posedge reset_p_i) begin
        if (reset_p_i) begin
            reply_valid_o <= 1'b0;
            reply_is_icmp_o <= 1'b0;
            peer_mac_o <= 48'd0;
            peer_ip_o <= 32'd0;
            ip_id_o <= 16'd0;
            icmp_length_o <= 16'd0;
            icmp_message_o <= {(MAX_ICMP_BYTES*8){1'b0}};
            reply_overflow_o <= 1'b0;
            pending_valid_q <= 1'b0;
            pending_is_icmp_q <= 1'b0;
        end else begin
            reply_overflow_o <= 1'b0;
            if (reply_valid_o && reply_ready_i)
                reply_valid_o <= 1'b0;

            if (state_q == CHECK_RESULT) begin
                pending_is_icmp_q <= icmp_header_ok && ip_checksum_ok &&
                                     icmp_checksum_ok;
                pending_valid_q <= !frame_overflow_q &&
                    crc_result == 32'h2144df1c &&
                    (arp_request_ok ||
                     (icmp_header_ok && ip_checksum_ok && icmp_checksum_ok));
            end else if (state_q == COMMIT) begin
                pending_valid_q <= 1'b0;
                if (pending_valid_q) begin
                    if (!reply_valid_o || reply_ready_i) begin
                        reply_valid_o <= 1'b1;
                        reply_is_icmp_o <= pending_is_icmp_q;
                        peer_mac_o <= {frame_mem[6], frame_mem[7], frame_mem[8],
                                       frame_mem[9], frame_mem[10], frame_mem[11]};
                        peer_ip_o <= pending_is_icmp_q ?
                            {frame_mem[26], frame_mem[27], frame_mem[28], frame_mem[29]} :
                            {frame_mem[28], frame_mem[29], frame_mem[30], frame_mem[31]};
                        ip_id_o <= {frame_mem[18], frame_mem[19]};
                        icmp_length_o <= pending_is_icmp_q ? icmp_length : 16'd0;
                        for (copy_index = 0; copy_index < MAX_ICMP_BYTES;
                             copy_index = copy_index + 1) begin
                            // 长度随记录一并提交；TX 不会读取长度之外的缓存字节。
                            if (pending_is_icmp_q)
                                icmp_message_o[MAX_ICMP_BYTES*8-1-copy_index*8 -: 8]
                                    <= frame_mem[34+copy_index];
                            else
                                icmp_message_o[MAX_ICMP_BYTES*8-1-copy_index*8 -: 8]
                                    <= 8'h00;
                        end
                    end else begin
                        reply_overflow_o <= 1'b1;
                    end
                end
            end
        end
    end

endmodule
