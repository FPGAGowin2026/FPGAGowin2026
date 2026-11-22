module hex1_udp_command_rx #(
    parameter [47:0] LOCAL_MAC = 48'h02_00_00_00_07_20,
    parameter [31:0] LOCAL_IP = 32'hC0_00_02_0A,
    parameter [31:0] BROADCAST_IP = 32'hC0_00_02_FF,
    parameter [15:0] LOCAL_PORT = 16'd6103
) (
    input  wire       clk50,
    input  wire       reset_n,
    input  wire       rgmii_rx_clk,
    input  wire [3:0] rgmii_rxd,
    input  wire       rgmii_rx_ctl,
    output reg        cmd_valid,
    output reg  [3:0] cmd,
    output wire [23:0] rx_debug
);
    localparam [31:0] ETH_CRC_RESIDUE = 32'h2144_DF1C;
    localparam [47:0] ETH_BROADCAST_MAC = 48'hFF_FF_FF_FF_FF_FF;

    (* ASYNC_REG = "TRUE" *) reg [1:0] rx_reset_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] clk50_reset_sync;
    wire rx_reset = rx_reset_sync[1];
    wire clk50_reset_n = clk50_reset_sync[1];

    // GW5A IDDR aligns both samples to the rising edge: Q0=low, Q1=high nibble.
    wire ctl_rise_sample, ctl_fall_sample;
    wire [4:0] rx_inputs = {rgmii_rx_ctl, rgmii_rxd};
    wire [4:0] rx_delayed;
    reg [1:0] ctl_q;
    reg [7:0] byte_q;
    wire ctl_rise = ctl_q[1];
    wire ctl_fall = ctl_q[0];
    wire [3:0] rgmii_rxd_q0, rgmii_rxd_q1;
    IDDR control_capture (
        .Q0 (ctl_rise_sample), .Q1 (ctl_fall_sample), .D(rx_delayed[4]), .CLK(rgmii_rx_clk)
    );
    genvar rx_lane;
    generate
        // Fixed matched input delay closes the actual RX hold deficit caused
        // by the pad-to-IDDR clock insertion, without changing PHY/SDC windows.
        for (rx_lane = 0; rx_lane < 5; rx_lane = rx_lane + 1) begin : rx_input_delay
            IODELAY #(.C_STATIC_DLY(32)) delay_input (
                .DI(rx_inputs[rx_lane]), .DO(rx_delayed[rx_lane]), .DF(),
                .SDTAP(1'b0), .VALUE(1'b0), .DLYSTEP(8'd0)
            );
        end
        for (rx_lane = 0; rx_lane < 4; rx_lane = rx_lane + 1) begin : rx_data_capture
            IDDR data_capture (
                .Q0(rgmii_rxd_q0[rx_lane]), .Q1(rgmii_rxd_q1[rx_lane]),
                .D(rx_delayed[rx_lane]), .CLK(rgmii_rx_clk)
            );
        end
    endgenerate

    // Move aligned bytes out of IOLOGIC before preamble/CRC decode fanout.
    always @(posedge rgmii_rx_clk or negedge reset_n) begin
        if (!reset_n) begin byte_q <= 0; ctl_q <= 0; end
        else begin
            byte_q <= {rgmii_rxd_q1, rgmii_rxd_q0};
            ctl_q <= {ctl_rise_sample, ctl_fall_sample};
        end
    end

    reg rx_in_frame;
    reg [3:0] preamble_count;
    reg sfd_seen;
    reg frame_bad;
    reg rx_error_seen;
    reg [6:0] frame_byte_count;
    reg [47:0] dst_mac;
    reg [15:0] ether_type;
    reg [159:0] ip_header;
    reg [15:0] udp_src_port;
    reg [15:0] udp_dst_port;
    reg [15:0] udp_length;
    reg [15:0] udp_checksum;
    reg [39:0] udp_payload;
    reg [5:0] dv_start_count;
    reg [5:0] sfd_count;
    reg [5:0] crc_length_count;
    reg [5:0] rx_clock_heartbeat;
    wire [7:0] rx_byte = byte_q;
    wire byte_valid = ctl_rise;
    // No idle byte exists between SFD and the destination MAC at 1 Gb/s.
    wire crc_init = rx_in_frame && byte_valid && !sfd_seen &&
                    preamble_count == 7 && rx_byte == 8'hD5;
    wire [31:0] crc_value;
    wire [5:0] dv_start_gray = dv_start_count ^ (dv_start_count >> 1);
    wire [5:0] sfd_gray = sfd_count ^ (sfd_count >> 1);
    wire [5:0] crc_length_gray = crc_length_count ^ (crc_length_count >> 1);
    wire [5:0] rx_clock_heartbeat_gray = rx_clock_heartbeat ^ (rx_clock_heartbeat >> 1);

    // 调试字节依次为 RX 时钟心跳、DV 起始、SFD 和 CRC/帧长通过计数，均为 Gray 编码。
    assign rx_debug = {rx_clock_heartbeat_gray, dv_start_gray, sfd_gray, crc_length_gray};

    crc32_d8 frame_crc (
        .clk(rgmii_rx_clk), .reset_p(rx_reset), .data(rx_byte),
        .crc_init(crc_init), .crc_en(sfd_seen && byte_valid),
        .crc_result(crc_value)
    );

    reg [16:0] ip_pairs [0:4], udp_pairs [0:5];
    reg [17:0] ip_groups [0:1], udp_groups [0:2];
    reg [16:0] ip_tail;
    reg [15:0] udp_tail_a, udp_tail_b;
    reg [19:0] ip_sum, udp_sum;
    reg [16:0] ip_fold, udp_fold;
    reg ip_checksum_ok, udp_checksum_ok;
    integer pair_index;

    // All fields are complete at byte 46. Five checksum stages settle before
    // byte 64/FCS acceptance, keeping the 125 MHz receive/command path short.
    always @(posedge rgmii_rx_clk or negedge reset_n) begin
        if (!reset_n) begin
            for (pair_index = 0; pair_index < 5; pair_index = pair_index + 1)
                ip_pairs[pair_index] <= 0;
            for (pair_index = 0; pair_index < 6; pair_index = pair_index + 1)
                udp_pairs[pair_index] <= 0;
            ip_groups[0] <= 0; ip_groups[1] <= 0;
            udp_groups[0] <= 0; udp_groups[1] <= 0; udp_groups[2] <= 0;
            ip_tail <= 0; udp_tail_a <= 0; udp_tail_b <= 0;
            ip_sum <= 0; udp_sum <= 0; ip_fold <= 0; udp_fold <= 0;
            ip_checksum_ok <= 0; udp_checksum_ok <= 0;
        end else begin
            for (pair_index = 0; pair_index < 5; pair_index = pair_index + 1)
                ip_pairs[pair_index] <= {1'b0, ip_header[159-pair_index*32 -: 16]} +
                                        {1'b0, ip_header[143-pair_index*32 -: 16]};
            udp_pairs[0] <= {1'b0, ip_header[63:48]} + {1'b0, ip_header[47:32]};
            udp_pairs[1] <= {1'b0, ip_header[31:16]} + {1'b0, ip_header[15:0]};
            udp_pairs[2] <= 17'h00011 + {1'b0, udp_length};
            udp_pairs[3] <= {1'b0, udp_src_port} + {1'b0, udp_dst_port};
            udp_pairs[4] <= {1'b0, udp_length} + {1'b0, udp_checksum};
            udp_pairs[5] <= {1'b0, udp_payload[39:24]} + {1'b0, udp_payload[23:8]};
            udp_tail_a <= {udp_payload[7:0], 8'd0};

            ip_groups[0] <= {1'b0, ip_pairs[0]} + {1'b0, ip_pairs[1]};
            ip_groups[1] <= {1'b0, ip_pairs[2]} + {1'b0, ip_pairs[3]};
            ip_tail <= ip_pairs[4];
            udp_groups[0] <= {1'b0, udp_pairs[0]} + {1'b0, udp_pairs[1]};
            udp_groups[1] <= {1'b0, udp_pairs[2]} + {1'b0, udp_pairs[3]};
            udp_groups[2] <= {1'b0, udp_pairs[4]} + {1'b0, udp_pairs[5]};
            udp_tail_b <= udp_tail_a;

            ip_sum <= ({2'd0, ip_groups[0]} + {2'd0, ip_groups[1]}) + {3'd0, ip_tail};
            udp_sum <= ({2'd0, udp_groups[0]} + {2'd0, udp_groups[1]}) +
                       ({2'd0, udp_groups[2]} + {4'd0, udp_tail_b});
            ip_fold <= {1'b0, ip_sum[15:0]} + {13'd0, ip_sum[19:16]};
            udp_fold <= {1'b0, udp_sum[15:0]} + {13'd0, udp_sum[19:16]};
            ip_checksum_ok <= ({1'b0, ip_fold[15:0]} + ip_fold[16]) == 17'h0FFFF;
            udp_checksum_ok <= ({1'b0, udp_fold[15:0]} + udp_fold[16]) == 17'h0FFFF;
        end
    end

    wire [31:0] packet_destination_ip = ip_header[31:0];
    wire mac_destination_ok =
        ((packet_destination_ip == LOCAL_IP) &&
         ((dst_mac == LOCAL_MAC) || (dst_mac == ETH_BROADCAST_MAC))) ||
        ((packet_destination_ip == BROADCAST_IP) &&
         (dst_mac == ETH_BROADCAST_MAC));
    wire packet_fields_match =
        mac_destination_ok && (ether_type == 16'h0800) &&
        (ip_header[159:152] == 8'h45) &&
        ({ip_header[143:136], ip_header[135:128]} == 16'd33) &&
        ((ip_header[111:96] & 16'hBFFF) == 16'd0) &&
        (ip_header[87:80] == 8'h11) &&
        ip_checksum_ok &&
        (udp_dst_port == LOCAL_PORT) &&
        (udp_length == 16'd13) &&
        (udp_payload[39:8] == 32'h48_45_58_31) &&
        (udp_payload[7:0] <= 8'd10) &&
        (udp_checksum == 0 || udp_checksum_ok);

    reg packet_fields_ok;
    always @(posedge rgmii_rx_clk or negedge reset_n)
        if (!reset_n) packet_fields_ok <= 0;
        else packet_fields_ok <= packet_fields_match;

    reg [3:0] mailbox_cmd;
    reg request_toggle_rx;
    (* ASYNC_REG = "TRUE" *) reg [1:0] acknowledge_sync_rx;
    reg acknowledge_toggle_50;
    (* ASYNC_REG = "TRUE" *) reg [1:0] request_sync_50;

    task reset_reception;
        begin
            rx_in_frame <= 1'b0;
            preamble_count <= 4'd0;
            sfd_seen <= 1'b0;
            frame_bad <= 1'b0;
            rx_error_seen <= 1'b0;
            frame_byte_count <= 7'd0;
            dst_mac <= 48'd0;
            ether_type <= 16'd0;
            ip_header <= 160'd0;
            udp_src_port <= 16'd0;
            udp_dst_port <= 16'd0;
            udp_length <= 16'd0;
            udp_checksum <= 16'd0;
            udp_payload <= 40'd0;
            dv_start_count <= 6'd0;
            sfd_count <= 6'd0;
            crc_length_count <= 6'd0;
            rx_clock_heartbeat <= 6'd0;
            mailbox_cmd <= 4'd0;
            request_toggle_rx <= 1'b0;
        end
    endtask

    always @(posedge rgmii_rx_clk or negedge reset_n) begin
        if (!reset_n) begin
            rx_reset_sync <= 2'b11;
            acknowledge_sync_rx <= 2'b00;
            reset_reception;
        end else begin
            rx_reset_sync <= {rx_reset_sync[0], 1'b0};
            acknowledge_sync_rx <= {acknowledge_sync_rx[0], acknowledge_toggle_50};

            if (rx_reset) begin
                reset_reception;
            end else begin
                rx_clock_heartbeat <= rx_clock_heartbeat + 1'b1;
                if (rx_in_frame && (ctl_rise ^ ctl_fall))
                    rx_error_seen <= 1'b1;

                if (!ctl_rise) begin
                    if (rx_in_frame) begin
                        if (sfd_seen &&
                            (frame_byte_count == 7'd64) &&
                            (crc_value == ETH_CRC_RESIDUE) &&
                            crc_length_count != 6'd63)
                            crc_length_count <= crc_length_count + 1'b1;
                        if (sfd_seen && !frame_bad && !rx_error_seen &&
                            !(ctl_rise ^ ctl_fall) &&
                            (frame_byte_count == 7'd64) &&
                            (crc_value == ETH_CRC_RESIDUE) && packet_fields_ok &&
                            (acknowledge_sync_rx[1] == request_toggle_rx)) begin
                            mailbox_cmd <= udp_payload[3:0];
                            request_toggle_rx <= !request_toggle_rx;
                        end
                        rx_in_frame <= 1'b0;
                        preamble_count <= 4'd0;
                        sfd_seen <= 1'b0;
                        frame_byte_count <= 7'd0;
                    end
                end else if (!rx_in_frame) begin
                    // Consume the first complete preamble byte on DV assertion.
                    if (dv_start_count != 6'd63)
                        dv_start_count <= dv_start_count + 1'b1;
                    rx_in_frame <= 1'b1;
                    preamble_count <= rx_byte == 8'h55 ? 4'd1 : 4'd0;
                    sfd_seen <= 1'b0;
                    frame_bad <= rx_byte != 8'h55;
                    rx_error_seen <= ctl_rise ^ ctl_fall;
                    frame_byte_count <= 7'd0;
                    dst_mac <= 48'd0;
                    ether_type <= 16'd0;
                    ip_header <= 160'd0;
                    udp_src_port <= 16'd0;
                    udp_dst_port <= 16'd0;
                    udp_length <= 16'd0;
                    udp_checksum <= 16'd0;
                    udp_payload <= 40'd0;
                end else begin
                    if (!sfd_seen) begin
                        if (rx_byte == 8'h55 && preamble_count < 4'd7) begin
                            preamble_count <= preamble_count + 1'b1;
                        end else if (rx_byte == 8'hD5 && preamble_count == 4'd7) begin
                            if (sfd_count != 6'd63)
                                sfd_count <= sfd_count + 1'b1;
                            sfd_seen <= 1'b1;
                            frame_byte_count <= 7'd0;
                        end else begin
                            frame_bad <= 1'b1;
                            preamble_count <= 4'd0;
                        end
                    end else begin
                        if (frame_byte_count != 7'h7F)
                            frame_byte_count <= frame_byte_count + 1'b1;
                        // Fields are consumed only at end of frame; assemble each in network order.
                        if (frame_byte_count < 6)
                            dst_mac <= {dst_mac[39:0], rx_byte};
                        else if (frame_byte_count >= 12 && frame_byte_count < 14)
                            ether_type <= {ether_type[7:0], rx_byte};
                        else if (frame_byte_count >= 14 && frame_byte_count < 34)
                            ip_header <= {ip_header[151:0], rx_byte};
                        else if (frame_byte_count >= 34 && frame_byte_count < 36)
                            udp_src_port <= {udp_src_port[7:0], rx_byte};
                        else if (frame_byte_count >= 36 && frame_byte_count < 38)
                            udp_dst_port <= {udp_dst_port[7:0], rx_byte};
                        else if (frame_byte_count >= 38 && frame_byte_count < 40)
                            udp_length <= {udp_length[7:0], rx_byte};
                        else if (frame_byte_count >= 40 && frame_byte_count < 42)
                            udp_checksum <= {udp_checksum[7:0], rx_byte};
                        else if (frame_byte_count >= 42 && frame_byte_count < 47)
                            udp_payload <= {udp_payload[31:0], rx_byte};
                    end
                end
            end
        end
    end

    always @(posedge clk50 or negedge reset_n) begin
        if (!reset_n) begin
            clk50_reset_sync <= 2'b00;
            request_sync_50 <= 2'b00;
            acknowledge_toggle_50 <= 1'b0;
            cmd_valid <= 1'b0;
            cmd <= 4'd0;
        end else begin
            clk50_reset_sync <= {clk50_reset_sync[0], 1'b1};
            if (!clk50_reset_n) begin
                request_sync_50 <= 2'b00;
                acknowledge_toggle_50 <= 1'b0;
                cmd_valid <= 1'b0;
                cmd <= 4'd0;
            end else begin
                request_sync_50 <= {request_sync_50[0], request_toggle_rx};
                cmd_valid <= 1'b0;
                if (request_sync_50[1] != acknowledge_toggle_50) begin
                    cmd <= mailbox_cmd;
                    cmd_valid <= 1'b1;
                    acknowledge_toggle_50 <= request_sync_50[1];
                end
            end
        end
    end
endmodule
