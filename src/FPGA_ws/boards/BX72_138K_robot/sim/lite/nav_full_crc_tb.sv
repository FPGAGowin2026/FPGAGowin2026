`timescale 1ns/1ps

// 对生产 CRC32、应用 CRC16 和 IPv4 头校验做独立逐位/折叠参考比对。
module nav_full_crc_tb;
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg reset_p = 1'b1;

    reg [7:0] crc32_data = 8'd0;
    reg crc32_init = 1'b0;
    reg crc32_en = 1'b0;
    wire [31:0] crc32_result;
    reg [31:0] crc32_reference = 32'hffff_ffff;
    reg [31:0] lfsr = 32'h72c5_184b;
    reg [7:0] sample_byte;
    integer index;
    integer bit_index;

    crc32_d8 production_crc32 (
        .clk(clk), .reset_p(reset_p), .data(crc32_data),
        .crc_init(crc32_init), .crc_en(crc32_en), .crc_result(crc32_result)
    );

    reg [15:0] app_crc16_reference = 16'hffff;
    reg [15:0] app_crc16_parallel;
    reg [15:0] app_crc16_serial;
    reg [15:0] app_crc16_work;

    // 网络输入保持空闲；层次函数调用直接检验生产CRC16函数。
    wire reply_valid, control_valid, auto_valid, heartbeat_valid;
    wire [255:0] reply_data, task_data;
    wire [127:0] diagnostics;
    wire [47:0] velocity;
    wire auto_enable;
    wire [63:0] control_token;
    wire good_toggle;
    wire [63:0] raw_prefix;
    bx72_udp_request_rx app_checker (
        .reset_p(reset_p), .rx_clk(clk), .gmii_rxd(8'd0), .gmii_rxdv(1'b0), .gmii_rxer(1'b0),
        .tx_clk(clk), .reply_ready(1'b0), .reply_valid(reply_valid), .reply_data(reply_data),
        .diagnostics_tx_o(diagnostics), .control_clk(clk), .control_valid_o(control_valid),
        .control_velocity_o(velocity), .auto_mode_valid_o(auto_valid), .auto_enable_o(auto_enable),
        .task_fragment_valid_o(), .task_fragment_o(task_data),
        .control_token_o(control_token), .control_heartbeat_o(heartbeat_valid),
        .training_reset_p_i(reset_p), .good_toggle_o(good_toggle), .raw_prefix_tx_o(raw_prefix)
    );

    reg ip_cal_en = 1'b0;
    reg [3:0] ip_version = 4'd4;
    reg [3:0] ip_header_length = 4'd5;
    reg [7:0] ip_tos = 8'd0;
    reg [15:0] ip_total_length = 16'd0;
    reg [15:0] ip_id = 16'd0;
    reg ip_reserved = 1'b0, ip_df = 1'b0, ip_mf = 1'b0;
    reg [12:0] ip_fragment_offset = 13'd0;
    reg [7:0] ip_ttl = 8'd64;
    reg [7:0] ip_protocol = 8'd17;
    reg [31:0] ip_source = 32'd0, ip_destination = 32'd0;
    wire [15:0] ip_checksum;
    wire ip_checksum_done;

    ip_checksum production_ip_checksum (
        .clk(clk), .reset_p(reset_p), .cal_en(ip_cal_en),
        .IP_ver(ip_version), .IP_hdr_len(ip_header_length), .IP_tos(ip_tos),
        .IP_total_len(ip_total_length), .IP_id(ip_id), .IP_rsv(ip_reserved),
        .IP_df(ip_df), .IP_mf(ip_mf), .IP_frag_offset(ip_fragment_offset),
        .IP_ttl(ip_ttl), .IP_protocol(ip_protocol),
        .src_ip(ip_source), .dst_ip(ip_destination), .checksum(ip_checksum),
        .cal_done_o(ip_checksum_done)
    );

    function [31:0] crc32_reference_step;
        input [31:0] current;
        input [7:0] value;
        reg [31:0] work;
        integer bit_number;
        begin
            work = current ^ {24'd0, value};
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1)
                work = work[0] ? ((work >> 1) ^ 32'hedb8_8320) : (work >> 1);
            crc32_reference_step = work;
        end
    endfunction

    function [15:0] crc16_reference_step;
        input [15:0] current;
        input [7:0] value;
        reg [15:0] work;
        integer bit_number;
        begin
            work = current ^ {value, 8'd0};
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1)
                work = work[15] ? ((work << 1) ^ 16'h1021) : (work << 1);
            crc16_reference_step = work;
        end
    endfunction

    function [15:0] ipv4_checksum_reference;
        input [3:0] version;
        input [3:0] header_length;
        input [7:0] tos;
        input [15:0] total_length;
        input [15:0] identification;
        input reserved_bit, dont_fragment, more_fragments;
        input [12:0] fragment_offset;
        input [7:0] ttl;
        input [7:0] protocol;
        input [31:0] source_address;
        input [31:0] destination_address;
        reg [31:0] sum;
        begin
            sum = {version, header_length, tos} + total_length + identification +
                  {reserved_bit, dont_fragment, more_fragments, fragment_offset} +
                  {ttl, protocol} + source_address[31:16] + source_address[15:0] +
                  destination_address[31:16] + destination_address[15:0];
            while (sum[31:16] != 0)
                sum = {16'd0, sum[15:0]} + sum[31:16];
            ipv4_checksum_reference = ~sum[15:0];
        end
    endfunction

    task automatic check_ip_checksum;
        input [3:0] version;
        input [3:0] header_length;
        input [7:0] tos;
        input [15:0] total_length;
        input [15:0] identification;
        input reserved_bit, dont_fragment, more_fragments;
        input [12:0] fragment_offset;
        input [7:0] ttl;
        input [7:0] protocol;
        input [31:0] source_address;
        input [31:0] destination_address;
        reg [15:0] expected;
        begin
            expected = ipv4_checksum_reference(version, header_length, tos, total_length,
                identification, reserved_bit, dont_fragment, more_fragments,
                fragment_offset, ttl, protocol, source_address, destination_address);
            @(negedge clk);
            ip_version = version;
            ip_header_length = header_length;
            ip_tos = tos;
            ip_total_length = total_length;
            ip_id = identification;
            ip_reserved = reserved_bit;
            ip_df = dont_fragment;
            ip_mf = more_fragments;
            ip_fragment_offset = fragment_offset;
            ip_ttl = ttl;
            ip_protocol = protocol;
            ip_source = source_address;
            ip_destination = destination_address;
            ip_cal_en = 1'b1;
            @(posedge clk); #1 ip_cal_en = 1'b0;
            if (ip_checksum_done)
                $fatal(1, "IP checksum valid asserted on the input-pair stage");
            // 新的pair输入寄存级使结果在cal_en采样后的第4个沿有效。
            repeat (2) begin
                @(posedge clk); #1;
                if (ip_checksum_done)
                    $fatal(1, "IP checksum valid asserted before the result stage");
            end
            @(posedge clk); #1;
            if (!ip_checksum_done)
                $fatal(1, "IP checksum valid pulse missing");
            if (ip_checksum !== expected)
                $fatal(1, "IP checksum mismatch got=%h expected=%h", ip_checksum, expected);
            @(posedge clk); #1;
            if (ip_checksum_done)
                $fatal(1, "IP checksum valid must be a one-cycle pulse");
        end
    endtask

    initial begin
        repeat (3) @(posedge clk);
        #1 reset_p = 1'b0;

        @(negedge clk); crc32_init = 1'b1;
        @(posedge clk); #1 crc32_init = 1'b0;
        crc32_reference = 32'hffff_ffff;
        for (index = 0; index < 512; index = index + 1) begin
            @(negedge clk);
            sample_byte = lfsr[7:0];
            lfsr = {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};
            crc32_data = sample_byte;
            crc32_en = 1'b1;
            crc32_reference = crc32_reference_step(crc32_reference, sample_byte);
            @(posedge clk); #1;
            if (crc32_result !== ~crc32_reference)
                $fatal(1, "CRC32 byte %0d mismatch got=%h expected=%h",
                       index, crc32_result, ~crc32_reference);

            app_crc16_parallel = app_checker.crc_byte(app_crc16_reference, sample_byte);
            app_crc16_serial = crc16_reference_step(app_crc16_reference, sample_byte);
            if (app_crc16_parallel !== app_crc16_serial)
                $fatal(1, "application CRC16 byte %0d mismatch got=%h expected=%h",
                       index, app_crc16_parallel, app_crc16_serial);
            app_crc16_reference = app_crc16_serial;
        end
        @(negedge clk); crc32_en = 1'b0;

        check_ip_checksum(4'hf, 4'hf, 8'hff, 16'hffff, 16'hffff,
            1'b1, 1'b1, 1'b1, 13'h1fff, 8'hff, 8'hff, 32'hffff_ffff, 32'hffff_ffff);
        check_ip_checksum(4'd4, 4'd5, 8'hb7, 16'hffff, 16'hffff,
            1'b1, 1'b1, 1'b1, 13'h1fff, 8'hff, 8'hff, 32'hffff_ffff, 32'hffff_ffff);
        check_ip_checksum(4'd4, 4'd5, 8'h2e, 16'h1234, 16'habcd,
            1'b0, 1'b1, 1'b0, 13'h1a35, 8'h40, 8'h11,
            32'hc0a8_0004, 32'he000_0101);
        check_ip_checksum(4'd4, 4'd5, 8'h00, 16'h05dc, 16'h0042,
            1'b0, 1'b0, 1'b0, 13'h0000, 8'h80, 8'h06,
            32'h0a00_0001, 32'h0a00_00fe);

        $display("PASS: nav_full_crc_tb CRC32=512bytes CRC16=512bytes checksum=4headers");
        $finish;
    end
endmodule
