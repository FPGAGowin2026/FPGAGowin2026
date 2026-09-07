`timescale 1ns/1ps

module tb_fpga_control_packet_rx;
    reg clk_i = 1'b0;
    reg rst_n_i = 1'b0;
    reg payload_valid_i = 1'b0;
    reg [7:0] payload_byte_i = 8'd0;
    reg packet_done_i = 1'b0;
    reg packet_error_i = 1'b0;

    wire control_valid_o;
    wire packet_rejected_o;
    wire duplicate_sequence_o;
    wire [31:0] sequence_o;
    wire [63:0] timestamp_us_o;
    wire signed [31:0] linear_mm_s_o;
    wire signed [31:0] angular_mrad_s_o;
    wire estop_o;

    reg [7:0] packet [0:30];
    integer checks = 0;

    fpga_control_packet_rx dut (
        .clk_i(clk_i),
        .rst_n_i(rst_n_i),
        .payload_valid_i(payload_valid_i),
        .payload_byte_i(payload_byte_i),
        .packet_done_i(packet_done_i),
        .packet_error_i(packet_error_i),
        .control_valid_o(control_valid_o),
        .packet_rejected_o(packet_rejected_o),
        .duplicate_sequence_o(duplicate_sequence_o),
        .sequence_o(sequence_o),
        .timestamp_us_o(timestamp_us_o),
        .linear_mm_s_o(linear_mm_s_o),
        .angular_mrad_s_o(angular_mrad_s_o),
        .estop_o(estop_o)
    );

    always #5 clk_i = ~clk_i;

    function automatic [15:0] crc16_next;
        input [15:0] crc_in;
        input [7:0] data_in;
        integer bit_index;
        reg [15:0] crc;
        begin
            crc = crc_in ^ {data_in, 8'h00};
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1)
                crc = crc[15] ? ((crc << 1) ^ 16'h1021) : (crc << 1);
            crc16_next = crc;
        end
    endfunction

    task automatic build_packet;
        input [31:0] seq_value;
        input signed [31:0] linear;
        input signed [31:0] angular;
        input [7:0] flags;
        integer i;
        reg [15:0] crc;
        begin
            packet[0]=8'h46; packet[1]=8'h50; packet[2]=8'h47; packet[3]=8'h41;
            packet[4]=8'h01; packet[5]=8'h01; packet[6]=8'h00; packet[7]=8'h09;
            packet[8]=seq_value[31:24]; packet[9]=seq_value[23:16];
            packet[10]=seq_value[15:8]; packet[11]=seq_value[7:0];
            packet[12]=8'h01; packet[13]=8'h02; packet[14]=8'h03; packet[15]=8'h04;
            packet[16]=8'h05; packet[17]=8'h06; packet[18]=8'h07; packet[19]=8'h08;
            packet[20]=linear[31:24]; packet[21]=linear[23:16];
            packet[22]=linear[15:8]; packet[23]=linear[7:0];
            packet[24]=angular[31:24]; packet[25]=angular[23:16];
            packet[26]=angular[15:8]; packet[27]=angular[7:0];
            packet[28]=flags;
            crc = 16'hffff;
            for (i = 0; i < 29; i = i + 1)
                crc = crc16_next(crc, packet[i]);
            packet[29]=crc[15:8]; packet[30]=crc[7:0];
        end
    endtask

    task automatic send_packet;
        input bad_udp_fcs;
        begin
            send_packet_bytes(31, bad_udp_fcs);
        end
    endtask

    task automatic send_packet_bytes;
        input integer byte_count;
        input bad_udp_fcs;
        integer i;
        begin
            for (i = 0; i < byte_count; i = i + 1) begin
                @(negedge clk_i);
                payload_valid_i = 1'b1;
                payload_byte_i = (i < 31) ? packet[i] : 8'h00;
            end
            @(negedge clk_i);
            payload_valid_i = 1'b0;
            packet_error_i = bad_udp_fcs;
            packet_done_i = 1'b1;
            @(negedge clk_i);
            packet_done_i = 1'b0;
            packet_error_i = 1'b0;
        end
    endtask

    task automatic update_packet_crc;
        integer i;
        reg [15:0] crc;
        begin
            crc = 16'hffff;
            for (i = 0; i < 29; i = i + 1)
                crc = crc16_next(crc, packet[i]);
            packet[29] = crc[15:8];
            packet[30] = crc[7:0];
        end
    endtask

    task automatic expect_accept;
        input [31:0] seq_value;
        input signed [31:0] linear;
        input signed [31:0] angular;
        input expected_estop;
        begin
            checks = checks + 1;
            if (!control_valid_o || packet_rejected_o || sequence_o !== seq_value ||
                linear_mm_s_o !== linear || angular_mrad_s_o !== angular ||
                estop_o !== expected_estop) begin
                $display("FAIL accept check=%0d valid=%b rejected=%b seq=%h linear=%0d angular=%0d estop=%b",
                         checks, control_valid_o, packet_rejected_o, sequence_o,
                         linear_mm_s_o, angular_mrad_s_o, estop_o);
                $fatal(1);
            end
        end
    endtask

    initial begin
        repeat (3) @(posedge clk_i);
        rst_n_i = 1'b1;

        // 文档黄金向量，同时检查符号数、大端字节序和急停位。
        build_packet(32'h01020304, -32'sd1000, 32'sd2500, 8'h01);
        if (packet[29] !== 8'h85 || packet[30] !== 8'haa)
            $fatal(1, "FAIL golden CRC=%02x%02x", packet[29], packet[30]);
        send_packet(1'b0);
        #1 expect_accept(32'h01020304, -32'sd1000, 32'sd2500, 1'b1);

        // 相同序号不得覆盖最近一次命令。
        build_packet(32'h01020304, 32'sd999, 32'sd888, 8'h00);
        send_packet(1'b0);
        #1;
        checks = checks + 1;
        if (!packet_rejected_o || !duplicate_sequence_o || control_valid_o ||
            linear_mm_s_o !== -32'sd1000)
            $fatal(1, "FAIL duplicate sequence rejection");

        // 应用层 CRC 错误和以太网 FCS 错误都不得提交。
        build_packet(32'h01020305, 32'sd10, 32'sd20, 8'h00);
        packet[30] = packet[30] ^ 8'h01;
        send_packet(1'b0);
        #1;
        checks = checks + 1;
        if (!packet_rejected_o || control_valid_o)
            $fatal(1, "FAIL application CRC rejection");

        build_packet(32'h01020305, 32'sd10, 32'sd20, 8'h00);
        send_packet(1'b1);
        #1;
        checks = checks + 1;
        if (!packet_rejected_o || control_valid_o)
            $fatal(1, "FAIL ethernet FCS rejection");

        // 头部正确性和固定总长必须独立拒绝，且不改变最近一次有效命令。
        build_packet(32'h01020305, 32'sd10, 32'sd20, 8'h00);
        packet[4] = 8'h02;
        update_packet_crc();
        send_packet(1'b0);
        #1;
        checks = checks + 1;
        if (!packet_rejected_o || control_valid_o)
            $fatal(1, "FAIL header rejection");

        build_packet(32'h01020305, 32'sd10, 32'sd20, 8'h00);
        send_packet_bytes(30, 1'b0);
        #1;
        checks = checks + 1;
        if (!packet_rejected_o || control_valid_o)
            $fatal(1, "FAIL short packet rejection");

        send_packet_bytes(32, 1'b0);
        #1;
        checks = checks + 1;
        if (!packet_rejected_o || control_valid_o)
            $fatal(1, "FAIL long packet rejection");

        // 最近一次有效命令仍保持，下一更新序号可正常恢复。
        send_packet(1'b0);
        #1 expect_accept(32'h01020305, 32'sd10, 32'sd20, 1'b0);

        // 半范围序号规则允许 0xffffffff 后的 0，避免长时间运行后锁死。
        @(negedge clk_i); rst_n_i = 1'b0;
        @(negedge clk_i); rst_n_i = 1'b1;
        build_packet(32'hffffffff, 32'sd30, 32'sd40, 8'h00);
        send_packet(1'b0);
        #1 expect_accept(32'hffffffff, 32'sd30, 32'sd40, 1'b0);
        build_packet(32'h00000000, 32'sd50, 32'sd60, 8'h00);
        send_packet(1'b0);
        #1 expect_accept(32'h00000000, 32'sd50, 32'sd60, 1'b0);

        $display("PASS: fpga_control_packet_rx %0d checks", checks);
        $finish;
    end
endmodule
