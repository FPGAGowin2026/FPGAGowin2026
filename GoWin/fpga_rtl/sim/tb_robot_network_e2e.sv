`timescale 1ns/1ps

module Gowin_PLL(
    input  wire clkin,
    input  wire init_clk,
    output wire clkout0
);
    assign clkout0 = clkin;
endmodule

module tb_robot_network_e2e;
    localparam [47:0] LOCAL_MAC = 48'h02_00_00_00_00_02;
    localparam [31:0] LOCAL_IP = 32'hc0_a8_32_02;
    localparam [47:0] HOST_MAC = 48'h02_00_00_00_00_01;
    localparam [31:0] HOST_IP = 32'hc0_a8_32_01;
    localparam integer FAULT_NONE = 0;
    localparam integer FAULT_FCS = 1;
    localparam integer FAULT_IP = 2;
    localparam integer FAULT_UDP = 3;
    localparam integer FAULT_APP = 4;
    localparam integer FAULT_MAC = 5;
    localparam integer FAULT_DEST_IP = 6;
    localparam integer FAULT_PORT = 7;

    reg reset_n = 1'b0;
    reg system_clk_i = 1'b0;
    reg rgmii_rx_clk_i = 1'b0;
    reg system_clock_enable = 1'b1;
    reg network_clock_enable = 1'b1;
    reg [3:0] rgmii_rxd = 4'd0;
    reg rgmii_rxdv = 1'b0;
    wire eth_rst_n;
    wire rgmii_tx_clk;
    wire [3:0] rgmii_txd;
    wire rgmii_txen;

    reg requested_mode_i = 1'b1;
    reg motion_stopped_i = 1'b1;
    reg physical_estop_i = 1'b0;
    reg critical_fault_i = 1'b0;
    reg front_sample_valid_i = 1'b0;
    reg [15:0] front_distance_mm_i = 16'd1000;
    reg left_sample_valid_i = 1'b0;
    reg [15:0] left_distance_mm_i = 16'd1100;
    reg right_sample_valid_i = 1'b0;
    reg [15:0] right_distance_mm_i = 16'd1200;
    reg imu_sample_start_i = 1'b0;
    reg imu_byte_valid_i = 1'b0;
    reg [7:0] imu_byte_data_i = 8'd0;
    reg path_start_i = 1'b0;
    reg path_abort_i = 1'b0;
    reg avoidance_tick_i = 1'b0;
    reg [31:0] segment_progress_i = 32'd0;
    reg motor_sample_tick_i = 1'b0;
    reg signed [31:0] left_ticks_i = -32'sd123;
    reg signed [31:0] right_ticks_i = 32'sd456;
    reg signed [31:0] left_velocity_mm_s_i = 32'sd111;
    reg signed [31:0] right_velocity_mm_s_i = 32'sd222;
    reg signed [31:0] accel_x_mm_s2_i = -32'sd10;
    reg signed [31:0] accel_y_mm_s2_i = 32'sd20;
    reg signed [31:0] accel_z_mm_s2_i = 32'sd30;
    reg signed [31:0] gyro_x_mrad_s_i = -32'sd40;
    reg signed [31:0] gyro_y_mrad_s_i = 32'sd50;
    reg signed [31:0] gyro_z_mrad_s_i = 32'sd60;

    wire control_packet_accepted_o;
    wire control_packet_rejected_o;
    wire motion_enabled_o;
    wire signed [31:0] final_linear_mm_s_o;
    wire signed [31:0] final_angular_mrad_s_o;
    wire software_estop_o;
    wire control_timeout_pulse_o;
    wire [31:0] status_bits_o;
    wire [31:0] fault_bits_o;
    wire rx_control_fifo_full_o;
    wire tx_frame_fifo_full_o;

    reg [7:0] host_frame [0:255];
    integer host_frame_length;
    reg [7:0] tx_capture [0:255];
    integer tx_count = 0;
    integer captured_length = 0;
    integer completed_frames = 0;
    integer accepted_count = 0;
    integer rejected_count = 0;
    integer timeout_count = 0;
    integer icmp_reply_count = 0;
    integer checks = 0;
    integer errors = 0;
    reg tx_capturing = 1'b0;
    reg [15:0] last_icmp_sequence = 16'd0;

    GSR GSR(.GSRI(1'b1));

    acg720_robot_network_rgmii #(
        .SYSTEM_CLOCK_HZ(100000000),
        .CONTROL_TIMEOUT_CYCLES(2000),
        .RANGE_TIMEOUT_CYCLES(100000),
        .TELEMETRY_PERIOD_CYCLES(500),
        .STATUS_PERIOD_CYCLES(5000),
        .TIMEBASE_CYCLES_PER_US(100)
    ) dut (
        .reset_n(reset_n), .system_clk_i(system_clk_i),
        .rgmii_rx_clk_i(rgmii_rx_clk_i), .rgmii_rxd(rgmii_rxd),
        .rgmii_rxdv(rgmii_rxdv), .eth_rst_n(eth_rst_n),
        .rgmii_tx_clk(rgmii_tx_clk), .rgmii_txd(rgmii_txd),
        .rgmii_txen(rgmii_txen), .requested_mode_i(requested_mode_i),
        .motion_stopped_i(motion_stopped_i), .physical_estop_i(physical_estop_i),
        .critical_fault_i(critical_fault_i),
        .front_sample_valid_i(front_sample_valid_i),
        .front_distance_mm_i(front_distance_mm_i),
        .left_sample_valid_i(left_sample_valid_i),
        .left_distance_mm_i(left_distance_mm_i),
        .right_sample_valid_i(right_sample_valid_i),
        .right_distance_mm_i(right_distance_mm_i),
        .imu_sample_start_i(imu_sample_start_i),
        .imu_byte_valid_i(imu_byte_valid_i), .imu_byte_data_i(imu_byte_data_i),
        .path_start_i(path_start_i), .path_abort_i(path_abort_i),
        .avoidance_tick_i(avoidance_tick_i), .segment_progress_i(segment_progress_i),
        .motor_sample_tick_i(motor_sample_tick_i), .left_ticks_i(left_ticks_i),
        .right_ticks_i(right_ticks_i), .left_velocity_mm_s_i(left_velocity_mm_s_i),
        .right_velocity_mm_s_i(right_velocity_mm_s_i),
        .accel_x_mm_s2_i(accel_x_mm_s2_i), .accel_y_mm_s2_i(accel_y_mm_s2_i),
        .accel_z_mm_s2_i(accel_z_mm_s2_i), .gyro_x_mrad_s_i(gyro_x_mrad_s_i),
        .gyro_y_mrad_s_i(gyro_y_mrad_s_i), .gyro_z_mrad_s_i(gyro_z_mrad_s_i),
        .control_packet_accepted_o(control_packet_accepted_o),
        .control_packet_rejected_o(control_packet_rejected_o),
        .motion_enabled_o(motion_enabled_o),
        .final_linear_mm_s_o(final_linear_mm_s_o),
        .final_angular_mrad_s_o(final_angular_mrad_s_o),
        .software_estop_o(software_estop_o),
        .control_timeout_pulse_o(control_timeout_pulse_o),
        .status_bits_o(status_bits_o), .fault_bits_o(fault_bits_o),
        .rx_control_fifo_full_o(rx_control_fifo_full_o),
        .tx_frame_fifo_full_o(tx_frame_fifo_full_o)
    );

    always #4 begin
        if (network_clock_enable)
            rgmii_rx_clk_i = ~rgmii_rx_clk_i;
    end

    initial begin
        #1;
        forever #5 begin
            if (system_clock_enable)
                system_clk_i = ~system_clk_i;
        end
    end

    always @(posedge system_clk_i) begin
        if (control_packet_accepted_o)
            accepted_count = accepted_count + 1;
        if (control_packet_rejected_o)
            rejected_count = rejected_count + 1;
        if (control_timeout_pulse_o)
            timeout_count = timeout_count + 1;
    end

    always @(posedge rgmii_rx_clk_i) begin
        #1;
        if (dut.gmii_txen) begin
            if (!tx_capturing) begin
                tx_capturing = 1'b1;
                tx_count = 0;
            end
            if (tx_count < 256)
                tx_capture[tx_count] = dut.gmii_txd;
            tx_count = tx_count + 1;
        end else if (tx_capturing) begin
            tx_capturing = 1'b0;
            captured_length = tx_count;
            completed_frames = completed_frames + 1;
            if (tx_count >= 50 && {tx_capture[20], tx_capture[21]} == 16'h0800 &&
                tx_capture[31] == 8'h01 && tx_capture[42] == 8'h00) begin
                icmp_reply_count = icmp_reply_count + 1;
                last_icmp_sequence = {tx_capture[48], tx_capture[49]};
            end
        end
    end

    function automatic [31:0] crc32_byte;
        input [31:0] crc_in;
        input [7:0] data;
        reg [31:0] crc;
        integer bit_index;
        begin
            crc = crc_in;
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
                if (crc[0] ^ data[bit_index])
                    crc = (crc >> 1) ^ 32'hedb88320;
                else
                    crc = crc >> 1;
            end
            crc32_byte = crc;
        end
    endfunction

    function automatic [15:0] crc16_byte;
        input [15:0] crc_in;
        input [7:0] data;
        reg [15:0] crc;
        integer bit_index;
        begin
            crc = crc_in ^ {data, 8'h00};
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1)
                crc = crc[15] ? ((crc << 1) ^ 16'h1021) : (crc << 1);
            crc16_byte = crc;
        end
    endfunction

    function automatic [15:0] fold_sum;
        input [31:0] sum_in;
        reg [16:0] once;
        reg [16:0] twice;
        begin
            once = sum_in[31:16] + sum_in[15:0];
            twice = once[16] + once[15:0];
            fold_sum = twice[16] + twice[15:0];
        end
    endfunction

    task automatic check_true;
        input condition;
        input string description;
        begin
            checks = checks + 1;
            if (!condition) begin
                errors = errors + 1;
                $display("[FAIL] %s", description);
            end else begin
                $display("[PASS] %s", description);
            end
        end
    endtask

    task automatic clear_host_frame;
        integer index;
        begin
            for (index = 0; index < 256; index = index + 1)
                host_frame[index] = 8'h00;
        end
    endtask

    task automatic finalize_host_fcs;
        input integer frame_length;
        input corrupt_fcs;
        integer index;
        reg [31:0] crc;
        begin
            crc = 32'hffffffff;
            for (index = 8; index < frame_length - 4; index = index + 1)
                crc = crc32_byte(crc, host_frame[index]);
            crc = ~crc;
            host_frame[frame_length-4] = crc[7:0] ^ (corrupt_fcs ? 8'h01 : 8'h00);
            host_frame[frame_length-3] = crc[15:8];
            host_frame[frame_length-2] = crc[23:16];
            host_frame[frame_length-1] = crc[31:24];
        end
    endtask

    task automatic build_arp_request;
        integer index;
        begin
            clear_host_frame();
            host_frame_length = 72;
            for (index = 0; index < 7; index = index + 1)
                host_frame[index] = 8'h55;
            host_frame[7] = 8'hd5;
            for (index = 8; index < 14; index = index + 1)
                host_frame[index] = 8'hff;
            host_frame[14] = HOST_MAC[47:40]; host_frame[15] = HOST_MAC[39:32];
            host_frame[16] = HOST_MAC[31:24]; host_frame[17] = HOST_MAC[23:16];
            host_frame[18] = HOST_MAC[15:8]; host_frame[19] = HOST_MAC[7:0];
            host_frame[20] = 8'h08; host_frame[21] = 8'h06;
            host_frame[22] = 8'h00; host_frame[23] = 8'h01;
            host_frame[24] = 8'h08; host_frame[25] = 8'h00;
            host_frame[26] = 8'h06; host_frame[27] = 8'h04;
            host_frame[28] = 8'h00; host_frame[29] = 8'h01;
            host_frame[30] = HOST_MAC[47:40]; host_frame[31] = HOST_MAC[39:32];
            host_frame[32] = HOST_MAC[31:24]; host_frame[33] = HOST_MAC[23:16];
            host_frame[34] = HOST_MAC[15:8]; host_frame[35] = HOST_MAC[7:0];
            host_frame[36] = HOST_IP[31:24]; host_frame[37] = HOST_IP[23:16];
            host_frame[38] = HOST_IP[15:8]; host_frame[39] = HOST_IP[7:0];
            host_frame[46] = LOCAL_IP[31:24]; host_frame[47] = LOCAL_IP[23:16];
            host_frame[48] = LOCAL_IP[15:8]; host_frame[49] = LOCAL_IP[7:0];
            finalize_host_fcs(host_frame_length, 1'b0);
        end
    endtask

    task automatic build_icmp_request;
        input [15:0] ip_id;
        input [15:0] echo_sequence;
        input [7:0] data_base;
        integer index;
        reg [31:0] sum;
        reg [15:0] checksum;
        begin
            clear_host_frame();
            host_frame_length = 72;
            for (index = 0; index < 7; index = index + 1)
                host_frame[index] = 8'h55;
            host_frame[7] = 8'hd5;
            host_frame[8] = LOCAL_MAC[47:40]; host_frame[9] = LOCAL_MAC[39:32];
            host_frame[10] = LOCAL_MAC[31:24]; host_frame[11] = LOCAL_MAC[23:16];
            host_frame[12] = LOCAL_MAC[15:8]; host_frame[13] = LOCAL_MAC[7:0];
            host_frame[14] = HOST_MAC[47:40]; host_frame[15] = HOST_MAC[39:32];
            host_frame[16] = HOST_MAC[31:24]; host_frame[17] = HOST_MAC[23:16];
            host_frame[18] = HOST_MAC[15:8]; host_frame[19] = HOST_MAC[7:0];
            host_frame[20] = 8'h08; host_frame[21] = 8'h00;
            host_frame[22] = 8'h45; host_frame[23] = 8'h00;
            host_frame[24] = 8'h00; host_frame[25] = 8'h24;
            host_frame[26] = ip_id[15:8]; host_frame[27] = ip_id[7:0];
            host_frame[28] = 8'h00; host_frame[29] = 8'h00;
            host_frame[30] = 8'h40; host_frame[31] = 8'h01;
            host_frame[34] = HOST_IP[31:24]; host_frame[35] = HOST_IP[23:16];
            host_frame[36] = HOST_IP[15:8]; host_frame[37] = HOST_IP[7:0];
            host_frame[38] = LOCAL_IP[31:24]; host_frame[39] = LOCAL_IP[23:16];
            host_frame[40] = LOCAL_IP[15:8]; host_frame[41] = LOCAL_IP[7:0];
            sum = 32'h4500 + 32'h0024 + ip_id + 32'h0000 + 32'h4001 +
                  HOST_IP[31:16] + HOST_IP[15:0] + LOCAL_IP[31:16] + LOCAL_IP[15:0];
            checksum = ~fold_sum(sum);
            host_frame[32] = checksum[15:8]; host_frame[33] = checksum[7:0];
            host_frame[42] = 8'h08; host_frame[43] = 8'h00;
            host_frame[46] = 8'hbe; host_frame[47] = 8'hef;
            host_frame[48] = echo_sequence[15:8];
            host_frame[49] = echo_sequence[7:0];
            for (index = 0; index < 8; index = index + 1)
                host_frame[50+index] = data_base + index;
            sum = 32'd0;
            for (index = 42; index < 58; index = index + 2)
                sum = sum + {host_frame[index], host_frame[index+1]};
            checksum = ~fold_sum(sum);
            host_frame[44] = checksum[15:8]; host_frame[45] = checksum[7:0];
            finalize_host_fcs(host_frame_length, 1'b0);
        end
    endtask

    task automatic build_control;
        input [31:0] seq_value;
        input signed [31:0] linear;
        input signed [31:0] angular;
        input [7:0] flags;
        input integer fault_kind;
        integer index;
        reg [15:0] app_crc;
        reg [31:0] sum;
        reg [15:0] checksum;
        begin
            clear_host_frame();
            host_frame_length = 85;
            for (index = 0; index < 7; index = index + 1)
                host_frame[index] = 8'h55;
            host_frame[7] = 8'hd5;
            host_frame[8] = fault_kind == FAULT_MAC ? 8'h12 : LOCAL_MAC[47:40];
            host_frame[9] = LOCAL_MAC[39:32]; host_frame[10] = LOCAL_MAC[31:24];
            host_frame[11] = LOCAL_MAC[23:16]; host_frame[12] = LOCAL_MAC[15:8];
            host_frame[13] = LOCAL_MAC[7:0];
            host_frame[14] = HOST_MAC[47:40]; host_frame[15] = HOST_MAC[39:32];
            host_frame[16] = HOST_MAC[31:24]; host_frame[17] = HOST_MAC[23:16];
            host_frame[18] = HOST_MAC[15:8]; host_frame[19] = HOST_MAC[7:0];
            host_frame[20] = 8'h08; host_frame[21] = 8'h00;
            host_frame[22] = 8'h45; host_frame[23] = 8'h00;
            host_frame[24] = 8'h00; host_frame[25] = 8'h3b;
            host_frame[26] = 8'h00; host_frame[27] = seq_value[7:0];
            host_frame[28] = 8'h00; host_frame[29] = 8'h00;
            host_frame[30] = 8'h40; host_frame[31] = 8'h11;
            host_frame[34] = HOST_IP[31:24]; host_frame[35] = HOST_IP[23:16];
            host_frame[36] = HOST_IP[15:8]; host_frame[37] = HOST_IP[7:0];
            host_frame[38] = fault_kind == FAULT_DEST_IP ? 8'hc0 : LOCAL_IP[31:24];
            host_frame[39] = fault_kind == FAULT_DEST_IP ? 8'ha8 : LOCAL_IP[23:16];
            host_frame[40] = fault_kind == FAULT_DEST_IP ? 8'h32 : LOCAL_IP[15:8];
            host_frame[41] = fault_kind == FAULT_DEST_IP ? 8'h03 : LOCAL_IP[7:0];
            host_frame[42] = 8'h13; host_frame[43] = 8'h89;
            host_frame[44] = fault_kind == FAULT_PORT ? 8'h13 : 8'h13;
            host_frame[45] = fault_kind == FAULT_PORT ? 8'h8a : 8'h88;
            host_frame[46] = 8'h00; host_frame[47] = 8'h27;
            host_frame[50] = 8'h46; host_frame[51] = 8'h50;
            host_frame[52] = 8'h47; host_frame[53] = 8'h41;
            host_frame[54] = 8'h01; host_frame[55] = 8'h01;
            host_frame[56] = 8'h00; host_frame[57] = 8'h09;
            host_frame[58] = seq_value[31:24]; host_frame[59] = seq_value[23:16];
            host_frame[60] = seq_value[15:8]; host_frame[61] = seq_value[7:0];
            host_frame[62] = 8'h01; host_frame[63] = 8'h02;
            host_frame[64] = 8'h03; host_frame[65] = 8'h04;
            host_frame[66] = 8'h05; host_frame[67] = 8'h06;
            host_frame[68] = 8'h07; host_frame[69] = 8'h08;
            host_frame[70] = linear[31:24]; host_frame[71] = linear[23:16];
            host_frame[72] = linear[15:8]; host_frame[73] = linear[7:0];
            host_frame[74] = angular[31:24]; host_frame[75] = angular[23:16];
            host_frame[76] = angular[15:8]; host_frame[77] = angular[7:0];
            host_frame[78] = flags;
            app_crc = 16'hffff;
            for (index = 50; index < 79; index = index + 1)
                app_crc = crc16_byte(app_crc, host_frame[index]);
            host_frame[79] = app_crc[15:8] ^ (fault_kind == FAULT_APP ? 8'h01 : 8'h00);
            host_frame[80] = app_crc[7:0];

            sum = 32'h4500 + 32'h003b + {16'd0, host_frame[26], host_frame[27]} +
                  32'h0000 + 32'h4011 + HOST_IP[31:16] + HOST_IP[15:0] +
                  {host_frame[38], host_frame[39]} + {host_frame[40], host_frame[41]};
            checksum = ~fold_sum(sum);
            host_frame[32] = checksum[15:8] ^ (fault_kind == FAULT_IP ? 8'h01 : 8'h00);
            host_frame[33] = checksum[7:0];

            sum = HOST_IP[31:16] + HOST_IP[15:0] +
                  {host_frame[38], host_frame[39]} + {host_frame[40], host_frame[41]} +
                  32'h0011 + 32'h0027;
            for (index = 42; index < 81; index = index + 2) begin
                if (index == 48)
                    sum = sum + 16'h0000;
                else if (index == 80)
                    sum = sum + {host_frame[index], 8'h00};
                else
                    sum = sum + {host_frame[index], host_frame[index+1]};
            end
            checksum = ~fold_sum(sum);
            if (checksum == 16'h0000)
                checksum = 16'hffff;
            host_frame[48] = checksum[15:8] ^ (fault_kind == FAULT_UDP ? 8'h01 : 8'h00);
            host_frame[49] = checksum[7:0];
            finalize_host_fcs(host_frame_length, fault_kind == FAULT_FCS);
        end
    endtask

    task automatic drive_rgmii_byte;
        input [7:0] data;
        input valid;
        begin
            @(negedge rgmii_rx_clk_i);
            rgmii_rxd <= data[3:0];
            rgmii_rxdv <= valid;
            @(posedge rgmii_rx_clk_i);
            rgmii_rxd <= data[7:4];
            rgmii_rxdv <= valid;
        end
    endtask

    task automatic send_host_frame;
        integer index;
        begin
            for (index = 0; index < host_frame_length; index = index + 1)
                drive_rgmii_byte(host_frame[index], 1'b1);
            for (index = 0; index < 16; index = index + 1)
                drive_rgmii_byte(8'h00, 1'b0);
        end
    endtask

    task automatic wait_for_frame_kind;
        input integer kind;
        output reg found;
        integer previous_count;
        integer guard;
        reg is_match;
        begin
            found = 1'b0;
            previous_count = completed_frames;
            guard = 0;
            while (!found && guard < 30000) begin
                @(posedge rgmii_rx_clk_i);
                guard = guard + 1;
                if (completed_frames != previous_count) begin
                    is_match = 1'b0;
                    if (kind == 1)
                        is_match = captured_length >= 22 &&
                                  {tx_capture[20], tx_capture[21]} == 16'h0806;
                    else if (kind == 2)
                        is_match = captured_length >= 42 &&
                                  {tx_capture[20], tx_capture[21]} == 16'h0800 &&
                                  tx_capture[31] == 8'h01;
                    else if (kind == 3)
                        is_match = captured_length >= 58 && tx_capture[55] == 8'h02;
                    else if (kind == 4)
                        is_match = captured_length >= 58 && tx_capture[55] == 8'h03;
                    if (is_match)
                        found = 1'b1;
                    previous_count = completed_frames;
                end
            end
        end
    endtask

    task automatic check_capture_fcs;
        integer index;
        reg [31:0] crc;
        reg [31:0] expected;
        begin
            crc = 32'hffffffff;
            for (index = 8; index < captured_length - 4; index = index + 1)
                crc = crc32_byte(crc, tx_capture[index]);
            expected = ~crc;
            if ({tx_capture[captured_length-1], tx_capture[captured_length-2],
                 tx_capture[captured_length-3], tx_capture[captured_length-4]} != expected)
                $display("[INFO] captured_length=%0d FCS=%08x expected=%08x",
                         captured_length,
                         {tx_capture[captured_length-1], tx_capture[captured_length-2],
                          tx_capture[captured_length-3], tx_capture[captured_length-4]},
                         expected);
            check_true({tx_capture[captured_length-1], tx_capture[captured_length-2],
                        tx_capture[captured_length-3], tx_capture[captured_length-4]} == expected,
                       "返回帧 FCS 字节级正确");
        end
    endtask

    task automatic check_ipv4_checksum;
        integer index;
        reg [31:0] sum;
        begin
            sum = 32'd0;
            for (index = 22; index < 42; index = index + 2)
                sum = sum + {tx_capture[index], tx_capture[index+1]};
            check_true(fold_sum(sum) == 16'hffff, "返回 IPv4 checksum 正确");
        end
    endtask

    task automatic check_application_crc;
        input integer packet_length;
        integer index;
        reg [15:0] crc;
        begin
            crc = 16'hffff;
            for (index = 50; index < 50 + packet_length - 2; index = index + 1)
                crc = crc16_byte(crc, tx_capture[index]);
            check_true({tx_capture[50+packet_length-2], tx_capture[50+packet_length-1]} == crc,
                       "返回应用层 CRC16 正确");
        end
    endtask

    task automatic wait_for_accept_count;
        input integer target;
        integer guard;
        begin
            guard = 0;
            while (accepted_count < target && guard < 10000) begin
                @(posedge system_clk_i);
                guard = guard + 1;
            end
            // 接受脉冲和安全门控输出均由非阻塞赋值提交，跨过本拍 NBA 后再检查。
            #1;
            check_true(accepted_count >= target, "Control 通过 CDC 后被 robot_rtl_core 接受");
        end
    endtask

    task automatic pulse_ranges;
        begin
            @(negedge system_clk_i);
            front_sample_valid_i <= 1'b1;
            left_sample_valid_i <= 1'b1;
            right_sample_valid_i <= 1'b1;
            @(negedge system_clk_i);
            front_sample_valid_i <= 1'b0;
            left_sample_valid_i <= 1'b0;
            right_sample_valid_i <= 1'b0;
        end
    endtask

    initial begin : run_test
        integer before_accept;
        integer before_reject;
        integer target_accept;
        integer before_icmp;
        integer guard;
        integer index;
        reg found;
        reg [31:0] sum;
        reg [15:0] app_crc;

        repeat (8) @(posedge rgmii_rx_clk_i);
        reset_n = 1'b1;
        repeat (8) @(posedge system_clk_i);
        pulse_ranges();
        repeat (8) drive_rgmii_byte(8'h00, 1'b0);

        build_arp_request();
        send_host_frame();
        wait_for_frame_kind(1, found);
        check_true(found, "ARP Request 产生 Reply");
        if (found) begin
            check_true(captured_length == 72, "ARP Reply 为完整 72 字节 RGMII/GMII 帧");
            check_true({tx_capture[8],tx_capture[9],tx_capture[10],tx_capture[11],
                        tx_capture[12],tx_capture[13]} == HOST_MAC &&
                       {tx_capture[14],tx_capture[15],tx_capture[16],tx_capture[17],
                        tx_capture[18],tx_capture[19]} == LOCAL_MAC,
                       "ARP Reply MAC 端点正确");
            check_true({tx_capture[28],tx_capture[29]} == 16'h0002 &&
                       {tx_capture[36],tx_capture[37],tx_capture[38],tx_capture[39]} == LOCAL_IP &&
                       {tx_capture[46],tx_capture[47],tx_capture[48],tx_capture[49]} == HOST_IP,
                       "ARP Reply opcode 与协议地址正确");
            check_capture_fcs();
        end

        build_icmp_request(16'h1234, 16'h0007, 8'ha0);
        send_host_frame();
        wait_for_frame_kind(2, found);
        check_true(found, "ICMP Echo Request 产生 Echo Reply");
        if (found) begin
            check_true(captured_length == 72 && tx_capture[42] == 8'h00 &&
                       tx_capture[43] == 8'h00, "ICMP Echo Reply 类型、代码和长度正确");
            check_ipv4_checksum();
            sum = 32'd0;
            for (index = 42; index < 58; index = index + 2)
                sum = sum + {tx_capture[index], tx_capture[index+1]};
            check_true(fold_sum(sum) == 16'hffff, "ICMP Echo Reply checksum 正确");
            check_true(tx_capture[46] == 8'hbe && tx_capture[47] == 8'hef &&
                       tx_capture[50] == 8'ha0 && tx_capture[57] == 8'ha7,
                       "ICMP 标识、序号和数据原样返回");
            check_capture_fcs();
        end

        // 流水化 checksum 会增加固定准备延迟；连续请求仍须逐帧返回且不串帧。
        before_icmp = icmp_reply_count;
        build_icmp_request(16'h1235, 16'h0008, 8'hb0);
        send_host_frame();
        build_icmp_request(16'h1236, 16'h0009, 8'hc0);
        send_host_frame();
        guard = 0;
        while (icmp_reply_count < before_icmp + 2 && guard < 30000) begin
            @(posedge rgmii_rx_clk_i);
            guard = guard + 1;
        end
        #1;
        check_true(icmp_reply_count == before_icmp + 2 &&
                   last_icmp_sequence == 16'h0009,
                   "连续 ICMP 请求按序产生两帧独立 Reply");
        if (icmp_reply_count == before_icmp + 2) begin
            check_ipv4_checksum();
            sum = 32'd0;
            for (index = 42; index < 58; index = index + 2)
                sum = sum + {tx_capture[index], tx_capture[index+1]};
            check_true(fold_sum(sum) == 16'hffff &&
                       tx_capture[50] == 8'hc0 && tx_capture[57] == 8'hc7,
                       "连续 ICMP 最后一帧 checksum 与数据未受前帧污染");
            check_capture_fcs();
        end

        for (index = FAULT_FCS; index <= FAULT_PORT; index = index + 1) begin
            before_accept = accepted_count;
            before_reject = rejected_count;
            build_control(32'hffff_fffe, 32'sd500, 32'sd100, 8'd0, index);
            send_host_frame();
            repeat (250) @(posedge system_clk_i);
            check_true(accepted_count == before_accept,
                       "错误 FCS/IP/UDP/应用校验或目标端点不提交 Control");
            if (index == FAULT_APP)
                check_true(rejected_count > before_reject,
                           "应用层 CRC 错误由 fpga_control_packet_rx 拒绝");
        end

        target_accept = accepted_count + 1;
        build_control(32'hffff_fffe, 32'sd500, 32'sd100, 8'd0, FAULT_NONE);
        send_host_frame();
        wait_for_accept_count(target_accept);
        repeat (8) @(posedge system_clk_i);
        check_true(motion_enabled_o && final_linear_mm_s_o == 32'sd500 &&
                   final_angular_mrad_s_o == 32'sd100,
                   "合法 Control 驱动 robot_rtl_core 逻辑目标");

        front_distance_mm_i = 16'd400;
        pulse_ranges();
        repeat (3) @(posedge system_clk_i);
        check_true(motion_enabled_o && final_linear_mm_s_o == 32'sd250 &&
                   final_angular_mrad_s_o == 32'sd100 && !status_bits_o[4],
                   "前向慢行区域将可观察线速度减半");

        front_distance_mm_i = 16'd100;
        pulse_ranges();
        repeat (3) @(posedge system_clk_i);
        check_true(motion_enabled_o && final_linear_mm_s_o == 32'sd0 &&
                   final_angular_mrad_s_o == 32'sd100 && status_bits_o[4],
                   "前向停车区域归零线速度并进入 Status 安全位");

        front_distance_mm_i = 16'd1000;
        pulse_ranges();
        repeat (3) @(posedge system_clk_i);
        check_true(final_linear_mm_s_o == 32'sd500 && !status_bits_o[4],
                   "测距恢复后安全限制仅释放当前有效命令");

        wait_for_frame_kind(4, found);
        check_true(found, "10 Hz Status 调度产生完整 UDP 帧");
        if (found) begin
            check_true(captured_length == 92 && tx_capture[55] == 8'h03 &&
                       {tx_capture[56],tx_capture[57]} == 16'h0010,
                       "Status 报文长度和类型正确");
            check_true(tx_capture[70] == 8'h01 && tx_capture[71] == 8'h01 &&
                       {tx_capture[82],tx_capture[83],tx_capture[84],tx_capture[85]} ==
                       32'hffff_fffe, "Status 模式、运动门控和最近 Control 序号正确");
            check_ipv4_checksum();
            check_application_crc(38);
            check_capture_fcs();
        end

        wait_for_frame_kind(3, found);
        check_true(found, "100 Hz Telemetry 调度产生完整 UDP 帧");
        if (found) begin
            check_true(captured_length == 126 && tx_capture[55] == 8'h02 &&
                       {tx_capture[56],tx_capture[57]} == 16'h0032,
                       "Telemetry 报文长度和类型正确");
            check_true({tx_capture[70],tx_capture[71],tx_capture[72],tx_capture[73]} ==
                       32'hffff_ff85 &&
                       {tx_capture[74],tx_capture[75],tx_capture[76],tx_capture[77]} ==
                       32'h0000_01c8, "Telemetry 编码器 ticks 字节序正确");
            check_true({tx_capture[110],tx_capture[111]} == 16'd1000 &&
                       {tx_capture[112],tx_capture[113]} == 16'd1100 &&
                       {tx_capture[114],tx_capture[115]} == 16'd1200,
                       "Telemetry 返回 robot_rtl_core 测距快照");
            check_true({tx_capture[48],tx_capture[49]} == 16'h0000,
                       "IPv4 UDP 发送 checksum 零值符合冻结协议边界");
            check_ipv4_checksum();
            check_application_crc(72);
            check_capture_fcs();
        end

        before_accept = accepted_count;
        before_reject = rejected_count;
        build_control(32'hffff_fffe, 32'sd600, 32'sd0, 8'd0, FAULT_NONE);
        send_host_frame();
        repeat (150) @(posedge system_clk_i);
        check_true(accepted_count == before_accept && rejected_count > before_reject,
                   "重复序号被拒绝");
        before_reject = rejected_count;
        build_control(32'hffff_fffd, 32'sd600, 32'sd0, 8'd0, FAULT_NONE);
        send_host_frame();
        repeat (150) @(posedge system_clk_i);
        check_true(accepted_count == before_accept && rejected_count > before_reject,
                   "倒序序号被拒绝");

        target_accept = accepted_count + 1;
        build_control(32'hffff_ffff, 32'sd610, 32'sd0, 8'd0, FAULT_NONE);
        send_host_frame();
        wait_for_accept_count(target_accept);
        target_accept = accepted_count + 1;
        build_control(32'h0000_0000, 32'sd620, 32'sd0, 8'd0, FAULT_NONE);
        send_host_frame();
        wait_for_accept_count(target_accept);
        check_true(final_linear_mm_s_o == 32'sd620, "序号 0xffffffff 到 0 回绕被接受");

        before_accept = accepted_count;
        build_control(32'h0000_0001, 32'sd700, -32'sd30, 8'd0, FAULT_FCS);
        send_host_frame();
        build_control(32'h0000_0001, 32'sd701, -32'sd31, 8'd0, FAULT_NONE);
        send_host_frame();
        wait_for_accept_count(before_accept + 1);
        check_true(final_linear_mm_s_o == 32'sd701 &&
                   final_angular_mrad_s_o == -32'sd31,
                   "坏 FCS 整帧拒绝且不污染下一合法 Control");

        target_accept = accepted_count + 1;
        build_control(32'h0000_0002, 32'sd800, 32'sd0, 8'h01, FAULT_NONE);
        send_host_frame();
        wait_for_accept_count(target_accept);
        check_true(software_estop_o && !motion_enabled_o && final_linear_mm_s_o == 0,
                   "软件急停立即关闭逻辑运动输出");
        target_accept = accepted_count + 1;
        build_control(32'h0000_0003, 32'sd810, 32'sd0, 8'h00, FAULT_NONE);
        send_host_frame();
        wait_for_accept_count(target_accept);
        check_true(!software_estop_o && motion_enabled_o && final_linear_mm_s_o == 32'sd810,
                   "解除软件急停后必须由新 Control 恢复");

        physical_estop_i = 1'b1;
        repeat (3) @(posedge system_clk_i);
        check_true(!motion_enabled_o && final_linear_mm_s_o == 0,
                   "物理急停门控逻辑目标");
        physical_estop_i = 1'b0;
        repeat (3) @(posedge system_clk_i);
        check_true(!motion_enabled_o, "物理急停释放后旧命令不自动恢复");
        target_accept = accepted_count + 1;
        build_control(32'h0000_0004, 32'sd820, 32'sd0, 8'h00, FAULT_NONE);
        send_host_frame();
        wait_for_accept_count(target_accept);

        critical_fault_i = 1'b1;
        repeat (3) @(posedge system_clk_i);
        check_true(!motion_enabled_o && fault_bits_o[0], "严重故障关闭运动并进入 fault_bits");
        critical_fault_i = 1'b0;
        repeat (3) @(posedge system_clk_i);
        check_true(!motion_enabled_o, "严重故障解除后旧命令不自动恢复");
        target_accept = accepted_count + 1;
        build_control(32'h0000_0005, 32'sd830, 32'sd0, 8'h00, FAULT_NONE);
        send_host_frame();
        wait_for_accept_count(target_accept);

        before_accept = timeout_count;
        repeat (2100) @(posedge system_clk_i);
        check_true(timeout_count > before_accept && !motion_enabled_o && fault_bits_o[1],
                   "Control 看门狗超时关闭运动并锁存故障");
        target_accept = accepted_count + 1;
        build_control(32'h0000_0006, 32'sd840, 32'sd0, 8'h00, FAULT_NONE);
        send_host_frame();
        wait_for_accept_count(target_accept);
        check_true(!fault_bits_o[1] && motion_enabled_o,
                   "新合法 Control 清除看门狗超时状态");

        system_clock_enable = 1'b0;
        repeat (4) @(posedge rgmii_rx_clk_i);
        for (index = 0; index < 5; index = index + 1) begin
            build_control(32'h0000_0007 + index, 32'sd901 + index,
                          32'sd0, 8'h00, FAULT_NONE);
            send_host_frame();
        end
        repeat (20) @(posedge rgmii_rx_clk_i);
        check_true(rx_control_fifo_full_o, "系统时钟暂停时 Control CDC FIFO 到达满状态");
        before_accept = accepted_count;
        system_clock_enable = 1'b1;
        target_accept = before_accept + 4;
        wait_for_accept_count(target_accept);
        check_true(final_linear_mm_s_o == 32'sd904 && fault_bits_o[4],
                   "RX FIFO 满整帧拒绝并在恢复后无污染地排空四帧");

        network_clock_enable = 1'b0;
        repeat (3500) @(posedge system_clk_i);
        check_true(tx_frame_fifo_full_o && fault_bits_o[5],
                   "网络时钟暂停时 Telemetry/Status CDC FIFO 背压与满状态生效");
        network_clock_enable = 1'b1;
        wait_for_frame_kind(3, found);
        check_true(found, "网络时钟恢复后完整 Telemetry 帧继续发送");
        if (found)
            check_capture_fcs();

        reset_n = 1'b0;
        repeat (6) @(posedge system_clk_i);
        check_true(!motion_enabled_o && final_linear_mm_s_o == 0,
                   "异步复位立即回到安全零输出");
        reset_n = 1'b1;
        repeat (8) @(posedge system_clk_i);
        pulse_ranges();
        target_accept = accepted_count + 1;
        build_control(32'h0000_0000, 32'sd333, 32'sd44, 8'h00, FAULT_NONE);
        send_host_frame();
        wait_for_accept_count(target_accept);
        check_true(motion_enabled_o && final_linear_mm_s_o == 32'sd333,
                   "复位清空序号与 CDC 状态后可重新建立闭环");

        check_true(status_bits_o[13:12] == 2'b00 || fault_bits_o[5],
                   "两个异步时钟域恢复后接口保持可传输状态");

        if (errors != 0)
            $fatal(1, "机器人网络端到端自检失败：%0d/%0d 项失败", errors, checks);
        $display("[PASS] 机器人网络端到端自检全部通过：%0d 项检查", checks);
        $finish;
    end

endmodule
