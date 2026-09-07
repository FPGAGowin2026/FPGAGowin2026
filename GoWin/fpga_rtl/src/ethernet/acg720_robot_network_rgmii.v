`timescale 1ns/1ps

// ACG720 机器人网络纯 RTL 集成顶层。RGMII 与机器人系统时钟独立；
// 未确定的电机、IMU、测距和急停物理引脚不在此模块分配。
module acg720_robot_network_rgmii #(
    parameter [47:0] LOCAL_MAC = 48'h02_00_00_00_00_02,
    parameter [31:0] LOCAL_IP = 32'hc0_a8_32_02,
    parameter [15:0] CONTROL_PORT = 16'd5000,
    parameter [15:0] TELEMETRY_PORT = 16'd5001,
    parameter integer SYSTEM_CLOCK_HZ = 100000000,
    parameter integer CONTROL_TIMEOUT_CYCLES = SYSTEM_CLOCK_HZ / 4,
    parameter integer RANGE_TIMEOUT_CYCLES = SYSTEM_CLOCK_HZ / 10,
    parameter integer TELEMETRY_PERIOD_CYCLES = SYSTEM_CLOCK_HZ / 100,
    parameter integer STATUS_PERIOD_CYCLES = SYSTEM_CLOCK_HZ / 10,
    parameter integer TIMEBASE_CYCLES_PER_US = SYSTEM_CLOCK_HZ / 1000000
) (
    input  wire               reset_n,
    input  wire               system_clk_i,
    input  wire               rgmii_rx_clk_i,
    input  wire [3:0]         rgmii_rxd,
    input  wire               rgmii_rxdv,
    output wire               eth_rst_n,
    output wire               rgmii_tx_clk,
    output wire [3:0]         rgmii_txd,
    output wire               rgmii_txen,

    input  wire               requested_mode_i,
    input  wire               motion_stopped_i,
    input  wire               physical_estop_i,
    input  wire               critical_fault_i,
    input  wire               front_sample_valid_i,
    input  wire [15:0]        front_distance_mm_i,
    input  wire               left_sample_valid_i,
    input  wire [15:0]        left_distance_mm_i,
    input  wire               right_sample_valid_i,
    input  wire [15:0]        right_distance_mm_i,
    input  wire               imu_sample_start_i,
    input  wire               imu_byte_valid_i,
    input  wire [7:0]         imu_byte_data_i,
    input  wire               path_start_i,
    input  wire               path_abort_i,
    input  wire               avoidance_tick_i,
    input  wire [31:0]        segment_progress_i,
    input  wire               motor_sample_tick_i,
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

    output wire               control_packet_accepted_o,
    output wire               control_packet_rejected_o,
    output wire               motion_enabled_o,
    output wire signed [31:0] final_linear_mm_s_o,
    output wire signed [31:0] final_angular_mrad_s_o,
    output wire               software_estop_o,
    output wire               control_timeout_pulse_o,
    output wire [31:0]        status_bits_o,
    output wire [31:0]        fault_bits_o,
    output wire               rx_control_fifo_full_o,
    output wire               tx_frame_fifo_full_o
);
    localparam integer CONTROL_RECORD_WIDTH = 48 + 32 + 16 + 248;
    localparam integer TX_RECORD_WIDTH = 48 + 32 + 7 + 576;
    localparam [2:0] REPLAY_IDLE = 3'd0;
    localparam [2:0] REPLAY_WAIT = 3'd1;
    localparam [2:0] REPLAY_BYTES = 3'd2;
    localparam [2:0] REPLAY_DONE = 3'd3;
    localparam [2:0] REPLAY_RESULT = 3'd4;
    localparam [2:0] ENCODE_IDLE = 3'd0;
    localparam [2:0] ENCODE_TELEMETRY = 3'd1;
    localparam [2:0] ENCODE_STATUS = 3'd2;
    localparam [2:0] ENCODE_ENQUEUE = 3'd3;
    localparam [1:0] NET_TX_IDLE = 2'd0;
    localparam [1:0] NET_TX_WAIT_RECORD = 2'd1;
    localparam [1:0] NET_TX_UDP = 2'd2;
    localparam [1:0] NET_TX_AUX = 2'd3;

    wire net_clk;
    reg [1:0] net_reset_sync_q;
    reg [1:0] system_reset_sync_q;
    wire net_rst_n = net_reset_sync_q[1];
    wire system_rst_n = system_reset_sync_q[1];
    wire net_reset_p = !net_rst_n;

    wire gmii_rx_clk;
    wire [7:0] gmii_rxd;
    wire gmii_rxdv;
    wire gmii_rxer;
    wire gmii_tx_clk;
    wire [7:0] gmii_txd;
    wire gmii_txen;
    reg  [7:0] gmii_txd_q;
    reg        gmii_txen_q;

    wire [47:0] udp_peer_mac;
    wire [31:0] udp_peer_ip;
    wire [15:0] udp_peer_port;
    wire [15:0] udp_payload_length;
    wire [7:0] udp_rx_byte;
    wire udp_rx_valid;
    wire udp_rx_done;
    wire udp_rx_error;

    reg [247:0] control_payload_q;
    reg [5:0] control_payload_count_q;
    wire control_fifo_wr_full;
    wire control_fifo_wr_overflow;
    wire control_fifo_wr_valid = udp_rx_done && !udp_rx_error &&
                                 control_payload_count_q == 6'd31;
    wire [CONTROL_RECORD_WIDTH-1:0] control_fifo_wr_data = {
        udp_peer_mac, udp_peer_ip, udp_peer_port, control_payload_q
    };
    wire [CONTROL_RECORD_WIDTH-1:0] control_fifo_rd_data;
    wire control_fifo_rd_valid;
    wire control_fifo_rd_empty;
    wire control_fifo_rd_ready;

    reg [2:0] replay_state_q;
    reg [5:0] replay_index_q;
    reg [CONTROL_RECORD_WIDTH-1:0] replay_record_q;
    wire core_udp_payload_valid = replay_state_q == REPLAY_BYTES;
    wire [7:0] core_udp_payload_byte =
        replay_record_q[247-replay_index_q*8 -: 8];
    wire core_udp_packet_done = replay_state_q == REPLAY_DONE;

    reg peer_valid_q;
    reg [47:0] accepted_peer_mac_q;
    reg [31:0] accepted_peer_ip_q;

    wire active_mode;
    wire switch_pending;
    wire control_command_active;
    wire [31:0] last_control_sequence;
    wire imu_sample_error;
    wire [15:0] front_range_mm;
    wire [15:0] left_range_mm;
    wire [15:0] right_range_mm;
    wire ranges_valid;
    wire [1:0] safety_level;
    wire avoidance_active;
    wire path_active;
    wire path_finished;
    wire path_progress_enable;
    wire path_paused;
    wire [1:0] path_segment_opcode;
    wire [7:0] path_segment_index;
    wire safety_stop = safety_level == 2'd2;

    reg rx_overflow_net_q;
    reg rx_overflow_sync1_q;
    reg rx_overflow_sync2_q;
    reg rx_fifo_full_sync1_q;
    reg rx_fifo_full_sync2_q;
    reg control_timeout_latched_q;
    reg imu_error_latched_q;
    reg tx_backpressure_latched_q;

    reg [31:0] telemetry_counter_q;
    reg [31:0] status_counter_q;
    reg telemetry_pending_q;
    reg status_pending_q;
    reg [31:0] telemetry_sequence_q;
    reg [31:0] status_sequence_q;
    reg [31:0] timebase_divider_q;
    reg [9:0] microseconds_in_ms_q;
    reg [63:0] uptime_us_q;
    reg [31:0] uptime_ms_q;

    reg telemetry_start_q;
    wire telemetry_valid;
    wire [7:0] telemetry_data;
    wire telemetry_busy;
    wire telemetry_done;
    reg status_start_q;
    wire status_valid;
    wire [7:0] status_data;
    wire status_busy;
    wire status_done;

    reg [2:0] encode_state_q;
    reg outbound_is_status_q;
    reg [6:0] outbound_byte_count_q;
    reg [575:0] outbound_shift_q;
    reg [47:0] outbound_peer_mac_q;
    reg [31:0] outbound_peer_ip_q;
    wire [575:0] outbound_payload_aligned = outbound_is_status_q ?
        (outbound_shift_q << 272) : outbound_shift_q;
    wire [6:0] outbound_length = outbound_is_status_q ? 7'd38 : 7'd72;
    wire tx_fifo_wr_valid = encode_state_q == ENCODE_ENQUEUE;
    wire tx_fifo_wr_full;
    wire tx_fifo_wr_overflow;
    wire [TX_RECORD_WIDTH-1:0] tx_fifo_wr_data = {
        outbound_peer_mac_q, outbound_peer_ip_q,
        outbound_length, outbound_payload_aligned
    };
    wire [TX_RECORD_WIDTH-1:0] tx_fifo_rd_data;
    wire tx_fifo_rd_valid;
    wire tx_fifo_rd_empty;
    wire tx_fifo_rd_ready;

    wire aux_reply_valid;
    wire aux_reply_ready;
    wire aux_reply_is_icmp;
    wire [47:0] aux_peer_mac;
    wire [31:0] aux_peer_ip;
    wire [15:0] aux_ip_id;
    wire [15:0] aux_icmp_length;
    wire [511:0] aux_icmp_message;
    wire aux_reply_overflow;
    wire aux_tx_ready;
    wire aux_tx_busy;
    wire aux_tx_done;
    wire aux_gmii_txen;
    wire [7:0] aux_gmii_txd;

    reg [1:0] net_tx_state_q;
    reg udp_tx_start_q;
    reg udp_output_seen_q;
    reg [TX_RECORD_WIDTH-1:0] tx_record_q;
    reg [575:0] udp_payload_shift_q;
    wire udp_tx_done;
    wire udp_payload_request;
    wire [47:0] tx_record_mac = tx_record_q[TX_RECORD_WIDTH-1 -: 48];
    wire [31:0] tx_record_ip = tx_record_q[TX_RECORD_WIDTH-49 -: 32];
    wire [6:0] tx_record_length = tx_record_q[582:576];
    wire [7:0] udp_tx_payload_byte = udp_payload_shift_q[575:568];
    wire udp_gmii_txen;
    wire [7:0] udp_gmii_txd;

    Gowin_PLL network_clock_pll (
        .clkin(rgmii_rx_clk_i), .init_clk(rgmii_rx_clk_i), .clkout0(net_clk)
    );

    assign eth_rst_n = 1'b1;

    always @(posedge net_clk or negedge reset_n) begin
        if (!reset_n)
            net_reset_sync_q <= 2'b00;
        else
            net_reset_sync_q <= {net_reset_sync_q[0], 1'b1};
    end

    always @(posedge system_clk_i or negedge reset_n) begin
        if (!reset_n)
            system_reset_sync_q <= 2'b00;
        else
            system_reset_sync_q <= {system_reset_sync_q[0], 1'b1};
    end

    rgmii_to_gmii rgmii_receiver (
        .reset(net_reset_p), .rgmii_rx_clk(net_clk), .rgmii_rxd(rgmii_rxd),
        .rgmii_rxdv(rgmii_rxdv), .gmii_rx_clk(gmii_rx_clk),
        .gmii_rxdv(gmii_rxdv), .gmii_rxd(gmii_rxd), .gmii_rxer(gmii_rxer)
    );

    eth_udp_rx_gmii #(
        .ACCEPT_BROADCAST_MAC(1'b0)
    ) control_udp_receiver (
        .reset_p(net_reset_p), .local_mac(LOCAL_MAC), .local_ip(LOCAL_IP),
        .local_port(CONTROL_PORT), .clk125m_o(), .exter_mac(udp_peer_mac),
        .exter_ip(udp_peer_ip), .exter_port(udp_peer_port),
        .rx_data_length(udp_payload_length), .data_overflow_i(control_fifo_wr_full),
        .payload_valid_o(udp_rx_valid), .payload_dat_o(udp_rx_byte),
        .one_pkt_done(udp_rx_done), .pkt_error(udp_rx_error),
        .debug_crc_check(), .gmii_rx_clk(gmii_rx_clk),
        .gmii_rxdv(gmii_rxdv), .gmii_rxd(gmii_rxd)
    );

    always @(posedge net_clk or negedge net_rst_n) begin
        if (!net_rst_n) begin
            control_payload_q <= 248'd0;
            control_payload_count_q <= 6'd0;
        end else begin
            if (udp_rx_valid) begin
                control_payload_q <= {control_payload_q[239:0], udp_rx_byte};
                if (control_payload_count_q != 6'h3f)
                    control_payload_count_q <= control_payload_count_q + 1'b1;
            end
            if (udp_rx_done)
                control_payload_count_q <= 6'd0;
        end
    end

    async_frame_fifo #(
        .DATA_WIDTH(CONTROL_RECORD_WIDTH), .ADDR_WIDTH(2)
    ) control_cdc_fifo (
        .wr_clk_i(net_clk), .wr_rst_n_i(net_rst_n),
        .wr_valid_i(control_fifo_wr_valid), .wr_data_i(control_fifo_wr_data),
        .wr_full_o(control_fifo_wr_full), .wr_overflow_o(control_fifo_wr_overflow),
        .rd_clk_i(system_clk_i), .rd_rst_n_i(system_rst_n),
        .rd_ready_i(control_fifo_rd_ready), .rd_data_o(control_fifo_rd_data),
        .rd_valid_o(control_fifo_rd_valid), .rd_empty_o(control_fifo_rd_empty)
    );
    assign rx_control_fifo_full_o = control_fifo_wr_full;

    assign control_fifo_rd_ready = replay_state_q == REPLAY_IDLE &&
                                   !control_fifo_rd_empty;
    always @(posedge system_clk_i or negedge system_rst_n) begin
        if (!system_rst_n) begin
            replay_state_q <= REPLAY_IDLE;
            replay_index_q <= 6'd0;
            replay_record_q <= {CONTROL_RECORD_WIDTH{1'b0}};
        end else begin
            case (replay_state_q)
                REPLAY_IDLE:
                    if (control_fifo_rd_ready)
                        replay_state_q <= REPLAY_WAIT;
                REPLAY_WAIT:
                    if (control_fifo_rd_valid) begin
                        replay_record_q <= control_fifo_rd_data;
                        replay_index_q <= 6'd0;
                        replay_state_q <= REPLAY_BYTES;
                    end
                REPLAY_BYTES:
                    if (replay_index_q == 6'd30)
                        replay_state_q <= REPLAY_DONE;
                    else
                        replay_index_q <= replay_index_q + 1'b1;
                REPLAY_DONE: replay_state_q <= REPLAY_RESULT;
                REPLAY_RESULT: replay_state_q <= REPLAY_IDLE;
                default: replay_state_q <= REPLAY_IDLE;
            endcase
        end
    end

    robot_rtl_core #(
        .CONTROL_TIMEOUT_CYCLES(CONTROL_TIMEOUT_CYCLES),
        .RANGE_TIMEOUT_CYCLES(RANGE_TIMEOUT_CYCLES)
    ) robot_core (
        .clk_i(system_clk_i), .rst_n_i(system_rst_n),
        .requested_mode_i(requested_mode_i), .motion_stopped_i(motion_stopped_i),
        .physical_estop_i(physical_estop_i), .critical_fault_i(critical_fault_i),
        .udp_payload_valid_i(core_udp_payload_valid),
        .udp_payload_byte_i(core_udp_payload_byte),
        .udp_packet_done_i(core_udp_packet_done), .udp_packet_error_i(1'b0),
        .front_sample_valid_i(front_sample_valid_i),
        .front_distance_mm_i(front_distance_mm_i),
        .left_sample_valid_i(left_sample_valid_i), .left_distance_mm_i(left_distance_mm_i),
        .right_sample_valid_i(right_sample_valid_i),
        .right_distance_mm_i(right_distance_mm_i),
        .imu_sample_start_i(imu_sample_start_i), .imu_byte_valid_i(imu_byte_valid_i),
        .imu_byte_data_i(imu_byte_data_i), .imu_byte_ready_o(),
        .imu_sample_data_o(), .imu_sample_valid_o(),
        .camera_pclk_i(system_clk_i), .camera_vsync_i(1'b0),
        .camera_href_i(1'b0), .camera_data_i(8'd0),
        .camera_frame_valid_o(), .camera_line_valid_o(),
        .camera_pixel_valid_o(), .camera_pixel_rgb565_o(),
        .path_start_i(path_start_i), .path_abort_i(path_abort_i),
        .avoidance_tick_i(avoidance_tick_i), .segment_progress_i(segment_progress_i),
        .motor_sample_tick_i(motor_sample_tick_i),
        .left_velocity_mm_s_i(left_velocity_mm_s_i),
        .right_velocity_mm_s_i(right_velocity_mm_s_i),
        .active_mode_o(active_mode), .switch_pending_o(switch_pending),
        .motion_enabled_o(motion_enabled_o),
        .final_linear_mm_s_o(final_linear_mm_s_o),
        .final_angular_mrad_s_o(final_angular_mrad_s_o),
        .left_target_mm_s_o(), .right_target_mm_s_o(),
        .left_pwm_duty_o(), .left_direction_o(),
        .right_pwm_duty_o(), .right_direction_o(),
        .front_range_mm_o(front_range_mm), .left_range_mm_o(left_range_mm),
        .right_range_mm_o(right_range_mm), .ranges_valid_o(ranges_valid),
        .safety_level_o(safety_level), .avoidance_active_o(avoidance_active),
        .path_active_o(path_active), .path_finished_o(path_finished),
        .path_progress_enable_o(path_progress_enable), .path_paused_o(path_paused),
        .path_segment_opcode_o(path_segment_opcode),
        .segment_index_o(path_segment_index), .software_estop_o(software_estop_o),
        .control_command_active_o(control_command_active),
        .control_timeout_pulse_o(control_timeout_pulse_o),
        .last_control_sequence_o(last_control_sequence),
        .imu_sample_error_o(imu_sample_error),
        .control_packet_accepted_o(control_packet_accepted_o),
        .control_packet_rejected_o(control_packet_rejected_o)
    );

    always @(posedge system_clk_i or negedge system_rst_n) begin
        if (!system_rst_n) begin
            peer_valid_q <= 1'b0;
            accepted_peer_mac_q <= 48'd0;
            accepted_peer_ip_q <= 32'd0;
        end else if (control_packet_accepted_o) begin
            peer_valid_q <= 1'b1;
            accepted_peer_mac_q <= replay_record_q[CONTROL_RECORD_WIDTH-1 -: 48];
            accepted_peer_ip_q <= replay_record_q[CONTROL_RECORD_WIDTH-49 -: 32];
        end
    end

    always @(posedge net_clk or negedge net_rst_n) begin
        if (!net_rst_n)
            rx_overflow_net_q <= 1'b0;
        else if (control_fifo_wr_overflow ||
                 (udp_rx_valid && control_fifo_wr_full) || aux_reply_overflow)
            rx_overflow_net_q <= 1'b1;
    end

    always @(posedge system_clk_i or negedge system_rst_n) begin
        if (!system_rst_n) begin
            rx_overflow_sync1_q <= 1'b0;
            rx_overflow_sync2_q <= 1'b0;
            rx_fifo_full_sync1_q <= 1'b0;
            rx_fifo_full_sync2_q <= 1'b0;
            control_timeout_latched_q <= 1'b0;
            imu_error_latched_q <= 1'b0;
            tx_backpressure_latched_q <= 1'b0;
        end else begin
            rx_overflow_sync1_q <= rx_overflow_net_q;
            rx_overflow_sync2_q <= rx_overflow_sync1_q;
            rx_fifo_full_sync1_q <= control_fifo_wr_full;
            rx_fifo_full_sync2_q <= rx_fifo_full_sync1_q;
            if (control_packet_accepted_o)
                control_timeout_latched_q <= 1'b0;
            else if (control_timeout_pulse_o)
                control_timeout_latched_q <= 1'b1;
            if (imu_sample_error)
                imu_error_latched_q <= 1'b1;
            if ((encode_state_q == ENCODE_ENQUEUE && tx_fifo_wr_full) ||
                tx_fifo_wr_overflow)
                tx_backpressure_latched_q <= 1'b1;
        end
    end

    // udp_protocol.md 冻结位：13 TX满，12 RX满，11 严重故障，10 切换中，
    // 9 绕障，8 路径完成，7 路径活动，6 测距有效，5 Control有效，
    // 4 安全停车，3 软件急停，2 物理急停，1 运动使能，0 活动模式。
    assign status_bits_o = {
        18'd0,
        tx_fifo_wr_full,
        rx_fifo_full_sync2_q,
        critical_fault_i,
        switch_pending,
        avoidance_active,
        path_finished,
        path_active,
        ranges_valid,
        control_command_active,
        safety_stop,
        software_estop_o,
        physical_estop_i,
        motion_enabled_o,
        active_mode
    };

    // 冻结故障位：5 TX背压，4 RX溢出，3 测距无效，2 IMU样本错误，
    // 1 Control超时，0 严重故障；锁存位只由指定条件或复位清除。
    assign fault_bits_o = {
        26'd0,
        tx_backpressure_latched_q,
        rx_overflow_sync2_q,
        !ranges_valid,
        imu_error_latched_q,
        control_timeout_latched_q,
        critical_fault_i
    };

    fpga_telemetry_packet_tx telemetry_encoder (
        .clk_i(system_clk_i), .rst_n_i(system_rst_n),
        .start_i(telemetry_start_q), .ready_i(1'b1),
        .sequence_i(telemetry_sequence_q), .timestamp_us_i(uptime_us_q),
        .left_ticks_i(left_ticks_i), .right_ticks_i(right_ticks_i),
        .left_velocity_mm_s_i(left_velocity_mm_s_i),
        .right_velocity_mm_s_i(right_velocity_mm_s_i),
        .accel_x_mm_s2_i(accel_x_mm_s2_i), .accel_y_mm_s2_i(accel_y_mm_s2_i),
        .accel_z_mm_s2_i(accel_z_mm_s2_i), .gyro_x_mrad_s_i(gyro_x_mrad_s_i),
        .gyro_y_mrad_s_i(gyro_y_mrad_s_i), .gyro_z_mrad_s_i(gyro_z_mrad_s_i),
        .range_front_mm_i(front_range_mm), .range_left_mm_i(left_range_mm),
        .range_right_mm_i(right_range_mm), .status_bits_i(status_bits_o),
        .valid_o(telemetry_valid), .data_o(telemetry_data),
        .busy_o(telemetry_busy), .done_o(telemetry_done)
    );

    fpga_status_packet_tx status_encoder (
        .clk_i(system_clk_i), .rst_n_i(system_rst_n),
        .start_i(status_start_q), .ready_i(1'b1),
        .sequence_i(status_sequence_q), .timestamp_us_i(uptime_us_q),
        .mode_i({7'd0, active_mode}),
        .motion_enabled_i({7'd0, motion_enabled_o}),
        .estop_i({7'd0, physical_estop_i || software_estop_o}),
        .safety_stop_i({7'd0, safety_stop}), .fault_bits_i(fault_bits_o),
        .uptime_ms_i(uptime_ms_q), .last_control_sequence_i(last_control_sequence),
        .valid_o(status_valid), .data_o(status_data),
        .busy_o(status_busy), .done_o(status_done)
    );

    always @(posedge system_clk_i or negedge system_rst_n) begin
        if (!system_rst_n) begin
            timebase_divider_q <= 32'd0;
            microseconds_in_ms_q <= 10'd0;
            uptime_us_q <= 64'd0;
            uptime_ms_q <= 32'd0;
        end else if (timebase_divider_q >= TIMEBASE_CYCLES_PER_US - 1) begin
            timebase_divider_q <= 32'd0;
            uptime_us_q <= uptime_us_q + 1'b1;
            if (microseconds_in_ms_q == 10'd999) begin
                microseconds_in_ms_q <= 10'd0;
                uptime_ms_q <= uptime_ms_q + 1'b1;
            end else begin
                microseconds_in_ms_q <= microseconds_in_ms_q + 1'b1;
            end
        end else begin
            timebase_divider_q <= timebase_divider_q + 1'b1;
        end
    end

    always @(posedge system_clk_i or negedge system_rst_n) begin
        if (!system_rst_n) begin
            telemetry_counter_q <= 32'd0;
            status_counter_q <= 32'd0;
            telemetry_pending_q <= 1'b0;
            status_pending_q <= 1'b0;
            telemetry_sequence_q <= 32'd0;
            status_sequence_q <= 32'd0;
            telemetry_start_q <= 1'b0;
            status_start_q <= 1'b0;
            encode_state_q <= ENCODE_IDLE;
            outbound_is_status_q <= 1'b0;
            outbound_byte_count_q <= 7'd0;
            outbound_shift_q <= 576'd0;
            outbound_peer_mac_q <= 48'd0;
            outbound_peer_ip_q <= 32'd0;
        end else begin
            telemetry_start_q <= 1'b0;
            status_start_q <= 1'b0;

            if (telemetry_counter_q >= TELEMETRY_PERIOD_CYCLES - 1) begin
                telemetry_counter_q <= 32'd0;
                telemetry_pending_q <= 1'b1;
            end else begin
                telemetry_counter_q <= telemetry_counter_q + 1'b1;
            end
            if (status_counter_q >= STATUS_PERIOD_CYCLES - 1) begin
                status_counter_q <= 32'd0;
                status_pending_q <= 1'b1;
            end else begin
                status_counter_q <= status_counter_q + 1'b1;
            end
            if (control_packet_accepted_o) begin
                telemetry_pending_q <= 1'b1;
                status_pending_q <= 1'b1;
            end

            case (encode_state_q)
                ENCODE_IDLE: begin
                    outbound_shift_q <= 576'd0;
                    outbound_byte_count_q <= 7'd0;
                    if (peer_valid_q && status_pending_q) begin
                        status_pending_q <= 1'b0;
                        status_start_q <= 1'b1;
                        outbound_is_status_q <= 1'b1;
                        outbound_peer_mac_q <= accepted_peer_mac_q;
                        outbound_peer_ip_q <= accepted_peer_ip_q;
                        encode_state_q <= ENCODE_STATUS;
                    end else if (peer_valid_q && telemetry_pending_q) begin
                        telemetry_pending_q <= 1'b0;
                        telemetry_start_q <= 1'b1;
                        outbound_is_status_q <= 1'b0;
                        outbound_peer_mac_q <= accepted_peer_mac_q;
                        outbound_peer_ip_q <= accepted_peer_ip_q;
                        encode_state_q <= ENCODE_TELEMETRY;
                    end
                end
                ENCODE_TELEMETRY: begin
                    if (telemetry_valid) begin
                        outbound_shift_q <= {outbound_shift_q[567:0], telemetry_data};
                        outbound_byte_count_q <= outbound_byte_count_q + 1'b1;
                    end
                    if (telemetry_done)
                        encode_state_q <= ENCODE_ENQUEUE;
                end
                ENCODE_STATUS: begin
                    if (status_valid) begin
                        outbound_shift_q <= {outbound_shift_q[567:0], status_data};
                        outbound_byte_count_q <= outbound_byte_count_q + 1'b1;
                    end
                    if (status_done)
                        encode_state_q <= ENCODE_ENQUEUE;
                end
                ENCODE_ENQUEUE: begin
                    if (!tx_fifo_wr_full) begin
                        if (outbound_is_status_q)
                            status_sequence_q <= status_sequence_q + 1'b1;
                        else
                            telemetry_sequence_q <= telemetry_sequence_q + 1'b1;
                        encode_state_q <= ENCODE_IDLE;
                    end
                end
                default: encode_state_q <= ENCODE_IDLE;
            endcase
        end
    end

    async_frame_fifo #(
        .DATA_WIDTH(TX_RECORD_WIDTH), .ADDR_WIDTH(2)
    ) tx_cdc_fifo (
        .wr_clk_i(system_clk_i), .wr_rst_n_i(system_rst_n),
        .wr_valid_i(tx_fifo_wr_valid), .wr_data_i(tx_fifo_wr_data),
        .wr_full_o(tx_fifo_wr_full), .wr_overflow_o(tx_fifo_wr_overflow),
        .rd_clk_i(net_clk), .rd_rst_n_i(net_rst_n),
        .rd_ready_i(tx_fifo_rd_ready), .rd_data_o(tx_fifo_rd_data),
        .rd_valid_o(tx_fifo_rd_valid), .rd_empty_o(tx_fifo_rd_empty)
    );
    assign tx_frame_fifo_full_o = tx_fifo_wr_full;

    eth_arp_icmp_rx_gmii #(
        .LOCAL_MAC(LOCAL_MAC), .LOCAL_IP(LOCAL_IP)
    ) auxiliary_receiver (
        .clk_i(net_clk), .reset_p_i(net_reset_p),
        .gmii_rxdv_i(gmii_rxdv), .gmii_rxd_i(gmii_rxd),
        .reply_valid_o(aux_reply_valid), .reply_ready_i(aux_reply_ready),
        .reply_is_icmp_o(aux_reply_is_icmp), .peer_mac_o(aux_peer_mac),
        .peer_ip_o(aux_peer_ip), .ip_id_o(aux_ip_id),
        .icmp_length_o(aux_icmp_length), .icmp_message_o(aux_icmp_message),
        .reply_overflow_o(aux_reply_overflow)
    );

    assign aux_reply_ready = net_tx_state_q == NET_TX_IDLE && aux_tx_ready;
    eth_arp_icmp_tx_gmii #(
        .LOCAL_MAC(LOCAL_MAC), .LOCAL_IP(LOCAL_IP)
    ) auxiliary_transmitter (
        .clk_i(net_clk), .reset_p_i(net_reset_p),
        .request_valid_i(aux_reply_valid && net_tx_state_q == NET_TX_IDLE),
        .request_ready_o(aux_tx_ready), .request_is_icmp_i(aux_reply_is_icmp),
        .peer_mac_i(aux_peer_mac), .peer_ip_i(aux_peer_ip), .ip_id_i(aux_ip_id),
        .icmp_length_i(aux_icmp_length), .icmp_message_i(aux_icmp_message),
        .busy_o(aux_tx_busy), .done_o(aux_tx_done),
        .gmii_txen_o(aux_gmii_txen), .gmii_txd_o(aux_gmii_txd)
    );

    assign tx_fifo_rd_ready = net_tx_state_q == NET_TX_IDLE &&
                              !aux_reply_valid && !tx_fifo_rd_empty;

    always @(posedge net_clk or negedge net_rst_n) begin
        if (!net_rst_n) begin
            net_tx_state_q <= NET_TX_IDLE;
            udp_tx_start_q <= 1'b0;
            udp_output_seen_q <= 1'b0;
            tx_record_q <= {TX_RECORD_WIDTH{1'b0}};
            udp_payload_shift_q <= 576'd0;
        end else begin
            udp_tx_start_q <= 1'b0;
            if (net_tx_state_q == NET_TX_UDP && udp_gmii_txen)
                udp_output_seen_q <= 1'b1;
            if (udp_payload_request)
                udp_payload_shift_q <= {udp_payload_shift_q[567:0], 8'd0};

            case (net_tx_state_q)
                NET_TX_IDLE: begin
                    if (aux_reply_valid && aux_tx_ready)
                        net_tx_state_q <= NET_TX_AUX;
                    else if (tx_fifo_rd_ready)
                        net_tx_state_q <= NET_TX_WAIT_RECORD;
                end
                NET_TX_WAIT_RECORD: begin
                    if (tx_fifo_rd_valid) begin
                        tx_record_q <= tx_fifo_rd_data;
                        // 发送期间顺序移位，避免 576 位变量索引进入 125 MHz TX 路径。
                        udp_payload_shift_q <= tx_fifo_rd_data[575:0];
                        udp_tx_start_q <= 1'b1;
                        udp_output_seen_q <= 1'b0;
                        net_tx_state_q <= NET_TX_UDP;
                    end
                end
                NET_TX_UDP:
                    // 官方 UDP TX 的 tx_done 早于最后一级 GMII 流水线排空；
                    // 仲裁器以真实 txen 的下降沿释放整帧，避免截断 FCS。
                    if (udp_output_seen_q && !udp_gmii_txen)
                        net_tx_state_q <= NET_TX_IDLE;
                NET_TX_AUX:
                    if (aux_tx_done)
                        net_tx_state_q <= NET_TX_IDLE;
                default: net_tx_state_q <= NET_TX_IDLE;
            endcase
        end
    end

    eth_udp_tx_gmii telemetry_udp_transmitter (
        .clk125m(net_clk), .reset_p(net_reset_p),
        .tx_en_pulse(udp_tx_start_q), .tx_done(udp_tx_done),
        .dst_mac(tx_record_mac), .src_mac(LOCAL_MAC),
        .dst_ip(tx_record_ip), .src_ip(LOCAL_IP),
        .dst_port(TELEMETRY_PORT), .src_port(TELEMETRY_PORT),
        .data_length({9'd0, tx_record_length}),
        .payload_req_o(udp_payload_request), .payload_dat_i(udp_tx_payload_byte),
        .gmii_tx_clk(), .gmii_txen(udp_gmii_txen), .gmii_txd(udp_gmii_txd)
    );

    assign gmii_tx_clk = net_clk;
    assign gmii_txen = gmii_txen_q;
    assign gmii_txd = gmii_txd_q;

    // 数据与使能一起寄存，使帧内容选择逻辑不直接驱动 ODDR。
    always @(posedge net_clk or negedge net_rst_n) begin
        if (!net_rst_n) begin
            gmii_txen_q <= 1'b0;
            gmii_txd_q <= 8'h00;
        end else begin
            gmii_txen_q <= net_tx_state_q == NET_TX_AUX ? aux_gmii_txen :
                           net_tx_state_q == NET_TX_UDP ? udp_gmii_txen : 1'b0;
            gmii_txd_q <= net_tx_state_q == NET_TX_AUX ? aux_gmii_txd :
                          net_tx_state_q == NET_TX_UDP ? udp_gmii_txd : 8'h00;
        end
    end

    gmii_to_rgmii rgmii_transmitter (
        .reset_n(net_rst_n), .gmii_tx_clk(gmii_tx_clk),
        .gmii_txd(gmii_txd), .gmii_txen(gmii_txen), .gmii_txer(1'b0),
        .rgmii_tx_clk(rgmii_tx_clk), .rgmii_txd(rgmii_txd),
        .rgmii_txen(rgmii_txen)
    );

endmodule
