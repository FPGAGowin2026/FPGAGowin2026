`timescale 1ns/1ps

module tb_robot_rtl_core;
    reg clk_i = 1'b0;
    reg camera_pclk_i = 1'b0;
    reg rst_n_i = 1'b0;
    reg requested_mode_i = 1'b0;
    reg motion_stopped_i = 1'b1;
    reg physical_estop_i = 1'b0;
    reg critical_fault_i = 1'b0;
    reg udp_payload_valid_i = 1'b0;
    reg [7:0] udp_payload_byte_i = 8'd0;
    reg udp_packet_done_i = 1'b0;
    reg udp_packet_error_i = 1'b0;
    reg front_sample_valid_i = 1'b0;
    reg [15:0] front_distance_mm_i = 16'd0;
    reg left_sample_valid_i = 1'b0;
    reg [15:0] left_distance_mm_i = 16'd0;
    reg right_sample_valid_i = 1'b0;
    reg [15:0] right_distance_mm_i = 16'd0;
    reg imu_sample_start_i = 1'b0;
    reg imu_byte_valid_i = 1'b0;
    reg [7:0] imu_byte_data_i = 8'd0;
    reg camera_vsync_i = 1'b1;
    reg camera_href_i = 1'b0;
    reg [7:0] camera_data_i = 8'd0;
    reg path_start_i = 1'b0;
    reg path_abort_i = 1'b0;
    reg avoidance_tick_i = 1'b1;
    reg [31:0] segment_progress_i = 32'd0;
    reg motor_sample_tick_i = 1'b0;
    reg signed [31:0] left_velocity_mm_s_i = 32'sd0;
    reg signed [31:0] right_velocity_mm_s_i = 32'sd0;

    wire imu_byte_ready_o;
    wire [95:0] imu_sample_data_o;
    wire imu_sample_valid_o;
    wire camera_frame_valid_o;
    wire camera_line_valid_o;
    wire camera_pixel_valid_o;
    wire [15:0] camera_pixel_rgb565_o;
    wire active_mode_o;
    wire switch_pending_o;
    wire motion_enabled_o;
    wire signed [31:0] final_linear_mm_s_o;
    wire signed [31:0] final_angular_mrad_s_o;
    wire signed [31:0] left_target_mm_s_o;
    wire signed [31:0] right_target_mm_s_o;
    wire [15:0] left_pwm_duty_o;
    wire left_direction_o;
    wire [15:0] right_pwm_duty_o;
    wire right_direction_o;
    wire [15:0] front_range_mm_o;
    wire [15:0] left_range_mm_o;
    wire [15:0] right_range_mm_o;
    wire ranges_valid_o;
    wire [1:0] safety_level_o;
    wire avoidance_active_o;
    wire path_active_o;
    wire path_finished_o;
    wire path_progress_enable_o;
    wire path_paused_o;
    wire [1:0] path_segment_opcode_o;
    wire [7:0] segment_index_o;
    wire software_estop_o;
    wire control_timeout_pulse_o;
    wire control_packet_accepted_o;
    wire control_packet_rejected_o;

    reg [7:0] packet [0:30];
    integer checks = 0;
    integer guard = 0;

    robot_rtl_core #(
        .CONTROL_TIMEOUT_CYCLES(100), .RANGE_TIMEOUT_CYCLES(200),
        .AVOID_STOP_TICKS(1), .AVOID_BACKUP_TICKS(1),
        .AVOID_TURN_TICKS(1), .AVOID_BYPASS_TICKS(1), .AVOID_RECOVER_TICKS(1)
    ) dut (
        .clk_i(clk_i), .rst_n_i(rst_n_i),
        .requested_mode_i(requested_mode_i), .motion_stopped_i(motion_stopped_i),
        .physical_estop_i(physical_estop_i), .critical_fault_i(critical_fault_i),
        .udp_payload_valid_i(udp_payload_valid_i), .udp_payload_byte_i(udp_payload_byte_i),
        .udp_packet_done_i(udp_packet_done_i), .udp_packet_error_i(udp_packet_error_i),
        .front_sample_valid_i(front_sample_valid_i), .front_distance_mm_i(front_distance_mm_i),
        .left_sample_valid_i(left_sample_valid_i), .left_distance_mm_i(left_distance_mm_i),
        .right_sample_valid_i(right_sample_valid_i), .right_distance_mm_i(right_distance_mm_i),
        .imu_sample_start_i(imu_sample_start_i), .imu_byte_valid_i(imu_byte_valid_i),
        .imu_byte_data_i(imu_byte_data_i), .imu_byte_ready_o(imu_byte_ready_o),
        .imu_sample_data_o(imu_sample_data_o), .imu_sample_valid_o(imu_sample_valid_o),
        .camera_pclk_i(camera_pclk_i), .camera_vsync_i(camera_vsync_i),
        .camera_href_i(camera_href_i), .camera_data_i(camera_data_i),
        .camera_frame_valid_o(camera_frame_valid_o),
        .camera_line_valid_o(camera_line_valid_o),
        .camera_pixel_valid_o(camera_pixel_valid_o),
        .camera_pixel_rgb565_o(camera_pixel_rgb565_o),
        .path_start_i(path_start_i), .path_abort_i(path_abort_i),
        .avoidance_tick_i(avoidance_tick_i), .segment_progress_i(segment_progress_i),
        .motor_sample_tick_i(motor_sample_tick_i),
        .left_velocity_mm_s_i(left_velocity_mm_s_i),
        .right_velocity_mm_s_i(right_velocity_mm_s_i),
        .active_mode_o(active_mode_o), .switch_pending_o(switch_pending_o),
        .motion_enabled_o(motion_enabled_o),
        .final_linear_mm_s_o(final_linear_mm_s_o),
        .final_angular_mrad_s_o(final_angular_mrad_s_o),
        .left_target_mm_s_o(left_target_mm_s_o), .right_target_mm_s_o(right_target_mm_s_o),
        .left_pwm_duty_o(left_pwm_duty_o), .left_direction_o(left_direction_o),
        .right_pwm_duty_o(right_pwm_duty_o), .right_direction_o(right_direction_o),
        .front_range_mm_o(front_range_mm_o), .left_range_mm_o(left_range_mm_o),
        .right_range_mm_o(right_range_mm_o), .ranges_valid_o(ranges_valid_o),
        .safety_level_o(safety_level_o), .avoidance_active_o(avoidance_active_o),
        .path_active_o(path_active_o), .path_finished_o(path_finished_o),
        .path_progress_enable_o(path_progress_enable_o), .path_paused_o(path_paused_o),
        .path_segment_opcode_o(path_segment_opcode_o), .segment_index_o(segment_index_o),
        .software_estop_o(software_estop_o),
        .control_timeout_pulse_o(control_timeout_pulse_o),
        .control_packet_accepted_o(control_packet_accepted_o),
        .control_packet_rejected_o(control_packet_rejected_o)
    );

    always #5 clk_i = ~clk_i;
    always #4 camera_pclk_i = ~camera_pclk_i;

    function automatic [15:0] crc16_next;
        input [15:0] crc_in;
        input [7:0] data_in;
        integer bit_index;
        reg [15:0] work;
        begin
            work = crc_in ^ {data_in, 8'h00};
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1)
                work = work[15] ? ((work << 1) ^ 16'h1021) : (work << 1);
            crc16_next = work;
        end
    endfunction

    task automatic build_control_packet;
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
            for (i = 12; i < 20; i = i + 1) packet[i] = i;
            packet[20]=linear[31:24]; packet[21]=linear[23:16];
            packet[22]=linear[15:8]; packet[23]=linear[7:0];
            packet[24]=angular[31:24]; packet[25]=angular[23:16];
            packet[26]=angular[15:8]; packet[27]=angular[7:0]; packet[28]=flags;
            crc = 16'hffff;
            for (i = 0; i < 29; i = i + 1) crc = crc16_next(crc, packet[i]);
            packet[29]=crc[15:8]; packet[30]=crc[7:0];
        end
    endtask

    task automatic send_control_packet;
        input packet_error;
        integer i;
        begin
            for (i = 0; i < 31; i = i + 1) begin
                @(negedge clk_i); udp_payload_valid_i = 1'b1; udp_payload_byte_i = packet[i];
            end
            @(negedge clk_i);
            udp_payload_valid_i = 1'b0;
            udp_packet_error_i = packet_error;
            udp_packet_done_i = 1'b1;
            @(negedge clk_i);
            udp_packet_done_i = 1'b0;
            udp_packet_error_i = 1'b0;
        end
    endtask

    task automatic update_ranges;
        input [15:0] front;
        input [15:0] left;
        input [15:0] right;
        begin
            @(negedge clk_i);
            front_distance_mm_i = front; left_distance_mm_i = left; right_distance_mm_i = right;
            front_sample_valid_i = 1'b1; left_sample_valid_i = 1'b1; right_sample_valid_i = 1'b1;
            @(negedge clk_i);
            front_sample_valid_i = 1'b0; left_sample_valid_i = 1'b0; right_sample_valid_i = 1'b0;
        end
    endtask

    task automatic pulse_motor_sample;
        begin
            @(negedge clk_i); motor_sample_tick_i = 1'b1;
            @(negedge clk_i); motor_sample_tick_i = 1'b0;
        end
    endtask

    task automatic send_imu_byte;
        input [7:0] value;
        begin
            @(negedge clk_i); imu_byte_valid_i = 1'b1; imu_byte_data_i = value;
            @(negedge clk_i); imu_byte_valid_i = 1'b0;
        end
    endtask

    initial begin
        repeat (4) @(posedge clk_i);
        rst_n_i = 1'b1;

        update_ranges(16'd1000, 16'd800, 16'd400);
        #1;
        checks = checks + 1;
        if (!ranges_valid_o || front_range_mm_o !== 16'd1000)
            $fatal(1, "FAIL range integration");

        // 摄像头和 IMU 前端在系统骨架中保持原子提交接口。
        @(negedge camera_pclk_i); camera_vsync_i = 1'b0; camera_href_i = 1'b1; camera_data_i = 8'h12;
        @(negedge camera_pclk_i); camera_data_i = 8'h34;
        @(posedge camera_pclk_i); #1;
        checks = checks + 1;
        if (!camera_pixel_valid_o || camera_pixel_rgb565_o !== 16'h1234)
            $fatal(1, "FAIL camera integration");

        @(negedge clk_i); imu_sample_start_i = 1'b1;
        @(negedge clk_i); imu_sample_start_i = 1'b0;
        send_imu_byte(8'h00); send_imu_byte(8'h01); send_imu_byte(8'h00); send_imu_byte(8'h02);
        send_imu_byte(8'h00); send_imu_byte(8'h03); send_imu_byte(8'h00); send_imu_byte(8'h04);
        send_imu_byte(8'h00); send_imu_byte(8'h05); send_imu_byte(8'h00); send_imu_byte(8'h06);
        #1;
        checks = checks + 1;
        if (!imu_sample_valid_o || imu_sample_data_o !== 96'h0001_0002_0003_0004_0005_0006)
            $fatal(1, "FAIL IMU integration");

        // FPGA 独立模式从内部 ROM 装载首个 MOVE，并接入轮速 PI。
        @(negedge clk_i); path_start_i = 1'b1;
        @(negedge clk_i); path_start_i = 1'b0;
        repeat (3) @(posedge clk_i); #1;
        checks = checks + 1;
        if (active_mode_o || !motion_enabled_o || final_linear_mm_s_o !== 32'sd300 ||
            !path_active_o || !path_progress_enable_o || path_segment_opcode_o !== 2'd0)
            $fatal(1, "FAIL FPGA path integration linear=%0d", final_linear_mm_s_o);
        pulse_motor_sample(); #1;
        checks = checks + 1;
        if (left_pwm_duty_o == 0 || right_pwm_duty_o == 0)
            $fatal(1, "FAIL motor PI integration");

        update_ranges(16'd400, 16'd800, 16'd400); #1;
        checks = checks + 1;
        if (final_linear_mm_s_o !== 32'sd150 || safety_level_o !== 2'd1)
            $fatal(1, "FAIL FPGA slow zone");

        update_ranges(16'd100, 16'd800, 16'd300);
        repeat (4) @(posedge clk_i); #1;
        checks = checks + 1;
        if (!avoidance_active_o || path_progress_enable_o || !path_paused_o ||
            !(final_linear_mm_s_o < 0 || final_angular_mrad_s_o != 0))
            $fatal(1, "FAIL autonomous avoidance integration state=%0d linear=%0d",
                   dut.avoidance_state, final_linear_mm_s_o);

        // 清障后恢复原 MOVE，并完整执行内部 ROM 的 MOVE/ROTATE/MOVE/STOP。
        update_ranges(16'd1000, 16'd800, 16'd300);
        guard = 0;
        while (avoidance_active_o && guard < 12) begin
            @(posedge clk_i); #1; guard = guard + 1;
        end
        if (avoidance_active_o) $fatal(1, "FAIL avoidance did not finish");

        segment_progress_i = 32'd1000;
        @(posedge clk_i); #1;
        if (segment_index_o !== 8'd1) $fatal(1, "FAIL demo MOVE 0");
        segment_progress_i = 32'd0;
        @(posedge clk_i); #1;
        segment_progress_i = 32'd1571;
        @(posedge clk_i); #1;
        if (segment_index_o !== 8'd2) $fatal(1, "FAIL demo ROTATE 1");
        segment_progress_i = 32'd0;
        @(posedge clk_i); #1;
        segment_progress_i = 32'd500;
        @(posedge clk_i); #1;
        if (segment_index_o !== 8'd3) $fatal(1, "FAIL demo MOVE 2");
        segment_progress_i = 32'd0;
        repeat (2) @(posedge clk_i); #1;
        checks = checks + 1;
        if (!path_finished_o) $fatal(1, "FAIL demo STOP/path finish");

        // 重新启动一条未完成路径；离开 FPGA 模式必须中止，不能后台推进或复用。
        @(negedge clk_i); path_start_i = 1'b1;
        @(negedge clk_i); path_start_i = 1'b0;
        repeat (3) @(posedge clk_i); #1;
        if (!path_active_o) $fatal(1, "FAIL path restart before mode switch");

        // 切到 Jetson；切换保护期间不得透传任一来源。
        requested_mode_i = 1'b1;
        @(posedge clk_i); #1;
        if (!switch_pending_o || motion_enabled_o || path_active_o)
            $fatal(1, "FAIL switch guard/path abort");
        @(posedge clk_i); #1;
        if (!active_mode_o || switch_pending_o) $fatal(1, "FAIL Jetson mode switch");

        // Jetson 活动模式忽略独立路径启动脉冲。
        @(negedge clk_i); path_start_i = 1'b1;
        @(negedge clk_i); path_start_i = 1'b0;
        repeat (2) @(posedge clk_i); #1;
        checks = checks + 1;
        if (path_active_o || avoidance_active_o)
            $fatal(1, "FAIL FPGA path isolation in Jetson mode");
        update_ranges(16'd1000, 16'd800, 16'd400);

        build_control_packet(32'd1, 32'sd500, 32'sd0, 8'h00);
        send_control_packet(1'b0);
        repeat (2) @(posedge clk_i); #1;
        checks = checks + 1;
        if (!motion_enabled_o || final_linear_mm_s_o !== 32'sd500)
            $fatal(1, "FAIL Jetson command integration linear=%0d", final_linear_mm_s_o);

        // 软件急停报文清零缓存；只有后续新的非急停报文才能恢复。
        build_control_packet(32'd2, 32'sd900, 32'sd0, 8'h01);
        send_control_packet(1'b0);
        repeat (2) @(posedge clk_i); #1;
        checks = checks + 1;
        if (!software_estop_o || motion_enabled_o || final_linear_mm_s_o !== 0)
            $fatal(1, "FAIL software estop integration");
        build_control_packet(32'd3, 32'sd500, 32'sd0, 8'h00);
        send_control_packet(1'b0);
        repeat (2) @(posedge clk_i); #1;
        checks = checks + 1;
        if (software_estop_o || !motion_enabled_o || final_linear_mm_s_o !== 32'sd500)
            $fatal(1, "FAIL fresh command after estop release");

        // 坏以太网 FCS 报文不提交；既有有效命令在看门狗窗口内保持。
        build_control_packet(32'd4, 32'sd900, 32'sd0, 8'h00);
        send_control_packet(1'b1);
        #1;
        checks = checks + 1;
        if (!control_packet_rejected_o || final_linear_mm_s_o !== 32'sd500)
            $fatal(1, "FAIL bad-FCS atomic rejection");

        update_ranges(16'd400, 16'd800, 16'd400); #1;
        checks = checks + 1;
        if (final_linear_mm_s_o !== 32'sd250 || safety_level_o !== 2'd1)
            $fatal(1, "FAIL Jetson safety limiter");

        // 测距停车给出合法零目标时，两个 PI 都必须立即清积分和 PWM。
        pulse_motor_sample();
        update_ranges(16'd100, 16'd800, 16'd400); #1;
        checks = checks + 1;
        if (final_linear_mm_s_o !== 0 || left_pwm_duty_o !== 0 || right_pwm_duty_o !== 0)
            $fatal(1, "FAIL range stop PWM clear");

        physical_estop_i = 1'b1;
        @(posedge clk_i); #1;
        checks = checks + 1;
        if (motion_enabled_o || final_linear_mm_s_o !== 0 || left_pwm_duty_o !== 0)
            $fatal(1, "FAIL physical estop integration");
        physical_estop_i = 1'b0;
        @(posedge clk_i); #1;
        checks = checks + 1;
        if (motion_enabled_o || final_linear_mm_s_o !== 0)
            $fatal(1, "FAIL stale command after physical estop release");

        // 急停释放后只有新报文可恢复。
        update_ranges(16'd1000, 16'd800, 16'd400);
        build_control_packet(32'd5, 32'sd500, 32'sd0, 8'h00);
        send_control_packet(1'b0);
        repeat (2) @(posedge clk_i); #1;
        checks = checks + 1;
        if (!motion_enabled_o || final_linear_mm_s_o !== 32'sd500)
            $fatal(1, "FAIL fresh command after physical estop");

        critical_fault_i = 1'b1;
        @(posedge clk_i); #1;
        critical_fault_i = 1'b0;
        @(posedge clk_i); #1;
        checks = checks + 1;
        if (motion_enabled_o || final_linear_mm_s_o !== 0)
            $fatal(1, "FAIL stale command after critical fault release");

        build_control_packet(32'd6, 32'sd500, 32'sd0, 8'h00);
        send_control_packet(1'b0);
        repeat (2) @(posedge clk_i); #1;
        checks = checks + 1;
        if (!motion_enabled_o || final_linear_mm_s_o !== 32'sd500)
            $fatal(1, "FAIL fresh command after critical fault");

        // 无新控制报文超过窗口后关闭协同运动使能。
        repeat (110) @(posedge clk_i);
        #1;
        checks = checks + 1;
        if (motion_enabled_o || final_linear_mm_s_o !== 0)
            $fatal(1, "FAIL watchdog timeout");

        $display("PASS: robot_rtl_core %0d integration checks", checks);
        $finish;
    end
endmodule
