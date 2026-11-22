`timescale 1ns/1ps
// 前向近障保护：按 5 度保留观测证据，CPU 失效时仍可独立停车。
module bx72_lite_guard #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer SAMPLE_TIMEOUT_MS = 500,
    parameter integer SECTOR_TIMEOUT_MS = 500,
    parameter integer COMMAND_TIMEOUT_MS = 100,
    parameter integer CLEAR_DISTANCE_MM = 12000,
    // 现场记录的最大相邻角距为1.72度；超过3度视为覆盖缺口。
    parameter integer MAX_SAMPLE_GAP_CDEG = 300
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        sample_valid_i,
    input  wire        point_valid_i,
    input  wire [15:0] angle_cdeg_i,
    input  wire [15:0] distance_mm_i,
    input  wire        packet_error_i,
    input  wire        command_valid_i,
    input  wire [47:0] command_velocity_i,
    input  wire        autonomous_i,
    input  wire [15:0] front_stop_mm_i, front_slow_mm_i,
    input  wire        kill_i,
    output wire        command_valid_o,
    output wire [47:0] command_velocity_o,
    output wire        stop_o,
    output wire        slow_o,
    output wire        missing_o,
    output wire        obstacle_o,
    output wire        lidar_fresh_o
);
    localparam integer MS_DIVISOR = CLK_HZ / 1000;

    reg [15:0] ms_divider;
    reg [15:0] sensor_age_ms;
    reg [7:0] command_age_ms;
    reg command_seen;
    reg [15:0] front_age_0, front_age_7;
    reg [1:0] front_seen;
    reg gap_error_pending;
    // 前方两个扇区按 5 度保存证据，其他角度的缺口不阻止已重新观测方向更新。
    reg [17:0] front_bin_seen, front_bin_hit, front_bin_unknown;
    reg [15:0] front_bin_distance [0:17];
    // 平坦寄存器避免 Gowin 1.9.12.04 将扫描距离、角度数组综合错位。
    reg [18*16-1:0] front_bin_sweep_min;
    reg [15:0] front_bin_distance_angle [0:17];
    reg [18*16-1:0] front_bin_sweep_angle;
    // A missing ray elsewhere in the bin cannot invalidate observed clearance at the held angle.
    reg [17:0] front_bin_reobserved;
    reg [15:0] previous_angle_cdeg;
    reg previous_angle_valid, previous_hit;
    integer age_index;

    // 板内唯一数据源是N10解析器，已保证有效样本角度小于36000。
    wire valid_sample = (sample_valid_i || point_valid_i) && !packet_error_i;
    wire valid_hit = point_valid_i && distance_mm_i != 0 && distance_mm_i <= CLEAR_DISTANCE_MM;
    wire scan_wrap = valid_sample && previous_angle_valid &&
                     previous_angle_cdeg > 16'd30000 && angle_cdeg_i < 16'd6000;
    wire angle_backwards = previous_angle_valid && !scan_wrap && angle_cdeg_i < previous_angle_cdeg;
    wire [16:0] angle_gap = scan_wrap ? 17'd36000 + {1'b0,angle_cdeg_i} - {1'b0,previous_angle_cdeg} :
                            {1'b0,angle_cdeg_i} - {1'b0,previous_angle_cdeg};
    wire gap_unknown = previous_angle_valid &&
                       (gap_error_pending || angle_backwards || angle_gap > MAX_SAMPLE_GAP_CDEG);
    wire [17:0] front_bin_sample, front_bin_previous_gap, front_bin_current_gap;
    wire [17:0] front_bin_stop, front_bin_slow, front_bin_covered;
    genvar front_bin;
    generate for (front_bin=0; front_bin<18; front_bin=front_bin+1) begin: front_coverage_decode
        localparam integer BIN_BEGIN = front_bin < 9 ? front_bin * 500 : 31500 + (front_bin - 9) * 500;
        localparam integer BIN_END = BIN_BEGIN + 500;
        assign front_bin_sample[front_bin] = angle_cdeg_i >= BIN_BEGIN && angle_cdeg_i < BIN_END;
        assign front_bin_previous_gap[front_bin] = gap_unknown && scan_wrap && previous_angle_cdeg < BIN_END;
        assign front_bin_current_gap[front_bin] = !gap_unknown ? 1'b0 :
            scan_wrap ? angle_cdeg_i > BIN_BEGIN :
            angle_backwards ? 1'b1 :
            ((previous_angle_cdeg < BIN_END && angle_cdeg_i > BIN_BEGIN) ||
             (gap_error_pending && front_bin_sample[front_bin]));
        // Consecutive valid rays cover the held angle despite scan jitter;
        // an invalid ray or excessive angular gap cannot provide clearance.
        assign front_bin_covered[front_bin] = valid_hit &&
            front_bin_distance[front_bin] != 16'hffff &&
            ((angle_cdeg_i == front_bin_distance_angle[front_bin]) ||
             (previous_angle_valid && previous_hit && !gap_unknown &&
              (scan_wrap ? (front_bin_distance_angle[front_bin] >= previous_angle_cdeg ||
                            front_bin_distance_angle[front_bin] <= angle_cdeg_i) :
                           (front_bin_distance_angle[front_bin] >= previous_angle_cdeg &&
                            front_bin_distance_angle[front_bin] <= angle_cdeg_i))));
        assign front_bin_stop[front_bin] = front_bin_distance[front_bin] <= front_stop_mm_i;
        assign front_bin_slow[front_bin] = front_bin_distance[front_bin] <= front_slow_mm_i;
    end endgenerate

    always @(posedge clk) begin
        if (rst) begin
            ms_divider <= 0;
            sensor_age_ms <= 16'hffff;
            command_age_ms <= 8'hff;
            command_seen <= 1'b0;
            front_seen <= 0;
            front_age_0 <= 16'hffff;
            front_age_7 <= 16'hffff;
            front_bin_seen <= 0;
            front_bin_reobserved <= 0;
            front_bin_hit <= 0;
            front_bin_unknown <= 18'h3ffff;
            gap_error_pending <= 1'b0;
            previous_angle_cdeg <= 0;
            previous_angle_valid <= 1'b0;
            previous_hit <= 1'b0;
            for (age_index = 0; age_index < 18; age_index = age_index + 1) begin
                front_bin_distance[age_index] <= 16'hffff;
                front_bin_distance_angle[age_index] <= 0;
                front_bin_sweep_angle[age_index*16 +: 16] <= 0;
                front_bin_sweep_min[age_index*16 +: 16] <= 16'hffff;
            end
        end else begin
            if (ms_divider == MS_DIVISOR - 1) begin
                ms_divider <= 0;
                if (sensor_age_ms != 16'hffff) sensor_age_ms <= sensor_age_ms + 1'b1;
                if (command_age_ms != 8'hff) command_age_ms <= command_age_ms + 1'b1;
                if (front_age_0 != 16'hffff) front_age_0 <= front_age_0 + 1'b1;
                if (front_age_7 != 16'hffff) front_age_7 <= front_age_7 + 1'b1;
            end else ms_divider <= ms_divider + 1'b1;

            if (packet_error_i) gap_error_pending <= 1'b1;
            if (valid_sample) begin
                gap_error_pending <= 1'b0;
                sensor_age_ms <= 0;
                if (valid_hit) begin
                    if (angle_cdeg_i < 16'd4500) begin
                        front_age_0 <= 0;
                        front_seen[0] <= 1'b1;
                    end else if (angle_cdeg_i >= 16'd31500) begin
                        front_age_7 <= 0;
                        front_seen[1] <= 1'b1;
                    end
                end
                previous_angle_cdeg <= angle_cdeg_i;
                previous_angle_valid <= 1'b1;
                previous_hit <= valid_hit;
                if (scan_wrap) begin
                    for (age_index = 0; age_index < 18; age_index = age_index + 1) begin
                        if (front_bin_seen[age_index] && front_bin_hit[age_index] &&
                            ((!front_bin_unknown[age_index] && !front_bin_previous_gap[age_index]) ||
                             front_bin_reobserved[age_index] || front_bin_covered[age_index])) begin
                            if (front_bin_sample[age_index] && valid_hit && distance_mm_i < front_bin_sweep_min[age_index*16 +: 16])
                                begin
                                front_bin_distance[age_index] <= distance_mm_i;
                                front_bin_distance_angle[age_index] <= angle_cdeg_i;
                            end else begin
                                front_bin_distance[age_index] <= front_bin_sweep_min[age_index*16 +: 16];
                                front_bin_distance_angle[age_index] <= front_bin_sweep_angle[age_index*16 +: 16];
                            end
                        end else if (front_bin_sample[age_index] && valid_hit && distance_mm_i < front_bin_distance[age_index]) begin
                            front_bin_distance[age_index] <= distance_mm_i;
                            front_bin_distance_angle[age_index] <= angle_cdeg_i;
                        end
                        front_bin_sweep_angle[age_index*16 +: 16] <= angle_cdeg_i;
                        front_bin_sweep_min[age_index*16 +: 16] <= (front_bin_sample[age_index] && valid_hit) ? distance_mm_i : 16'hffff;
                    end
                    // Wrap coverage on the ending side has already been consumed above.
                    front_bin_reobserved <= front_bin_covered & front_bin_sample;
                    front_bin_seen <= front_bin_sample;
                    front_bin_hit <= valid_hit ? front_bin_sample : 18'd0;
                    front_bin_unknown <= front_bin_current_gap | (valid_hit ? 18'd0 : front_bin_sample);
                end else begin
                    front_bin_seen <= front_bin_seen | front_bin_sample;
                    front_bin_unknown <= front_bin_unknown | front_bin_current_gap |
                                         (valid_hit ? 18'd0 : front_bin_sample);
                    if (valid_hit) begin
                        front_bin_hit <= front_bin_hit | front_bin_sample;
                        for (age_index = 0; age_index < 18; age_index = age_index + 1)
                            if (front_bin_sample[age_index]) begin
                                if (distance_mm_i < front_bin_sweep_min[age_index*16 +: 16]) begin
                                    front_bin_sweep_min[age_index*16 +: 16] <= distance_mm_i;
                                    front_bin_sweep_angle[age_index*16 +: 16] <= angle_cdeg_i;
                                end
                                if (distance_mm_i < front_bin_distance[age_index]) begin
                                    front_bin_distance[age_index] <= distance_mm_i;
                                    front_bin_distance_angle[age_index] <= angle_cdeg_i;
                                end
                            end
                    end
                    front_bin_reobserved <= front_bin_reobserved | front_bin_covered;
                end
            end

            if (command_valid_i) begin
                command_age_ms <= 0;
                command_seen <= 1'b1;
            end
        end
    end

    /* 正常命令经四拍完成原子锁存、方向解码、并行威胁比较和速度/valid发布。
     * kill、命令过期及雷达超时绕过流水，立即输出零速。
     */
    reg signed [15:0] s0_vx, s0_vy, s0_wz;
    reg s0_valid, s0_autonomous;
    reg [47:0] s1_velocity;
    reg s1_valid, s1_forward, s1_rotation, s1_autonomous;
    reg [47:0] s2_velocity;
    reg s2_valid, s2_unsafe, s2_slow, s2_autonomous;
    reg [47:0] s3_velocity;
    reg s3_valid, s3_stop, s3_slow;
    wire command_stale = !command_seen || command_age_ms >= COMMAND_TIMEOUT_MS;
    wire immediate_stop = kill_i || command_stale || !lidar_fresh_o ||
        front_stop_mm_i == 0 || front_slow_mm_i < front_stop_mm_i;

    // 仅前进或旋转检查前方；纯侧移、后退不做距离停车。
    wire s1_check_front = s1_rotation || s1_forward;
    wire s1_sector_data_missing = s1_check_front &&
        (!front_seen[0] || front_age_0 >= SECTOR_TIMEOUT_MS ||
         !front_seen[1] || front_age_7 >= SECTOR_TIMEOUT_MS);
    wire s1_stop_threat = s1_check_front && (|front_bin_stop);
    wire s1_slow_threat = s1_check_front && (|front_bin_slow);
    // Removing forward/turning components retains the checked lateral envelope.
    wire s2_lateral_escape = s2_slow && s2_autonomous && s2_velocity[31:16] != 0;

    assign missing_o = !lidar_fresh_o || s1_sector_data_missing;
    assign obstacle_o = s1_stop_threat;
    assign lidar_fresh_o = sensor_age_ms < SAMPLE_TIMEOUT_MS;
    assign stop_o = immediate_stop || s3_stop;
    assign slow_o = s3_slow && !immediate_stop;
    assign command_valid_o = immediate_stop || s3_valid;
    assign command_velocity_o = immediate_stop ? 48'd0 : s3_velocity;

    always @(posedge clk) begin
        if (rst) begin
            s0_vx <= 0; s0_vy <= 0; s0_wz <= 0;
            s0_valid <= 0; s0_autonomous <= 0;
            s1_velocity <= 0;
            s1_valid <= 0; s1_forward <= 0; s1_rotation <= 0; s1_autonomous <= 0;
            s2_velocity <= 0; s2_valid <= 0; s2_unsafe <= 0; s2_slow <= 0; s2_autonomous <= 0;
            s3_velocity <= 0; s3_valid <= 0; s3_stop <= 0; s3_slow <= 0;
        end else begin
            // 第0拍：原子锁存三轴命令及有效标记。
            s0_vx <= command_velocity_i[47:32];
            s0_vy <= command_velocity_i[31:16];
            s0_wz <= command_velocity_i[15:0];
            s0_valid <= command_valid_i;
            s0_autonomous <= autonomous_i;

            // 第1拍：记录前进和旋转状态。
            s1_velocity <= {s0_vx, s0_vy, s0_wz};
            s1_valid <= s0_valid;
            s1_autonomous <= s0_autonomous;
            s1_forward <= s0_vx > 0;
            s1_rotation <= s0_wz != 0;

            // 第2拍：各前向角度并行比较产生威胁位图。
            s2_velocity <= s1_velocity;
            s2_valid <= s1_valid;
            s2_autonomous <= s1_autonomous;
            s2_unsafe <= immediate_stop || s1_sector_data_missing || s1_stop_threat;
            s2_slow <= s1_slow_threat;

            // 第3拍：同批发布速度和valid，无串行最小距离比较链。
            s3_valid <= s2_valid || s2_unsafe || s2_slow;
            s3_stop <= s2_unsafe;
            s3_slow <= s2_slow && !s2_unsafe;
            if (s2_unsafe) s3_velocity <= 48'd0;
            else if (s2_lateral_escape)
                s3_velocity <= {16'd0, s2_velocity[31:16], 16'd0};
            else if (s2_slow)
                s3_velocity <= {$signed(s2_velocity[47:32]) >>> 1,
                                $signed(s2_velocity[31:16]) >>> 1,
                                $signed(s2_velocity[15:0]) >>> 1};
            else s3_velocity <= s2_velocity;
        end
    end
endmodule
