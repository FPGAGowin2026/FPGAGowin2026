`timescale 1ns/1ps
// 轻量近障保护：按8个45度扇区保存最近距离与采样年龄，CPU失效时仍可独立停车。
module bx72_lite_guard #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer SAMPLE_TIMEOUT_MS = 500,
    parameter integer SECTOR_TIMEOUT_MS = 500,
    parameter integer COMMAND_TIMEOUT_MS = 100,
    parameter integer FRONT_STOP_DISTANCE_MM = 150,
    parameter integer SLOW_DISTANCE_MM = 700,
    parameter integer ROTATION_SLOW_DISTANCE_MM = 300,
    parameter integer CLEAR_DISTANCE_MM = 8000,
    // 现场记录的最大相邻角距为1.72度；超过3度视为覆盖缺口。
    parameter integer MAX_SAMPLE_GAP_CDEG = 300,
    // 临时调试构建可显式定义；默认保留近障停车及减速。
`ifdef BX72_TEMP_BYPASS_PROXIMITY
    parameter integer BYPASS_PROXIMITY = 1
`else
    parameter integer BYPASS_PROXIMITY = 0
`endif
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
    input  wire        kill_i,
    output wire        command_valid_o,
    output wire [47:0] command_velocity_o,
    output wire        stop_o,
    output wire        slow_o,
    output wire        lidar_fresh_o
);
    localparam integer MS_DIVISOR = CLK_HZ / 1000;

    reg [15:0] ms_divider;
    reg [15:0] sensor_age_ms;
    reg [7:0] command_age_ms;
    reg command_seen;
    reg [15:0] sector_distance [0:7];
    reg [15:0] sector_age_ms [0:7];
    reg [7:0] sector_seen;
    reg [7:0] sweep_seen;
    reg [7:0] sweep_hit;
    reg [7:0] sweep_unknown;
    reg gap_error_pending;
    reg [15:0] sweep_min_distance [0:7];
    reg [15:0] previous_angle_cdeg;
    reg previous_angle_valid;
    integer age_index;

    function automatic [2:0] sector_index(input [15:0] angle_cdeg);
        begin
            if      (angle_cdeg < 16'd4500)  sector_index = 3'd0;
            else if (angle_cdeg < 16'd9000)  sector_index = 3'd1;
            else if (angle_cdeg < 16'd13500) sector_index = 3'd2;
            else if (angle_cdeg < 16'd18000) sector_index = 3'd3;
            else if (angle_cdeg < 16'd22500) sector_index = 3'd4;
            else if (angle_cdeg < 16'd27000) sector_index = 3'd5;
            else if (angle_cdeg < 16'd31500) sector_index = 3'd6;
            else                             sector_index = 3'd7;
        end
    endfunction

    // 板内唯一数据源是N10解析器，已保证有效样本角度小于36000。
    wire valid_sample = (sample_valid_i || point_valid_i) && !packet_error_i;
    wire [2:0] sample_sector = sector_index(angle_cdeg_i);
    wire valid_hit = point_valid_i && distance_mm_i != 0 && distance_mm_i <= CLEAR_DISTANCE_MM;
    wire [7:0] sample_sector_mask = 8'b1 << sample_sector;
    wire scan_wrap = valid_sample && previous_angle_valid &&
                     previous_angle_cdeg > 16'd30000 && angle_cdeg_i < 16'd6000;
    wire [2:0] previous_sector = sector_index(previous_angle_cdeg);
    wire angle_backwards = previous_angle_valid && !scan_wrap && angle_cdeg_i < previous_angle_cdeg;
    wire [16:0] angle_gap = scan_wrap ? 17'd36000 + {1'b0,angle_cdeg_i} - {1'b0,previous_angle_cdeg} :
                            {1'b0,angle_cdeg_i} - {1'b0,previous_angle_cdeg};
    wire gap_unknown = previous_angle_valid &&
                       (gap_error_pending || angle_backwards || angle_gap > MAX_SAMPLE_GAP_CDEG);
    // 缺口跨扇区时，只标记该扇区内实际超过阈值的未覆盖角度。
    // 回卷时把末尾和开头分别归入结束圈和新圈。
    wire [7:0] previous_sweep_gap, current_sweep_gap;
    genvar coverage_sector;
    generate for (coverage_sector=0; coverage_sector<8; coverage_sector=coverage_sector+1) begin: coverage_decode
        localparam integer SECTOR_BEGIN = coverage_sector * 4500;
        localparam integer SECTOR_END = (coverage_sector + 1) * 4500;
        wire tail_gap = {1'b0,previous_angle_cdeg} + MAX_SAMPLE_GAP_CDEG < SECTOR_END;
        wire head_gap = {1'b0,angle_cdeg_i} > SECTOR_BEGIN + MAX_SAMPLE_GAP_CDEG;
        wire previous_end = previous_sector == coverage_sector;
        wire current_end = sample_sector == coverage_sector;
        assign previous_sweep_gap[coverage_sector] = gap_unknown && scan_wrap &&
            ((coverage_sector > previous_sector) || (previous_end && (tail_gap || gap_error_pending)));
        assign current_sweep_gap[coverage_sector] = !gap_unknown ? 1'b0 :
            scan_wrap ? ((coverage_sector < sample_sector) || (current_end && (head_gap || gap_error_pending))) :
            angle_backwards ? 1'b1 :
            (previous_sector == sample_sector) ? previous_end :
            ((coverage_sector > previous_sector && coverage_sector < sample_sector) ||
             (previous_end && (tail_gap || gap_error_pending)) ||
             (current_end && (head_gap || gap_error_pending)));
    end endgenerate

    always @(posedge clk) begin
        if (rst) begin
            ms_divider <= 0;
            sensor_age_ms <= 16'hffff;
            command_age_ms <= 8'hff;
            command_seen <= 1'b0;
            sector_seen <= 0;
            sweep_seen <= 0;
            sweep_hit <= 0;
            // 第一圈可能从任意角度开始，不能用于覆盖更近的旧值。
            sweep_unknown <= 8'hff;
            gap_error_pending <= 1'b0;
            previous_angle_cdeg <= 0;
            previous_angle_valid <= 1'b0;
            for (age_index = 0; age_index < 8; age_index = age_index + 1) begin
                sector_distance[age_index] <= 16'hffff;
                sector_age_ms[age_index] <= 16'hffff;
                sweep_min_distance[age_index] <= 16'hffff;
            end
        end else begin
            if (ms_divider == MS_DIVISOR - 1) begin
                ms_divider <= 0;
                if (sensor_age_ms != 16'hffff) sensor_age_ms <= sensor_age_ms + 1'b1;
                if (command_age_ms != 8'hff) command_age_ms <= command_age_ms + 1'b1;
                for (age_index = 0; age_index < 8; age_index = age_index + 1)
                    if (sector_age_ms[age_index] != 16'hffff)
                        sector_age_ms[age_index] <= sector_age_ms[age_index] + 1'b1;
            end else ms_divider <= ms_divider + 1'b1;

            if (packet_error_i) gap_error_pending <= 1'b1;
            if (valid_sample) begin
                gap_error_pending <= 1'b0;
                sensor_age_ms <= 0;
                if (valid_hit) begin
                    sector_age_ms[sample_sector] <= 0;
                    sector_seen[sample_sector] <= 1'b1;
                end
                previous_angle_cdeg <= angle_cdeg_i;
                previous_angle_valid <= 1'b1;
                if (scan_wrap) begin
                    for (age_index = 0; age_index < 8; age_index = age_index + 1) begin
                        if (sweep_seen[age_index] && sweep_hit[age_index] &&
                            !sweep_unknown[age_index] && !previous_sweep_gap[age_index] &&
                            !(valid_hit && sample_sector == age_index))
                            sector_distance[age_index] <= sweep_min_distance[age_index];
                    end
                    for (age_index = 0; age_index < 8; age_index = age_index + 1)
                        sweep_min_distance[age_index] <= 16'hffff;
                    sweep_seen <= sample_sector_mask;
                    sweep_hit <= valid_hit ? sample_sector_mask : 8'd0;
                    sweep_unknown <= current_sweep_gap;
                    if (valid_hit) sweep_min_distance[sample_sector] <= distance_mm_i;
                end else begin
                    sweep_seen <= sweep_seen | sample_sector_mask;
                    sweep_unknown <= sweep_unknown | current_sweep_gap;
                    if (valid_hit) begin
                        sweep_hit <= sweep_hit | sample_sector_mask;
                        if (distance_mm_i < sweep_min_distance[sample_sector])
                            sweep_min_distance[sample_sector] <= distance_mm_i;
                    end
                end
                if (valid_hit) begin
                    /* 近点立即生效；更远的值仅在该扇区角度完整且有有效回波时替换。 */
                    if (scan_wrap && sweep_seen[sample_sector] && sweep_hit[sample_sector] &&
                        !sweep_unknown[sample_sector] && !previous_sweep_gap[sample_sector]) begin
                        if (sweep_min_distance[sample_sector] < distance_mm_i)
                            sector_distance[sample_sector] <= sweep_min_distance[sample_sector];
                        else
                            sector_distance[sample_sector] <= distance_mm_i;
                    end else if (distance_mm_i < sector_distance[sample_sector])
                        sector_distance[sample_sector] <= distance_mm_i;
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
    reg s0_valid;
    reg [47:0] s1_velocity;
    reg s1_valid, s1_forward, s1_rotation;
    reg [47:0] s2_velocity;
    reg s2_valid, s2_unsafe, s2_slow;
    reg [47:0] s3_velocity;
    reg s3_valid, s3_stop, s3_slow;
    wire command_stale = !command_seen || command_age_ms >= COMMAND_TIMEOUT_MS;
    wire immediate_stop = kill_i || command_stale || !lidar_fresh_o;

    wire [7:0] sector_fresh_bits;
    wire [7:0] sector_stop_bits;
    wire [7:0] sector_slow_bits;
    wire [7:0] sector_rotation_slow_bits;
    genvar threat_sector;
    generate for (threat_sector = 0; threat_sector < 8; threat_sector = threat_sector + 1) begin: threat_decode
        assign sector_fresh_bits[threat_sector] = sector_seen[threat_sector] &&
            sector_age_ms[threat_sector] < SECTOR_TIMEOUT_MS;
        assign sector_stop_bits[threat_sector] = sector_distance[threat_sector] <= FRONT_STOP_DISTANCE_MM;
        assign sector_slow_bits[threat_sector] = sector_distance[threat_sector] <= SLOW_DISTANCE_MM;
        assign sector_rotation_slow_bits[threat_sector] = sector_distance[threat_sector] <= ROTATION_SLOW_DISTANCE_MM;
    end endgenerate

    // 仅前进或旋转检查前方；纯侧移、后退不做距离停车。
    wire [7:0] s1_selection_mask = (s1_rotation || s1_forward) ? 8'h81 : 8'h00;
    wire s1_sector_data_missing = s1_selection_mask != 0 &&
        ((sector_fresh_bits & s1_selection_mask) != s1_selection_mask);
    wire s1_stop_threat = (sector_stop_bits & s1_selection_mask) != 0;
    wire s1_slow_threat = ((s1_forward ? sector_slow_bits : sector_rotation_slow_bits) & s1_selection_mask) != 0;

    assign lidar_fresh_o = sensor_age_ms < SAMPLE_TIMEOUT_MS;
    assign stop_o = immediate_stop || s3_stop;
    assign slow_o = s3_slow && !immediate_stop;
    assign command_valid_o = immediate_stop || s3_valid;
    assign command_velocity_o = immediate_stop ? 48'd0 : s3_velocity;

    always @(posedge clk) begin
        if (rst) begin
            s0_vx <= 0; s0_vy <= 0; s0_wz <= 0;
            s0_valid <= 0;
            s1_velocity <= 0;
            s1_valid <= 0; s1_forward <= 0; s1_rotation <= 0;
            s2_velocity <= 0; s2_valid <= 0; s2_unsafe <= 0; s2_slow <= 0;
            s3_velocity <= 0; s3_valid <= 0; s3_stop <= 0; s3_slow <= 0;
        end else begin
            // 第0拍：原子锁存三轴命令及有效标记。
            s0_vx <= command_velocity_i[47:32];
            s0_vy <= command_velocity_i[31:16];
            s0_wz <= command_velocity_i[15:0];
            s0_valid <= command_valid_i;

            // 第1拍：记录前进和旋转状态。
            s1_velocity <= {s0_vx, s0_vy, s0_wz};
            s1_valid <= s0_valid;
            s1_forward <= s0_vx > 0;
            s1_rotation <= s0_wz != 0;

            // 第2拍：八个并行比较产生威胁位图。
            s2_velocity <= s1_velocity;
            s2_valid <= s1_valid;
            s2_unsafe <= immediate_stop || s1_sector_data_missing || (!BYPASS_PROXIMITY && s1_stop_threat);
            s2_slow <= !BYPASS_PROXIMITY && (s1_forward || s1_rotation) && s1_slow_threat;

            // 第3拍：同批发布速度和valid，无串行最小距离比较链。
            s3_valid <= s2_valid || s2_unsafe || s2_slow;
            s3_stop <= s2_unsafe;
            s3_slow <= s2_slow && !s2_unsafe;
            if (s2_unsafe) s3_velocity <= 48'd0;
            else if (s2_slow)
                s3_velocity <= {$signed(s2_velocity[47:32]) >>> 1,
                                $signed(s2_velocity[31:16]) >>> 1,
                                $signed(s2_velocity[15:0]) >>> 1};
            else s3_velocity <= s2_velocity;
        end
    end
endmodule
