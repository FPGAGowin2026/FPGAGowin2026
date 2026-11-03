// ==================== ov5647_autocal_eye_select.v ====================
`timescale 1ns/1ps
// 32x32 tap 通过图的最大内接矩形选择器。
module ov5647_autocal_eye_select (
    input  wire          byte_clk,
    input  wire          reset_n,
    input  wire          start,
    input  wire [1023:0] pass_map,
    input  wire [1023:0] blocked_map,
    output reg           busy,
    output reg           done,
    output reg           found,
    output reg  [4:0]    top_out,
    output reg  [4:0]    bottom_out,
    output reg  [4:0]    left_out,
    output reg  [4:0]    right_out,
    output reg  [4:0]    center_row_index,
    output reg  [4:0]    center_col_index,
    output reg  [7:0]    center_tap_lane0,
    output reg  [7:0]    center_tap_lane1
);
    localparam [3:0] ST_IDLE          = 4'd0;
    localparam [3:0] ST_ROW_READ      = 4'd1;
    localparam [3:0] ST_ROW_APPLY     = 4'd2;
    localparam [3:0] ST_COL           = 4'd3;
    localparam [3:0] ST_CAND_AREA     = 4'd4;
    localparam [3:0] ST_CAND_COMPARE  = 4'd5;
    localparam [3:0] ST_CAND_UPDATE   = 4'd6;
    localparam [3:0] ST_NEXT          = 4'd7;
    localparam [3:0] ST_DONE          = 4'd8;

    reg [3:0] state;
    reg [4:0] top_index;
    reg [4:0] bottom_index;
    reg [4:0] col_index;
    reg [31:0] row_intersection;
    reg [4:0] run_start;
    reg [5:0] run_length;
    reg [11:0] best_area;
    reg [5:0] best_min_dim;
    reg [4:0] best_top;
    reg [4:0] best_bottom;
    reg [4:0] best_left;
    reg [4:0] best_right;
    reg [31:0] current_row_mask_q;
    reg [5:0] proposal_length;
    reg [5:0] proposal_height;
    reg [5:0] proposal_min_dim;
    reg [11:0] proposal_area;
    reg [4:0] proposal_top;
    reg [4:0] proposal_bottom;
    reg [4:0] proposal_left;
    reg [4:0] proposal_right;
    reg proposal_area_greater_q;
    reg proposal_area_equal_q;
    reg proposal_dim_greater_q;
    reg proposal_dim_equal_q;
    reg proposal_top_less_q;
    reg proposal_top_equal_q;
    reg proposal_left_less_q;

    wire [9:0] row_base = {bottom_index, 5'b00000};
    wire [31:0] current_row_mask =
        pass_map[row_base +: 32] & ~blocked_map[row_base +: 32];
    wire cell_pass = row_intersection[col_index];
    wire [5:0] span_length = cell_pass ? (run_length + 6'd1) : run_length;
    wire [4:0] span_left = (run_length == 0) ? col_index : run_start;
    // 有效跨度右端不超过 31；先在 6 位中计算，再显式取列索引低 5 位。
    wire [5:0] span_right = {1'b0, span_left} + span_length - 6'd1;
    wire span_end = !cell_pass || (col_index == 5'd31);
    wire [5:0] rect_height = {1'b0, bottom_index} - {1'b0, top_index} + 6'd1;
    wire [5:0] candidate_min_dim =
        (span_length < rect_height) ? span_length : rect_height;
    wire [11:0] proposal_area_next =
        {6'd0, proposal_length} * {6'd0, proposal_height};
    wire candidate_better = proposal_area_greater_q ||
        (proposal_area_equal_q && proposal_dim_greater_q) ||
        (proposal_area_equal_q && proposal_dim_equal_q && proposal_top_less_q) ||
        (proposal_area_equal_q && proposal_dim_equal_q && proposal_top_equal_q &&
         proposal_left_less_q);
    wire [5:0] center_row_sum = {1'b0, best_top} + {1'b0, best_bottom};
    wire [5:0] center_col_sum = {1'b0, best_left} + {1'b0, best_right};

    always @(posedge byte_clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= ST_IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            found <= 1'b0;
            top_out <= 5'd0;
            bottom_out <= 5'd0;
            left_out <= 5'd0;
            right_out <= 5'd0;
            center_row_index <= 5'd0;
            center_col_index <= 5'd0;
            center_tap_lane0 <= 8'd0;
            center_tap_lane1 <= 8'd0;
            top_index <= 5'd0;
            bottom_index <= 5'd0;
            col_index <= 5'd0;
            row_intersection <= 32'd0;
            run_start <= 5'd0;
            run_length <= 6'd0;
            best_area <= 12'd0;
            best_min_dim <= 6'd0;
            best_top <= 5'd0;
            best_bottom <= 5'd0;
            best_left <= 5'd0;
            best_right <= 5'd0;
            current_row_mask_q <= 32'd0;
            proposal_length <= 6'd0;
            proposal_height <= 6'd0;
            proposal_min_dim <= 6'd0;
            proposal_area <= 12'd0;
            proposal_top <= 5'd0;
            proposal_bottom <= 5'd0;
            proposal_left <= 5'd0;
            proposal_right <= 5'd0;
            proposal_area_greater_q <= 1'b0;
            proposal_area_equal_q <= 1'b0;
            proposal_dim_greater_q <= 1'b0;
            proposal_dim_equal_q <= 1'b0;
            proposal_top_less_q <= 1'b0;
            proposal_top_equal_q <= 1'b0;
            proposal_left_less_q <= 1'b0;
        end else begin
            done <= 1'b0;
            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;
                        found <= 1'b0;
                        top_index <= 5'd0;
                        bottom_index <= 5'd0;
                        col_index <= 5'd0;
                        row_intersection <= 32'hffff_ffff;
                        run_start <= 5'd0;
                        run_length <= 6'd0;
                        best_area <= 12'd0;
                        best_min_dim <= 6'd0;
                        best_top <= 5'd31;
                        best_bottom <= 5'd31;
                        best_left <= 5'd31;
                        best_right <= 5'd31;
                        state <= ST_ROW_READ;
                    end
                end

                ST_ROW_READ: begin
                    // 先单独寄存32列掩码，避免变量行读取与列交集串在一拍。
                    current_row_mask_q <= current_row_mask;
                    state <= ST_ROW_APPLY;
                end

                ST_ROW_APPLY: begin
                    row_intersection <= row_intersection & current_row_mask_q;
                    col_index <= 5'd0;
                    run_start <= 5'd0;
                    run_length <= 6'd0;
                    state <= ST_COL;
                end

                ST_COL: begin
                    if (cell_pass) begin
                        if (run_length == 0)
                            run_start <= col_index;
                        run_length <= run_length + 6'd1;
                    end else begin
                        run_length <= 6'd0;
                    end

                    if (span_end && (span_length != 0)) begin
                        proposal_length <= span_length;
                        proposal_height <= rect_height;
                        proposal_min_dim <= candidate_min_dim;
                        proposal_top <= top_index;
                        proposal_bottom <= bottom_index;
                        proposal_left <= span_left;
                        proposal_right <= span_right[4:0];
                        state <= ST_CAND_AREA;
                    end else begin
                        if (col_index == 5'd31)
                            state <= ST_NEXT;
                        else
                            col_index <= col_index + 5'd1;
                    end
                end

                ST_CAND_AREA: begin
                    proposal_area <= proposal_area_next;
                    state <= ST_CAND_COMPARE;
                end

                ST_CAND_COMPARE: begin
                    proposal_area_greater_q <= proposal_area > best_area;
                    proposal_area_equal_q <= proposal_area == best_area;
                    proposal_dim_greater_q <= proposal_min_dim > best_min_dim;
                    proposal_dim_equal_q <= proposal_min_dim == best_min_dim;
                    proposal_top_less_q <= proposal_top < best_top;
                    proposal_top_equal_q <= proposal_top == best_top;
                    proposal_left_less_q <= proposal_left < best_left;
                    state <= ST_CAND_UPDATE;
                end

                ST_CAND_UPDATE: begin
                    if (candidate_better) begin
                        best_area <= proposal_area;
                        best_min_dim <= proposal_min_dim;
                        best_top <= proposal_top;
                        best_bottom <= proposal_bottom;
                        best_left <= proposal_left;
                        best_right <= proposal_right;
                    end
                    if (col_index == 5'd31)
                        state <= ST_NEXT;
                    else begin
                        col_index <= col_index + 5'd1;
                        state <= ST_COL;
                    end
                end

                ST_NEXT: begin
                    if (bottom_index != 5'd31) begin
                        bottom_index <= bottom_index + 5'd1;
                        state <= ST_ROW_READ;
                    end else if (top_index != 5'd31) begin
                        top_index <= top_index + 5'd1;
                        bottom_index <= top_index + 5'd1;
                        row_intersection <= 32'hffff_ffff;
                        state <= ST_ROW_READ;
                    end else begin
                        state <= ST_DONE;
                    end
                end

                ST_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    found <= (best_area != 0);
                    if (best_area != 0) begin
                        top_out <= best_top;
                        bottom_out <= best_bottom;
                        left_out <= best_left;
                        right_out <= best_right;
                        center_row_index <= center_row_sum[5:1];
                        center_col_index <= center_col_sum[5:1];
                        // 网格步长为 8 tap，inclusive 矩形中心为 4*(lo+hi)。
                        center_tap_lane0 <= {center_row_sum, 2'b00};
                        center_tap_lane1 <= {center_col_sum, 2'b00};
                    end
                    state <= ST_IDLE;
                end

                default: begin
                    state <= ST_IDLE;
                    busy <= 1'b0;
                    found <= 1'b0;
                end
            endcase
        end
    end
endmodule

// ==================== ov5647_dphy_autocal.v ====================
`timescale 1ns/1ps
// 双 lane IODELAY 自动训练：每个 HS burst 只按首个合格 CSI 头评分。
module ov5647_dphy_autocal #(
    parameter integer SKIP_BURSTS = 2,
    parameter integer POINT_BURSTS = 4096,
    parameter integer VERIFY_BURSTS = 4096,
    parameter integer VERIFY_MIN_CYCLES = 109375000,
    parameter integer FRAME_TIMEOUT_CYCLES = 100000000,
    parameter integer LOAD_TIMEOUT_CYCLES = 50000000,
    parameter integer RECOVERY_WAIT_CYCLES = 109375000
)(
    input  wire        byte_clk,
    input  wire        reset_n,
    input  wire        start_enable,
    input  wire        burst_active,
    input  wire        burst_hs_started,
    input  wire [1:0]  lp_lane0_raw,
    input  wire [1:0]  lp_lane1_raw,
    input  wire        qualified_header,
    input  wire        good_eof_toggle_pixel,
    input  wire        bad_guard_eof_toggle_pixel,
    output reg  [7:0]  tap_lane0,
    output reg  [7:0]  tap_lane1,
    output reg         delay_load_lane0,
    output reg         delay_load_lane1,
    output reg         ready,
    output reg         fault,
    output wire [3:0]  state,
    output reg  [4:0]  scan_row,
    output reg  [4:0]  scan_col,
    output reg  [12:0] verify_count,
    output reg  [15:0] good_frame_count,
    output reg  [31:0] burst_total_count,
    output reg  [31:0] bad_burst_count,
    output wire [3:0] last_failure_reason_o,
    output wire [7:0] retry_count_o
);
    localparam [3:0] ST_WAIT_START      = 4'd0;
    localparam [3:0] ST_SCAN_REQUEST    = 4'd1;
    localparam [3:0] ST_SCAN_LOAD_WAIT  = 4'd2;
    localparam [3:0] ST_SCAN_SKIP       = 4'd3;
    localparam [3:0] ST_SCAN_SAMPLE     = 4'd4;
    localparam [3:0] ST_SELECT_START    = 4'd5;
    localparam [3:0] ST_SELECT_WAIT     = 4'd6;
    localparam [3:0] ST_VERIFY_LOAD     = 4'd7;
    localparam [3:0] ST_VERIFY_LOAD_WAIT= 4'd8;
    localparam [3:0] ST_VERIFY          = 4'd9;
    localparam [3:0] ST_MASK_FAILED     = 4'd10;
    localparam [3:0] ST_RESELECT        = 4'd11;
    localparam [3:0] ST_READY           = 4'd12;
    localparam [3:0] ST_FAULT           = 4'd13;

    reg [3:0] state_reg;
    assign state = state_reg;

    localparam integer BURST_WINDOW_LIMIT0 =
        (POINT_BURSTS > VERIFY_BURSTS) ? POINT_BURSTS : VERIFY_BURSTS;
    localparam integer BURST_WINDOW_LIMIT =
        (BURST_WINDOW_LIMIT0 > SKIP_BURSTS) ? BURST_WINDOW_LIMIT0 : SKIP_BURSTS;
    localparam integer BURST_WINDOW_WIDTH =
        (BURST_WINDOW_LIMIT > 1) ? $clog2(BURST_WINDOW_LIMIT + 1) : 1;
    localparam integer WATCHDOG_LIMIT0 =
        (FRAME_TIMEOUT_CYCLES > LOAD_TIMEOUT_CYCLES) ?
        FRAME_TIMEOUT_CYCLES : LOAD_TIMEOUT_CYCLES;
    localparam integer WATCHDOG_LIMIT =
        (WATCHDOG_LIMIT0 > RECOVERY_WAIT_CYCLES) ? WATCHDOG_LIMIT0 : RECOVERY_WAIT_CYCLES;
    localparam integer WATCHDOG_WIDTH =
        (WATCHDOG_LIMIT > 1) ? $clog2(WATCHDOG_LIMIT + 1) : 1;
    localparam integer VERIFY_CYCLE_WIDTH =
        (VERIFY_MIN_CYCLES > 1) ? $clog2(VERIFY_MIN_CYCLES + 1) : 1;
    localparam [BURST_WINDOW_WIDTH-1:0] SKIP_BURST_COUNT = SKIP_BURSTS;
    localparam [BURST_WINDOW_WIDTH-1:0] POINT_BURST_COUNT = POINT_BURSTS;
    localparam [BURST_WINDOW_WIDTH-1:0] VERIFY_BURST_COUNT = VERIFY_BURSTS;
    localparam [WATCHDOG_WIDTH-1:0] FRAME_TIMEOUT_COUNT = FRAME_TIMEOUT_CYCLES;
    localparam [WATCHDOG_WIDTH-1:0] FRAME_TIMEOUT_LAST = FRAME_TIMEOUT_CYCLES - 1;
    localparam [WATCHDOG_WIDTH-1:0] LOAD_TIMEOUT_LAST = LOAD_TIMEOUT_CYCLES - 1;
    localparam [WATCHDOG_WIDTH-1:0] RECOVERY_WAIT_LAST = RECOVERY_WAIT_CYCLES - 1;
    localparam [VERIFY_CYCLE_WIDTH-1:0] VERIFY_MIN_COUNT = VERIFY_MIN_CYCLES;

    // LP11、burst 和像素域 guard EOF toggle 都在 byte_clk 内二级同步后使用。
    (* ASYNC_REG = "TRUE" *) reg [1:0] lp0_meta;
    (* ASYNC_REG = "TRUE" *) reg [1:0] lp0_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] lp1_meta;
    (* ASYNC_REG = "TRUE" *) reg [1:0] lp1_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] good_eof_sync;
    reg good_eof_sync_d;
    wire good_eof_pulse = good_eof_sync[1] ^ good_eof_sync_d;
    (* ASYNC_REG = "TRUE" *) reg [1:0] bad_guard_eof_sync;
    reg bad_guard_eof_sync_d;
    wire bad_guard_eof_pulse = bad_guard_eof_sync[1] ^ bad_guard_eof_sync_d;
    wire [15:0] good_frame_count_effective =
        good_frame_count + (good_eof_pulse ? 16'd1 : 16'd0);
    wire safe_to_load = (lp0_sync == 2'b11) && (lp1_sync == 2'b11) &&
                        (lp_lane0_raw == 2'b11) && (lp_lane1_raw == 2'b11) &&
                        !burst_active;

    // 提前使能的 LP-01 可能被中止；只有实际进入 HS 的完整 burst 才计数。
    // 首个合格 SP/AV 头由 header gate 已筛掉 ECC 错误。
    reg burst_active_d;
    reg burst_hs_seen;
    reg burst_header_seen;
    wire burst_rise = burst_active && !burst_active_d;
    wire burst_fall = !burst_active && burst_active_d;
    wire completed_burst = burst_fall && (burst_hs_seen || burst_hs_started);
    wire bad_burst_event = completed_burst && !(burst_header_seen || qualified_header);
    always @(posedge byte_clk or negedge reset_n) begin
        if (!reset_n) begin
            burst_active_d <= 1'b0;
            burst_hs_seen <= 1'b0;
            burst_header_seen <= 1'b0;
            burst_total_count <= 32'd0;
            bad_burst_count <= 32'd0;
        end else begin
            burst_active_d <= burst_active;
            if (burst_rise) begin
                burst_hs_seen <= burst_hs_started;
                burst_header_seen <= 1'b0;
            end else begin
                if (burst_active && burst_hs_started)
                    burst_hs_seen <= 1'b1;
                if (qualified_header && (burst_active || burst_active_d))
                    burst_header_seen <= 1'b1;
            end

            if (completed_burst) begin
                burst_total_count <= burst_total_count + 32'd1;
                if (bad_burst_event)
                    bad_burst_count <= bad_burst_count + 32'd1;
            end
        end
    end

    always @(posedge byte_clk or negedge reset_n) begin
        if (!reset_n) begin
            lp0_meta <= 2'b00;
            lp0_sync <= 2'b00;
            lp1_meta <= 2'b00;
            lp1_sync <= 2'b00;
            good_eof_sync <= 2'b00;
            good_eof_sync_d <= 1'b0;
            bad_guard_eof_sync <= 2'b00;
            bad_guard_eof_sync_d <= 1'b0;
            good_frame_count <= 16'd0;
        end else begin
            lp0_meta <= lp_lane0_raw;
            lp0_sync <= lp0_meta;
            lp1_meta <= lp_lane1_raw;
            lp1_sync <= lp1_meta;
            good_eof_sync <= {good_eof_sync[0], good_eof_toggle_pixel};
            good_eof_sync_d <= good_eof_sync[1];
            bad_guard_eof_sync <= {bad_guard_eof_sync[0], bad_guard_eof_toggle_pixel};
            bad_guard_eof_sync_d <= bad_guard_eof_sync[1];
            if (good_eof_pulse)
                good_frame_count <= good_frame_count + 16'd1;
        end
    end

    reg [1023:0] pass_map;
    reg [7:0] requested_tap0;
    reg [7:0] requested_tap1;
    reg tap_load_request;
    reg eye_start;
    wire eye_busy;
    wire eye_done;
    wire eye_found;
    wire [4:0] eye_top;
    wire [4:0] eye_bottom;
    wire [4:0] eye_left;
    wire [4:0] eye_right;
    wire [4:0] eye_center_row;
    wire [4:0] eye_center_col;
    wire [7:0] eye_tap0;
    wire [7:0] eye_tap1;

    ov5647_autocal_eye_select eye_select (
        .byte_clk(byte_clk), .reset_n(reset_n), .start(eye_start),
        .pass_map(pass_map), .blocked_map(1024'd0),
        .busy(eye_busy), .done(eye_done), .found(eye_found),
        .top_out(eye_top), .bottom_out(eye_bottom),
        .left_out(eye_left), .right_out(eye_right),
        .center_row_index(eye_center_row), .center_col_index(eye_center_col),
        .center_tap_lane0(eye_tap0), .center_tap_lane1(eye_tap1)
    );

    // 两个 tap 同时更新；DLYSTEP 保持一整拍后，才在 LP11 给 VALUE 单拍。
    localparam [1:0] LD_IDLE   = 2'd0;
    localparam [1:0] LD_SAFE   = 2'd1;
    localparam [1:0] LD_SETTLE = 2'd2;
    localparam [1:0] LD_DONE   = 2'd3;
    reg [1:0] load_state;
    reg [7:0] load_target0;
    reg [7:0] load_target1;
    reg loader_done;
    always @(posedge byte_clk or negedge reset_n) begin
        if (!reset_n) begin
            load_state <= LD_IDLE;
            load_target0 <= 8'd0;
            load_target1 <= 8'd0;
            tap_lane0 <= 8'd0;
            tap_lane1 <= 8'd0;
            delay_load_lane0 <= 1'b0;
            delay_load_lane1 <= 1'b0;
            loader_done <= 1'b0;
        end else begin
            delay_load_lane0 <= 1'b0;
            delay_load_lane1 <= 1'b0;
            loader_done <= 1'b0;
            // 超时进入故障后撤销未完成的请求，重扫不能消费旧 loader_done。
            if (state_reg == ST_FAULT) begin
                load_state <= LD_IDLE;
            end else case (load_state)
                LD_IDLE: begin
                    if (tap_load_request) begin
                        load_target0 <= requested_tap0;
                        load_target1 <= requested_tap1;
                        load_state <= LD_SAFE;
                    end
                end
                LD_SAFE: begin
                    if (safe_to_load) begin
                        tap_lane0 <= load_target0;
                        tap_lane1 <= load_target1;
                        load_state <= LD_SETTLE;
                    end
                end
                LD_SETTLE: begin
                    if (safe_to_load) begin
                        delay_load_lane0 <= 1'b1;
                        delay_load_lane1 <= 1'b1;
                        load_state <= LD_DONE;
                    end else begin
                        load_state <= LD_SAFE;
                    end
                end
                LD_DONE: begin
                    loader_done <= 1'b1;
                    load_state <= LD_IDLE;
                end
                default: load_state <= LD_IDLE;
            endcase
        end
    end

    reg [BURST_WINDOW_WIDTH-1:0] point_skip_base;
    reg [BURST_WINDOW_WIDTH-1:0] point_sample_base;
    reg [BURST_WINDOW_WIDTH-1:0] point_bad_base;
    reg [BURST_WINDOW_WIDTH-1:0] verify_skip_base;
    reg [BURST_WINDOW_WIDTH-1:0] verify_sample_base;
    reg [BURST_WINDOW_WIDTH-1:0] verify_bad_base;
    reg [VERIFY_CYCLE_WIDTH-1:0] verify_elapsed_cycles;
    reg [15:0] verify_good_base;
    reg verify_armed;
    reg verify_skip_done;
    reg verify_sample_done;
    reg [2:0] attempt_count;
    reg [7:0] retry_count;
    // 0=unknown/none, 1=no eye, 2=bad CSI burst, 3=bad raw EOF,
    // 4=HS burst timeout, 5=good EOF timeout, 6=tap-load timeout.
    reg [3:0] last_failure_reason;
    assign last_failure_reason_o = last_failure_reason;
    assign retry_count_o = retry_count;
    reg [4:0] selected_center_row;
    reg [4:0] selected_center_col;
    reg [1:0] mask_dr;
    reg [1:0] mask_dc;
    reg [WATCHDOG_WIDTH-1:0] frame_watchdog;
    reg [WATCHDOG_WIDTH-1:0] state_watchdog;

    // 模计数差仅用于检测有界阈值；长时间等待时由 verify_*_done 锁存跨过状态。
    wire [BURST_WINDOW_WIDTH-1:0] point_skip_elapsed =
        burst_total_count[BURST_WINDOW_WIDTH-1:0] - point_skip_base;
    wire [BURST_WINDOW_WIDTH-1:0] point_sample_elapsed =
        burst_total_count[BURST_WINDOW_WIDTH-1:0] - point_sample_base;
    wire [BURST_WINDOW_WIDTH-1:0] verify_skip_elapsed =
        burst_total_count[BURST_WINDOW_WIDTH-1:0] - verify_skip_base;
    wire [BURST_WINDOW_WIDTH-1:0] verify_sample_elapsed =
        burst_total_count[BURST_WINDOW_WIDTH-1:0] - verify_sample_base;
    wire verify_skip_reached = verify_skip_done ||
                               (verify_skip_elapsed >= SKIP_BURST_COUNT);
    wire verify_sample_reached = verify_sample_done ||
                                 (verify_sample_elapsed >= VERIFY_BURST_COUNT);
    wire [VERIFY_CYCLE_WIDTH-1:0] verify_elapsed_cycles_next =
        (verify_elapsed_cycles < VERIFY_MIN_COUNT) ?
        (verify_elapsed_cycles + 1'b1) : verify_elapsed_cycles;
    wire [9:0] scan_map_index = {scan_row, scan_col};
    wire [5:0] mask_row_sum = {1'b0, selected_center_row} + {4'd0, mask_dr};
    wire [5:0] mask_col_sum = {1'b0, selected_center_col} + {4'd0, mask_dc};
    wire mask_row_valid = (mask_row_sum >= 1) && (mask_row_sum <= 32);
    wire mask_col_valid = (mask_col_sum >= 1) && (mask_col_sum <= 32);
    wire [9:0] mask_map_index = {mask_row_sum[4:0] - 5'd1,
                                 mask_col_sum[4:0] - 5'd1};

    always @(posedge byte_clk or negedge reset_n) begin
        if (!reset_n) begin
            state_reg <= ST_WAIT_START;
            scan_row <= 5'd0;
            scan_col <= 5'd0;
            verify_count <= 13'd0;
            ready <= 1'b0;
            fault <= 1'b0;
            pass_map <= 1024'd0;
            requested_tap0 <= 8'd0;
            requested_tap1 <= 8'd0;
            tap_load_request <= 1'b0;
            eye_start <= 1'b0;
            point_skip_base <= {BURST_WINDOW_WIDTH{1'b0}};
            point_sample_base <= {BURST_WINDOW_WIDTH{1'b0}};
            point_bad_base <= {BURST_WINDOW_WIDTH{1'b0}};
            verify_skip_base <= {BURST_WINDOW_WIDTH{1'b0}};
            verify_sample_base <= {BURST_WINDOW_WIDTH{1'b0}};
            verify_bad_base <= {BURST_WINDOW_WIDTH{1'b0}};
            verify_elapsed_cycles <= {VERIFY_CYCLE_WIDTH{1'b0}};
            verify_good_base <= 16'd0;
            verify_armed <= 1'b0;
            verify_skip_done <= 1'b0;
            verify_sample_done <= 1'b0;
            attempt_count <= 3'd0;
            retry_count <= 8'd0;
            last_failure_reason <= 4'd0;
            selected_center_row <= 5'd0;
            selected_center_col <= 5'd0;
            mask_dr <= 2'd0;
            mask_dc <= 2'd0;
            frame_watchdog <= {WATCHDOG_WIDTH{1'b0}};
            state_watchdog <= {WATCHDOG_WIDTH{1'b0}};
        end else begin
            tap_load_request <= 1'b0;
            eye_start <= 1'b0;

            // 首次启动由外部同步的 sensor_init_done && dphy_reset_n 开启。
            case (state_reg)
                ST_WAIT_START: begin
                    state_watchdog <= 32'd0;
                    if (start_enable) begin
                        scan_row <= 5'd0;
                        scan_col <= 5'd0;
                        pass_map <= 1024'd0;
                        attempt_count <= 3'd0;
                        last_failure_reason <= 4'd0;
                        ready <= 1'b0;
                        fault <= 1'b0;
                        state_reg <= ST_SCAN_REQUEST;
                    end
                end

                ST_SCAN_REQUEST: begin
                    requested_tap0 <= {scan_row, 3'b000};
                    requested_tap1 <= {scan_col, 3'b000};
                    tap_load_request <= 1'b1;
                    state_watchdog <= 32'd0;
                    state_reg <= ST_SCAN_LOAD_WAIT;
                end

                ST_SCAN_LOAD_WAIT: begin
                    if (loader_done) begin
                        point_skip_base <= burst_total_count[BURST_WINDOW_WIDTH-1:0];
                        state_watchdog <= 32'd0;
                        state_reg <= ST_SCAN_SKIP;
                    end else if (state_watchdog >= LOAD_TIMEOUT_LAST) begin
                        last_failure_reason <= 4'd6;
                        state_watchdog <= {WATCHDOG_WIDTH{1'b0}};
                        fault <= 1'b1;
                        state_reg <= ST_FAULT;
                    end else begin
                        state_watchdog <= state_watchdog + 1'b1;
                    end
                end

                ST_SCAN_SKIP: begin
                    if (point_skip_elapsed >= SKIP_BURST_COUNT) begin
                        point_sample_base <= burst_total_count[BURST_WINDOW_WIDTH-1:0];
                        point_bad_base <= bad_burst_count[BURST_WINDOW_WIDTH-1:0];
                        state_watchdog <= 32'd0;
                        state_reg <= ST_SCAN_SAMPLE;
                    end else if (state_watchdog >= FRAME_TIMEOUT_LAST) begin
                        last_failure_reason <= 4'd4;
                        state_watchdog <= {WATCHDOG_WIDTH{1'b0}};
                        fault <= 1'b1;
                        state_reg <= ST_FAULT;
                    end else begin
                        state_watchdog <= state_watchdog + 1'b1;
                    end
                end

                ST_SCAN_SAMPLE: begin
                    if (point_sample_elapsed >= POINT_BURST_COUNT) begin
                        pass_map[scan_map_index] <=
                            (bad_burst_count[BURST_WINDOW_WIDTH-1:0] == point_bad_base);
                        state_watchdog <= 32'd0;
                        if (scan_col != 5'd31) begin
                            scan_col <= scan_col + 5'd1;
                            state_reg <= ST_SCAN_REQUEST;
                        end else if (scan_row != 5'd31) begin
                            scan_col <= 5'd0;
                            scan_row <= scan_row + 5'd1;
                            state_reg <= ST_SCAN_REQUEST;
                        end else begin
                            state_reg <= ST_SELECT_START;
                        end
                    end else if (state_watchdog >= FRAME_TIMEOUT_LAST) begin
                        last_failure_reason <= 4'd4;
                        state_watchdog <= {WATCHDOG_WIDTH{1'b0}};
                        fault <= 1'b1;
                        state_reg <= ST_FAULT;
                    end else begin
                        state_watchdog <= state_watchdog + 1'b1;
                    end
                end

                ST_SELECT_START: begin
                    eye_start <= 1'b1;
                    state_reg <= ST_SELECT_WAIT;
                end

                ST_SELECT_WAIT: begin
                    if (eye_done) begin
                        if (!eye_found) begin
                            last_failure_reason <= 4'd1;
                            state_watchdog <= {WATCHDOG_WIDTH{1'b0}};
                            fault <= 1'b1;
                            state_reg <= ST_FAULT;
                        end else begin
                            requested_tap0 <= eye_tap0;
                            requested_tap1 <= eye_tap1;
                            selected_center_row <= eye_center_row;
                            selected_center_col <= eye_center_col;
                            if (attempt_count == 0)
                                attempt_count <= 3'd1;
                            tap_load_request <= 1'b1;
                            state_watchdog <= 32'd0;
                            state_reg <= ST_VERIFY_LOAD_WAIT;
                        end
                    end
                end

                ST_VERIFY_LOAD_WAIT: begin
                    if (loader_done) begin
                        verify_skip_base <= burst_total_count[BURST_WINDOW_WIDTH-1:0];
                        verify_armed <= 1'b0;
                        verify_skip_done <= 1'b0;
                        verify_sample_done <= 1'b0;
                        verify_elapsed_cycles <= {VERIFY_CYCLE_WIDTH{1'b0}};
                        verify_count <= 13'd0;
                        frame_watchdog <= 32'd0;
                        state_watchdog <= 32'd0;
                        state_reg <= ST_VERIFY;
                    end else if (state_watchdog >= LOAD_TIMEOUT_LAST) begin
                        last_failure_reason <= 4'd6;
                        state_watchdog <= {WATCHDOG_WIDTH{1'b0}};
                        fault <= 1'b1;
                        state_reg <= ST_FAULT;
                    end else begin
                        state_watchdog <= state_watchdog + 1'b1;
                    end
                end

                ST_VERIFY: begin
                    // 低位差值只需跨过有界阈值；之后用锁存位保持结果，
                    // 避免长时间等 EOF 时模计数器回绕清掉已达到标记。
                    if (verify_skip_elapsed >= SKIP_BURST_COUNT)
                        verify_skip_done <= 1'b1;
                    if (verify_armed &&
                        (verify_sample_elapsed >= VERIFY_BURST_COUNT))
                        verify_sample_done <= 1'b1;

                    if (good_eof_pulse)
                        frame_watchdog <= 32'd0;
                    else if (frame_watchdog < FRAME_TIMEOUT_COUNT)
                        frame_watchdog <= frame_watchdog + 1'b1;

                    if (completed_burst)
                        state_watchdog <= 32'd0;
                    else if (state_watchdog < FRAME_TIMEOUT_COUNT)
                        state_watchdog <= state_watchdog + 1'b1;

                    if (!verify_armed) begin
                        if (verify_skip_reached) begin
                            if (good_eof_pulse) begin
                                // tap 切换后的残帧不计入验证；首个原始 good EOF 只负责置位基线。
                                verify_sample_base <= burst_total_count[BURST_WINDOW_WIDTH-1:0];
                                verify_bad_base <= bad_burst_count[BURST_WINDOW_WIDTH-1:0];
                                verify_good_base <= good_frame_count_effective;
                                verify_armed <= 1'b1;
                                verify_sample_done <= 1'b0;
                                verify_elapsed_cycles <= {VERIFY_CYCLE_WIDTH{1'b0}};
                                verify_count <= 13'd0;
                                frame_watchdog <= 32'd0;
                                state_watchdog <= 32'd0;
                            end else if (frame_watchdog >= FRAME_TIMEOUT_LAST) begin
                                // 等待首个干净原始帧也必须有界，前置坏帧不反馈到训练器。
                                last_failure_reason <= 4'd5;
                                state_reg <= ST_MASK_FAILED;
                                mask_dr <= 2'd0;
                                mask_dc <= 2'd0;
                            end
                        end else if (state_watchdog >= FRAME_TIMEOUT_LAST) begin
                            last_failure_reason <= 4'd4;
                            state_watchdog <= {WATCHDOG_WIDTH{1'b0}};
                            fault <= 1'b1;
                            state_reg <= ST_FAULT;
                        end else begin
                            state_watchdog <= state_watchdog + 1'b1;
                        end
                    end else begin
                        verify_elapsed_cycles <= verify_elapsed_cycles_next;
                        if (verify_sample_reached)
                            verify_count <= VERIFY_BURSTS;
                        else
                            verify_count <= verify_sample_elapsed;

                        if ((bad_burst_count[BURST_WINDOW_WIDTH-1:0] != verify_bad_base) || bad_burst_event ||
                            bad_guard_eof_pulse) begin
                            // post-arm 原始坏 EOF 或任意漏头均优先屏蔽，包括采样/计时同拍。
                            if (bad_guard_eof_pulse)
                                last_failure_reason <= 4'd3;
                            else
                                last_failure_reason <= 4'd2;
                            state_reg <= ST_MASK_FAILED;
                            mask_dr <= 2'd0;
                            mask_dc <= 2'd0;
                        end else if (verify_sample_reached &&
                                     (verify_elapsed_cycles_next >= VERIFY_MIN_COUNT) &&
                                     ((good_frame_count_effective - verify_good_base) >= 16'd2)) begin
                            ready <= 1'b1;
                            fault <= 1'b0;
                            state_reg <= ST_READY;
                        end else if (verify_sample_reached &&
                                     (frame_watchdog >= FRAME_TIMEOUT_LAST)) begin
                            last_failure_reason <= 4'd5;
                            state_reg <= ST_MASK_FAILED;
                            mask_dr <= 2'd0;
                            mask_dc <= 2'd0;
                        end else if (state_watchdog >= FRAME_TIMEOUT_LAST) begin
                            // 目标 tap 停止产生 HS burst 时不可无限等待。
                            last_failure_reason <= 4'd4;
                            state_reg <= ST_MASK_FAILED;
                            mask_dr <= 2'd0;
                            mask_dc <= 2'd0;
                        end
                    end
                end

                ST_MASK_FAILED: begin
                    if (attempt_count >= 3'd4) begin
                        state_watchdog <= {WATCHDOG_WIDTH{1'b0}};
                        fault <= 1'b1;
                        state_reg <= ST_FAULT;
                    end else begin
                        if (mask_row_valid && mask_col_valid)
                            // 扫描结束后只读取 pass_map；直接清除失败区域即可复用该位图。
                            pass_map[mask_map_index] <= 1'b0;

                        if (mask_dc != 2'd2) begin
                            mask_dc <= mask_dc + 2'd1;
                        end else if (mask_dr != 2'd2) begin
                            mask_dc <= 2'd0;
                            mask_dr <= mask_dr + 2'd1;
                        end else begin
                            mask_dc <= 2'd0;
                            mask_dr <= 2'd0;
                            attempt_count <= attempt_count + 3'd1;
                            if (retry_count != 8'hff)
                                retry_count <= retry_count + 1'b1;
                            state_reg <= ST_RESELECT;
                        end
                    end
                end

                ST_RESELECT: begin
                    eye_start <= 1'b1;
                    state_reg <= ST_SELECT_WAIT;
                end

                ST_READY: begin
                    ready <= 1'b1;
                end

                ST_FAULT: begin
                    ready <= 1'b0;
                    fault <= 1'b1;
                    // 故障和最后原因持续可见；冷却后从完整眼图重新训练。
                    if (!start_enable) begin
                        state_watchdog <= {WATCHDOG_WIDTH{1'b0}};
                    end else if (state_watchdog >= RECOVERY_WAIT_LAST) begin
                        scan_row <= 5'd0;
                        scan_col <= 5'd0;
                        pass_map <= 1024'd0;
                        attempt_count <= 3'd0;
                        verify_count <= 13'd0;
                        verify_armed <= 1'b0;
                        if (retry_count != 8'hff)
                            retry_count <= retry_count + 1'b1;
                        state_watchdog <= {WATCHDOG_WIDTH{1'b0}};
                        state_reg <= ST_SCAN_REQUEST;
                    end else begin
                        state_watchdog <= state_watchdog + 1'b1;
                    end
                end

                default: begin
                    state_watchdog <= {WATCHDOG_WIDTH{1'b0}};
                    fault <= 1'b1;
                    ready <= 1'b0;
                    state_reg <= ST_FAULT;
                end
            endcase
        end
    end
endmodule

// 字节域在 FE 时提交源帧账本；只统计通过头门控后实际送入 B2P 的字节。
// 该结果与像素域的逐行几何检查同时成立，才能允许延迟排空的帧通过。
module ov5647_source_frame_accounting #(
    parameter integer EXPECTED_LINES = 1080,
    parameter integer BYTES_PER_LINE = 2400
) (
    input wire byte_clk,
    input wire reset_n,
    input wire short_packet,
    input wire [5:0] data_type,
    input wire video_packet,
    input wire [1:0] payload_valid,
    output reg complete_snapshot,
    output reg eof_toggle
);
    localparam integer LINE_W = (EXPECTED_LINES > 1) ? $clog2(EXPECTED_LINES + 1) : 1;
    localparam integer BYTE_W = (BYTES_PER_LINE > 1) ? $clog2(BYTES_PER_LINE + 1) : 1;
    reg active, bad, packet_open;
    reg [LINE_W-1:0] completed_packets;
    reg [BYTE_W-1:0] payload_bytes;
    wire fs = short_packet && (data_type == 6'h00);
    wire fe = short_packet && (data_type == 6'h01);
    wire [BYTE_W:0] bytes_next = {1'b0, (video_packet ? {BYTE_W{1'b0}} : payload_bytes)} +
                                  payload_valid[0] + payload_valid[1];

    always @(posedge byte_clk or negedge reset_n) begin
        if (!reset_n) begin
            active <= 1'b0;
            bad <= 1'b0;
            packet_open <= 1'b0;
            completed_packets <= {LINE_W{1'b0}};
            payload_bytes <= {BYTE_W{1'b0}};
            complete_snapshot <= 1'b0;
            eof_toggle <= 1'b0;
        end else if (fs) begin
            // 漏 FE 后的新 FS 不能把两个源帧合并成好帧。
            active <= 1'b1;
            bad <= active || video_packet || (|payload_valid);
            packet_open <= 1'b0;
            completed_packets <= {LINE_W{1'b0}};
            payload_bytes <= {BYTE_W{1'b0}};
        end else if (fe) begin
            // 对同拍仍在传输的包保守拒收；正常 CSI 包之间有间隔。
            complete_snapshot <= active && !bad && !packet_open &&
                                 (completed_packets == EXPECTED_LINES) &&
                                 !video_packet && !(|payload_valid);
            eof_toggle <= !eof_toggle;
            active <= 1'b0;
            packet_open <= 1'b0;
        end else begin
            if (video_packet) begin
                if (!active || packet_open || completed_packets >= EXPECTED_LINES)
                    bad <= 1'b1;
                packet_open <= 1'b1;
                payload_bytes <= {BYTE_W{1'b0}};
            end
            if (|payload_valid) begin
                if (!active || (!packet_open && !video_packet) ||
                    (bytes_next > BYTES_PER_LINE)) begin
                    bad <= 1'b1;
                    packet_open <= 1'b0;
                end else if (bytes_next == BYTES_PER_LINE) begin
                    packet_open <= 1'b0;
                    payload_bytes <= {BYTE_W{1'b0}};
                    if (completed_packets < EXPECTED_LINES)
                        completed_packets <= completed_packets + 1'b1;
                    else
                        bad <= 1'b1;
                end else begin
                    payload_bytes <= bytes_next[BYTE_W-1:0];
                end
            end
        end
    end
endmodule

// 同步 FE toggle 后再等一拍采样保持到下一次 FE 的字节域结果。
module ov5647_source_status_cdc (
    input wire pixel_clk,
    input wire reset_n,
    input wire frame_start,
    input wire eof_toggle_byte,
    input wire complete_byte,
    output reg status_valid,
    output reg frame_complete
);
    (* ASYNC_REG="TRUE" *) reg [1:0] eof_sync, complete_sync;
    reg eof_seen, result_pending;
    always @(posedge pixel_clk or negedge reset_n) begin
        if (!reset_n) begin
            eof_sync <= 2'b00;
            complete_sync <= 2'b00;
            eof_seen <= 1'b0;
            result_pending <= 1'b0;
            status_valid <= 1'b0;
            frame_complete <= 1'b0;
        end else begin
            eof_sync <= {eof_sync[0], eof_toggle_byte};
            complete_sync <= {complete_sync[0], complete_byte};
            if (eof_sync[1] != eof_seen) begin
                eof_seen <= eof_sync[1];
                result_pending <= 1'b1;
            end else if (result_pending) begin
                result_pending <= 1'b0;
                frame_complete <= complete_sync[1];
                if (!frame_start)
                    status_valid <= 1'b1;
            end
            if (frame_start)
                status_valid <= 1'b0;
        end
    end
endmodule

// ==================== ov5647_frame_integrity_guard.v ====================
// 发布 RGB565 帧之前，检查 CSI 源帧的完整行数和行宽。
module ov5647_frame_integrity_guard #(
    parameter integer EXPECTED_LINES = 1080,
    parameter integer PAIRS_PER_LINE = 960,
    parameter integer USE_SOURCE_FRAME_COMPLETE = 0,
    // 最多等待两行输出时间加64拍跨域/启动余量；超限只能拒帧。
    parameter integer DRAIN_TIMEOUT_CYCLES = 2 * PAIRS_PER_LINE + 64,
    // 43.75 MHz 下约91 ms，约三个正常源帧周期；漏 FE 时有界拒帧。
    parameter integer FRAME_TIMEOUT_CYCLES = 4000000
) (
    input  wire pixel_clk,
    input  wire reset_n,
    input  wire frame_valid,
    input  wire line_valid,
    input  wire source_status_valid,
    input  wire source_frame_complete,
    output wire camera_vsync,
    output wire camera_frame_bad,
    output reg diag_pixel_geometry,
    output reg diag_frame_overlap,
    output reg diag_end_timeout,
    output reg diag_no_fe_timeout
);
    localparam integer LINE_COUNT_WIDTH = (EXPECTED_LINES > 1) ? $clog2(EXPECTED_LINES + 1) : 1;
    localparam integer PAIR_COUNT_WIDTH = (PAIRS_PER_LINE > 1) ? $clog2(PAIRS_PER_LINE + 1) : 1;
    localparam integer DRAIN_WIDTH = (DRAIN_TIMEOUT_CYCLES > 1) ? $clog2(DRAIN_TIMEOUT_CYCLES + 1) : 1;
    localparam integer FRAME_WIDTH = (FRAME_TIMEOUT_CYCLES > 1) ? $clog2(FRAME_TIMEOUT_CYCLES + 1) : 1;

    reg frame_active;
    reg frame_valid_d;
    reg line_valid_d;
    reg frame_end_pending;
    reg frame_bad_reg;
    reg [LINE_COUNT_WIDTH-1:0] completed_lines;
    reg [PAIR_COUNT_WIDTH-1:0] line_cycles;
    reg [DRAIN_WIDTH-1:0] drain_cycles;
    reg [FRAME_WIDTH-1:0] frame_cycles;

    wire frame_start = frame_valid && !frame_valid_d;
    wire frame_end = frame_valid_d && !frame_valid;
    wire pixel_geometry_error = (frame_active || frame_start) &&
        ((line_valid && !line_valid_d && !frame_start && completed_lines >= EXPECTED_LINES) ||
         (line_valid && line_valid_d && line_cycles >= PAIRS_PER_LINE) ||
         (!line_valid && line_valid_d &&
          (line_cycles != PAIRS_PER_LINE || completed_lines >= EXPECTED_LINES)));
    wire guard_end_timeout = frame_end_pending && drain_cycles >= DRAIN_TIMEOUT_CYCLES-1;
    wire guard_frame_timeout = frame_active && frame_cycles >= FRAME_TIMEOUT_CYCLES-1;

    // framebuffer 用高 VSYNC 表示消隐。FV 提前下降后继续等待最后 LV 排空，
    // 并先稳定坏帧标志，再结束采集窗口。
    assign camera_vsync = !frame_active;
    assign camera_frame_bad = frame_bad_reg;

    always @(posedge pixel_clk or negedge reset_n) begin
        if (!reset_n) begin
            frame_active <= 1'b0;
            frame_valid_d <= 1'b0;
            line_valid_d <= 1'b0;
            frame_end_pending <= 1'b0;
            frame_bad_reg <= 1'b0;
            completed_lines <= {LINE_COUNT_WIDTH{1'b0}};
            line_cycles <= {PAIR_COUNT_WIDTH{1'b0}};
            drain_cycles <= {DRAIN_WIDTH{1'b0}};
            frame_cycles <= {FRAME_WIDTH{1'b0}};
        end else begin
            frame_valid_d <= frame_valid;
            line_valid_d <= line_valid;

            if (frame_start) begin
                if (frame_active || frame_end_pending) begin
                    // 采集窗口尚未关闭时再次开始 FV，必须标记为坏帧。
                    frame_bad_reg <= 1'b1;
                end else begin
                    frame_active <= 1'b1;
                    frame_bad_reg <= 1'b0;
                    completed_lines <= {LINE_COUNT_WIDTH{1'b0}};
                    line_cycles <= {PAIR_COUNT_WIDTH{1'b0}};
                    drain_cycles <= {DRAIN_WIDTH{1'b0}};
                    frame_cycles <= {FRAME_WIDTH{1'b0}};
                end
            end

            if (frame_active || frame_start) begin
                if (line_valid && !line_valid_d) begin
                    line_cycles <= {{(PAIR_COUNT_WIDTH-1){1'b0}}, 1'b1};
                end else if (line_valid && line_valid_d) begin
                    if (line_cycles < PAIRS_PER_LINE)
                        line_cycles <= line_cycles + 1'b1;
                end else if (!line_valid && line_valid_d) begin
                    if (completed_lines < EXPECTED_LINES)
                        completed_lines <= completed_lines + 1'b1;

                    // FV 已结束时，这是末条源行；先锁存缺行/多行错误，再释放 VSYNC。
                    if ((USE_SOURCE_FRAME_COMPLETE == 0) && (frame_end_pending || frame_end) &&
                        (completed_lines + 1'b1 != EXPECTED_LINES))
                        frame_bad_reg <= 1'b1;
                    line_cycles <= {PAIR_COUNT_WIDTH{1'b0}};
                end
            end

            if (pixel_geometry_error)
                frame_bad_reg <= 1'b1;

            if (frame_end && frame_active) begin
                frame_end_pending <= 1'b1;
                drain_cycles <= {DRAIN_WIDTH{1'b0}};
                if ((USE_SOURCE_FRAME_COMPLETE == 0) && !line_valid && !line_valid_d &&
                    completed_lines != EXPECTED_LINES)
                    frame_bad_reg <= 1'b1;
            end

            // 完整字节账本允许等待尚未开始的尾行；超时、缺包及缺行一律拒收。
            if (frame_end_pending) begin
                if (drain_cycles < DRAIN_TIMEOUT_CYCLES)
                    drain_cycles <= drain_cycles + 1'b1;
                if (guard_end_timeout) begin
                    frame_bad_reg <= 1'b1;
                    frame_active <= 1'b0;
                    frame_end_pending <= 1'b0;
                end else if (!line_valid && !line_valid_d &&
                    ((USE_SOURCE_FRAME_COMPLETE == 0) ||
                     (source_status_valid &&
                      (!source_frame_complete || completed_lines == EXPECTED_LINES)))) begin
                    if ((completed_lines != EXPECTED_LINES) ||
                        ((USE_SOURCE_FRAME_COMPLETE != 0) && !source_frame_complete))
                        frame_bad_reg <= 1'b1;
                    frame_active <= 1'b0;
                    frame_end_pending <= 1'b0;
                end
            end

            // FV 长时间不结束时仍必须输出坏帧结束，不能无限占用帧缓冲。
            if (frame_active) begin
                if (frame_cycles < FRAME_TIMEOUT_CYCLES)
                    frame_cycles <= frame_cycles + 1'b1;
                if (guard_frame_timeout) begin
                    frame_bad_reg <= 1'b1;
                    frame_active <= 1'b0;
                    frame_end_pending <= 1'b0;
                end
            end
        end
    end

    // 旁路诊断只观察已有guard状态，不影响帧接收或坏帧判定。
    always @(posedge pixel_clk or negedge reset_n) begin
        if (!reset_n) begin
            diag_pixel_geometry <= 1'b0;
            diag_frame_overlap <= 1'b0;
            diag_end_timeout <= 1'b0;
            diag_no_fe_timeout <= 1'b0;
        end else begin
            if (frame_start && !frame_active && !frame_end_pending) begin
                diag_pixel_geometry <= 1'b0;
                diag_frame_overlap <= 1'b0;
                diag_end_timeout <= 1'b0;
                diag_no_fe_timeout <= 1'b0;
            end
            if (frame_start && (frame_active || frame_end_pending))
                diag_frame_overlap <= 1'b1;
            if (pixel_geometry_error)
                diag_pixel_geometry <= 1'b1;
            if (guard_end_timeout)
                diag_end_timeout <= 1'b1;
            if (guard_frame_timeout) begin
                if (frame_end_pending || frame_end)
                    diag_end_timeout <= 1'b1;
                else
                    diag_no_fe_timeout <= 1'b1;
            end
        end
    end
endmodule

// 各位记录一次或多次事件，不是帧数；像素域复位时清零。
module ov5647_guard_failure_flags (
    input wire pixel_clk, reset_n, guard_vsync, guard_bad, frame_allowed,
    input wire source_status_valid, source_frame_complete,
    input wire pixel_geometry, frame_overlap, end_timeout, no_fe_timeout,
    output reg [3:0] sticky
);
    reg guard_vsync_d;
    always @(posedge pixel_clk or negedge reset_n) begin
        if (!reset_n) begin
            guard_vsync_d <= 1'b1;
            sticky <= 4'b0000;
        end else begin
            guard_vsync_d <= guard_vsync;
            if (guard_vsync && !guard_vsync_d && guard_bad && frame_allowed) begin
                // 跨帧重叠时字节快照可能属于后一个帧，不做源/几何归因。
                if (!frame_overlap && source_status_valid && !source_frame_complete)
                    sticky[0] <= 1'b1;
                if (!frame_overlap && source_status_valid && source_frame_complete && pixel_geometry)
                    sticky[1] <= 1'b1;
                if (end_timeout) sticky[2] <= 1'b1;
                if (no_fe_timeout) sticky[3] <= 1'b1;
            end
        end
    end
endmodule

// ==================== ov5647_lp_hs_control.v ====================
// CSI-2 LP 请求限定的连续时钟接收控制器。
// 可在确认 STOP 后的 LP-01 阶段提前启动 byte aligner。
module ov5647_lp_hs_control #(
    parameter integer TAIL_CYCLES = 32,
    parameter integer EARLY_HS_ENABLE = 1
) (
    input  wire       byte_clk,
    input  wire       reset_n,
    input  wire [1:0] lp_lane0,
    output reg        data_hs_enable,
    output wire       data_hs_started
);
    localparam integer TAIL_COUNTER_WIDTH =
        (TAIL_CYCLES > 1) ? $clog2(TAIL_CYCLES + 1) : 1;
    localparam [TAIL_COUNTER_WIDTH-1:0] TAIL_LOAD = TAIL_CYCLES;

    localparam [2:0] ST_WAIT_LP11 = 3'd0;
    localparam [2:0] ST_WAIT_LP01 = 3'd1;
    localparam [2:0] ST_WAIT_LP00 = 3'd2;
    localparam [2:0] ST_HS_ACTIVE = 3'd3;
    localparam [2:0] ST_TAIL = 3'd4;
    localparam [2:0] ST_TAIL_RESTART_GAP = 3'd5;

    (* ASYNC_REG="TRUE" *) reg [1:0] lp_lane0_meta;
    (* ASYNC_REG="TRUE" *) reg [1:0] lp_lane0_sync;

    reg [2:0] state;
    // 与使能同属 byte_clk 域；LP-01 提前请求不表示已进入 HS。
    assign data_hs_started = (state == ST_HS_ACTIVE) || (state == ST_TAIL);
    reg       lp11_first_seen;
    reg       stop_lp11_first_seen;
    reg [TAIL_COUNTER_WIDTH-1:0] tail_count;

    // LP 输入经两级同步。复位值为 LP-00，不能伪造 LP-11 停止状态。
    always @(posedge byte_clk or negedge reset_n) begin
        if (!reset_n) begin
            lp_lane0_meta <= 2'b00;
            lp_lane0_sync <= 2'b00;
            state <= ST_WAIT_LP11;
            lp11_first_seen <= 1'b0;
            stop_lp11_first_seen <= 1'b0;
            tail_count <= {TAIL_COUNTER_WIDTH{1'b0}};
            data_hs_enable <= 1'b0;
        end else begin
            lp_lane0_meta <= lp_lane0;
            lp_lane0_sync <= lp_lane0_meta;

            case (state)
                ST_WAIT_LP11: begin
                    data_hs_enable <= 1'b0;
                    tail_count <= {TAIL_COUNTER_WIDTH{1'b0}};
                    stop_lp11_first_seen <= 1'b0;
                    if (lp_lane0_sync == 2'b11) begin
                        if (lp11_first_seen) begin
                            // 已采到连续两个 LP-11 样本，允许等待本次请求。
                            state <= ST_WAIT_LP01;
                            lp11_first_seen <= 1'b0;
                        end else begin
                            lp11_first_seen <= 1'b1;
                        end
                    end else begin
                        lp11_first_seen <= 1'b0;
                    end
                end

                ST_WAIT_LP01: begin
                    data_hs_enable <= 1'b0;
                    tail_count <= {TAIL_COUNTER_WIDTH{1'b0}};
                    stop_lp11_first_seen <= 1'b0;
                    case (lp_lane0_sync)
                        2'b11: state <= ST_WAIT_LP01;
                        2'b01: begin
                            state <= ST_WAIT_LP00;
                            if (EARLY_HS_ENABLE != 0)
                                data_hs_enable <= 1'b1;
                        end
                        default: begin
                            // LP-10 或无请求的 LP-00 会使本次空闲资格失效。
                            state <= ST_WAIT_LP11;
                            lp11_first_seen <= 1'b0;
                        end
                    endcase
                end

                ST_WAIT_LP00: begin
                    tail_count <= {TAIL_COUNTER_WIDTH{1'b0}};
                    stop_lp11_first_seen <= 1'b0;
                    case (lp_lane0_sync)
                        2'b00: begin
                            state <= ST_HS_ACTIVE;
                            data_hs_enable <= 1'b1;
                            lp11_first_seen <= 1'b0;
                        end
                        2'b01: begin
                            state <= ST_WAIT_LP00;
                            data_hs_enable <= (EARLY_HS_ENABLE != 0);
                        end
                        2'b11: begin
                            // 请求被 LP-11 中止；从当前样本重新累计空闲序列。
                            state <= ST_WAIT_LP11;
                            lp11_first_seen <= 1'b1;
                            data_hs_enable <= 1'b0;
                        end
                        default: begin
                            // LP-10 中止请求；之后必须重新观察 LP-11 STOP。
                            state <= ST_WAIT_LP11;
                            lp11_first_seen <= 1'b0;
                            data_hs_enable <= 1'b0;
                        end
                    endcase
                end

                ST_HS_ACTIVE: begin
                    data_hs_enable <= 1'b1;
                    tail_count <= {TAIL_COUNTER_WIDTH{1'b0}};
                    if (lp_lane0_sync == 2'b11) begin
                        if (stop_lp11_first_seen) begin
                            // 只有连续两个 LP-11 样本才确认 STOP 并开始排空。
                            state <= ST_TAIL;
                            tail_count <= TAIL_LOAD;
                            stop_lp11_first_seen <= 1'b0;
                        end else begin
                            stop_lp11_first_seen <= 1'b1;
                        end
                    end else begin
                        // HS 中的孤立 LP-01/10/11 均不关闭接收器。
                        stop_lp11_first_seen <= 1'b0;
                    end
                end

                ST_TAIL: begin
                    stop_lp11_first_seen <= 1'b0;
                    if (lp_lane0_sync == 2'b01) begin
                        // STOP 已确认后，新 LP-01 先关一拍以复位旧 byte aligner。
                        state <= ST_TAIL_RESTART_GAP;
                        tail_count <= {TAIL_COUNTER_WIDTH{1'b0}};
                        data_hs_enable <= 1'b0;
                        lp11_first_seen <= 1'b0;
                    end else if (tail_count <= {{(TAIL_COUNTER_WIDTH-1){1'b0}}, 1'b1}) begin
                        tail_count <= {TAIL_COUNTER_WIDTH{1'b0}};
                        data_hs_enable <= 1'b0;
                        if (lp_lane0_sync == 2'b11) begin
                            // 已确认的 LP-11 STOP 仍有效，可接受下一次 LP-01。
                            state <= ST_WAIT_LP01;
                            lp11_first_seen <= 1'b0;
                        end else begin
                            state <= ST_WAIT_LP11;
                            lp11_first_seen <= 1'b0;
                        end
                    end else begin
                        tail_count <= tail_count - 1'b1;
                        data_hs_enable <= 1'b1;
                    end
                end

                ST_TAIL_RESTART_GAP: begin
                    // 状态至少维持一个 byte_clk 周期，确保旧 aligner 被复位。
                    tail_count <= {TAIL_COUNTER_WIDTH{1'b0}};
                    stop_lp11_first_seen <= 1'b0;
                    data_hs_enable <= 1'b0;
                    case (lp_lane0_sync)
                        2'b01: begin
                            state <= ST_WAIT_LP00;
                            if (EARLY_HS_ENABLE != 0)
                                data_hs_enable <= 1'b1;
                        end
                        2'b00: begin
                            state <= ST_HS_ACTIVE;
                            data_hs_enable <= 1'b1;
                        end
                        2'b11: begin
                            state <= ST_WAIT_LP11;
                            lp11_first_seen <= 1'b1;
                        end
                        default: begin
                            state <= ST_WAIT_LP11;
                            lp11_first_seen <= 1'b0;
                        end
                    endcase
                end

                default: begin
                    state <= ST_WAIT_LP11;
                    lp11_first_seen <= 1'b0;
                    stop_lp11_first_seen <= 1'b0;
                    tail_count <= {TAIL_COUNTER_WIDTH{1'b0}};
                    data_hs_enable <= 1'b0;
                end
            endcase
        end
    end
endmodule

// ==================== csi2_burst_header_gate.v ====================
`timescale 1ns/1ps
// 按外部 burst 标记限定 CSI 事件，拦截 D-PHY READY 零尾产生的假 FS。
module csi2_burst_header_gate #(
    parameter [15:0] RAW10_WORD_COUNT = 16'd2400,
    parameter integer LOW_TAIL_CYCLES = 4,
    parameter integer ARM_DELAY_CYCLES = 4
)(
    input  wire        byte_clk,
    input  wire        reset_n,
    input  wire        burst_active,
    input  wire        csi_sp_en,
    input  wire        csi_lp_en,
    input  wire        csi_lp_av_en,
    input  wire        csi_ecc_ok,
    input  wire [1:0]  csi_vc,
    input  wire [5:0]  csi_dt,
    input  wire [15:0] csi_wc,
    input  wire [1:0]  csi_payload_dv,
    output wire        qualified_sp_en,
    output wire        qualified_lp_av_en,
    output wire [1:0]  qualified_payload_dv
);
    localparam integer TAIL_COUNT_WIDTH =
        (LOW_TAIL_CYCLES < 2) ? 1 : $clog2(LOW_TAIL_CYCLES + 1);
    localparam integer ARM_CYCLES = (ARM_DELAY_CYCLES < 1) ? 1 : ARM_DELAY_CYCLES;
    localparam integer ARM_COUNT_WIDTH = (ARM_CYCLES < 2) ? 1 : $clog2(ARM_CYCLES + 1);

    reg burst_active_d;
    reg burst_window;
    reg burst_armed;
    reg burst_event_seen;
    reg [TAIL_COUNT_WIDTH-1:0] low_tail_count;
    reg [ARM_COUNT_WIDTH-1:0] arm_count;
    reg av_authorized;
    reg payload_authorized;
    reg payload_seen;

    wire burst_rise = burst_active && !burst_active_d;
    wire burst_tail_close = !burst_active && burst_window &&
                            (low_tail_count >= LOW_TAIL_CYCLES-1);
    // HS 上升沿可能与旧 burst 的 CSI 尾事件同拍，禁止该拍组合放行。
    wire first_event = burst_window && burst_armed && !burst_event_seen && !burst_rise;
    wire header_good = csi_ecc_ok && (csi_vc == 2'b00);
    wire raw10_header_good = header_good && (csi_dt == 6'h2b) &&
                             (csi_wc == RAW10_WORD_COUNT);
    wire event_consumed = csi_sp_en || csi_lp_en;
    wire raw10_av_authorized = av_authorized ||
                               (first_event && csi_lp_en && raw10_header_good);

    // FS 的 WC 可合法回绕为零；有效性由 HS burst 和首头时序限定，不用 WC 猜包边界。
    // SP、ECC、VC、DT、WC 由官方 CSI RX 在同一 byte_clk 输出周期给出。
    assign qualified_sp_en = csi_sp_en && first_event && header_good;
    assign qualified_lp_av_en = csi_lp_av_en && raw10_av_authorized &&
                                  raw10_header_good;
    // B2P 仅接收合格 RAW10 长包的 payload，坏 ECC 和尾部字节均被屏蔽。
    assign qualified_payload_dv = payload_authorized ? csi_payload_dv : 2'b00;

    always @(posedge byte_clk or negedge reset_n) begin
        if (!reset_n) begin
            burst_active_d <= 1'b0;
            burst_window <= 1'b0;
            burst_armed <= 1'b0;
            burst_event_seen <= 1'b0;
            low_tail_count <= {TAIL_COUNT_WIDTH{1'b0}};
            arm_count <= {ARM_COUNT_WIDTH{1'b0}};
            av_authorized <= 1'b0;
            payload_authorized <= 1'b0;
            payload_seen <= 1'b0;
        end else begin
            burst_active_d <= burst_active;

            if (burst_rise) begin
                burst_window <= 1'b1;
                burst_armed <= 1'b0;
                burst_event_seen <= 1'b0;
                low_tail_count <= {TAIL_COUNT_WIDTH{1'b0}};
                arm_count <= {ARM_COUNT_WIDTH{1'b0}};
                av_authorized <= 1'b0;
                payload_authorized <= 1'b0;
                payload_seen <= 1'b0;
            end else begin
                if (burst_active) begin
                    burst_window <= 1'b1;
                    low_tail_count <= {TAIL_COUNT_WIDTH{1'b0}};
                end else if (burst_window) begin
                    if (burst_tail_close) begin
                        burst_window <= 1'b0;
                        burst_armed <= 1'b0;
                        burst_event_seen <= 1'b0;
                        low_tail_count <= {TAIL_COUNT_WIDTH{1'b0}};
                        av_authorized <= 1'b0;
                        payload_authorized <= 1'b0;
                        payload_seen <= 1'b0;
                    end else begin
                        low_tail_count <= low_tail_count + 1'b1;
                    end
                end else begin
                    burst_event_seen <= 1'b0;
                end

                // CSI 的旧注册事件在 HS 上升后仍可能排队；等 SoT 和头译码延迟过去再开窗。
                if (burst_window && !burst_armed) begin
                    if (arm_count >= ARM_CYCLES-1) begin
                        burst_armed <= 1'b1;
                    end else begin
                        arm_count <= arm_count + 1'b1;
                    end
                end

                if (burst_window && burst_armed && event_consumed)
                    burst_event_seen <= 1'b1;

                // LP_EN 与 LP_AV_EN 同拍时直接放行；若 AV 延后一拍则保留一次授权。
                if (csi_lp_av_en && av_authorized) begin
                    av_authorized <= 1'b0;
                end else if (first_event && csi_lp_en && raw10_header_good &&
                             !csi_lp_av_en) begin
                    av_authorized <= 1'b1;
                end

                // payload 授权从合格头开始，覆盖整个 payload，直到 DV 下降。
                if (first_event && csi_lp_en && raw10_header_good) begin
                    payload_authorized <= 1'b1;
                    payload_seen <= 1'b0;
                end else if (payload_authorized && (|csi_payload_dv)) begin
                    payload_seen <= 1'b1;
                end else if (payload_authorized && payload_seen) begin
                    payload_authorized <= 1'b0;
                    payload_seen <= 1'b0;
                end
            end
        end
    end
endmodule

// ==================== ov5647_csi2_capture.v ====================
// OV5647两lane CSI-2 RAW10接收；保留可靠收包链并输出640x360彩色YUYV422。
module ov5647_csi2_capture #(
    // RAW10双像素顺序和CFA相位应结合初始化表及色卡实测；默认保留当前板级假设。
    parameter integer RAW_PAIR_LOW_FIRST = 1,
    parameter integer BAYER_PATTERN = 0,
    // 静态线性RGB白平衡增益：每通道10位无符号UQ2.8，码值0..1023对应0..3.99609375，256表示1.0。
    parameter [9:0] WB_R_GAIN_UQ2_8 = 10'd256,
    parameter [9:0] WB_G_GAIN_UQ2_8 = 10'd256,
    parameter [9:0] WB_B_GAIN_UQ2_8 = 10'd256
) (
    input  wire clk50m,
    input  wire reset_n,
    input  wire pixel_reset_n,
    input  wire sensor_init_done,
    input  wire dphy_reset_n,
    input  wire mipi_clk_p,
    input  wire mipi_clk_n,
    input  wire mipi_lane0_p,
    input  wire mipi_lane0_n,
    input  wire mipi_lane1_p,
    input  wire mipi_lane1_n,
    input  wire [1:0] mipi_lp_clk,
    input  wire [1:0] mipi_lp_lane0,
    input  wire [1:0] mipi_lp_lane1,
    output wire pixel_clk,
    output wire camera_vsync,
    output wire camera_frame_bad,
    output wire camera_pixel_valid,
    output wire [15:0] camera_pixel,
    output wire [31:0] debug_status,
    output wire [31:0] autocal_burst_total_gray,
    output wire [31:0] autocal_control_word,
    output wire [31:0] autocal_bad_burst_gray,
    output wire [31:0] guard_bad_good_gray16,
    output wire [31:0] guard_total_frame_gray,
    output wire [3:0] guard_failure_sticky
);
    wire byte_clk;
    wire [7:0] lane0_data;
    wire [7:0] lane1_data;
    wire [3:0] dphy_ready;
    wire camera_pixel_pll_lock;
    wire csi_short_packet;
    wire csi_long_packet;
    wire csi_video_packet;
    wire csi_ecc_ok;
    wire [7:0] csi_ecc;
    wire [15:0] csi_word_count;
    wire [1:0] csi_virtual_channel;
    wire [5:0] csi_data_type;
    wire [15:0] csi_payload;
    wire [1:0] csi_payload_valid;
    wire raw_camera_pixel_valid;
    wire [15:0] raw_camera_pixel;
    wire [29:0] raw_camera_rgb10;
    wire qualified_short_packet;
    wire qualified_video_packet;
    wire [1:0] qualified_payload_valid;
    wire qualified_frame_short_packet;
    wire autocal_qualified_header;
    wire byte_frame_valid;
    wire pixel_line_valid;
    wire [19:0] raw10_pixels;
    (* ASYNC_REG="TRUE" *) reg [1:0] byte_init_sync;
    (* ASYNC_REG="TRUE" *) reg [1:0] pixel_init_sync;
    (* ASYNC_REG="TRUE" *) reg [1:0] pixel_pll_lock_sync;
    (* ASYNC_REG="TRUE" *) reg [1:0] pixel_frame_valid_sync;
    (* ASYNC_REG="TRUE" *) reg [1:0] byte_por_reset_sync;
    (* ASYNC_REG="TRUE" *) reg [1:0] autocal_start_sync;
    (* ASYNC_REG="TRUE" *) reg [1:0] pixel_autocal_ready_sync;
    wire autocal_ready;
    always @(posedge byte_clk or negedge reset_n)
        if (!reset_n) byte_init_sync <= 2'b00;
        else byte_init_sync <= {byte_init_sync[0], sensor_init_done};
    always @(posedge pixel_clk or negedge reset_n)
        if (!reset_n) pixel_init_sync <= 2'b00;
        else pixel_init_sync <= {pixel_init_sync[0], sensor_init_done};
    // 锁相环失锁时异步清零，同步等待新时钟稳定后再释放像素域复位。
    always @(posedge pixel_clk or negedge reset_n or negedge camera_pixel_pll_lock)
        if (!reset_n || !camera_pixel_pll_lock) pixel_pll_lock_sync <= 2'b00;
        else pixel_pll_lock_sync <= {pixel_pll_lock_sync[0], 1'b1};
    wire byte_pipeline_reset_n = byte_init_sync[1];
    wire pixel_pipeline_reset_n = pixel_reset_n && pixel_init_sync[1] && pixel_pll_lock_sync[1];
    wire pipeline_reset_n = byte_pipeline_reset_n && pixel_pipeline_reset_n;

    // 自动校准器使用 byte_clk 域同步释放的板级 POR；启动条件另行跨域同步。
    always @(posedge byte_clk or negedge reset_n)
        if (!reset_n) byte_por_reset_sync <= 2'b00;
        else byte_por_reset_sync <= {byte_por_reset_sync[0], 1'b1};
    always @(posedge byte_clk or negedge byte_por_reset_sync[1])
        if (!byte_por_reset_sync[1]) autocal_start_sync <= 2'b00;
        else autocal_start_sync <= {autocal_start_sync[0], (sensor_init_done && dphy_reset_n)};
    always @(posedge pixel_clk or negedge pixel_pipeline_reset_n)
        if (!pixel_pipeline_reset_n) pixel_autocal_ready_sync <= 2'b00;
        else pixel_autocal_ready_sync <= {pixel_autocal_ready_sync[0], autocal_ready};

    // 官方网表中的 O_FV 由 I_BYTE_CLK 驱动，进入像素域后必须经过两级同步。
    always @(posedge pixel_clk or negedge pixel_pipeline_reset_n)
        if (!pixel_pipeline_reset_n) pixel_frame_valid_sync <= 2'b00;
        else pixel_frame_valid_sync <= {pixel_frame_valid_sync[0], byte_frame_valid};
    wire pixel_frame_valid_aligned = pixel_frame_valid_sync[1];

    // 连续时钟下确认 LP-11 后，在 LP-01 提前启动每次突发的字节对齐。
    // 独立 LP 球位使用 LVCMOS10UD33，不能改用 3.3 V 输入阈值。
    wire data_hs_enable;
    wire data_hs_started;
    wire [7:0] autocal_tap_lane0;
    wire [7:0] autocal_tap_lane1;
    wire autocal_delay_load_lane0;
    wire autocal_delay_load_lane1;
    wire autocal_fault;
    wire [3:0] autocal_state;
    wire [4:0] autocal_scan_row;
    wire [4:0] autocal_scan_col;
    wire [12:0] autocal_verify_count;
    wire [15:0] autocal_good_frame_count;
    wire [31:0] autocal_burst_total_count;
    wire [31:0] autocal_bad_burst_count;
    wire raw_guard_vsync;
    wire raw_guard_bad;
    wire diag_pixel_geometry, diag_frame_overlap, diag_end_timeout, diag_no_fe_timeout;
    reg ready_latched_in_frame;
    reg frame_rgb_seen;
    reg raw_guard_vsync_d;
    reg good_eof_toggle_pixel;
    reg bad_guard_eof_toggle_pixel;
    reg [15:0] guard_good_frame_count;
    reg [15:0] guard_bad_frame_count;
    reg [31:0] guard_total_frame_count;
    ov5647_lp_hs_control lp_hs_control (
        .byte_clk(byte_clk),
        .reset_n(reset_n),
        .lp_lane0(mipi_lp_lane0),
        .data_hs_enable(data_hs_enable),
        .data_hs_started(data_hs_started)
    );

    Camera_DPHY_RX camera_dphy_rx (
        .reset_n(dphy_reset_n),
        .HS_CLK_P(mipi_clk_p),
        .HS_CLK_N(mipi_clk_n),
        .sclk_out(byte_clk),
        .HS_DATA0_P(mipi_lane0_p),
        .HS_DATA0_N(mipi_lane0_n),
        .data_out0(lane0_data),
        .HS_DATA1_P(mipi_lane1_p),
        .HS_DATA1_N(mipi_lane1_n),
        .data_out1(lane1_data),
        .clk_hs_en(1'b1),
        .data_hs_en(data_hs_enable),
        .delay_taps_lane0(autocal_tap_lane0),
        .delay_load_pulse_lane0(autocal_delay_load_lane0),
        .delay_taps_lane1(autocal_tap_lane1),
        .delay_load_pulse_lane1(autocal_delay_load_lane1),
        .clk_term_en(1'b0),
        .data_term_en(1'b0),
        .ready(dphy_ready),
        .sync_lane0(),
        .sync_lane1()
    );

    // RAW10 两 lane、Gear 8、2 pixels/clock：f_byte/f_pixel=20/(2*8)=1.25。
    // OV5647 1080p30 模式约 54.6875 MHz byte clock，对应 43.75 MHz pixel clock。
    Camera_Pixel_PLL camera_pixel_pll (
        .clkin(clk50m),
        .init_clk(clk50m),
        .clkout0(pixel_clk),
        .lock(camera_pixel_pll_lock)
    );

    DSI_CSI2_RX_TOP csi2_receiver (
        .I_RSTN(byte_pipeline_reset_n),
        .I_BYTE_CLK(byte_clk),
        .I_REF_DT(6'h2b),
        .I_READY(dphy_ready[0]),
        .I_DATA0(lane0_data),
        .I_DATA1(lane1_data),
        .O_SP_EN(csi_short_packet),
        .O_LP_EN(csi_long_packet),
        .O_LP_AV_EN(csi_video_packet),
        .O_ECC_OK(csi_ecc_ok),
        .O_ECC(csi_ecc),
        .O_WC(csi_word_count),
        .O_VC(csi_virtual_channel),
        .O_DT(csi_data_type),
        .O_PAYLOAD(csi_payload),
        .O_PAYLOAD_DV(csi_payload_valid)
    );

    // 4800=04 的普通模式每次 HS 突发一个包；拒绝 READY 零尾伪 FS 与坏 ECC 头。
    // 数据有效信号也必须受控，否则 B2P 仍会仅凭 payload 生成像素行。
    csi2_burst_header_gate header_gate (
        .byte_clk(byte_clk), .reset_n(byte_pipeline_reset_n),
        .burst_active(data_hs_enable),
        .csi_sp_en(csi_short_packet), .csi_lp_en(csi_long_packet),
        .csi_lp_av_en(csi_video_packet), .csi_ecc_ok(csi_ecc_ok),
        .csi_vc(csi_virtual_channel), .csi_dt(csi_data_type),
        .csi_wc(csi_word_count), .csi_payload_dv(csi_payload_valid),
        .qualified_sp_en(qualified_short_packet),
        .qualified_lp_av_en(qualified_video_packet),
        .qualified_payload_dv(qualified_payload_valid)
    );

    // 自动校准只把 RAW10 长包和帧起止短包计为有效；LS/LE 仍保留给原 B2P。
    assign qualified_frame_short_packet = qualified_short_packet &&
        ((csi_data_type == 6'h00) || (csi_data_type == 6'h01));
    assign autocal_qualified_header = qualified_frame_short_packet || qualified_video_packet;

    wire source_complete_byte;
    wire source_eof_toggle_byte;
    ov5647_source_frame_accounting #(
        .EXPECTED_LINES(1080), .BYTES_PER_LINE(2400)
    ) source_frame_accounting (
        .byte_clk(byte_clk), .reset_n(pipeline_reset_n),
        .short_packet(qualified_short_packet), .data_type(csi_data_type),
        .video_packet(qualified_video_packet), .payload_valid(qualified_payload_valid),
        .complete_snapshot(source_complete_byte), .eof_toggle(source_eof_toggle_byte)
    );

    wire source_status_valid_pixel, source_complete_pixel;
    reg pixel_frame_valid_d;
    wire pixel_frame_valid_rise = pixel_frame_valid_aligned && !pixel_frame_valid_d;
    ov5647_source_status_cdc source_status_cdc (
        .pixel_clk(pixel_clk), .reset_n(pixel_pipeline_reset_n),
        .frame_start(pixel_frame_valid_rise), .eof_toggle_byte(source_eof_toggle_byte),
        .complete_byte(source_complete_byte), .status_valid(source_status_valid_pixel),
        .frame_complete(source_complete_pixel)
    );

    // 训练评分只接收合格 RAW10/FS/FE；原始 good/bad guard EOF 独立跨域。
    // 每个tap采1024个真实RAW10 HS burst，约覆盖一帧1080行；保留完整32x32眼图。
    // 旧4096样本使完整扫描需要约四帧/ tap，实板每次上电长时间没有任何可发布帧。
    // 选中中心后仍保留原有2秒稳定时间、1024 burst验证及两帧good EOF门槛。
    wire [3:0] autocal_last_failure;
    wire [7:0] autocal_retry_count;
    ov5647_dphy_autocal #(
        .POINT_BURSTS(1024),
        .VERIFY_BURSTS(1024)
    ) autocal (
        .byte_clk(byte_clk),
        .reset_n(byte_por_reset_sync[1]),
        .start_enable(autocal_start_sync[1]),
        .burst_active(data_hs_enable),
        .burst_hs_started(data_hs_started),
        .lp_lane0_raw(mipi_lp_lane0),
        .lp_lane1_raw(mipi_lp_lane1),
        .qualified_header(autocal_qualified_header),
        .good_eof_toggle_pixel(good_eof_toggle_pixel),
        .bad_guard_eof_toggle_pixel(bad_guard_eof_toggle_pixel),
        .tap_lane0(autocal_tap_lane0),
        .tap_lane1(autocal_tap_lane1),
        .delay_load_lane0(autocal_delay_load_lane0),
        .delay_load_lane1(autocal_delay_load_lane1),
        .ready(autocal_ready),
        .fault(autocal_fault),
        .last_failure_reason_o(autocal_last_failure),
        .retry_count_o(autocal_retry_count),
        .state(autocal_state),
        .scan_row(autocal_scan_row),
        .scan_col(autocal_scan_col),
        .verify_count(autocal_verify_count),
        .good_frame_count(autocal_good_frame_count),
        .burst_total_count(autocal_burst_total_count),
        .bad_burst_count(autocal_bad_burst_count)
    );

    Byte_to_Pixel_Top raw10_converter (
        .I_RSTN(pipeline_reset_n),
        .I_BYTE_CLK(byte_clk),
        .I_PIXEL_CLK(pixel_clk),
        .I_SP_EN(qualified_short_packet),
        .I_LP_AV_EN(qualified_video_packet),
        .I_DT(csi_data_type),
        .I_WC(csi_word_count),
        .I_PAYLOAD_DV(qualified_payload_valid),
        .I_PAYLOAD(csi_payload),
        .O_FV(byte_frame_valid),
        .O_LV(pixel_line_valid),
        .O_PIXEL(raw10_pixels)
    );

    // 1080p RAW10 每帧须有 1080 条、每条 960 个 byte-to-pixel 周期。
    // O_FV 早于最后 O_LV 结束；源账本完整时允许尾行排空，缺失与超时均拒收。
    ov5647_frame_integrity_guard #(
        .EXPECTED_LINES(1080),
        .PAIRS_PER_LINE(960),
        .USE_SOURCE_FRAME_COMPLETE(1)
    ) frame_integrity_guard (
        .pixel_clk(pixel_clk),
        .reset_n(pixel_pipeline_reset_n),
        .frame_valid(pixel_frame_valid_aligned),
        .line_valid(pixel_line_valid),
        .source_status_valid(source_status_valid_pixel),
        .source_frame_complete(source_complete_pixel),
        .camera_vsync(raw_guard_vsync),
        .camera_frame_bad(raw_guard_bad),
        .diag_pixel_geometry(diag_pixel_geometry), .diag_frame_overlap(diag_frame_overlap),
        .diag_end_timeout(diag_end_timeout), .diag_no_fe_timeout(diag_no_fe_timeout)
    );

    ov5647_guard_failure_flags failure_diagnostics (
        .pixel_clk(pixel_clk), .reset_n(pixel_pipeline_reset_n),
        .guard_vsync(raw_guard_vsync), .guard_bad(raw_guard_bad),
        .frame_allowed(ready_latched_in_frame),
        .source_status_valid(source_status_valid_pixel), .source_frame_complete(source_complete_pixel),
        .pixel_geometry(diag_pixel_geometry), .frame_overlap(diag_frame_overlap),
        .end_timeout(diag_end_timeout), .no_fe_timeout(diag_no_fe_timeout),
        .sticky(guard_failure_sticky)
    );

    // 发布门控仅影响输出；guard good/bad EOF 仍只取原始 guard 状态。
    wire raw_camera_frame_bad = raw_guard_bad || !ready_latched_in_frame;

    always @(posedge pixel_clk or negedge pixel_pipeline_reset_n) begin
        if (!pixel_pipeline_reset_n) begin
            pixel_frame_valid_d <= 1'b0;
            ready_latched_in_frame <= 1'b0;
            frame_rgb_seen <= 1'b0;
            raw_guard_vsync_d <= 1'b1;
            good_eof_toggle_pixel <= 1'b0;
            bad_guard_eof_toggle_pixel <= 1'b0;
            guard_good_frame_count <= 16'd0;
            guard_bad_frame_count <= 16'd0;
            guard_total_frame_count <= 32'd0;
        end else begin
            pixel_frame_valid_d <= pixel_frame_valid_aligned;
            raw_guard_vsync_d <= raw_guard_vsync;
            if (pixel_frame_valid_rise) begin
                ready_latched_in_frame <= pixel_autocal_ready_sync[1];
                frame_rgb_seen <= 1'b0;
            end else if (raw_camera_pixel_valid) begin
                frame_rgb_seen <= 1'b1;
            end
            if (raw_guard_vsync && !raw_guard_vsync_d) begin
                guard_total_frame_count <= guard_total_frame_count + 1'b1;
                if (raw_guard_bad) begin
                    guard_bad_frame_count <= guard_bad_frame_count + 1'b1;
                    bad_guard_eof_toggle_pixel <= !bad_guard_eof_toggle_pixel;
                end else begin
                    guard_good_frame_count <= guard_good_frame_count + 1'b1;
                    good_eof_toggle_pixel <= !good_eof_toggle_pixel;
                end
            end
        end
    end

    wire [15:0] autocal_good_frame_gray =
        autocal_good_frame_count ^ (autocal_good_frame_count >> 1);
    // Register Gray codes before crossing clock domains. A combinational XOR
    // of a binary counter can glitch on carries and adds logic before sync1.
    reg [15:0] guard_good_frame_gray,guard_bad_frame_gray;
    always @(posedge pixel_clk or negedge pixel_pipeline_reset_n) begin
        if (!pixel_pipeline_reset_n) begin
            guard_good_frame_gray <= 0;
            guard_bad_frame_gray <= 0;
        end else begin
            guard_good_frame_gray <= guard_good_frame_count ^ (guard_good_frame_count >> 1);
            guard_bad_frame_gray <= guard_bad_frame_count ^ (guard_bad_frame_count >> 1);
        end
    end
    assign autocal_burst_total_gray = autocal_burst_total_count ^ (autocal_burst_total_count >> 1);
    // 重选候选或故障后重扫均累计重试次数，饱和保存到下次 POR。
    reg [31:0] autocal_control_q;
    always @(posedge byte_clk or negedge byte_por_reset_sync[1]) begin
        if (!byte_por_reset_sync[1]) autocal_control_q <= 0;
        else autocal_control_q <= {4'hC, autocal_last_failure,
            autocal_tap_lane0, autocal_tap_lane1, autocal_retry_count};
    end
    assign autocal_control_word = autocal_control_q;
    assign autocal_bad_burst_gray = autocal_bad_burst_count ^ (autocal_bad_burst_count >> 1);
    assign guard_bad_good_gray16 = {guard_bad_frame_gray, guard_good_frame_gray};
    assign guard_total_frame_gray = guard_total_frame_count ^ (guard_total_frame_count >> 1);

    // 固定3x3 RAW10转YUYV路径，在pixel_clk域直接处理。
    bx72_gowin_isp_pipeline #(
        .RAW_PAIR_LOW_FIRST(RAW_PAIR_LOW_FIRST),
        .BAYER_PATTERN(BAYER_PATTERN),
        .WB_R_GAIN_UQ2_8(WB_R_GAIN_UQ2_8),
        .WB_G_GAIN_UQ2_8(WB_G_GAIN_UQ2_8),
        .WB_B_GAIN_UQ2_8(WB_B_GAIN_UQ2_8)
    ) official_isp (
        .raw_clk(pixel_clk),.reset_n(pixel_pipeline_reset_n),
        .raw_frame_valid(pixel_frame_valid_aligned),.raw_line_valid(pixel_line_valid),
        .raw_pair(raw10_pixels),.raw_vsync(raw_guard_vsync),.raw_frame_bad(raw_camera_frame_bad),
        .camera_vsync(camera_vsync),.camera_frame_bad(camera_frame_bad),
        .camera_pixel_valid(camera_pixel_valid),.camera_pixel(camera_pixel),
        .raw_overflow());
    // 训练计数仍只使用原始 guard EOF；本状态表示 RAW 活动，不跨域读 ISP valid。
    assign raw_camera_pixel_valid = pixel_line_valid;

    // UART 侧再用两级同步采样此状态字；计数值使用源时钟域寄存 Gray 码跨域。
    assign debug_status = {autocal_state, autocal_ready, autocal_fault,
                           autocal_scan_row, autocal_scan_col, autocal_verify_count,
                           ready_latched_in_frame, frame_rgb_seen, 1'b0};

    wire unused_status = csi_long_packet ^ csi_virtual_channel[0];
endmodule
