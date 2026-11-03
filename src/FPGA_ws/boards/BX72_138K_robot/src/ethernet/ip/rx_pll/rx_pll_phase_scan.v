// RGMII RX 时钟相位训练控制器。
// 输入 good_toggle_i 每收到一帧完整校验并提交的请求翻转一次；该信号
// 必须在采样接收器复位时保持，不得由 sampling_reset_o 清零。
module rx_pll_phase_scan #(
    parameter integer PHASE_TICKS = 30000000, // 50 MHz 下每相位 0.6 秒
    parameter integer VERIFY_TICKS = 30000000,
    parameter integer SETTLE_TICKS = 1000,
    parameter integer PULSE_HIGH_TICKS = 4,
    parameter integer PULSE_LOW_TICKS = 4
) (
    input  wire        init_clk,
    input  wire        resetlink_i,
    input  wire        pll_lock_i,
    input  wire        good_toggle_i,
    output reg         pspulse_o,
    output wire        sampling_reset_o,
    output reg         ready_o,
    output reg  [3:0]  phase_index_o,
    output wire [15:0] good_mask_o
);
    localparam [3:0] ST_WAIT_LOCK = 4'd0;
    localparam [3:0] ST_SETTLE    = 4'd1;
    localparam [3:0] ST_MEASURE   = 4'd2;
    localparam [3:0] ST_PULSE_HI  = 4'd3;
    localparam [3:0] ST_PULSE_LO  = 4'd4;
    localparam [3:0] ST_EVALUATE  = 4'd5;
    localparam [3:0] ST_EVAL_START= 4'd6;
    localparam [3:0] ST_EVAL_WALK = 4'd7;
    localparam [3:0] ST_EVAL_DONE = 4'd8;
    localparam [3:0] ST_VERIFY    = 4'd9;
    localparam [3:0] ST_READY     = 4'd10;
    localparam       SETTLE_SCAN  = 1'b0;
    localparam       SETTLE_VERIFY= 1'b1;

    reg [3:0] state;
    reg settle_next;
    reg [15:0] good_mask;
    reg [31:0] timer;
    reg [7:0] phase_good_count;
    reg [1:0] verify_good_count;
    reg [4:0] move_steps;
    reg [1:0] pulse_count;
    reg [7:0] pulse_timer;
    reg [3:0] eval_start;
    reg [3:0] eval_offset;
    reg [4:0] eval_run_len;
    reg [4:0] eval_best_len;
    reg [3:0] eval_best_center;
    (* ASYNC_REG = "TRUE" *) reg reset_meta, reset_sync;
    (* ASYNC_REG = "TRUE" *) reg good_meta, good_sync;
    (* ASYNC_REG = "TRUE" *) reg lock_meta, lock_sync;
    reg good_seen;

    wire [3:0] selected_delta;
    wire [3:0] phase_zero_delta;
    wire [4:0] eval_center_full={1'b0,eval_start}+((eval_run_len-5'd1)>>1);

    assign sampling_reset_o = resetlink_i || reset_sync || !lock_sync ||
        !((state == ST_MEASURE) || (state == ST_VERIFY) || (state == ST_READY));
    assign good_mask_o = good_mask;
    assign selected_delta = eval_best_center - phase_index_o;
    assign phase_zero_delta = 4'd0 - phase_index_o;

    // PLL 控制脉冲使用单个触发器输出，避免多位状态译码毛刺触发额外移相。
    // ready 注册后再跨到 TX；异步断链仍由顶层 udp_rx_reset_p 立即停车和清 FIFO。
    always @(posedge init_clk) begin
        if(reset_sync || !lock_sync)begin pspulse_o<=0;ready_o<=0;end
        else begin pspulse_o<=(state==ST_PULSE_HI);ready_o<=(state==ST_READY);end
    end

    // 将一次有效 good-toggle 变更同步到控制时钟域。采样器复位不复位此同步器。
    // 外部 link reset 异步断言，init_clk 两级同步释放。
    always @(posedge init_clk or posedge resetlink_i) begin
        if (resetlink_i) begin
            reset_meta <= 1'b1;
            reset_sync <= 1'b1;
        end else begin
            reset_meta <= 1'b0;
            reset_sync <= reset_meta;
        end
    end

    always @(posedge init_clk) begin
        if (reset_sync) begin
            good_meta <= 1'b0;
            good_sync <= 1'b0;
            lock_meta <= 1'b0;
            lock_sync <= 1'b0;
        end else begin
            case (good_toggle_i)
                1'b1:    good_meta <= 1'b1;
                default: good_meta <= 1'b0;
            endcase
            good_sync <= good_meta;
            lock_meta <= pll_lock_i;
            lock_sync <= lock_meta;
        end
    end

    always @(posedge init_clk) begin
        if (reset_sync) begin
            state <= ST_WAIT_LOCK;
            settle_next <= SETTLE_SCAN;
            good_mask <= 16'd0;
            phase_index_o <= 4'd0;
            timer <= 32'd0;
            phase_good_count <= 8'd0;
            verify_good_count <= 2'd0;
            move_steps <= 5'd0;
            pulse_count <= 2'd0;
            pulse_timer <= 8'd0;
            eval_start <= 4'd0;
            eval_offset <= 4'd0;
            eval_run_len <= 5'd0;
            eval_best_len <= 5'd0;
            eval_best_center <= 4'd0;
            good_seen <= 1'b0;
        end else if (reset_sync || !lock_sync) begin
            state <= ST_WAIT_LOCK;
            settle_next <= SETTLE_SCAN;
            good_mask <= 16'd0;
            phase_index_o <= 4'd0;
            timer <= 32'd0;
            phase_good_count <= 8'd0;
            verify_good_count <= 2'd0;
            move_steps <= 5'd0;
            pulse_count <= 2'd0;
            pulse_timer <= 8'd0;
            eval_start <= 4'd0;
            eval_offset <= 4'd0;
            eval_run_len <= 5'd0;
            eval_best_len <= 5'd0;
            eval_best_center <= 4'd0;
            good_seen <= good_sync;
        end else begin
            case (state)
                ST_WAIT_LOCK: begin
                    good_mask <= 16'd0;
                    phase_index_o <= 4'd0;
                    settle_next <= SETTLE_SCAN;
                    timer <= 32'd0;
                    phase_good_count <= 8'd0;
                    verify_good_count <= 2'd0;
                    move_steps <= 5'd0;
                    pulse_count <= 2'd0;
                    pulse_timer <= 8'd0;
                    good_seen <= good_sync;
                    state <= ST_SETTLE;
                end

                ST_SETTLE: begin
                    if (timer >= SETTLE_TICKS - 1) begin
                        timer <= 32'd0;
                        good_seen <= good_sync;
                        if (settle_next == SETTLE_VERIFY) begin
                            verify_good_count <= 2'd0;
                            state <= ST_VERIFY;
                        end else begin
                            phase_good_count <= 8'd0;
                            state <= ST_MEASURE;
                        end
                    end else begin
                        timer <= timer + 1'b1;
                    end
                end

                ST_MEASURE: begin
                    if (good_sync != good_seen) begin
                        good_seen <= good_sync;
                        if (phase_good_count != 8'hff)
                            phase_good_count <= phase_good_count + 1'b1;
                    end

                    if (timer >= PHASE_TICKS - 1) begin
                        timer <= 32'd0;
                        if ((phase_good_count != 0) || (good_sync != good_seen))
                            good_mask[phase_index_o] <= 1'b1;
                        else
                            good_mask[phase_index_o] <= 1'b0;

                        if (phase_index_o == 4'd15) begin
                            eval_start <= 4'd0;
                            eval_best_len <= 5'd0;
                            eval_best_center <= 4'd0;
                            state <= ST_EVALUATE;
                        end else begin
                            phase_index_o <= phase_index_o + 1'b1;
                            move_steps <= 5'd1;
                            pulse_count <= 2'd0;
                            pulse_timer <= 8'd0;
                            settle_next <= SETTLE_SCAN;
                            state <= ST_PULSE_HI;
                        end
                    end else begin
                        timer <= timer + 1'b1;
                    end
                end

                ST_PULSE_HI: begin
                    if (pulse_timer >= PULSE_HIGH_TICKS - 1) begin
                        pulse_timer <= 8'd0;
                        state <= ST_PULSE_LO;
                    end else begin
                        pulse_timer <= pulse_timer + 1'b1;
                    end
                end

                ST_PULSE_LO: begin
                    if (pulse_timer >= PULSE_LOW_TICKS - 1) begin
                        pulse_timer <= 8'd0;
                        if (pulse_count == 2'd2) begin
                            pulse_count <= 2'd0;
                            if (move_steps > 1) begin
                                move_steps <= move_steps - 1'b1;
                                state <= ST_PULSE_HI;
                            end else begin
                                move_steps <= 5'd0;
                                timer <= 32'd0;
                                state <= ST_SETTLE;
                            end
                        end else begin
                            pulse_count <= pulse_count + 1'b1;
                            state <= ST_PULSE_HI;
                        end
                    end else begin
                        pulse_timer <= pulse_timer + 1'b1;
                    end
                end

                ST_EVALUATE: begin
                    // 每周期只检查一个 run 起点；全 good 图直接选择相位 0。
                    if (good_mask == 16'hffff) begin
                        eval_best_len <= 5'd16;
                        eval_best_center <= 4'd0;
                        state <= ST_EVAL_DONE;
                    end else begin
                        eval_start <= 4'd0;
                        eval_best_len <= 5'd0;
                        eval_best_center <= 4'd0;
                        state <= ST_EVAL_START;
                    end
                end

                ST_EVAL_START: begin
                    if (good_mask[eval_start] &&
                        !good_mask[(eval_start + 4'd15)]) begin
                        eval_run_len <= 5'd1;
                        eval_offset <= 4'd1;
                        state <= ST_EVAL_WALK;
                    end else if (eval_start == 4'd15) begin
                        state <= ST_EVAL_DONE;
                    end else begin
                        eval_start <= eval_start + 1'b1;
                    end
                end

                ST_EVAL_WALK: begin
                    if ((eval_offset == 4'd0) ||
                        !good_mask[(eval_start + eval_offset)]) begin
                        if (eval_run_len > eval_best_len) begin
                            eval_best_len <= eval_run_len;
                            eval_best_center <= eval_center_full[3:0]; // 循环相位按16取模
                        end
                        if (eval_start == 4'd15) begin
                            state <= ST_EVAL_DONE;
                        end else begin
                            eval_start <= eval_start + 1'b1;
                            state <= ST_EVAL_START;
                        end
                    end else begin
                        eval_run_len <= eval_run_len + 1'b1;
                        eval_offset <= eval_offset + 1'b1;
                    end
                end

                ST_EVAL_DONE: begin
                    if (eval_best_len == 0) begin
                        // 全环没有 good 点；回到 0 相位并重新扫描。
                        good_mask <= 16'd0;
                        phase_index_o <= 4'd0;
                        move_steps <= 5'd1;
                        pulse_count <= 2'd0;
                        pulse_timer <= 8'd0;
                        settle_next <= SETTLE_SCAN;
                        state <= ST_PULSE_HI;
                    end else begin
                        phase_index_o <= eval_best_center;
                        settle_next <= SETTLE_VERIFY;
                        if (selected_delta == 0) begin
                            timer <= 32'd0;
                            state <= ST_SETTLE;
                        end else begin
                            move_steps <= {1'b0, selected_delta};
                            pulse_count <= 2'd0;
                            pulse_timer <= 8'd0;
                            state <= ST_PULSE_HI;
                        end
                    end
                end

                ST_VERIFY: begin
                    if (good_sync != good_seen) begin
                        good_seen <= good_sync;
                        if (verify_good_count == 2'd2) begin
                            state <= ST_READY;
                        end else begin
                            verify_good_count <= verify_good_count + 1'b1;
                        end
                    end

                    if ((timer >= VERIFY_TICKS - 1) &&
                        !((good_sync != good_seen) && (verify_good_count == 2'd2))) begin
                        // 复核不够 3 帧，回到 0 相位完整重扫。
                        timer <= 32'd0;
                        good_mask <= 16'd0;
                        phase_good_count <= 8'd0;
                        phase_index_o <= 4'd0;
                        settle_next <= SETTLE_SCAN;
                        if (phase_index_o == 4'd0) begin
                            state <= ST_SETTLE;
                        end else begin
                            move_steps <= {1'b0, phase_zero_delta};
                            pulse_count <= 2'd0;
                            pulse_timer <= 8'd0;
                            state <= ST_PULSE_HI;
                        end
                    end else if (timer < VERIFY_TICKS - 1) begin
                        timer <= timer + 1'b1;
                    end
                end

                ST_READY: begin
                    // 链路/PLL 失锁会在上面的公共路径中清除 ready 并重新训练。
                    state <= ST_READY;
                end

                default: begin
                    state <= ST_WAIT_LOCK;
                    good_mask <= 16'd0;
                    phase_index_o <= 4'd0;
                    timer <= 32'd0;
                end
            endcase
        end
    end
endmodule
