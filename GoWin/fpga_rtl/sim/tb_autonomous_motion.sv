`timescale 1ns/1ps

// 单一自检覆盖安全限速、左右绕障和 MOVE/ROTATE/STOP 路径暂停恢复。
module tb_autonomous_motion;

    localparam OP_MOVE   = 2'd0;
    localparam OP_ROTATE = 2'd1;
    localparam OP_STOP   = 2'd2;

    localparam AVOID_IDLE    = 3'd0;
    localparam AVOID_STOP    = 3'd1;
    localparam AVOID_SELECT  = 3'd2;
    localparam AVOID_BACKUP  = 3'd3;
    localparam AVOID_TURN    = 3'd4;
    localparam AVOID_BYPASS  = 3'd5;
    localparam AVOID_RECOVER = 3'd6;

    reg clk_i = 1'b0;
    reg rst_n_i = 1'b0;
    reg start_i = 1'b0;
    reg abort_i = 1'b0;
    reg path_pause_i = 1'b0;
    reg motion_stopped_i = 1'b1;
    reg avoidance_tick_i = 1'b1;

    reg [15:0] front_range_i = 16'd1000;
    reg [15:0] left_range_i = 16'd800;
    reg [15:0] right_range_i = 16'd300;
    reg front_range_valid_i = 1'b1;
    reg left_range_valid_i = 1'b1;
    reg right_range_valid_i = 1'b1;

    wire segment_ready_o;
    reg segment_valid_i = 1'b0;
    reg [1:0] segment_opcode_i = OP_STOP;
    reg [31:0] segment_target_i = 32'd0;
    reg signed [31:0] segment_command_i = 32'sd0;
    reg segment_last_i = 1'b0;
    reg [31:0] segment_progress_i = 32'd0;

    wire path_active_o;
    wire path_finished_o;
    wire path_progress_enable_o;
    wire [7:0] segment_index_o;
    wire path_paused_o;
    wire avoidance_active_o;
    wire avoidance_done_o;
    wire avoid_left_o;
    wire [2:0] avoidance_state_o;
    wire [1:0] safety_level_o;
    wire command_valid_o;
    wire signed [31:0] linear_o;
    wire signed [31:0] angular_o;

    integer check_count = 0;

    autonomous_motion_controller #(
        .STOP_DISTANCE(16'd200),
        .SLOW_DISTANCE(16'd500),
        .STOP_TICKS(1),
        .BACKUP_TICKS(1),
        .TURN_TICKS(1),
        .BYPASS_TICKS(1),
        .RECOVER_TICKS(1),
        .BACKUP_COMMAND(100),
        .TURN_COMMAND(300),
        .BYPASS_COMMAND(150)
    ) dut (
        .clk_i(clk_i),
        .rst_n_i(rst_n_i),
        .start_i(start_i),
        .abort_i(abort_i),
        .path_pause_i(path_pause_i),
        .motion_stopped_i(motion_stopped_i),
        .avoidance_tick_i(avoidance_tick_i),
        .front_range_i(front_range_i),
        .left_range_i(left_range_i),
        .right_range_i(right_range_i),
        .front_range_valid_i(front_range_valid_i),
        .left_range_valid_i(left_range_valid_i),
        .right_range_valid_i(right_range_valid_i),
        .segment_ready_o(segment_ready_o),
        .segment_valid_i(segment_valid_i),
        .segment_opcode_i(segment_opcode_i),
        .segment_target_i(segment_target_i),
        .segment_command_i(segment_command_i),
        .segment_last_i(segment_last_i),
        .segment_progress_i(segment_progress_i),
        .path_active_o(path_active_o),
        .path_finished_o(path_finished_o),
        .path_progress_enable_o(path_progress_enable_o),
        .segment_index_o(segment_index_o),
        .path_paused_o(path_paused_o),
        .avoidance_active_o(avoidance_active_o),
        .avoidance_done_o(avoidance_done_o),
        .avoid_left_o(avoid_left_o),
        .avoidance_state_o(avoidance_state_o),
        .safety_level_o(safety_level_o),
        .command_valid_o(command_valid_o),
        .linear_o(linear_o),
        .angular_o(angular_o)
    );

    always #5 clk_i = ~clk_i;

    task check_command;
        input expected_valid;
        input signed [31:0] expected_linear;
        input signed [31:0] expected_angular;
        begin
            #1;
            check_count = check_count + 1;
            if (command_valid_o !== expected_valid ||
                linear_o !== expected_linear ||
                angular_o !== expected_angular) begin
                $display(
                    "FAIL check=%0d valid=%b linear=%0d angular=%0d state=%0d index=%0d",
                    check_count, command_valid_o, linear_o, angular_o,
                    avoidance_state_o, segment_index_o
                );
                $fatal(1);
            end
        end
    endtask

    task load_segment;
        input [1:0] opcode;
        input [31:0] target;
        input signed [31:0] command_value;
        input is_last;
        begin
            if (!segment_ready_o) begin
                $display("FAIL: segment source was not requested");
                $fatal(1);
            end
            segment_opcode_i = opcode;
            segment_target_i = target;
            segment_command_i = command_value;
            segment_last_i = is_last;
            segment_progress_i = 32'd0;
            segment_valid_i = 1'b1;
            @(posedge clk_i);
            #1 segment_valid_i = 1'b0;
        end
    endtask

    task wait_avoidance_state;
        input [2:0] expected_state;
        integer cycles;
        begin
            cycles = 0;
            while (avoidance_state_o != expected_state && cycles < 12) begin
                @(posedge clk_i);
                #1 cycles = cycles + 1;
            end
            if (avoidance_state_o != expected_state) begin
                $display("FAIL: avoidance state %0d not reached", expected_state);
                $fatal(1);
            end
        end
    endtask

    initial begin
        repeat (2) @(posedge clk_i);
        rst_n_i = 1'b1;

        // 启动三段路径并装载正向 MOVE。
        @(negedge clk_i);
        start_i = 1'b1;
        @(posedge clk_i);
        #1 start_i = 1'b0;
        load_segment(OP_MOVE, 32'd100, 32'sd200, 1'b0);
        check_command(1'b1, 32'sd200, 32'sd0);

        // 前向测距无效时保持路径和零输出，不凭无效侧距主动选择绕行方向。
        front_range_valid_i = 1'b0;
        check_command(1'b0, 32'sd0, 32'sd0);
        @(posedge clk_i);
        #1;
        if (!path_paused_o || avoidance_active_o)
            $fatal(1, "FAIL: invalid range did not hold path safely");
        front_range_valid_i = 1'b1;
        check_command(1'b1, 32'sd200, 32'sd0);

        // 接近区把正向速度缩半，正常区恢复原命令。
        front_range_i = 16'd400;
        check_command(1'b1, 32'sd100, 32'sd0);
        if (safety_level_o !== 2'd1) $fatal(1, "FAIL: slow level");
        front_range_i = 16'd1000;
        check_command(1'b1, 32'sd200, 32'sd0);

        // 危险区立即停车并暂停路径，随后选择空间更大的左侧。
        front_range_i = 16'd100;
        left_range_i = 16'd800;
        right_range_i = 16'd300;
        check_command(1'b0, 32'sd0, 32'sd0);
        if (!path_paused_o || path_progress_enable_o) $fatal(1, "FAIL: path not paused");

        wait_avoidance_state(AVOID_STOP);
        check_command(1'b1, 32'sd0, 32'sd0);
        wait_avoidance_state(AVOID_SELECT);
        wait_avoidance_state(AVOID_BACKUP);
        if (!avoid_left_o) $fatal(1, "FAIL: left side not selected");
        // 前方危险不应阻断后退。
        check_command(1'b1, -32'sd100, 32'sd0);

        wait_avoidance_state(AVOID_TURN);
        // 前方危险也不应阻断原地左转。
        check_command(1'b1, 32'sd0, 32'sd300);
        wait_avoidance_state(AVOID_BYPASS);
        check_command(1'b1, 32'sd0, 32'sd0);

        // 障碍未清除时 BYPASS 必须停住且不得提前进入恢复阶段。
        repeat (2) @(posedge clk_i);
        #1;
        if (avoidance_state_o !== AVOID_BYPASS) $fatal(1, "FAIL: blocked bypass advanced");
        front_range_i = 16'd1000;
        wait_avoidance_state(AVOID_RECOVER);
        check_command(1'b1, 32'sd0, -32'sd300);
        wait_avoidance_state(AVOID_IDLE);
        #1;
        if (!avoidance_done_o) $fatal(1, "FAIL: missing avoidance done pulse");
        // 绕障结束后恢复尚未完成的 MOVE。
        check_command(1'b1, 32'sd200, 32'sd0);

        // 完成 MOVE 后装载 ROTATE；外部暂停期间即使进度已达目标也不能推进。
        segment_progress_i = 32'd100;
        @(posedge clk_i);
        #1;
        if (!segment_ready_o || segment_index_o !== 8'd1) $fatal(1, "FAIL: MOVE did not advance");
        load_segment(OP_ROTATE, 32'd90, -32'sd250, 1'b0);
        check_command(1'b1, 32'sd0, -32'sd250);
        path_pause_i = 1'b1;
        segment_progress_i = 32'd90;
        check_command(1'b0, 32'sd0, 32'sd0);
        repeat (2) @(posedge clk_i);
        #1;
        if (segment_index_o !== 8'd1 || segment_ready_o) $fatal(1, "FAIL: paused segment advanced");
        path_pause_i = 1'b0;
        @(posedge clk_i);
        #1;
        if (!segment_ready_o || segment_index_o !== 8'd2) $fatal(1, "FAIL: ROTATE did not resume");

        // STOP 段等待实际停车确认后结束整条路径。
        motion_stopped_i = 1'b0;
        load_segment(OP_STOP, 32'd0, 32'sd0, 1'b1);
        check_command(1'b1, 32'sd0, 32'sd0);
        repeat (2) @(posedge clk_i);
        #1;
        if (path_finished_o) $fatal(1, "FAIL: STOP ignored motion state");
        motion_stopped_i = 1'b1;
        @(posedge clk_i);
        #1;
        if (!path_finished_o) $fatal(1, "FAIL: path did not finish");

        // 新路径再覆盖右侧空间更大时的确定性选择。
        @(negedge clk_i);
        start_i = 1'b1;
        @(posedge clk_i);
        #1 start_i = 1'b0;
        load_segment(OP_MOVE, 32'd50, 32'sd120, 1'b1);
        left_range_i = 16'd250;
        right_range_i = 16'd900;
        front_range_i = 16'd100;
        wait_avoidance_state(AVOID_BACKUP);
        if (avoid_left_o) $fatal(1, "FAIL: right side not selected");
        check_command(1'b1, -32'sd100, 32'sd0);
        wait_avoidance_state(AVOID_TURN);
        check_command(1'b1, 32'sd0, -32'sd300);
        front_range_i = 16'd1000;
        wait_avoidance_state(AVOID_IDLE);

        // abort_i 立即关闭命令并清除未完成路径。
        abort_i = 1'b1;
        check_command(1'b0, 32'sd0, 32'sd0);
        @(posedge clk_i);
        #1;
        if (path_active_o || avoidance_active_o) $fatal(1, "FAIL: abort did not clear state");

        $display("PASS: autonomous motion, %0d command checks", check_count);
        $finish;
    end

endmodule
