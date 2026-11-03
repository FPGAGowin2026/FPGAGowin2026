`timescale 1ns/1ps
// 40MHz真实时钟下核对电机采样计数；驱动器保持禁用，不产生运动输出。
module nav_full_motor_clock_tb;
    localparam integer CLK_HZ = 40_000_000;
    localparam integer SAMPLE_HZ = 50;
    localparam integer SAMPLE_CYCLES = CLK_HZ / SAMPLE_HZ;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg enable = 1'b0;
    reg signed [31:0] vx = 32'sd200;
    reg signed [31:0] vy = -32'sd100;
    reg signed [31:0] wz = 32'sd300;
    wire [3:0] motor_in1;
    wire [3:0] motor_in2;
    wire sample_valid;
    wire [127:0] position_ticks;
    wire [127:0] measured_mm_s;
    wire [127:0] target_mm_s;
    wire [3:0] encoder_invalid;
    reg sample_valid_in = 1'b0;
    reg point_valid = 1'b0;
    reg [15:0] radar_angle = 16'd0;
    reg [15:0] radar_distance = 16'd0;
    reg packet_error = 1'b0;
    reg command_valid = 1'b0;
    reg [47:0] guard_velocity = 48'd0;
    reg kill = 1'b0;
    wire guarded_valid;
    wire [47:0] guarded_velocity;
    wire guard_stop;
    wire guard_slow;
    wire lidar_fresh;

    integer cycle_count = 0;
    integer sample_count = 0;
    integer previous_sample_cycle = 0;
    integer interval_a = 0;
    integer interval_b = 0;

    always #12.5 clk = ~clk;

    mecanum_motor_control #(
        .CLK_HZ(CLK_HZ),
        .SAMPLE_HZ(SAMPLE_HZ),
        .PWM_PERIOD(2500)
    ) motor (
        .clk_i(clk), .rst_n_i(rst_n), .enable_i(enable),
        .vx_mm_s_i(vx), .vy_mm_s_i(vy), .wz_mrad_s_i(wz),
        .encoder_a_i(4'b0000), .encoder_b_i(4'b0000),
        .in1_o(motor_in1), .in2_o(motor_in2),
        .position_ticks_o(position_ticks), .measured_mm_s_o(measured_mm_s),
        .target_mm_s_o(target_mm_s), .encoder_invalid_o(encoder_invalid),
        .sample_valid_o(sample_valid)
    );

    bx72_lite_guard #(.CLK_HZ(CLK_HZ)) guard (
        .clk(clk), .rst(!rst_n),
        .sample_valid_i(sample_valid_in), .point_valid_i(point_valid),
        .angle_cdeg_i(radar_angle), .distance_mm_i(radar_distance),
        .packet_error_i(packet_error), .command_valid_i(command_valid),
        .command_velocity_i(guard_velocity), .kill_i(kill),
        .command_valid_o(guarded_valid), .command_velocity_o(guarded_velocity),
        .stop_o(guard_stop), .slow_o(guard_slow), .lidar_fresh_o(lidar_fresh)
    );

    always @(posedge clk) begin
        if (!rst_n) begin
            cycle_count = 0;
            sample_count = 0;
            previous_sample_cycle = 0;
            interval_a = 0;
            interval_b = 0;
        end else begin
            if (motor.velocity_valid !== 4'b0000 && motor.velocity_valid !== 4'b1111)
                $fatal(1, "four wheel velocity valid signals are not aligned");
            cycle_count = cycle_count + 1;
            if (sample_valid) begin
                if (sample_count == 1) interval_a = cycle_count - previous_sample_cycle;
                if (sample_count == 2) interval_b = cycle_count - previous_sample_cycle;
                previous_sample_cycle = cycle_count;
                sample_count = sample_count + 1;
            end
        end
    end

    task automatic radar_point(input [15:0] a, input [15:0] d);
        begin
            @(negedge clk);
            sample_valid_in = 1'b1;
            point_valid = 1'b1;
            radar_angle = a;
            radar_distance = d;
            @(negedge clk);
            sample_valid_in = 1'b0;
            point_valid = 1'b0;
        end
    endtask

    task automatic far_scan;
        integer sector;
        begin
            for (sector = 0; sector < 8; sector = sector + 1)
                radar_point(sector * 16'd4500, 16'd2000);
            radar_point(16'd0, 16'd2000);
        end
    endtask

    initial begin
        repeat (4) @(negedge clk);
        rst_n = 1'b1;

        // 只给闭环模块非零目标用于验证采样计时；enable保持0，物理输出须为00。
        wait (sample_count == 3);
        if (interval_a != SAMPLE_CYCLES || interval_b != SAMPLE_CYCLES)
            $fatal(1, "50Hz samples must be %0d cycles, observed %0d and %0d",
                   SAMPLE_CYCLES, interval_a, interval_b);
        if (motor_in1 !== 4'b0000 || motor_in2 !== 4'b0000)
            $fatal(1, "disabled motor output is not electrically idle");

        far_scan();
        command_valid = 1'b1;
        guard_velocity = {16'sd200, 16'sd0, 16'sd0};
        repeat (5) @(negedge clk);
        command_valid = 1'b0;
        if (!lidar_fresh || guard_stop || guarded_velocity[47:32] !== 16'sd200)
            $fatal(1, "far valid scan did not arm the guard command");
        kill = 1'b1;
        #1;
        if (!guard_stop || guarded_velocity !== 48'd0)
            $fatal(1, "kill did not immediately force zero velocity");

        $display("PASS: nav_full_motor_clock 40MHz, 50Hz=%0d cycles, disabled outputs idle, guard kill zeroes command",
                 SAMPLE_CYCLES);
        $finish;
    end

    initial begin
        #80_000_000;
        $fatal(1, "motor clock test timeout");
    end
endmodule
