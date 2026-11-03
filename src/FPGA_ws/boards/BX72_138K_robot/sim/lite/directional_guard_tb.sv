`timescale 1ns/1ps
module directional_guard_tb;
    reg clk = 0, rst = 1, sample = 0, hit = 0, command = 0, kill = 0;
    reg [15:0] angle = 0, distance = 0;
    reg [47:0] velocity = 0;
    wire [47:0] out_velocity;
    wire out_valid, stop, slow, fresh;
    always #10 clk = ~clk;
    bx72_lite_guard dut(.clk(clk), .rst(rst), .sample_valid_i(sample),
        .point_valid_i(hit), .angle_cdeg_i(angle), .distance_mm_i(distance),
        .packet_error_i(1'b0), .command_valid_i(command), .command_velocity_i(velocity),
        .kill_i(kill), .command_valid_o(out_valid), .command_velocity_o(out_velocity),
        .stop_o(stop), .slow_o(slow), .lidar_fresh_o(fresh));

    task automatic run_case(input integer obstacle_sector, input integer obstacle_mm,
                            input signed [15:0] vx, input signed [15:0] vy,
                            input expected_stop);
        integer i;
        begin
            @(negedge clk); rst = 1; sample = 0; command = 0;
            repeat (3) @(negedge clk);
            rst = 0;
            for (i = 0; i < 8; i = i + 1) begin
                @(negedge clk); sample = 1; hit = 1;
                angle = i * 4500 + 1000;
                distance = i == obstacle_sector ? obstacle_mm : 1000;
            end
            @(negedge clk); sample = 0; hit = 0; command = 1;
            velocity = {vx, vy, 16'sd0};
            repeat (7) @(negedge clk);
            if (stop !== expected_stop)
                $fatal(1, "sector=%0d distance=%0d vx=%0d vy=%0d stop=%b expected=%b",
                       obstacle_sector, obstacle_mm, vx, vy, stop, expected_stop);
        end
    endtask

    task automatic rotation_case(input [15:0] obstacle_angle, input [15:0] obstacle_mm,
                                 input signed [15:0] vx, input expected_stop,
                                 input signed [15:0] expected_vx, input signed [15:0] expected_wz);
        integer i;
        begin
            @(negedge clk); rst = 1; sample = 0; command = 0;
            repeat (3) @(negedge clk);
            rst = 0;
            for (i = 0; i < 8; i = i + 1) begin
                @(negedge clk); sample = 1; hit = 1;
                angle = i * 4500 + 1000; distance = 1000;
            end
            @(negedge clk); angle = obstacle_angle; distance = obstacle_mm;
            @(negedge clk); sample = 0; hit = 0; command = 1;
            velocity = {vx, 16'sd0, 16'sd300};
            repeat (7) @(negedge clk);
            if (stop !== expected_stop || $signed(out_velocity[15:0]) !== expected_wz ||
                $signed(out_velocity[47:32]) !== expected_vx)
                $fatal(1, "rotation angle=%0d distance=%0d stop=%b wz=%0d", obstacle_angle,
                       obstacle_mm, stop, $signed(out_velocity[15:0]));
        end
    endtask

    initial begin
        run_case(0, 150, 100, 0, 1);
        run_case(0, 151, 100, 0, 0);
        run_case(2, 100, 0, -100, 0);
        run_case(2, 101, 0, -100, 0);
        run_case(6, 100, 0, 100, 0);
        run_case(6, 101, 0, 100, 0);
        run_case(4, 50, 0, -100, 0);
        run_case(4, 50, -100, 0, 0);
        rotation_case(13200, 90, 0, 0, 0, 300);
        rotation_case(22800, 80, 0, 0, 0, 300);
        rotation_case(12800, 90, 0, 0, 0, 300);
        rotation_case(1000, 150, 0, 1, 0, 0);
        rotation_case(1000, 200, 0, 0, 0, 150);
        rotation_case(1000, 500, 0, 0, 0, 300);
        rotation_case(1000, 500, 400, 0, 200, 150);
        rotation_case(1000, 701, 400, 0, 400, 300);
        rotation_case(1000, 150, 400, 1, 0, 0);
        kill = 1; #1;
        if (!stop || out_velocity != 0) $fatal(1, "kill must immediately stop");
        $display("PASS directional_guard_tb");
        $finish;
    end
endmodule
