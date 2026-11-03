`timescale 1ns/1ps
// 轻量雷达保护回归源码；实施线程只写，不运行仿真。
module tb_bx72_lite_guard;
    reg clk = 0;
    always #10 clk = ~clk;
    reg rst = 1;
    reg sample_valid = 0;
    reg point_valid = 0;
    reg [15:0] angle_cdeg = 0;
    reg [15:0] distance_mm = 0;
    reg packet_error = 0;
    reg command_valid = 0;
    reg [47:0] command_velocity = 0;
    reg kill = 0;
    wire command_valid_out;
    wire [47:0] command_velocity_out;
    wire stop;
    wire slow;
    wire lidar_fresh;

    bx72_lite_guard dut(
        .clk(clk), .rst(rst),
        .sample_valid_i(sample_valid), .point_valid_i(point_valid),
        .angle_cdeg_i(angle_cdeg), .distance_mm_i(distance_mm),
        .packet_error_i(packet_error), .command_valid_i(command_valid),
        .command_velocity_i(command_velocity), .kill_i(kill),
        .command_valid_o(command_valid_out), .command_velocity_o(command_velocity_out),
        .stop_o(stop), .slow_o(slow), .lidar_fresh_o(lidar_fresh));

    task automatic send_sample(input [15:0] angle, input hit, input [15:0] distance);
        begin
            @(negedge clk);
            sample_valid = 1;
            point_valid = hit;
            angle_cdeg = angle;
            distance_mm = distance;
            @(negedge clk);
            sample_valid = 0;
            point_valid = 0;
        end
    endtask

    task automatic clear_scan;
        integer sector;
        begin
            for (sector = 0; sector < 8; sector = sector + 1)
                send_sample(sector * 4500, 0, 16'd8000);
            send_sample(16'd0, 0, 16'd8000);
        end
    endtask

    task automatic issue_forward;
        begin
            @(negedge clk);
            command_valid = 1;
            command_velocity = {16'sd100, 16'sd0, 16'sd0};
            #1;
        end
    endtask

    initial begin
        repeat (3) @(negedge clk);
        rst = 0;
        issue_forward();
        if (!stop || command_velocity_out !== 48'd0 || lidar_fresh)
            $fatal(1, "没有LiDAR采样时必须立即停止");

        command_valid = 0;
        command_velocity = 0;
        clear_scan();
        if (!lidar_fresh) $fatal(1, "无回波采样仍应刷新LiDAR新鲜度");

        send_sample(16'd0, 1, 16'd200);
        send_sample(16'd1000, 1, 16'd8000);
        issue_forward();
        if (!stop || command_velocity_out !== 48'd0)
            $fatal(1, "远点不得覆盖当前扫描中更近的障碍");

        command_velocity = {16'sd100, 16'sd100, 16'sd0};
        #1;
        if (!stop || command_velocity_out !== 48'd0)
            $fatal(1, "斜向行驶必须检查相邻扇区");
        command_velocity = {16'sd0, 16'sd0, 16'sd100};
        #1;
        if (!stop || command_velocity_out !== 48'd0)
            $fatal(1, "旋转必须检查周边所有扇区");

        command_valid = 0;
        command_velocity = 0;
        clear_scan();
        clear_scan();
        issue_forward();
        if (stop || command_velocity_out[47:32] !== 16'sd100)
            $fatal(1, "完整一周无回波覆盖后应允许前向命令");

        packet_error = 1;
        #1;
        if (stop || !lidar_fresh)
            $fatal(1, "短时雷达包错误不应立即停车");
        $display("PASS tb_bx72_lite_guard: min distance, diagonal/rotation, no-echo coverage");
        $finish;
    end
endmodule
