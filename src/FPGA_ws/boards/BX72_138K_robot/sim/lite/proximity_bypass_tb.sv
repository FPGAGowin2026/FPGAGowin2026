`timescale 1ns/1ps
module proximity_bypass_tb;
    reg clk=0, rst=1, sample=0, hit=0, command=0, kill=0, error=0;
    reg [15:0] angle=0, distance=0;
    reg [47:0] velocity={16'd100,16'd0,16'd0};
    wire [47:0] normal_v, bypass_v;
    wire normal_stop, bypass_stop, normal_slow, bypass_slow;
    always #5 clk=~clk;
    bx72_lite_guard #(.CLK_HZ(1000), .SAMPLE_TIMEOUT_MS(10), .COMMAND_TIMEOUT_MS(10), .SECTOR_TIMEOUT_MS(20), .BYPASS_PROXIMITY(0)) a
        (.clk(clk),.rst(rst),.sample_valid_i(sample),.point_valid_i(hit),.angle_cdeg_i(angle),.distance_mm_i(distance),.packet_error_i(error),.command_valid_i(command),.command_velocity_i(velocity),.kill_i(kill),.command_valid_o(),.command_velocity_o(normal_v),.stop_o(normal_stop),.slow_o(normal_slow),.lidar_fresh_o());
    bx72_lite_guard #(.CLK_HZ(1000), .SAMPLE_TIMEOUT_MS(10), .COMMAND_TIMEOUT_MS(10), .SECTOR_TIMEOUT_MS(20), .BYPASS_PROXIMITY(1)) b
        (.clk(clk),.rst(rst),.sample_valid_i(sample),.point_valid_i(hit),.angle_cdeg_i(angle),.distance_mm_i(distance),.packet_error_i(error),.command_valid_i(command),.command_velocity_i(velocity),.kill_i(kill),.command_valid_o(),.command_velocity_o(bypass_v),.stop_o(bypass_stop),.slow_o(bypass_slow),.lidar_fresh_o());
    integer i;
    initial begin
        repeat(2) @(negedge clk); rst=0;
        // 即使旁路距离阈值，缺少必需扇区仍必须停车。
        sample=1; hit=1; angle=1000; distance=50; command=1;
        repeat(7) @(negedge clk);
        if (!bypass_stop) $fatal(1,"missing sector accepted");
        for(i=1;i<8;i=i+1) begin angle=i*4500+1000; distance=1000; @(negedge clk); end
        sample=0; hit=0; repeat(6) @(negedge clk);
        if (!normal_stop || bypass_stop || bypass_v != velocity || bypass_slow) $fatal(1,"distance bypass failed");
        kill=1; #1; if (!bypass_stop || bypass_v != 0) $fatal(1,"kill bypassed"); kill=0;
        // 单次包错误不清除仍新鲜的旧值；带错误的样本不得刷新雷达时效。
        error=1;sample=1;hit=1;angle=1000;distance=1000;
        #1;if (bypass_stop || bypass_v != velocity) $fatal(1,"包错误提前清除了新鲜旧值");
        repeat(5) @(negedge clk);#1;
        if (!bypass_stop || bypass_v != 0) $fatal(1,"带错误样本错误地刷新了雷达时效");

        // 重新建好前方两个扇区，持续刷新雷达但不刷新命令，验证命令超时。
        rst=1;sample=0;hit=0;error=0;command=0;
        repeat(2) @(negedge clk);rst=0;
        sample=1;hit=1;command=1;angle=1000;distance=1000;
        @(negedge clk);angle=33000;
        @(negedge clk);angle=1000;
        repeat(5) @(negedge clk);#1;
        if (bypass_stop || bypass_v != velocity) $fatal(1,"命令看门狗测试未放行");
        command=0;
        for(i=0;i<12;i=i+1)begin angle=(i&1)?33000:1000;@(negedge clk);end
        #1;if (!bypass_stop || bypass_v != 0) $fatal(1,"命令超时未停车");
        $display("PASS proximity_bypass_tb"); $finish;
    end
endmodule
