`timescale 1ns/1ps
// 仿真时钟为1MHz；保留真实500ms雷达超时并缩短命令超时。
module nav_full_guard_tb;
    reg clk=0,rst=1,sample_valid=0,hit=0,packet_error=0,command_valid=0,kill=0;
    reg [15:0] angle=0,distance=0;
    reg [47:0] velocity=0;
    wire valid,stop,slow,fresh;
    wire [47:0] output_velocity;
    always #500 clk=~clk;
    bx72_lite_guard #(.CLK_HZ(1_000_000),.COMMAND_TIMEOUT_MS(2)) dut(
        .clk(clk),.rst(rst),.sample_valid_i(sample_valid),.point_valid_i(hit),
        .angle_cdeg_i(angle),.distance_mm_i(distance),.packet_error_i(packet_error),
        .command_valid_i(command_valid),.command_velocity_i(velocity),.kill_i(kill),
        .command_valid_o(valid),.command_velocity_o(output_velocity),
        .stop_o(stop),.slow_o(slow),.lidar_fresh_o(fresh));
    task automatic point(input [15:0] a,input h,input [15:0] d);
        begin
            @(negedge clk);sample_valid=1;hit=h;angle=a;distance=d;
            @(negedge clk);sample_valid=0;hit=0;
            #1;
        end
    endtask
    task automatic scan(input h,input [15:0] d);
        integer s;
        begin
            // 真实覆盖需包含扇区内采样；每度一个样本。
            for(s=0;s<36000;s=s+100)point(s,h,d);
            point(0,h,d);
        end
    endtask
    task automatic command(input signed [15:0] x,y,w);
        begin
            @(negedge clk);velocity={x,y,w};command_valid=1;
            @(negedge clk);command_valid=0;
            // 正常命令固定四拍；多留两拍检查最终稳定输出。
            repeat(5)@(negedge clk);
            #1;
        end
    endtask
    initial begin
        repeat(4)@(negedge clk);rst=0;
        command(100,0,0);
        if(!stop||output_velocity!==0)$fatal(1,"缺雷达必须停车");
        scan(1,2000);scan(1,2000);command(100,0,0);
        if(stop||slow||output_velocity[47:32]!==16'd100)$fatal(1,"有效远点覆盖未放行");
        point(0,1,150);command(100,0,0);
        if(!stop||output_velocity!==0)$fatal(1,"前向近障未停车");
        packet_error=1;point(0,1,2000);packet_error=0;command(100,0,0);
        if(!stop)$fatal(1,"坏采样不得清除已有近障");
        scan(0,0);scan(0,0);command(100,0,0);
        if(!stop)$fatal(1,"无回波错误地清除了已有近障");
        scan(1,2000);scan(1,2000);command(100,0,0);
        if(stop)$fatal(1,"完整有效远点扫描未释放旧近障");
        point(0,1,500);command(100,0,0);
        if(stop||!slow||output_velocity[47:32]!==16'd50)$fatal(1,"距离减速未输出半速");
        scan(1,2000);scan(1,2000);
        point(18000,1,200);command(-100,0,0);
        if(stop)$fatal(1,"后方遮挡不应阻止后退");
        command(0,0,100);
        if(stop)$fatal(1,"后方遮挡不应阻止旋转");
        scan(1,2000);scan(1,2000);
        point(27000,1,100);command(0,100,0);
        if(stop)$fatal(1,"纯横移不应因侧向距离停车");
        scan(1,2000);scan(1,2000);command(100,0,0);
        if(stop)$fatal(1,"超时测试前未放行");
        repeat(2750)@(negedge clk);
        #1;
        if(!stop||output_velocity!==0||!fresh)$fatal(1,"CPU命令超时未独立停车");
        command(100,0,0);
        if(stop)$fatal(1,"有效新命令未恢复");
        kill=1;#1;if(!stop||output_velocity!==0)$fatal(1,"kill未立即停车");
        kill=0;packet_error=1;sample_valid=1;hit=1;angle=0;distance=100;#1;
        if(stop||!fresh)$fatal(1,"短时坏包不应立即停车");
        @(negedge clk);packet_error=0;sample_valid=0;hit=0;
        command(100,0,0);
        if(stop||!fresh)$fatal(1,"短时坏包后应保留仍新鲜的有效值");
        command_valid=1;packet_error=1;sample_valid=1;angle=0;distance=2000;
        wait(dut.sensor_age_ms == 255); #1;
        if(stop||!fresh||!dut.sector_fresh_bits[0]||!dut.sector_fresh_bits[7])
            $fatal(1,"255ms后雷达与前向扇区仍应有效");
        wait(dut.sensor_age_ms == 499); #1;
        if(stop||!fresh||!dut.sector_fresh_bits[0]||!dut.sector_fresh_bits[7])
            $fatal(1,"500ms前雷达与前向扇区仍应有效");
        wait(dut.sensor_age_ms == 500); #1;
        if(!stop||fresh||output_velocity!==0)$fatal(1,"持续坏包必须按采样超时停车");
        packet_error=0;sample_valid=0;command_valid=0;
        point(0,1,2000);point(35000,1,2000);command(100,0,0);
        if(stop||!fresh)$fatal(1,"前向两个扇区有效采样后未恢复");
        command_valid=1;
        while (dut.sector_age_ms[7] < 500) begin
            point(0,1,2000);
        end
        repeat(6)@(negedge clk);#1;
        if(!fresh||dut.sector_fresh_bits[7]||!stop)
            $fatal(1,"其他扇区持续有效时前向扇区满500ms必须失效停车");
        point(35000,1,2000);command(100,0,0);
        if(stop||!fresh)$fatal(1,"过期扇区收到有效采样后未恢复");
        $display("PASS: nav_full_guard clock=1MHz front=150 side_and_rear=unprotected slowdown=half noecho=retain");
        $finish;
    end
    initial begin #1500000000;$fatal(1,"guard测试超时");end
endmodule
