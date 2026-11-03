`timescale 1ns/1ps
// 真实核心/guard/电机，CPU和健康输入用可控夹具，不仿真真实固件执行。
module vision_safety_boundaries_tb;
    reg clk=0,reset_n=0,command_valid=0,heartbeat=0;
    reg mode_valid=0,mode_enable=0,task_valid=0,link_down=0;
    reg [47:0] velocity=0;
    reg [63:0] token=64'h1234;
    reg radar_sample=0,manual_fresh_model=1;
    reg [41:0] radar_data=0;
    reg [31:0] now_model=100;
    wire [3:0] motor1,motor2;
    integer i;
    always #12.5 clk=~clk;
    bx72_lite_robot_core #(.CONTROL_CLK_HZ(40000000)) dut(
        .control_clk(clk),.loc_clk33m(clk),.reset_n(reset_n),
        .n10_uart_rx(1'b1),.imu_miso(1'b0),.imu_cs_n(),.imu_sclk(),.imu_mosi(),
        .encoder_a_i(4'd0),.encoder_b_i(4'd0),.motor_in1_o(motor1),.motor_in2_o(motor2),
        .command_valid_i(command_valid),.command_link_down_i(link_down),.command_heartbeat_i(heartbeat),
        .command_velocity_i(velocity),.auto_mode_valid_i(mode_valid),.auto_enable_i(mode_enable),
        .task_fragment_valid_i(task_valid),.task_fragment_i(256'd0),.command_token_i(token),
        .navigation_valid_o(),.navigation_payload_o(),.local_plan_valid_o(),.local_plan_payload_o(),
        .auto_mode_o(),.fusion_valid_o(),.imu_valid_o(),.motor_valid_o(),.lidar_valid_o(),
        .imu_payload_o(),.motor_payload_o(),.lidar_payload_o(),.fusion_payload_o(),
        .sensor_health_o(),.cpu_ready_o(),.cpu_fault_o(),.cpu_busy_o(),.control_reset_o(),.cpu_fault_code_o());
    defparam dut.motor_control_u.CLK_HZ=40000;
    defparam dut.motor_control_u.STARTUP_CYCLES=3;
    defparam dut.motor_control_u.PWM_PERIOD=32;

    task automatic store(input [11:0] address,input [31:0] value);
        begin @(negedge clk);dut.cpu.valid=1;dut.cpu.writing=1;dut.cpu.address=address;dut.cpu.data=value;
              @(negedge clk);dut.cpu.valid=0;dut.cpu.writing=0;end
    endtask
    task automatic send(input signed [15:0] w);
        begin @(negedge clk);command_valid=1;heartbeat=1;velocity={32'd0,w};
              @(negedge clk);command_valid=0;heartbeat=0;repeat(3)@(negedge clk);end
    endtask
    task automatic point(input [15:0] a);
        begin @(negedge clk);radar_sample=1;radar_data={1'b0,1'b1,8'd0,a,16'd2000};
              @(negedge clk);radar_sample=0;end
    endtask
    task automatic scan_far;
        integer k;
        begin for(k=0;k<8;k=k+1)point(k*4500+1000);point(0);end
    endtask
    task automatic snapshot;
        begin store(12'h108,0);end
    endtask
    task automatic stage(input signed [31:0] w,input bit autonomous);
        begin
            store(12'h200,0);store(12'h204,0);store(12'h208,w);
            store(12'h2a0,autonomous);store(12'h2a4,dut.snap_command_seq);
            store(12'h2a8,dut.snap_mode_seq);
        end
    endtask
    task automatic commit;
        begin store(12'h20c,32'h42583732);repeat(16)@(negedge clk);end
    endtask
    task automatic confirm(input signed [31:0] w,input bit autonomous);
        begin snapshot();stage(w,autonomous);commit();end
    endtask
    task automatic expect_stopped(input [511:0] label);
        begin repeat(8)@(negedge clk);#1;
            if(!dut.hard_kill || !dut.guard_stop || motor1!==0 || motor2!==0)
                $fatal(1,"%0s kill=%b guard=%b current=%b mode=%d/%d pwm=%h/%h",
                   label,dut.hard_kill,dut.guard_stop,dut.command_output_current,
                   dut.mode_sequence,dut.output_mode_seq,motor1,motor2);
        end
    endtask
    task automatic expect_run(input signed [15:0] w,input [511:0] label);
        begin repeat(20)@(negedge clk);#1;
            if(dut.hard_kill || dut.guard_stop || $signed(dut.guard_velocity[15:0])!==w)
                $fatal(1,"%0s kill=%b guard=%b current=%b velocity=%0d expected=%0d",
                   label,dut.hard_kill,dut.guard_stop,dut.command_output_current,
                   $signed(dut.guard_velocity[15:0]),w);
        end
    endtask
    task automatic restart_motion(input signed [15:0] w);
        begin
            send(0);confirm(0,0);expect_run(0,"fresh zero confirmed");
            send(w);expect_stopped("new nonzero requires fresh CPU confirmation");
            confirm(w,0);expect_run(w,"fresh nonzero confirmed");
        end
    endtask

    initial begin
        force dut.now_us=now_model;
        force dut.manual_fresh=manual_fresh_model;
        force dut.link_down=1'b0;
        force dut.pose_fresh=1'b1;force dut.imu_fresh=1'b1;force dut.wheel_ok=1'b1;
        force dut.radar_sample_ctrl=radar_sample;force dut.radar_data_ctrl=radar_data;
        force dut.radar_error_ctrl=1'b0;
        repeat(6)@(negedge clk);reset_n=1;repeat(10)@(negedge clk);scan_far();
        restart_motion(-16'sd236);
        repeat(2500)@(negedge clk);
        if(!dut.motor_control_u.controller_enabled)$fatal(1,"motor fixture did not enter active control");

        // 快速变化与非零反向属于同一已确认代次；先保持旧输出再提交新输出。
        for(i=0;i<100;i=i+1)begin
            send(i[0] ? -16'sd236 : -16'sd238);
            if(dut.hard_kill || !dut.command_output_current || !dut.motor_control_u.controller_enabled)
                $fatal(1,"fast in-epoch variation caused a stop at i=%0d",i);
        end
        send(16'sd236);expect_run(-16'sd236,"reversal retains old confirmed output before commit");
        confirm(16'sd236,0);expect_run(16'sd236,"reversal applies after CPU confirmation");
        $display("PASS continuity: fast variation and reverse sample/hold");

        // 零速立即建立屏障；之前暂存的非零提交不得重新放行。
        snapshot();stage(16'sd236,0);
        send(0);expect_stopped("zero must stop");
        commit();expect_stopped("late pre-zero nonzero commit cannot revive");
        send(-16'sd236);expect_stopped("new activation remains stopped");
        snapshot();stage(-16'sd236,0);
        send(-16'sd238);commit();
        expect_stopped("first activation cannot accept an old in-epoch snapshot");
        confirm(-16'sd238,0);expect_run(-16'sd238,"latest activation confirmed");
        $display("PASS zero fence and first-activation matching");

        // 模式变化让旧mode_sequence失效；新人工控制走零速和新提交。
        snapshot();stage(-16'sd238,0);
        @(negedge clk);mode_valid=1;mode_enable=0;
        @(negedge clk);mode_valid=0;
        expect_stopped("mode transition");
        commit();expect_stopped("old mode commit cannot revive");
        restart_motion(-16'sd236);
        $display("PASS mode boundary");

        // 实际任务FIFO到达同样阻止人工代次复用。
        snapshot();stage(-16'sd236,0);
        @(negedge clk);task_valid=1;
        @(negedge clk);task_valid=0;
        expect_stopped("task activity");
        commit();expect_stopped("old commit cannot cross task activity");
        wait(dut.task_available);store(12'h184,dut.task_fifo_epoch);wait(!dut.task_available);
        restart_motion(-16'sd236);
        $display("PASS task boundary");

        // 物理断链的最终引脚门控不等待确认；恢复本身不能复活旧意图。
        snapshot();stage(-16'sd236,0);
        @(negedge clk);link_down=1;#1;
        if(motor1!==0 || motor2!==0 || !dut.hard_kill)$fatal(1,"link loss did not immediately cut outputs");
        repeat(5)@(negedge clk);link_down=0;
        commit();expect_stopped("link restoration cannot revive old output");
        restart_motion(-16'sd236);
        $display("PASS link boundary");

        // 租约恢复和CPU旧写回不能代替新的人工命令。
        snapshot();stage(-16'sd236,0);
        @(negedge clk);manual_fresh_model=0;
        expect_stopped("lease expiry");
        @(negedge clk);manual_fresh_model=1;
        commit();expect_stopped("lease recovery cannot revive old output");
        restart_motion(-16'sd236);
        $display("PASS lease boundary");

        // session token变化直接闭锁并递增mode序号，屏蔽旧会话提交。
        snapshot();stage(-16'sd236,0);
        @(negedge clk);token=64'h5678;heartbeat=1;#1;
        if(!dut.hard_kill || motor1!==0 || motor2!==0)$fatal(1,"session boundary did not immediately stop");
        @(negedge clk);heartbeat=0;
        commit();expect_stopped("late pre-session-change commit");
        send(-16'sd238);expect_stopped("new session motion before confirmation");
        confirm(-16'sd238,0);expect_run(-16'sd238,"new session motion confirmed");
        $display("PASS manual session boundary");

        // 新会话第一个心跳同时带非零命令；边界同拍不能把它当成旧代次确认。
        snapshot();stage(-16'sd238,0);token=64'h7777;
        send(16'sd236);expect_stopped("session and motion in the same packet");
        confirm(16'sd236,0);expect_stopped("boundary packet alone cannot rearm");
        send(16'sd236);expect_stopped("fresh post-session intent awaits CPU");
        confirm(16'sd236,0);expect_run(16'sd236,"post-session intent confirmed");
        send(-16'sd238);confirm(-16'sd238,0);expect_run(-16'sd238,"source test fixture");
        $display("PASS simultaneous session and command boundary");

        // PC/ROS身份不传入RTL：这里只模拟bridge既有的零速+模式屏障。
        snapshot();stage(-16'sd238,0);
        send(0);expect_stopped("source handoff zero barrier");
        commit();expect_stopped("old source commit after zero barrier");
        @(negedge clk);mode_valid=1;mode_enable=0;@(negedge clk);mode_valid=0;
        send(16'sd236);expect_stopped("new source before confirmation");
        confirm(16'sd236,0);expect_run(16'sd236,"new source after confirmation");
        $display("PASS modeled bridge source barrier");

        // 自主模式下会话变更也禁止旧暂存自主提交复活。
        @(negedge clk);mode_valid=1;mode_enable=1;@(negedge clk);mode_valid=0;
        snapshot();stage(16'sd236,1);commit();expect_run(16'sd236,"autonomy fixture active");
        if(!dut.auto_enabled)$fatal(1,"autonomy fixture not active");
        snapshot();stage(16'sd236,1);
        @(negedge clk);token=64'h9abc;heartbeat=1;#1;
        if(!dut.hard_kill || motor1!==0 || motor2!==0)$fatal(1,"autonomy session boundary failed");
        @(negedge clk);heartbeat=0;
        commit();expect_stopped("late autonomous old session commit");
        if(dut.auto_enabled)$fatal(1,"stale autonomous commit revived old mode");
        restart_motion(-16'sd236);
        $display("PASS autonomous session boundary");

        // 仅推进模型时钟，验证原有100ms输出过期实际切断电机。
        now_model=100101;
        repeat(12)@(negedge clk);
        if(dut.output_fresh!==0)$fatal(1,"output freshness did not expire");
        expect_stopped("expired CPU output");
        $display("PASS original output freshness deadline");

        // 不强制manual_fresh，构造真实核心32位命令序号回绕。
        now_model=100;reset_n=0;release dut.manual_fresh;
        repeat(8)@(negedge clk);reset_n=1;repeat(10)@(negedge clk);scan_far();
        send(0);confirm(0,0);expect_run(0,"wrap fixture zero");
        @(negedge clk);
        dut.manual_command_epoch.sequence_o=32'hfffffffe;
        dut.manual_command_epoch.effective_sequence=32'hfffffffe;
        dut.output_command_seq=32'hfffffffe;
        send(-16'sd236);expect_stopped("wrap fixture first nonzero");
        confirm(-16'sd236,0);expect_run(-16'sd236,"wrap fixture motion");
        repeat(2500)@(negedge clk);
        if(!dut.motor_control_u.controller_enabled)$fatal(1,"wrap fixture motor disabled");
        send(-16'sd238);
        if(dut.command_sequence!==0 || dut.manual_fresh!==1'b1)
            $fatal(1,"sequence wrap invalidated live lease seq=%h fresh=%b",dut.command_sequence,dut.manual_fresh);
        expect_run(-16'sd236,"wrap preserves prior output");
        force dut.output_fresh=1'b1;
        now_model=200101;repeat(12)@(negedge clk);
        if(dut.manual_fresh!==1'b0)$fatal(1,"200ms manual lease no longer expires");
        expect_stopped("manual lease still expires after wrap");
        $display("PASS real core wrap and 200ms manual deadline");
        $display("PASS: actual core/guard/motor boundary regression with modeled CPU");
        $finish;
    end
    initial begin #10000000;$fatal(1,"boundary simulation timeout");end
endmodule
