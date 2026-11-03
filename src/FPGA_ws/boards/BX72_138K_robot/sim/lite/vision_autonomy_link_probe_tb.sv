`timescale 1ns/1ps
// 真实核心/guard/电机，CPU与健康输入为夹具；新自主提交测试不代表实车独立任务恢复。
module vision_autonomy_link_probe_tb;
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
        force dut.pose_fresh=1'b1;force dut.imu_fresh=1'b1;force dut.wheel_ok=1'b1;
        force dut.radar_sample_ctrl=radar_sample;force dut.radar_data_ctrl=radar_data;
        force dut.radar_error_ctrl=1'b0;
        repeat(6)@(negedge clk);reset_n=1;repeat(10)@(negedge clk);scan_far();
        restart_motion(-16'sd236);
        @(negedge clk);mode_valid=1;mode_enable=1;@(negedge clk);mode_valid=0;
        snapshot();stage(-16'sd236,1);commit();expect_run(-16'sd236,"autonomy active");
        if(!dut.auto_enabled)$fatal(1,"autonomy fixture failed");
        snapshot();stage(-16'sd236,1);
        @(negedge clk);link_down=1;
        repeat(5)@(negedge clk);link_down=0;
        // 即使出现新快照，之前已暂存的commit仍不得借它复活。
        snapshot();commit();repeat(30)@(negedge clk);
        $display("LINK_PROBE kill=%b auto=%b guard=%b link_fresh=%b velocity=%0d mode=%0d/%0d",dut.hard_kill,dut.auto_enabled,dut.guard_stop,dut.link_fresh,$signed(dut.guard_velocity[15:0]),dut.mode_sequence,dut.output_mode_seq);
        if(dut.auto_enabled || !dut.hard_kill)
            $fatal(1,"autonomous stale pre-link commit reactivated after physical link loss");
        if(dut.mode_sequence!=1)$fatal(1,"physical link fence changed firmware-visible mode sequence");
        snapshot();stage(-16'sd236,1);commit();
        expect_run(-16'sd236,"fresh post-link autonomous snapshot and commit");
        if(!dut.auto_enabled || dut.mode_sequence!=1)$fatal(1,"valid independent output did not recover");

        // 同拍断链与SNAPSHOT必须按断链优先，不能当成新快照。
        @(negedge clk);link_down=1;dut.cpu.valid=1;dut.cpu.writing=1;dut.cpu.address=12'h108;dut.cpu.data=0;#1;
        if(!dut.hard_kill || motor1!==0 || motor2!==0)$fatal(1,"concurrent link drop did not stop pins");
        @(negedge clk);dut.cpu.valid=0;dut.cpu.writing=0;
        repeat(4)@(negedge clk);link_down=0;
        stage(-16'sd236,1);commit();expect_stopped("snapshot concurrent with link drop cannot release output");
        if(dut.auto_enabled || dut.mode_sequence!=1)$fatal(1,"concurrent snapshot escaped fence");
        snapshot();stage(-16'sd236,1);commit();expect_run(-16'sd236,"snapshot after concurrent drop");
        $display("PASS: old autonomous output fenced; new snapshot can recover without mode change");

        // 只屏蔽输出期限，以实际链路期限检查心跳200ms边界及停车。
        restart_motion(-16'sd236);
        force dut.output_fresh=1'b1;
        now_model=200098;repeat(12)@(negedge clk);
        if(!dut.link_fresh)$fatal(1,"link lease expired before its early boundary");
        expect_run(-16'sd236,"manual output before link deadline");
        now_model=200099;repeat(12)@(negedge clk);
        if(dut.link_fresh)$fatal(1,"link lease survived its 200ms early boundary");
        expect_stopped("link deadline cuts manual motion");
        $display("PASS: real core 200ms link deadline and early stop");
        $finish;
    end
    initial begin #1000000;$fatal(1,"autonomous link probe timeout");end
endmodule
