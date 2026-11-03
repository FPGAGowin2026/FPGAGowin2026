`timescale 1ns/1ps
/* 真实 core/guard/motor，CPU用MMIO事务模型；不执行固件或连接实体设备。 */
module bx72_nav_platform #(parameter integer CLK_HZ=40000000)(
    input wire clk,rst,
    output wire cpu_ready_o,cpu_fault_o,output wire [7:0] fault_code_o,
    output wire [31:0] heartbeat_o,
    output wire io_valid_o,io_write_o,output wire [11:0] io_addr_o,
    output wire [31:0] io_wdata_o,output wire [3:0] io_wstrb_o,
    input wire io_ready_i,input wire [31:0] io_rdata_i
);
    reg valid=0,writing=0;
    reg [11:0] address=0;
    reg [31:0] data=0;
    assign cpu_ready_o=!rst;
    assign cpu_fault_o=0;
    assign fault_code_o=0;
    assign heartbeat_o=0;
    assign io_valid_o=valid;
    assign io_write_o=writing;
    assign io_addr_o=address;
    assign io_wdata_o=data;
    assign io_wstrb_o=4'hf;
endmodule

module vision_guard_feedback_tb;
    reg clk=0,reset_n=0,command_valid=0,heartbeat=0,mode_valid=0,mode_enable=0;
    reg [47:0] command_velocity=0;
    reg radar_sample=0;
    reg [41:0] radar_data=0;
    reg [31:0] now_us=100;
    reg test_pose_fresh=1,test_imu_fresh=1,test_wheel_fresh=1;
    wire [3:0] motor1,motor2;
    wire [543:0] navigation_payload;
    wire [831:0] fusion_payload;
    integer cycle;
    always #12.5 clk=~clk;
    integer trace_cycle=0;
    always @(negedge clk) begin
        trace_cycle=trace_cycle+1;
        #1;
        if (reset_n && (dut.hard_kill || dut.guard_stop) && (motor1!==4'd0 || motor2!==4'd0))
            $fatal(1,"guard or hard kill left nonzero PWM");
        if ($test$plusargs("TRACE") && trace_cycle < 200 &&
            (dut.output_commit || dut.guard_command_valid || dut.motor_control_u.remote_checked_valid))
            $display("TRACE cycle=%0d kill=%b guard=%b valid=%b gv=%h sync=%b armed=%b pending=%b checked=%b mode=%b commit=%b",
                trace_cycle,dut.hard_kill,dut.guard_stop,dut.guard_command_valid,dut.guard_velocity,
                dut.motor_control_u.remote_link_sync,dut.motor_control_u.remote_armed,
                dut.motor_control_u.remote_pending_valid,dut.motor_control_u.remote_checked_valid,
                dut.motor_control_u.remote_mode,dut.output_commit);
    end

    bx72_lite_robot_core #(.CONTROL_CLK_HZ(40_000_000)) dut(
        .control_clk(clk),.loc_clk33m(clk),.reset_n(reset_n),
        .n10_uart_rx(1'b1),.imu_miso(1'b0),.imu_cs_n(),.imu_sclk(),.imu_mosi(),
        .encoder_a_i(4'b0),.encoder_b_i(4'b0),.motor_in1_o(motor1),.motor_in2_o(motor2),
        .command_valid_i(command_valid),.command_link_down_i(1'b0),.command_heartbeat_i(heartbeat),
        .command_velocity_i(command_velocity),.auto_mode_valid_i(mode_valid),.auto_enable_i(mode_enable),
        .task_fragment_valid_i(1'b0),.task_fragment_i(256'd0),.command_token_i(64'h1234),
        .navigation_valid_o(),.navigation_payload_o(navigation_payload),.local_plan_valid_o(),.local_plan_payload_o(),
        .auto_mode_o(),.fusion_valid_o(),.imu_valid_o(),.motor_valid_o(),.lidar_valid_o(),
        .imu_payload_o(),.motor_payload_o(),.lidar_payload_o(),.fusion_payload_o(fusion_payload),
        .sensor_health_o(),.cpu_ready_o(),.cpu_fault_o(),.cpu_busy_o(),.control_reset_o(),.cpu_fault_code_o());
    /* 加速仿真中的电机启动与测速；保护器的生产40MHz时钟参数不变。 */
    defparam dut.motor_control_u.CLK_HZ=40000;
    defparam dut.motor_control_u.STARTUP_CYCLES=3;
    defparam dut.motor_control_u.PWM_PERIOD=32;

    task automatic store(input [11:0] addr,input [31:0] value);
        begin
            @(negedge clk);dut.cpu.valid=1;dut.cpu.writing=1;dut.cpu.address=addr;dut.cpu.data=value;
            @(negedge clk);dut.cpu.valid=0;dut.cpu.writing=0;
        end
    endtask
    task automatic send(input signed [15:0] x,input signed [15:0] y,input signed [15:0] w);
        begin
            @(negedge clk);command_valid=1;heartbeat=1;command_velocity={x,y,w};
            @(negedge clk);command_valid=0;heartbeat=0;
            repeat(4) @(negedge clk);
        end
    endtask
    task automatic point(input [15:0] a,input [15:0] distance);
        begin
            @(negedge clk);radar_sample=1;radar_data={1'b0,1'b1,8'd0,a,distance};
            @(negedge clk);radar_sample=0;
        end
    endtask
    task automatic scan_far;
        integer i;
        begin
            for(i=0;i<36000;i=i+100) point(i,2000);
            point(0,2000);
        end
    endtask
    task automatic snapshot;
        begin store(12'h108,0); end
    endtask
    task automatic stage_output(input signed [31:0] vx,input signed [31:0] vy,input signed [31:0] wz);
        begin
            store(12'h200,vx);store(12'h204,vy);store(12'h208,wz);
            store(12'h2a0,0);
            store(12'h2a4,dut.snap_command_seq);
            store(12'h2a8,dut.snap_mode_seq);
        end
    endtask
    task automatic finish_output;
        begin store(12'h20c,32'h42583732);repeat(10) @(negedge clk); end
    endtask
    task automatic finish_with_residual_snapshot;
        begin
            store(12'h20c,32'h42583732);
            wait(!dut.hard_kill && dut.guard_stop);
            @(negedge clk);
            if (dut.hard_kill || !dut.guard_stop)
                $fatal(1,"test fixture missed guard pipeline residual");
            if (navigation_payload[256]!==1'b0 || fusion_payload[359]!==1'b1)
                $fatal(1,"guard pipeline residual must be filtered only in kind8");
            dut.cpu.valid=1;dut.cpu.writing=1;dut.cpu.address=12'h108;dut.cpu.data=0;
            @(negedge clk);dut.cpu.valid=0;dut.cpu.writing=0;
            $display("TRACE residual cpu_status_guard=%b hard_kill=%b guard=%b",
                dut.snap_status[3],dut.hard_kill,dut.guard_stop);
            if(dut.snap_status[3]!==1'b0)
                $fatal(1,"guard pipeline residual fed back to CPU after internal kill");
            repeat(10) @(negedge clk);
        end
    endtask
    task automatic expect_killed(input [255:0] label);
        begin
            #1;
            if (!dut.hard_kill || !dut.guard_stop || dut.guard_velocity!==48'd0 ||
                motor1!==4'd0 || motor2!==4'd0)
                $fatal(1,"%0s kill=%b guard=%b velocity=%h pwm=%h/%h",label,
                    dut.hard_kill,dut.guard_stop,dut.guard_velocity,motor1,motor2);
        end
    endtask
    task automatic expect_released(input signed [15:0] expected_x);
        begin
            repeat(12) @(negedge clk);#1;
            if(dut.hard_kill || dut.guard_stop || $signed(dut.guard_velocity[47:32])!==expected_x)
                $fatal(1,"safe command not released kill=%b guard=%b vx=%0d expected=%0d seq=%0d out=%0d current=%b confirmed=%b after=%b mode=%0d/%0d seen=%b fresh=%b pose=%b imu=%b wheel=%b link=%b manual=%b",
                    dut.hard_kill,dut.guard_stop,$signed(dut.guard_velocity[47:32]),expected_x,
                    dut.command_sequence,dut.output_command_seq,dut.command_output_current,
                    dut.manual_output_confirmed,dut.manual_command_after_invalidation,
                    dut.mode_sequence,dut.output_mode_seq,dut.output_seen,dut.output_fresh,
                    dut.pose_fresh,dut.imu_fresh,dut.wheel_ok,dut.link_down,dut.manual_fresh);
        end
    endtask
    task automatic check_commit_bounds(input signed [31:0] x,y,w,input bit accepted);
        begin
            snapshot();stage_output(x,y,w);finish_output();repeat(12)@(negedge clk);#1;
            if (accepted) begin
                if (dut.hard_kill || dut.guard_stop || dut.committed_velocity!=={x[15:0],y[15:0],w[15:0]} ||
                    dut.motor_control_u.remote_vx!==x || dut.motor_control_u.remote_vy!==y || dut.motor_control_u.remote_wz!==w)
                    $fatal(1,"in-range commit lost at %0d,%0d,%0d",x,y,w);
            end else if (!dut.hard_kill || dut.committed_velocity!==48'd0 || motor1!==0 || motor2!==0) begin
                $fatal(1,"out-of-range commit escaped at %0d,%0d,%0d",x,y,w);
            end
        end
    endtask

    initial begin
        force dut.now_us=now_us;
        force dut.pose_fresh=test_pose_fresh;
        force dut.imu_fresh=test_imu_fresh;
        force dut.wheel_ok=test_wheel_fresh;
        force dut.radar_sample_ctrl=radar_sample;
        force dut.radar_data_ctrl=radar_data;
        force dut.radar_error_ctrl=1'b0;
        repeat(4) @(negedge clk);reset_n=1;
        repeat(8) @(negedge clk);
        if ($test$plusargs("AUTONOMY_ONLY")) begin
            /* 隔离人工租约与CPU尚未提交自主位的时序，不依赖其他场景进度。 */
            force dut.link_fresh=1'b0;
            force dut.manual_fresh=1'b0;
            force dut.output_fresh=1'b1;
            force dut.output_seen=1'b1;
            force dut.command_output_current=1'b1;
            repeat(5)@(negedge clk);
            if(!dut.hard_kill || dut.auto_enabled)$fatal(1,"expired manual fixture not stopped");
            store(12'h2a0,1);repeat(5)@(negedge clk);
            if(!dut.hard_kill || dut.auto_enabled)
                $fatal(1,"uncommitted autonomy escaped expired manual lease");
            $display("PASS vision_autonomy_commit_tb: uncommitted autonomy preserves expired manual lease");
            $finish;
        end
        if(navigation_payload[256]!==1'b0 || fusion_payload[359]!==1'b1)
            $fatal(1,"startup hard kill must be filtered only in kind8");
        scan_far();scan_far();
        send(0,0,0);snapshot();stage_output(0,0,0);finish_output();
        expect_released(0);
        if(navigation_payload[256]!==1'b0)$fatal(1,"zero-speed handshake did not clear guard status");

        /* 新速度等待CPU确认时，硬件必须立即停车，但CPU不能把等待本身当作近障。 */
        send(100,0,0);
        expect_killed("new velocity awaits CPU");
        if(navigation_payload[256]!==1'b0 || fusion_payload[359]!==1'b1)
            $fatal(1,"CPU output wait must be filtered only in kind8");
        snapshot();
        $display("TRACE waiting cpu_status_guard=%b hard_kill=%b guard=%b seq=%0d output=%0d",
            dut.snap_status[3],dut.hard_kill,dut.guard_stop,dut.command_sequence,dut.output_command_seq);
        if (!$test$plusargs("ATOMIC_ONLY") && dut.snap_status[3]!==1'b0)
            $fatal(1,"internal output wait fed back to CPU as guard stop");
        stage_output(100,0,0);
        if ($test$plusargs("ATOMIC_ONLY")) finish_output();
        else finish_with_residual_snapshot();
        expect_released(100);
        if(navigation_payload[256]!==1'b0)$fatal(1,"confirmed motion did not clear guard status");
        if (!$test$plusargs("ATOMIC_ONLY") &&
            (!dut.motor_control_u.remote_armed || dut.motor_control_u.remote_vx!==32'sd100))
            $fatal(1,"first CPU command lost in rearm: armed=%b vx=%0d",
                dut.motor_control_u.remote_armed,dut.motor_control_u.remote_vx);
        if ($test$plusargs("ATOMIC_ONLY")) begin
            /* 基线CPU曾将guard回授为零；补一帧零速建立可运动的对照夹具。 */
            snapshot();stage_output(0,0,0);finish_output();expect_released(0);
            snapshot();stage_output(100,0,0);finish_output();expect_released(100);
        end

        // MMIO is the trusted path's sole numeric boundary; exercise both
        // signs and each axis, including the exact allowed endpoints.
        check_commit_bounds(400,-400,800,1);
        check_commit_bounds(-400,400,-800,1);
        check_commit_bounds(401,0,0,0);check_commit_bounds(-401,0,0,0);
        check_commit_bounds(0,401,0,0);check_commit_bounds(0,-401,0,0);
        check_commit_bounds(0,0,801,0);check_commit_bounds(0,0,-801,0);
        check_commit_bounds(100,0,0,1);

        /* 流水线前须实际产生PWM，再检查速度切换的两条保护路径。 */
        begin : wait_pwm
            reg found;
            found=0;
            for(cycle=0;cycle<2000;cycle=cycle+1)begin
                @(negedge clk);#1;
                if(motor1!=0 || motor2!=0)found=1;
            end
            if(!found)$fatal(1,"test fixture no PWM mode=%b armed=%b remain=%0d command=%h vx=%0d sample=%b target=%h duty=%h",
                dut.motor_control_u.remote_mode,dut.motor_control_u.remote_armed,
                dut.motor_control_u.remaining,dut.motor_control_u.command,dut.motor_control_u.remote_vx,
                dut.motor_control_u.control.sample_valid_o,dut.motor_control_u.control.target_mm_s_o,
                dut.motor_control_u.control.group_duty[0]);
        end
        send(150,0,0);expect_released(100);
        snapshot();stage_output(150,0,0);
        repeat(10)begin @(negedge clk);expect_released(100); end
        finish_output();expect_released(150);
        if(dut.motor_control_u.remote_vx!==32'sd150)
            $fatal(1,"rearm lost the newly committed speed");

        send(0,0,0);expect_killed("first zero revokes motion");
        snapshot();stage_output(0,0,0);
        repeat(10)begin @(negedge clk);expect_killed("before stop commit"); end
        finish_output();expect_released(0);
        if(dut.motor_control_u.remote_vx!==0 || motor1!==0 || motor2!==0)
            $fatal(1,"rearm replayed motion after a committed stop");

        send(100,0,0);snapshot();stage_output(100,0,0);finish_output();expect_released(100);
        @(negedge clk);mode_valid=1;mode_enable=0;
        @(negedge clk);mode_valid=0;
        repeat(4)@(negedge clk);expect_killed("mode change revokes old output");
        send(100,0,0);expect_killed("fresh post-mode command awaits CPU");
        snapshot();stage_output(100,0,0);
        repeat(10)begin @(negedge clk);expect_killed("before new mode commit"); end
        finish_output();expect_released(100);
        if(dut.motor_control_u.remote_vx!==32'sd100)
            $fatal(1,"rearm lost the output confirmed in the new mode");

`ifndef BX72_VISION_BASELINE
        /* guard重审恰遇模式变化时，旧重审不能越过新模式确认。 */
        test_pose_fresh=0;repeat(3)@(negedge clk);expect_killed("prepare mode race");
        send(120,0,0);snapshot();stage_output(120,0,0);store(12'h20c,32'h42583732);
        test_pose_fresh=1;wait(dut.guard_recheck);
        @(negedge clk);mode_valid=1;mode_enable=0;
        @(negedge clk);mode_valid=0;
        repeat(4)@(negedge clk);expect_killed("mode changed during guard recheck");
        send(120,0,0);expect_killed("fresh post-race command awaits CPU");
        snapshot();stage_output(120,0,0);finish_output();expect_released(120);
        if(dut.motor_control_u.remote_vx!==32'sd120)
            $fatal(1,"new mode confirmation lost after concurrent recheck");

        /* guard重审恰遇首次零速时，旧非零不能在停车确认后复活。 */
        test_pose_fresh=0;repeat(3)@(negedge clk);expect_killed("prepare zero race");
        send(130,0,0);snapshot();stage_output(130,0,0);store(12'h20c,32'h42583732);
        test_pose_fresh=1;wait(dut.guard_recheck);
        send(0,0,0);expect_killed("stop arrived during guard recheck");
        snapshot();stage_output(0,0,0);finish_output();expect_released(0);
        repeat(12)@(negedge clk);
        if(dut.motor_control_u.remote_vx!==0 || motor1!==0 || motor2!==0)
            $fatal(1,"stale recheck revived motion after new stop");
        send(100,0,0);snapshot();stage_output(100,0,0);finish_output();expect_released(100);
`endif

        /* 已结束任务事件只影响展示，不再使已确认人工输出反复失效。 */
        store(12'h230,4);
        send(100,0,0);expect_released(100);
        snapshot();stage_output(100,0,0);finish_output();expect_released(100);
        send(100,0,0);expect_released(100);
        store(12'h230,0);snapshot();stage_output(100,0,0);finish_output();expect_released(100);
        send(100,0,0);expect_released(100);
        $display("TRACE completed-task display preserves confirmed manual epoch");

        /* 近障和内部等待同时存在时，所有周期PWM均为零；解除等待后近障仍守住。 */
        test_pose_fresh=0;repeat(3)@(negedge clk);
        send(150,0,0);expect_killed("prepare internal kill before obstacle");
        point(0,120);
        repeat(8)begin
            @(negedge clk);expect_killed("obstacle with internal kill");
            if(navigation_payload[256]!==1'b0 || fusion_payload[359]!==1'b1)
                $fatal(1,"internal kill must remain visible only in raw guard status");
        end
        snapshot();stage_output(150,0,0);test_pose_fresh=1;finish_output();
        repeat(20)begin @(negedge clk);#1;
            if(dut.hard_kill || !dut.guard_stop || dut.guard_velocity!==0 || motor1!==0 || motor2!==0)
                $fatal(1,"actual obstacle lost after output confirmation");
        end
        snapshot();
        if(dut.snap_status[3]!==1'b1)$fatal(1,"real obstacle must reach CPU after kill pipeline clears");
        if(navigation_payload[256]!==1'b1 || fusion_payload[359]!==1'b1)
            $fatal(1,"real obstacle must reach both filtered and raw telemetry");
        scan_far();scan_far();
        snapshot();stage_output(150,0,0);finish_output();expect_released(150);

`ifndef BX72_VISION_BASELINE
        /* 同拍COMMIT与guard重审不能使上一帧进入电机；只允许新COMMIT经guard后放行。 */
        test_pose_fresh=0;repeat(3)@(negedge clk);expect_killed("prepare commit/recheck race");
        snapshot();stage_output(-150,0,0);
        test_pose_fresh=1;
        wait(dut.guard_recheck);
        @(negedge clk);dut.cpu.valid=1;dut.cpu.writing=1;dut.cpu.address=12'h20c;dut.cpu.data=32'h42583732;
        #1;if(dut.guard_recheck)$fatal(1,"COMMIT must suppress same-cycle old guard recheck");
        @(negedge clk);dut.cpu.valid=0;dut.cpu.writing=0;
        repeat(20)begin @(negedge clk);#1;
            if(dut.motor_command_valid && dut.motor_command_velocity!==48'd0 &&
               $signed(dut.motor_command_velocity[47:32])!==-16'sd150)
                $fatal(1,"old velocity reached motor during new COMMIT/recheck collision");
        end
        expect_released(-150);
        if(dut.motor_control_u.remote_vx!==-32'sd150)$fatal(1,"new COMMIT was lost during guard recheck");
`endif

        test_pose_fresh=0;repeat(3)@(negedge clk);expect_killed("pose expiry");
        test_pose_fresh=1;repeat(3)@(negedge clk);expect_released(-150);
        test_imu_fresh=0;repeat(3)@(negedge clk);expect_killed("imu expiry");
        test_imu_fresh=1;repeat(3)@(negedge clk);expect_released(-150);
        test_wheel_fresh=0;repeat(3)@(negedge clk);expect_killed("wheel expiry");
        test_wheel_fresh=1;repeat(3)@(negedge clk);expect_released(-150);
        now_us=100101;repeat(5)@(negedge clk);expect_killed("100ms CPU output expiry");
        if(dut.output_fresh)$fatal(1,"old output became fresh at 100ms");
        snapshot();stage_output(150,0,0);finish_output();expect_released(150);
        now_us=200099;repeat(5)@(negedge clk);expect_killed("200ms lease expiry");
        if(!dut.output_fresh || dut.manual_fresh || dut.link_fresh)
            $fatal(1,"lease expiry must stop independently while CPU output remains fresh");
        store(12'h2a0,1);repeat(10)@(negedge clk);
        expect_killed("uncommitted autonomy must not bypass expired manual lease");
        snapshot();store(12'h2a4,dut.snap_command_seq);store(12'h2a8,dut.snap_mode_seq);
        send(0,0,0);expect_killed("new manual stop supersedes staged autonomy");
        store(12'h20c,32'h42583732);repeat(8)@(negedge clk);
        if(dut.auto_enabled || !dut.hard_kill || motor1!==0 || motor2!==0)
            $fatal(1,"stale autonomous commit overrode newer manual command");
        snapshot();stage_output(0,0,0);finish_output();expect_released(0);
        snapshot();stage_output(0,0,0);store(12'h2a0,1);
        @(negedge clk);mode_valid=1;mode_enable=0;
        @(negedge clk);mode_valid=0;
        store(12'h20c,32'h42583732);repeat(8)@(negedge clk);
        if(dut.auto_enabled || !dut.hard_kill || motor1!==0 || motor2!==0)
            $fatal(1,"stale autonomous commit overrode a newer mode");
        snapshot();stage_output(0,0,0);finish_output();expect_released(0);
        $display("PASS vision_guard_feedback_tb: pending, atomic commit, proximity+kill, sensor and lease stops");
        $finish;
    end
    initial begin #5000000;$fatal(1,"vision guard feedback test timeout");end
endmodule
