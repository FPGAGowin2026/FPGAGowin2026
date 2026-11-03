`timescale 1ns/1ps
/* 真实core、任务FIFO、guard及电机；CPU由MMIO事务模型驱动。 */
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

module task_overflow_recovery_tb;
    reg clk=0,reset_n=0,command_valid=0,heartbeat=0,mode_valid=0;
    reg [47:0] command_velocity=0;
    reg task_valid=0;
    reg [255:0] task_data=0;
    reg radar_sample=0;
    reg [41:0] radar_data=0;
    reg [31:0] now_us=100;
    wire [3:0] motor1,motor2;
    reg expect_zero=0;
    reg [255:0] held_head;
    reg [31:0] old_epoch,old_motion_sequence;
    integer i;
    always #12.5 clk=~clk;
    always @(negedge clk) begin
        #1;
        if(reset_n && (dut.hard_kill||dut.guard_stop) && (motor1!==0||motor2!==0))
            $fatal(1,"kill or real guard left nonzero PWM");
        if(expect_zero && !dut.hard_kill && !dut.guard_stop &&
           dut.motor_command_valid && dut.motor_command_velocity!==48'd0)
            $fatal(1,"old motion crossed overflow recovery");
    end

    bx72_lite_robot_core #(.CONTROL_CLK_HZ(40_000_000)) dut(
        .control_clk(clk),.loc_clk33m(clk),.reset_n(reset_n),
        .n10_uart_rx(1'b1),.imu_miso(1'b0),.imu_cs_n(),.imu_sclk(),.imu_mosi(),
        .encoder_a_i(4'b0),.encoder_b_i(4'b0),.motor_in1_o(motor1),.motor_in2_o(motor2),
        .command_valid_i(command_valid),.command_link_down_i(1'b0),.command_heartbeat_i(heartbeat),
        .command_velocity_i(command_velocity),.auto_mode_valid_i(mode_valid),.auto_enable_i(1'b0),
        .task_fragment_valid_i(task_valid),.task_fragment_i(task_data),.command_token_i(64'h1234),
        .navigation_valid_o(),.navigation_payload_o(),.local_plan_valid_o(),.local_plan_payload_o(),
        .auto_mode_o(),.fusion_valid_o(),.imu_valid_o(),.motor_valid_o(),.lidar_valid_o(),
        .imu_payload_o(),.motor_payload_o(),.lidar_payload_o(),.fusion_payload_o(),
        .sensor_health_o(),.cpu_ready_o(),.cpu_fault_o(),.cpu_busy_o(),.control_reset_o(),.cpu_fault_code_o());
    defparam dut.motor_control_u.CLK_HZ=40000;
    defparam dut.motor_control_u.STARTUP_CYCLES=3;
    defparam dut.motor_control_u.PWM_PERIOD=32;

    task automatic store(input [11:0] addr,input [31:0] value);
        begin
            @(negedge clk);dut.cpu.valid=1;dut.cpu.writing=1;dut.cpu.address=addr;dut.cpu.data=value;
            @(negedge clk);dut.cpu.valid=0;dut.cpu.writing=0;
        end
    endtask
    task automatic send(input signed [15:0] x);
        begin
            @(negedge clk);command_valid=1;heartbeat=1;command_velocity={x,32'd0};
            @(negedge clk);command_valid=0;heartbeat=0;
            repeat(4)@(negedge clk);
        end
    endtask
    task automatic output_commit(input signed [31:0] x);
        begin
            store(12'h108,0);
            store(12'h200,x);store(12'h204,0);store(12'h208,0);
            store(12'h2a0,0);store(12'h2a4,dut.snap_command_seq);store(12'h2a8,dut.snap_mode_seq);
            store(12'h20c,32'h42583732);
            repeat(16)@(negedge clk);
        end
    endtask
    task automatic point(input [15:0] a,input [15:0] distance);
        begin
            @(negedge clk);radar_sample=1;radar_data={1'b0,1'b1,8'd0,a,distance};
            @(negedge clk);radar_sample=0;
        end
    endtask
    task automatic scan_far;
        integer n;
        begin
            for(n=0;n<8;n=n+1) point(n*4500+1000,2000);
            point(0,2000);
        end
    endtask
    task automatic push(input [7:0] index);
        begin
            @(negedge clk);task_valid=1;task_data=0;task_data[95:88]=index;
            @(negedge clk);task_valid=0;
        end
    endtask
    task automatic expect_head(input [7:0] index);
        begin
            wait(dut.task_available);#1;
            if(dut.task_head[95:88]!==index)
                $fatal(1,"unexpected task fragment %0d wanted %0d",dut.task_head[95:88],index);
            store(12'h184,dut.task_fifo_epoch);
        end
    endtask

    initial begin
        force dut.now_us=now_us;
        force dut.pose_fresh=1'b1;
        force dut.imu_fresh=1'b1;
        force dut.wheel_ok=1'b1;
        force dut.radar_sample_ctrl=radar_sample;
        force dut.radar_data_ctrl=radar_data;
        force dut.radar_error_ctrl=1'b0;
        repeat(4)@(negedge clk);reset_n=1;
        repeat(8)@(negedge clk);
        scan_far();scan_far();
        send(0);output_commit(0);
        send(100);output_commit(100);
        if(dut.hard_kill || dut.guard_stop || dut.motor_control_u.remote_vx!==32'sd100)
            $fatal(1,"fixture failed to establish confirmed manual command");

        /* 16项RAM及预取槽均填满，下一分片发生丢失。 */
        for(i=0;i<17;i=i+1)push(i);
        if(!dut.task_full||!dut.task_available)$fatal(1,"fixture failed to fill task FIFO");
        held_head=dut.task_head;
        push(7);
        repeat(6)@(negedge clk);
        expect_zero=1;
        if(!dut.task_overflow||!dut.hard_kill||motor1!==0||motor2!==0)
            $fatal(1,"task overflow failed to stop motion");
        if(dut.task_available)$fatal(1,"overflowed task FIFO was not flushed");
        if(dut.task_head!==held_head)$fatal(1,"overflow changed a fragment being read by the CPU");
        push(1);push(2);
        if(dut.task_available)$fatal(1,"task fragment entered while overflow was pending");

        /* 模拟固件的恢复顺序：先提交零速，再应答丢片。 */
        output_commit(0);
        if(dut.committed_velocity!==0||!dut.task_overflow)$fatal(1,"zero must commit before acknowledgement");

        /* 应答同拍到达的分片仍丢弃，随后仅接收新分片。 */
        @(negedge clk);dut.cpu.valid=1;dut.cpu.writing=1;dut.cpu.address=12'h188;dut.cpu.data=1;
        task_valid=1;task_data=0;task_data[95:88]=0;
        @(negedge clk);dut.cpu.valid=0;dut.cpu.writing=0;task_valid=0;
        repeat(20)@(negedge clk);
        if(dut.task_overflow||dut.hard_kill||dut.task_available||dut.motor_control_u.remote_vx!==0)
            $fatal(1,"recovery did not release safely with an empty FIFO");

        /* 幸存后缀和下一条完整任务分别按原样出队；固件独立验证拒绝后缀。 */
        for(i=1;i<8;i=i+1)push(i);
        for(i=1;i<8;i=i+1)expect_head(i);
        repeat(3)@(negedge clk);
        if(dut.task_available)$fatal(1,"suffix FIFO did not drain");
        for(i=0;i<8;i=i+1)push(i);
        for(i=0;i<8;i=i+1)expect_head(i);
        repeat(3)@(negedge clk);
        if(dut.task_available)$fatal(1,"complete task FIFO did not drain");

        /* 旧读事务不能在零速接管之后误弹出新任务的队首。 */
        push(0);wait(dut.task_available);old_epoch=dut.task_fifo_epoch;
        old_motion_sequence=dut.manual_motion_sequence;
        send(0);
        if(dut.task_available)$fatal(1,"manual stop did not flush old fragments");
        if(dut.manual_motion_sequence!==old_motion_sequence)
            $fatal(1,"zero stop must not count as a manual movement takeover");
        push(1);wait(dut.task_available);
        store(12'h184,old_epoch);
        if(!dut.task_available || dut.task_head[95:88]!==8'd1)
            $fatal(1,"stale pop consumed post-takeover fragment");
        expect_head(1);

        /* 模式在先的后续分片可入队；模式变化前的分片已撤销。 */
        push(0);wait(dut.task_available);old_epoch=dut.task_fifo_epoch;
        @(negedge clk);mode_valid=1;
        @(negedge clk);mode_valid=0;
        if(dut.task_available)$fatal(1,"mode change did not flush old fragments");
        push(0);wait(dut.task_available);
        store(12'h184,old_epoch);
        if(!dut.task_available || dut.task_head[95:88]!==8'd0)
            $fatal(1,"stale pop consumed post-mode fragment");
        expect_head(0);

        /* 两拍之间的非零再归零仍须在一次快照中报告。 */
        send(100);send(0);store(12'h108,0);
        if(dut.snap_manual_motion_seq!==old_motion_sequence+1'b1)
            $fatal(1,"nonzero manual takeover was lost after a subsequent zero");

        /* 新的零速及非零人工命令须重新确认；近障依旧停车。 */
        send(0);output_commit(0);
        expect_zero=0;
        send(100);output_commit(100);
        if(dut.hard_kill||dut.guard_stop||dut.motor_control_u.remote_vx!==32'sd100)
            $fatal(1,"new confirmed manual control did not recover");
        for(i=0;i<17;i=i+1)push(i);
        push(7);repeat(6)@(negedge clk);expect_zero=1;
        if(!dut.task_overflow||!dut.hard_kill||dut.task_available)
            $fatal(1,"a second overflow must stop and flush again");
        output_commit(0);store(12'h188,1);repeat(20)@(negedge clk);
        if(dut.task_overflow||dut.hard_kill||dut.motor_control_u.remote_vx!==0)
            $fatal(1,"a second overflow did not recover with zero output");
        send(0);output_commit(0);
        expect_zero=0;send(100);output_commit(100);
        if(dut.hard_kill||dut.guard_stop||dut.motor_control_u.remote_vx!==32'sd100)
            $fatal(1,"manual control did not recover after the second overflow");
        point(0,120);
        repeat(12)@(negedge clk);
        if(!dut.guard_stop||motor1!==0||motor2!==0)$fatal(1,"front obstacle protection changed");
        $display("PASS task_overflow_recovery_tb: flush, recovery order, in-flight drop, new fragments, manual, guard");
        $finish;
    end
    initial begin #5000000;$fatal(1,"task overflow recovery test timeout");end
endmodule
