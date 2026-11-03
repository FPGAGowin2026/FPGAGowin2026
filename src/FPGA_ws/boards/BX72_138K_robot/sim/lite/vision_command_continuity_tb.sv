`timescale 1ns/1ps

// 生产核心、保护和电机使用原始RTL；软核以确定时延的MMIO事务替身模拟。
// MMIO平台替身复用 shared_lidar_queue_tb.sv。


module vision_command_continuity_tb;
    parameter integer SIM_HZ=400000;
    localparam integer INTERVAL=SIM_HZ/20;       // 命令每50ms
    localparam integer CPU_PERIOD=SIM_HZ/100;  // CPU模型每10ms拍快照
    localparam integer CPU_DELAY=SIM_HZ/500;    // 快照后2ms提交
    localparam integer PWM_SCALE=SIM_HZ/16000;  // 保持16kHz PWM
    reg clk=0,reset_n=0,command_valid=0,heartbeat=0;
    reg [47:0] velocity=0;
    reg radar_sample=0,scan_enabled=0,cpu_enabled=0,measuring=0;
    reg [41:0] radar_data=0;
    wire [3:0] motor1,motor2;
    integer i;
    always #(500000000/SIM_HZ) clk=~clk;

    bx72_lite_robot_core #(.CONTROL_CLK_HZ(SIM_HZ)) dut(
        .control_clk(clk),.loc_clk33m(clk),.reset_n(reset_n),
        .n10_uart_rx(1'b1),.imu_miso(1'b0),.imu_cs_n(),.imu_sclk(),.imu_mosi(),
        .encoder_a_i(4'd0),.encoder_b_i(4'd0),.motor_in1_o(motor1),.motor_in2_o(motor2),
        .command_valid_i(command_valid),.command_link_down_i(1'b0),
        .command_heartbeat_i(heartbeat),.command_velocity_i(velocity),
        .auto_mode_valid_i(1'b0),.auto_enable_i(1'b0),
        .task_fragment_valid_i(1'b0),.task_fragment_i(256'd0),.command_token_i(64'h1234),
        .navigation_valid_o(),.navigation_payload_o(),.local_plan_valid_o(),.local_plan_payload_o(),
        .auto_mode_o(),.fusion_valid_o(),.imu_valid_o(),.motor_valid_o(),.lidar_valid_o(),
        .imu_payload_o(),.motor_payload_o(),.lidar_payload_o(),.fusion_payload_o(),
        .sensor_health_o(),.cpu_ready_o(),.cpu_fault_o(),.cpu_busy_o(),.control_reset_o(),.cpu_fault_code_o());
    defparam dut.motor_control_u.PWM_PERIOD=PWM_SCALE;

    task automatic store(input [11:0] addr,input [31:0] value);
        begin
            @(negedge clk); dut.cpu.valid=1; dut.cpu.writing=1; dut.cpu.address=addr; dut.cpu.data=value;
            @(negedge clk); dut.cpu.valid=0; dut.cpu.writing=0;
        end
    endtask
    task automatic send_w(input signed [15:0] w);
        begin
            @(negedge clk); velocity={16'd0,16'd0,w}; command_valid=1; heartbeat=1;
            @(negedge clk); command_valid=0; heartbeat=0;
        end
    endtask
    task automatic point(input [15:0] a);
        begin
            @(negedge clk); radar_sample=1; radar_data={1'b0,1'b1,8'd0,a,16'd2000};
            @(negedge clk); radar_sample=0;
        end
    endtask
    task automatic far_scan;
        integer k;
        begin
            for(k=0;k<8;k=k+1) point(k*4500+1000);
            point(0);
        end
    endtask

    // 与两个场景相同的CPU模型：每轮等待10ms读快照，再等待2ms写回；未执行真实固件。
    initial begin
        forever begin
            wait(cpu_enabled);
            repeat(CPU_PERIOD) @(negedge clk);
            if(cpu_enabled) begin
                store(12'h108,0);
                repeat(CPU_DELAY) @(negedge clk);
                store(12'h200,{{16{dut.snap_remote_velocity[47]}},dut.snap_remote_velocity[47:32]});
                store(12'h204,{{16{dut.snap_remote_velocity[31]}},dut.snap_remote_velocity[31:16]});
                store(12'h208,{{16{dut.snap_remote_velocity[15]}},dut.snap_remote_velocity[15:0]});
                store(12'h2a4,dut.snap_command_seq);
                store(12'h2a8,dut.snap_mode_seq);
                store(12'h2a0,0);
                store(12'h20c,32'h42583732);
            end
        end
    end
    initial begin
        forever begin
            wait(scan_enabled);
            repeat(SIM_HZ/10) @(negedge clk);
            if(scan_enabled) far_scan;
        end
    end

    integer cycles,current_low,kill_high,guard_high,disabled,disable_edges,history_nonzero_on_disable;
    integer drive_cycles,max_abs_target,max_abs_effort,min_target,prev_enable;
    integer signed target,effort;
    reg [63:0] sum_abs_target,sum_abs_effort;
    integer fixed_disable,fixed_kill,fixed_current_low;
    reg [63:0] fixed_sum_target,fixed_sum_effort;

    task automatic clear_stats;
        begin
            cycles=0;current_low=0;kill_high=0;guard_high=0;disabled=0;
            disable_edges=0;history_nonzero_on_disable=0;drive_cycles=0;
            max_abs_target=0;max_abs_effort=0;min_target=2147483647;
            sum_abs_target=0;sum_abs_effort=0;
            prev_enable=dut.motor_control_u.controller_enabled;
        end
    endtask
    always @(negedge clk) begin
        #1;
        if(measuring) begin
            cycles=cycles+1;
            if(!dut.command_output_current)current_low=current_low+1;
            if(dut.hard_kill)kill_high=kill_high+1;
            if(dut.guard_stop)guard_high=guard_high+1;
            if(!dut.motor_control_u.controller_enabled)disabled=disabled+1;
            target=$signed(dut.motor_control_u.control.target_mm_s_o[31:0]);
            if(target<0)target=-target;
            effort=$signed(dut.motor_control_u.control.group_controller.engine.effort_q[0]);
            if(effort<0)effort=-effort;
            if(prev_enable&&!dut.motor_control_u.controller_enabled)begin
                disable_edges=disable_edges+1;
                if(effort!=0)history_nonzero_on_disable=history_nonzero_on_disable+1;
            end
            prev_enable=dut.motor_control_u.controller_enabled;
            if(target>max_abs_target)max_abs_target=target;
            if(target<min_target)min_target=target;
            if(effort>max_abs_effort)max_abs_effort=effort;
            sum_abs_target=sum_abs_target+target;
            sum_abs_effort=sum_abs_effort+effort;
            if(motor1[2]^motor2[2])drive_cycles=drive_cycles+1;
        end
    end

    task automatic run_phase(input integer changed);
        integer n;
        reg signed [15:0] w;
        begin
            measuring=0;cpu_enabled=0;scan_enabled=0;
            // 等待上一轮CPU及雷达模型退出各自事务后再重置。
            repeat(CPU_PERIOD+CPU_DELAY+30) @(negedge clk);
            reset_n=0;command_valid=0;heartbeat=0;velocity=0;radar_sample=0;
            repeat(8) @(negedge clk);reset_n=1;repeat(8) @(negedge clk);
            far_scan();scan_enabled=1;cpu_enabled=1;
            send_w(0);
            repeat(SIM_HZ*3/100) @(negedge clk);
            send_w(-16'sd236);
            repeat(SIM_HZ*3/20) @(negedge clk);
            if(!dut.motor_control_u.controller_enabled || dut.hard_kill || dut.guard_stop)
                $fatal(1,"phase fixture not released: changed=%0d enabled=%b kill=%b guard=%b current=%b",
                    changed,dut.motor_control_u.controller_enabled,dut.hard_kill,dut.guard_stop,dut.command_output_current);
            @(posedge clk);clear_stats();measuring=1;
            for(n=0;n<8;n=n+1)begin
                w=(changed && (n%2==0)) ? -16'sd238 : -16'sd236;
                send_w(w);
                repeat(INTERVAL-2) @(negedge clk);
            end
            @(posedge clk);measuring=0;
            $display("RESULT phase=%0s sim_hz=%0d pwm_period=%0d cycles=%0d current_low=%0d kill_high=%0d guard_high=%0d controller_disabled=%0d disabled_edges=%0d history_nonzero_on_disable=%0d target_abs_sum=%0d target_min=%0d target_max=%0d effort_abs_sum=%0d effort_max=%0d active_pwm_cycles=%0d",
                changed?"jitter":"constant",SIM_HZ,PWM_SCALE,cycles,current_low,kill_high,guard_high,
                disabled,disable_edges,history_nonzero_on_disable,sum_abs_target,min_target,max_abs_target,
                sum_abs_effort,max_abs_effort,drive_cycles);
            if(!changed)begin
                fixed_disable=disable_edges;fixed_kill=kill_high;fixed_current_low=current_low;
                fixed_sum_target=sum_abs_target;fixed_sum_effort=sum_abs_effort;
            end else begin
                if(fixed_disable!=0 || fixed_kill!=0 || fixed_current_low!=0)
                    $fatal(1,"constant fixture unexpectedly stopped");

                if(disable_edges!=0 || current_low!=0 || kill_high!=0 || guard_high!=0 || disabled!=0)
                    $fatal(1,"candidate unexpectedly stopped inside a confirmed nonzero epoch");
                if(sum_abs_target!=fixed_sum_target || sum_abs_effort!=fixed_sum_effort)
                    $fatal(1,"candidate failed to preserve the same quantized wheel target and history");

            end
        end
    endtask

    initial begin
        // 保持已有测试的传感器夹具；不强制命令确认、保护或电机使能信号。
        force dut.now_us=32'd100;
        force dut.manual_fresh=1'b1;
        force dut.link_down=1'b0;
        force dut.pose_fresh=1'b1;
        force dut.imu_fresh=1'b1;
        force dut.wheel_ok=1'b1;
        force dut.output_fresh=1'b1;
        force dut.radar_sample_ctrl=radar_sample;
        force dut.radar_data_ctrl=radar_data;
        force dut.radar_error_ctrl=1'b0;
        run_phase(0);
        run_phase(1);
        $display("PASS: real core/guard/motor comparison with modeled CPU confirmation");
        $finish;
    end
    initial begin
        #3000000000;
        $fatal(1,"simulation timeout");
    end
endmodule
