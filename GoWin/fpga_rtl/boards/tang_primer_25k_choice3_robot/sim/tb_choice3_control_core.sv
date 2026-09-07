`timescale 1ns/1ps

module tb_choice3_control_core;
    reg clk=0, rst_n=0, estop_n=1;
    reg [3:0] peripheral_health=0;
    reg arm=0, disarm=0, clear_fault=0, route_start=0;
    reg manual_valid=0;reg [2:0] manual_motion=0;
    reg [15:0] vl_mm=16'd800, us_mm=16'd800;
    reg vl_valid=0, us_valid=0, us_timeout=0;
    reg [3:0] ir=0, enc_a=0, enc_b=0;
    wire [3:0] pwm,in1,in2,deadtime;
    wire stby,armed,motor_enable,arm_rejected;
    wire [7:0] health,fault_now,fault_latched;
    wire [3:0] nav_state;
    wire route_done,front_valid,front_blocked,sample_tick;
    wire [15:0] front_mm;
    wire [63:0] duties;
    integer errors=0;

    // 独立速度估计器探针：覆盖正常值与正/负饱和边界。
    reg velocity_probe_rst_n=0,velocity_probe_sample=0;
    reg signed [31:0] velocity_probe_position=0;
    wire signed [31:0] velocity_probe_value;
    wire velocity_probe_valid;

    always #5 clk=~clk;

    choice3_control_core #(
        .CLOCK_HZ(4000),.SAMPLE_HZ(100),.PWM_PERIOD(512),
        .DEADTIME_CYCLES(6),.STALL_SAMPLE_LIMIT(20),
        .RANGE_MAX_AGE_TICKS(20),.NAV_LEG1_TICKS(10000),
        .NAV_TURN_TICKS(10000),.NAV_LEG2_TICKS(10000),
        .NAV_AVOID_BACK_SAMPLES(2),.NAV_AVOID_TURN_SAMPLES(2)
    ) dut (
        .clk_i(clk),.rst_n_i(rst_n),.estop_n_i(estop_n),
        .peripheral_health_i(peripheral_health),.arm_pulse_i(arm),
        .disarm_pulse_i(disarm),.clear_fault_pulse_i(clear_fault),
        .route_start_pulse_i(route_start),.manual_command_valid_i(manual_valid),
        .manual_motion_i(manual_motion),.vl53_distance_mm_i(vl_mm),
        .vl53_valid_i(vl_valid),.ultrasonic_distance_mm_i(us_mm),
        .ultrasonic_valid_i(us_valid),.ultrasonic_timeout_i(us_timeout),
        .ir_active_i(ir),.encoder_a_i(enc_a),.encoder_b_i(enc_b),
        .motor_pwm_o(pwm),.motor_in1_o(in1),.motor_in2_o(in2),
        .motor_stby_o(stby),.armed_o(armed),.motor_enable_o(motor_enable),
        .health_o(health),.fault_now_o(fault_now),
        .fault_latched_o(fault_latched),.arm_rejected_pulse_o(arm_rejected),
        .nav_state_o(nav_state),.route_done_o(route_done),
        .fused_front_mm_o(front_mm),.fused_front_valid_o(front_valid),
        .front_blocked_o(front_blocked),.pwm_duty_flat_o(duties),
        .deadtime_active_o(deadtime),.manual_mode_o(),.manual_motion_o(),
        .sample_tick_o(sample_tick));

    encoder_velocity_estimator #(.MM_PER_TICK_Q16(32'sd131072),.SAMPLE_HZ(1))
      u_velocity_probe(.clk_i(clk),.rst_n_i(velocity_probe_rst_n),
      .sample_tick_i(velocity_probe_sample),.position_ticks_i(velocity_probe_position),
      .velocity_mm_s_o(velocity_probe_value),.velocity_valid_o(velocity_probe_valid));

    task automatic check(input bit condition,input string message);
        begin
            if(!condition) begin $display("FAIL: %s",message); errors=errors+1; end
        end
    endtask
    task automatic pulse_arm;
        begin @(negedge clk);arm=1;@(negedge clk);arm=0; end
    endtask
    task automatic pulse_route;
        begin @(negedge clk);route_start=1;@(negedge clk);route_start=0; end
    endtask
    task automatic issue_manual(input [2:0] motion);
        begin @(negedge clk);manual_motion=motion;manual_valid=1;
              @(negedge clk);manual_valid=0;#1;end
    endtask
    task automatic publish_ranges(input [15:0] v,input [15:0] u);
        begin @(negedge clk);vl_mm=v;us_mm=u;vl_valid=1;us_valid=1;
              @(negedge clk);vl_valid=0;us_valid=0; end
    endtask
    task automatic wait_sample;
        // sample_tick 由时序逻辑产生；再等一个 clk 上升沿，确保消费者已提交结果。
        begin @(posedge sample_tick); @(posedge clk); #1; end
    endtask
    task automatic set_ab(input integer wheel,input bit a,input bit b);
        begin enc_a[wheel]=a;enc_b[wheel]=b;repeat(4)@(posedge clk);end
    endtask
    task automatic forward_cycle(input integer wheel);
        begin set_ab(wheel,0,1);set_ab(wheel,1,1);set_ab(wheel,1,0);set_ab(wheel,0,0);end
    endtask
    task automatic reverse_half(input integer wheel);
        begin set_ab(wheel,1,0);set_ab(wheel,1,1);end
    endtask
    task automatic probe_velocity(input signed [31:0] position);
        begin
            @(negedge clk);velocity_probe_position=position;velocity_probe_sample=1;
            @(negedge clk);velocity_probe_sample=0;
            repeat(2)@(posedge clk);#1;
        end
    endtask
    task automatic reset_velocity_probe;
        begin
            @(negedge clk);velocity_probe_rst_n=0;velocity_probe_position=0;
            repeat(2)@(negedge clk);velocity_probe_rst_n=1;
        end
    endtask

    initial begin
        if($test$plusargs("DUMP")) begin
            $dumpfile("reports/tb_choice3_control_core.vcd");
            $dumpvars(0,tb_choice3_control_core);
        end
        repeat(5)@(posedge clk); rst_n=1; repeat(5)@(posedge clk);
        check(!armed && !stby && pwm==0 && in1==0 && in2==0,
              "复位后必须保持电机安全关闭");

        reset_velocity_probe();probe_velocity(32'sd100);
        check(velocity_probe_valid&&velocity_probe_value==32'sd200,
              "速度估计正常 Q16 换算错误");
        reset_velocity_probe();probe_velocity(32'sh7fff_ffff);
        check(velocity_probe_value==32'sh7fff_ffff,"速度估计正溢出未饱和");
        reset_velocity_probe();probe_velocity(-32'sh8000_0000);
        check(velocity_probe_value==-32'sh8000_0000,"速度估计负溢出未饱和");

        issue_manual(1);repeat(2)@(posedge clk);
        check(!dut.manual_mode_o&&dut.target[0]==0,"未武装手动命令被非法缓存");

        // 外设缺失时武装必须失败。
        pulse_arm(); #1;
        check(!armed && !stby,"外设缺失时不得武装");
        check(fault_latched[3:0]!=0,"缺失外设必须锁存故障");

        peripheral_health=4'hf;
        publish_ranges(16'd700,16'd650);
        repeat(4)@(posedge clk);
        check(health[4:0]==5'h1f && front_valid && front_mm==650,
              "健康位和双测距最小值融合错误");
        estop_n=0;pulse_arm();#1;
        check(!armed&&!stby&&arm_rejected,"急停按下时 arm 未被显式拒绝");
        estop_n=1;
        @(negedge clk);clear_fault=1;@(negedge clk);clear_fault=0;
        repeat(2)@(posedge clk);
        pulse_arm(); #1;
        check(armed && motor_enable && stby,"健康后显式武装失败");
        pulse_route(); wait_sample(); wait_sample();
        check(nav_state==1,"航点状态机未进入第一直行段");
        check(duties[15:0]!=0 && duties[31:16]!=0 &&
              duties[47:32]!=0 && duties[63:48]!=0,
              "四路 PI 未产生独立 PWM 请求");

        // 显式动作入口符号验收：前、后、弧转、原地旋转，且与路线互斥。
        issue_manual(1);check(dut.target[0]>0&&dut.target[3]>0,"手动前进目标符号错误");
        issue_manual(2);check(dut.target[0]<0&&dut.target[3]<0,"手动后退目标符号错误");
        issue_manual(3);check(dut.target[0]>0&&dut.target[2]>dut.target[0],"手动左弧转目标错误");
        issue_manual(4);check(dut.target[0]>dut.target[2]&&dut.target[2]>0,"手动右弧转目标错误");
        issue_manual(5);check(dut.target[0]<0&&dut.target[2]>0,"手动左原地旋转符号错误");
        issue_manual(6);check(dut.target[0]>0&&dut.target[2]<0,"手动右原地旋转符号错误");
        issue_manual(1);ir=4'b0001;#1;
        check(dut.target[0]==0&&!stby&&pwm==0,"手动前进遇前障碍未即时禁止驱动");
        ir=0;issue_manual(0);check(dut.target[0]==0,"手动停止命令无效");
        pulse_route();wait_sample();check(dut.manual_mode_o==0&&nav_state==1,
              "自动路线与手动模式未互斥");

        // 给不同轮子注入不同编码器增量；下一次 PI 计算应使用各自速度。
        fork
            forward_cycle(0);
            begin repeat(8)@(posedge clk);set_ab(1,0,1);set_ab(1,1,1);end
            reverse_half(3);
        join
        wait_sample();
        // 速度估计同样是四拍流水。
        repeat(4)@(posedge clk);#1;
        $display("INFO injected_velocity=%0d,%0d,%0d,%0d",
                 dut.velocity[0],dut.velocity[1],dut.velocity[2],dut.velocity[3]);
        check(dut.velocity[0] != dut.velocity[1] &&
              dut.velocity[1] != dut.velocity[2] &&
              dut.velocity[3] < 0,
              "四路速度估计未保持独立编码器状态");
        wait_sample();
        // PI 为四拍流水；速度估计在 sample 边沿提交，随后再等待流水完成。
        repeat(4)@(posedge clk);#1;
        $display("INFO independent_duties=%0d,%0d,%0d,%0d",
                 duties[15:0],duties[31:16],duties[47:32],duties[63:48]);
        check(duties[15:0]!=duties[31:16] || duties[31:16]!=duties[47:32],
              "四个速度环疑似共享了测量状态");

        // IR 前障碍进入后退，随后方向翻转必须出现全关断死区。
        ir=4'b0001; wait_sample(); wait_sample(); wait_sample();
        check(nav_state==4 || nav_state==5,"融合避障未进入后退/转向状态");
        wait(deadtime!=0); #1;
        check((pwm & deadtime)==0 && (in1 & deadtime)==0 && (in2 & deadtime)==0,
              "换向死区内仍有桥臂输出");
        repeat(6) begin @(posedge clk); #1;
            check((in1 & in2)==0,"同一电机 IN1/IN2 不得同时有效");
        end
        ir=0;

        // 保持 raw sensor_ok 但停止发布样本，freshness 到期必须撤武装。
        issue_manual(1);
        repeat(21) wait_sample(); #1;
        check(!motor_enable && !stby && !armed,"测距样本老化未 fail-safe 撤武装");
        check(fault_now[3:2]==2'b11,"测距老化未反映到健康故障位");
        publish_ranges(16'd700,16'd700); repeat(3)@(posedge clk);
        check(health[3:2]==2'b11,"新样本未恢复 freshness");
        check(!armed && !stby,"健康恢复后发生了非法自动重武装");

        pulse_arm();#1;
        check(!dut.manual_mode_o&&dut.target[0]==0,"健康故障恢复后旧手动动作被恢复");
        pulse_route(); wait_sample();
        check(armed && stby,"恢复后显式重新武装失败");
        // 在两个时钟沿之间按下急停，组合输出必须立即安全。
        @(negedge clk); #2 estop_n=0; #1;
        check(!stby && pwm==0 && in1==0 && in2==0,"急停未立即切断全部电机输出");
        estop_n=1; repeat(3)@(posedge clk);
        check(!armed && fault_latched[7],"急停未撤武装或未锁存 bit7");
        $display("INFO final health=%b now=%b latched=%b nav=%b duties=%h",
                 health,fault_now,fault_latched,nav_state,duties);
        check((^{health,fault_now,fault_latched,nav_state,duties}) !== 1'bx,
              "控制状态出现 X/Z");

        if(errors==0) $display("PASS: tb_choice3_control_core");
        else begin $display("FAIL: tb_choice3_control_core errors=%0d",errors);$fatal(1);end
        $finish;
    end

    initial begin #2_000_000;$display("FAIL: timeout");$fatal(1);end
endmodule
