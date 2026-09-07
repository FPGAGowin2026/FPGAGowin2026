`timescale 1ns/1ps
module tb_choice3_demo_system;
    reg clk=0,rst_n=0;
    reg wb_cyc=0,wb_stb=0,wb_we=0;reg [31:0] wb_adr=0,wb_wdata=0;reg [3:0] wb_sel=4'hf;
    wire [31:0] wb_rdata;wire wb_ack;
    wire heartbeat_write,arm_cmd,disarm_cmd,route_cmd,clear_cmd,manual_valid;
    wire [2:0] manual_motion;wire [31:0] heartbeat,scratch,txn_count,error_count;
    wire cpu_alive,cpu_fault,allow_commit,allow_start,path_abort;
    wire [3:0] pwm,in1,in2,deadtime,enc_a,enc_b,ir;
    wire stby,armed,motor_enable,arm_rejected,route_done,front_blocked,fused_valid,sample_tick;
    wire [7:0] health,fault_now,fault_latched;wire [3:0] nav_state;
    wire [15:0] fused_mm,primary_mm,secondary_mm;wire primary_valid,secondary_valid,secondary_timeout;
    wire [63:0] duty_flat;wire signed [15:0] roll_deg,pitch_deg,yaw_deg;wire attitude_valid;
    reg video_frame_start=0;wire [6:0] pass_low;wire [7:0] demo_pass={1'b1,pass_low};
    integer errors=0;reg [31:0] readback;
    always #5 clk=~clk;
    task check(input bit cond,input string msg);if(!cond)begin $display("FAIL: %s",msg);errors=errors+1;end endtask
    task wb_write(input [31:0] address,input [31:0] data);
      begin @(negedge clk);wb_adr=address;wb_wdata=data;wb_we=1;wb_cyc=1;wb_stb=1;
        while(!wb_ack)@(negedge clk);@(negedge clk);wb_cyc=0;wb_stb=0;wb_we=0;end endtask
    task wb_read(input [31:0] address,output [31:0] data);
      begin @(negedge clk);wb_adr=address;wb_we=0;wb_cyc=1;wb_stb=1;
        while(!wb_ack)@(negedge clk);data=wb_rdata;@(negedge clk);wb_cyc=0;wb_stb=0;end endtask

    choice3_robot_service_regs u_regs(.wb_clk_i(clk),.wb_rst_i(!rst_n),
      .wb_cyc_i(wb_cyc),.wb_stb_i(wb_stb),.wb_we_i(wb_we),.wb_adr_i(wb_adr),
      .wb_dat_i(wb_wdata),.wb_sel_i(wb_sel),.wb_dat_o(wb_rdata),.wb_ack_o(wb_ack),
      .health_i(health),.fault_i(fault_now|fault_latched),.nav_state_i(nav_state),
      .route_done_i(route_done),.drive_enabled_i(stby),.armed_i(armed),.demo_mode_i(1'b1),
      .cpu_watchdog_fault_i(cpu_fault),.fused_range_mm_i(fused_mm),
      .fused_range_valid_i(fused_valid),.front_blocked_i(front_blocked),
      .roll_deg_i(roll_deg),.pitch_deg_i(pitch_deg),.yaw_deg_i(yaw_deg),
      .demo_pass_i(demo_pass),.heartbeat_o(heartbeat),.heartbeat_write_o(heartbeat_write),
      .arm_pulse_o(arm_cmd),.disarm_pulse_o(disarm_cmd),.route_start_pulse_o(route_cmd),
      .clear_fault_pulse_o(clear_cmd),.manual_command_valid_o(manual_valid),
      .manual_motion_o(manual_motion),.scratch_o(scratch),
      .transaction_count_o(txn_count),.error_count_o(error_count));
    softcore_service_watchdog #(.TIMEOUT_CYCLES(20000)) u_watchdog(.clk_i(clk),
      .safety_reset_n_i(rst_n),.heartbeat_write_i(heartbeat_write),.alive_o(cpu_alive),
      .fault_o(cpu_fault),.allow_commit_o(allow_commit),.allow_start_o(allow_start),
      .path_abort_o(path_abort));
    choice3_control_core #(.CLOCK_HZ(100_000),.SAMPLE_HZ(1_000),.PWM_PERIOD(100),
      .DEADTIME_CYCLES(2),.STALL_SAMPLE_LIMIT(30),.NAV_LEG1_TICKS(20),
      .NAV_TURN_TICKS(10),.NAV_LEG2_TICKS(20),.NAV_AVOID_BACK_SAMPLES(3),
      .NAV_AVOID_TURN_SAMPLES(4),.REQUIRED_HEALTH_MASK(8'h17)) u_control(
      .clk_i(clk),.rst_n_i(rst_n),.estop_n_i(cpu_alive),.peripheral_health_i(4'hf),
      .arm_pulse_i(arm_cmd),.disarm_pulse_i(disarm_cmd|path_abort),
      .clear_fault_pulse_i(clear_cmd),.route_start_pulse_i(route_cmd&allow_start),
      .manual_command_valid_i(manual_valid),.manual_motion_i(manual_motion),
      .vl53_distance_mm_i(primary_mm),.vl53_valid_i(primary_valid),
      .ultrasonic_distance_mm_i(secondary_mm),.ultrasonic_valid_i(secondary_valid),
      .ultrasonic_timeout_i(secondary_timeout),.ir_active_i(ir),.encoder_a_i(enc_a),
      .encoder_b_i(enc_b),.motor_pwm_o(pwm),.motor_in1_o(in1),.motor_in2_o(in2),
      .motor_stby_o(stby),.armed_o(armed),.motor_enable_o(motor_enable),
      .health_o(health),.fault_now_o(fault_now),.fault_latched_o(fault_latched),
      .arm_rejected_pulse_o(arm_rejected),.nav_state_o(nav_state),.route_done_o(route_done),
      .fused_front_mm_o(fused_mm),.fused_front_valid_o(fused_valid),
      .front_blocked_o(front_blocked),.pwm_duty_flat_o(duty_flat),
      .deadtime_active_o(deadtime),.manual_mode_o(),.manual_motion_o(),.sample_tick_o(sample_tick));
    choice3_demo_stimulus #(.ENERGY_PER_STEP(50),.OBSTACLE_SLOW_START(3),
      .OBSTACLE_BLOCK_START(5),.OBSTACLE_BLOCK_END(7)) u_demo(.clk_i(clk),.rst_n_i(rst_n),
      .sample_tick_i(sample_tick),.motor_pwm_i(pwm),.motor_in1_i(in1),.motor_in2_i(in2),
      .nav_state_i(nav_state),.route_done_i(route_done),.video_frame_start_i(video_frame_start),
      .encoder_a_o(enc_a),.encoder_b_o(enc_b),.vl53_distance_mm_o(primary_mm),
      .vl53_valid_o(primary_valid),.ultrasonic_distance_mm_o(secondary_mm),
      .ultrasonic_valid_o(secondary_valid),.ultrasonic_timeout_o(secondary_timeout),
      .ir_active_o(ir),.roll_deg_o(roll_deg),.pitch_deg_o(pitch_deg),.yaw_deg_o(yaw_deg),
      .attitude_valid_o(attitude_valid),.pass_mask_o(pass_low));

    initial begin
      repeat(5)@(posedge clk);rst_n=1;@(negedge clk);video_frame_start=1;@(negedge clk);video_frame_start=0;
      repeat(500)@(posedge clk);
      wb_write(32'h2000_000c,32'h1);repeat(5)@(posedge clk);check(cpu_alive,"CPU 心跳未建立");
      wb_write(32'h2000_2000,32'h1);repeat(5)@(posedge clk);check(armed,"健康 DEMO 未武装");
      wb_write(32'h2000_2000,32'h4);
      fork : wait_route
        begin repeat(15000)@(posedge clk);$display("FAIL: demo route timeout nav=%0d pass=%h health=%h fault=%h duty=%h enc=%b%b",
          nav_state,demo_pass,health,fault_now,duty_flat,enc_a,enc_b);$fatal(1);end
        begin wait(demo_pass==8'hff);disable wait_route;end
      join
      check(route_done&&nav_state==6,"固定航点路线未完成");
      check(pass_low[2]&&pass_low[3]&&pass_low[5],"慢行/硬停/绕障覆盖不完整");
      wb_read(32'h2000_1010,readback);check(readback==32'hff,"软核 DEMO 证据寄存器错误");
      wb_read(32'h2000_1008,readback);check((^readback)!==1'bx,"姿态快照含 X");
      if(errors==0)$display("PASS: tb_choice3_demo_system");
      else begin $display("FAIL: tb_choice3_demo_system errors=%0d",errors);$fatal(1);end
      $finish;
    end
endmodule
