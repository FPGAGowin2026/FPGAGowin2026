`timescale 1ns/1ps

// 赛题三控制与安全平面。传感器物理总线由顶层驱动，本模块仅接收原子样本。
module choice3_control_core #(
    parameter integer CLOCK_HZ = 50_000_000,
    parameter integer SAMPLE_HZ = 100,
    parameter integer PWM_PERIOD = 1000,
    parameter integer DEADTIME_CYCLES = 5000,
    parameter integer STALL_SAMPLE_LIMIT = 100,
    parameter integer RANGE_MAX_AGE_TICKS = 50,
    parameter integer NAV_LEG1_TICKS = 400,
    parameter integer NAV_TURN_TICKS = 180,
    parameter integer NAV_LEG2_TICKS = 400,
    parameter integer NAV_AVOID_BACK_SAMPLES = 20,
    parameter integer NAV_AVOID_TURN_SAMPLES = 35,
    // 摄像头、IMU、主测距、编码器为必需；备测距 bit3 可选。
    parameter [7:0] REQUIRED_HEALTH_MASK = 8'h17
) (
    input  wire clk_i, input wire rst_n_i, input wire estop_n_i,
    // [0]摄像头 [1]IMU [2]VL53 [3]超声波；编码器健康在本模块生成 bit4。
    input  wire [3:0] peripheral_health_i,
    input  wire arm_pulse_i, input wire disarm_pulse_i,
    input  wire clear_fault_pulse_i, input wire route_start_pulse_i,
    input  wire manual_command_valid_i,input wire [2:0] manual_motion_i,
    input  wire [15:0] vl53_distance_mm_i, input wire vl53_valid_i,
    input  wire [15:0] ultrasonic_distance_mm_i,
    input  wire ultrasonic_valid_i, input wire ultrasonic_timeout_i,
    input  wire [3:0] ir_active_i,
    input  wire [3:0] encoder_a_i, input wire [3:0] encoder_b_i,
    output wire [3:0] motor_pwm_o, output wire [3:0] motor_in1_o,
    output wire [3:0] motor_in2_o, output wire motor_stby_o,
    output wire armed_o, output wire motor_enable_o,
    output wire [7:0] health_o, output wire [7:0] fault_now_o,
    output wire [7:0] fault_latched_o, output wire arm_rejected_pulse_o,
    output wire [3:0] nav_state_o, output wire route_done_o,
    output wire [15:0] fused_front_mm_o, output wire fused_front_valid_o,
    output wire front_blocked_o,
    output wire [63:0] pwm_duty_flat_o,
    output wire [3:0] deadtime_active_o,
    output reg manual_mode_o,output reg [2:0] manual_motion_o,
    output wire sample_tick_o
);
    localparam integer SAMPLE_DIV_RAW = CLOCK_HZ / SAMPLE_HZ;
    localparam integer SAMPLE_DIV = (SAMPLE_DIV_RAW < 1) ? 1 : SAMPLE_DIV_RAW;
    localparam integer SAMPLE_W = (SAMPLE_DIV < 2) ? 1 : $clog2(SAMPLE_DIV);
    reg [SAMPLE_W-1:0] sample_count_q;
    reg sample_tick_q;
    assign sample_tick_o = sample_tick_q;

    wire front_slow, left_blocked, right_blocked, vl_fresh, us_fresh;
    wire [3:0] encoder_step, encoder_invalid, encoder_ready;
    wire signed [31:0] position [0:3];
    wire signed [31:0] velocity [0:3];
    wire [3:0] velocity_valid;
    wire signed [31:0] nav_target [0:3];
    reg signed [31:0] target [0:3];
    reg signed [31:0] pi_target_q [0:3];
    reg pi_enable_q;
    wire [15:0] duty [0:3];
    wire [3:0] pi_direction;
    wire encoder_health;
    wire [3:0] stall_fault;
    wire manual_hazard_stop=(manual_motion_o==1&&front_blocked_o)||
      (manual_motion_o==2&&(left_blocked||right_blocked))||
      (manual_motion_o==3&&(front_blocked_o||left_blocked))||
      (manual_motion_o==4&&(front_blocked_o||right_blocked))||
      ((manual_motion_o==5||manual_motion_o==6)&&(left_blocked||right_blocked));
    wire drive_enable=motor_enable_o&&!(manual_mode_o&&manual_hazard_stop);
    // 测距器不只要求“初始化成功”，还要求最近收到过有效样本。
    wire [7:0] health_w = {3'b111, encoder_health,
                           (peripheral_health_i[3] && us_fresh),
                           (peripheral_health_i[2] && vl_fresh),
                           peripheral_health_i[1:0]};
    assign health_o = health_w;
    assign pwm_duty_flat_o = {duty[3],duty[2],duty[1],duty[0]};

    integer target_index;
    // PI 只读取已寄存的仲裁目标/使能，切断 freshness->导航->64位PI 的长路径。
    // raw drive_enable 仍直接门控桥驱动与顶层功率输出，故健康/急停关断不增加延迟。
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin
            pi_enable_q<=1'b0;
            for(target_index=0;target_index<4;target_index=target_index+1)
                pi_target_q[target_index]<=32'sd0;
        end else begin
            pi_enable_q<=drive_enable;
            for(target_index=0;target_index<4;target_index=target_index+1)
                pi_target_q[target_index]<=target[target_index];
        end
    end

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin sample_count_q<=0; sample_tick_q<=0; end
        else begin
            sample_tick_q<=0;
            if (sample_count_q >= SAMPLE_DIV-1) begin
                sample_count_q<=0; sample_tick_q<=1;
            end else sample_count_q<=sample_count_q+1'b1;
        end
    end

    // 手动动作与自动路线互斥；任何手动命令都会让导航FSM回到IDLE。
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i)begin manual_mode_o<=0;manual_motion_o<=0;end
        else if(!armed_o||disarm_pulse_i)begin manual_mode_o<=0;manual_motion_o<=0;end
        else if(route_start_pulse_i)begin manual_mode_o<=0;manual_motion_o<=0;end
        else if(manual_command_valid_i)begin
            manual_motion_o<=manual_motion_i;manual_mode_o<=(manual_motion_i!=0);
        end
    end
    always @(manual_mode_o or manual_motion_o or front_blocked_o or front_slow or
             nav_target[0] or nav_target[1] or nav_target[2] or nav_target[3]) begin
        target[0]=nav_target[0];target[1]=nav_target[1];
        target[2]=nav_target[2];target[3]=nav_target[3];
        if(manual_mode_o)begin
            case(manual_motion_o)
                1:begin // 前进：近障碍立即停，慢区限速。
                    target[0]=front_blocked_o?0:(front_slow?100:220);
                    target[1]=target[0];target[2]=target[0];target[3]=target[0];end
                2:begin target[0]=-180;target[1]=-180;target[2]=-180;target[3]=-180;end // 后退
                3:begin target[0]=80;target[1]=80;target[2]=220;target[3]=220;end // 左弧转
                4:begin target[0]=220;target[1]=220;target[2]=80;target[3]=80;end // 右弧转
                5:begin target[0]=-180;target[1]=-180;target[2]=180;target[3]=180;end // 左原地
                6:begin target[0]=180;target[1]=180;target[2]=-180;target[3]=-180;end // 右原地
                default:begin target[0]=0;target[1]=0;target[2]=0;target[3]=0;end
            endcase
        end
    end

    choice3_range_fusion #(.MAX_AGE_TICKS(RANGE_MAX_AGE_TICKS)) u_fusion (
        .clk_i(clk_i),.rst_n_i(rst_n_i),.sample_tick_i(sample_tick_q),
        .vl53_distance_mm_i(vl53_distance_mm_i),.vl53_valid_i(vl53_valid_i),
        .vl53_sensor_ok_i(peripheral_health_i[2]),
        .ultrasonic_distance_mm_i(ultrasonic_distance_mm_i),
        .ultrasonic_valid_i(ultrasonic_valid_i),
        .ultrasonic_timeout_i(ultrasonic_timeout_i),.ir_active_i(ir_active_i),
        .fused_front_mm_o(fused_front_mm_o),.fused_front_valid_o(fused_front_valid_o),
        .front_blocked_o(front_blocked_o),.front_slow_o(front_slow),
        .left_blocked_o(left_blocked),.right_blocked_o(right_blocked),
        .vl53_fresh_o(vl_fresh),.ultrasonic_fresh_o(us_fresh));

    genvar g;
    generate for (g=0;g<4;g=g+1) begin: g_wheel
        choice3_quadrature_decoder u_decoder (
            .clk_i(clk_i),.rst_n_i(rst_n_i),.encoder_a_i(encoder_a_i[g]),
            .encoder_b_i(encoder_b_i[g]),.position_ticks_o(position[g]),
            .step_valid_o(encoder_step[g]),.invalid_transition_o(encoder_invalid[g]),
            .baseline_ready_o(encoder_ready[g]));
        encoder_velocity_estimator #(.MM_PER_TICK_Q16(32'sd6554),.SAMPLE_HZ(SAMPLE_HZ))
          u_velocity (.clk_i(clk_i),.rst_n_i(rst_n_i),.sample_tick_i(sample_tick_q),
            .position_ticks_i(position[g]),.velocity_mm_s_o(velocity[g]),
            .velocity_valid_o(velocity_valid[g]));
        wheel_speed_controller #(.KP(32'sd8),.KI(32'sd1),.GAIN_SHIFT(4),
          .PWM_MAX(PWM_PERIOD-1)) u_pi (
            .clk_i(clk_i),.rst_n_i(rst_n_i),.enable_i(pi_enable_q),
            .sample_tick_i(sample_tick_q),.target_velocity_mm_s_i(pi_target_q[g]),
            .measured_velocity_mm_s_i(velocity[g]),.pwm_duty_o(duty[g]),
            .direction_o(pi_direction[g]));
    end endgenerate

    choice3_encoder_health #(.STALL_SAMPLE_LIMIT(STALL_SAMPLE_LIMIT)) u_encoder_health (
        .clk_i(clk_i),.rst_n_i(rst_n_i),.clear_fault_pulse_i(clear_fault_pulse_i),
        .motion_enabled_i(drive_enable),.sample_tick_i(sample_tick_q),
        .baseline_ready_i(encoder_ready),.encoder_step_i(encoder_step),
        .pwm_duty_flat_i(pwm_duty_flat_o),.health_ok_o(encoder_health),
        .stall_fault_o(stall_fault));

    choice3_safety_supervisor #(.REQUIRED_HEALTH_MASK(REQUIRED_HEALTH_MASK)) u_safety (
        .clk_i(clk_i),.rst_n_i(rst_n_i),.estop_n_i(estop_n_i),
        .health_i(health_w),.arm_pulse_i(arm_pulse_i),
        .disarm_pulse_i(disarm_pulse_i),.clear_fault_pulse_i(clear_fault_pulse_i),
        .armed_o(armed_o),.motor_enable_o(motor_enable_o),
        .fault_now_o(fault_now_o),.fault_latched_o(fault_latched_o),
        .arm_rejected_pulse_o(arm_rejected_pulse_o));

    choice3_navigation_fsm #(.LEG1_TICKS(NAV_LEG1_TICKS),
      .TURN_TICKS(NAV_TURN_TICKS),.LEG2_TICKS(NAV_LEG2_TICKS),
      .AVOID_BACK_SAMPLES(NAV_AVOID_BACK_SAMPLES),
      .AVOID_TURN_SAMPLES(NAV_AVOID_TURN_SAMPLES)) u_nav (
        .clk_i(clk_i),.rst_n_i(rst_n_i),.sample_tick_i(sample_tick_q),
        .enable_i(motor_enable_o),.start_pulse_i(route_start_pulse_i),
        .stop_pulse_i(disarm_pulse_i||manual_command_valid_i),.front_blocked_i(front_blocked_o),
        .front_slow_i(front_slow),.left_blocked_i(left_blocked),
        .right_blocked_i(right_blocked),.wheel0_ticks_i(position[0]),
        .wheel1_ticks_i(position[1]),.wheel2_ticks_i(position[2]),
        .wheel3_ticks_i(position[3]),.target0_mm_s_o(nav_target[0]),
        .target1_mm_s_o(nav_target[1]),.target2_mm_s_o(nav_target[2]),
        .target3_mm_s_o(nav_target[3]),.state_o(nav_state_o),.route_done_o(route_done_o));

    choice3_tb6612_quad_bridge #(.PWM_PERIOD(PWM_PERIOD),
      .DEADTIME_CYCLES(DEADTIME_CYCLES)) u_bridge (
        .clk_i(clk_i),.rst_n_i(rst_n_i),.enable_i(drive_enable),
        .duty_flat_i(pwm_duty_flat_o),.direction_i(pi_direction),
        .pwm_o(motor_pwm_o),.in1_o(motor_in1_o),.in2_o(motor_in2_o),
        .stby_o(motor_stby_o),.deadtime_active_o(deadtime_active_o));
endmodule

