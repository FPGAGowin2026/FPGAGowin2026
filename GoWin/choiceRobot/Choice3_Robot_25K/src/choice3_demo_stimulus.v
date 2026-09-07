`timescale 1ns/1ps

// 可综合片上场景：用真实控制器 PWM/方向闭合简化电机模型，并注入慢行、停车和绕障。
// 该模块只服务无外设板测；顶层在 DEMO 模式强制真实电机管脚保持禁能。
module choice3_demo_stimulus #(
    parameter integer ENERGY_PER_STEP = 1600,
    // 与默认 400 tick 第一航段匹配：先在约 60 ms 进入慢行，
    // 再注入硬障碍，确保实板自测覆盖减速、停车和绕行，而非先到达航点。
    parameter integer OBSTACLE_SLOW_START = 6,
    parameter integer OBSTACLE_BLOCK_START = 10,
    parameter integer OBSTACLE_BLOCK_END = 14
) (
    input  wire clk_i,
    input  wire rst_n_i,
    input  wire sample_tick_i,
    input  wire [3:0] motor_pwm_i,
    input  wire [3:0] motor_in1_i,
    input  wire [3:0] motor_in2_i,
    input  wire [3:0] nav_state_i,
    input  wire route_done_i,
    input  wire video_frame_start_i,
    output reg  [3:0] encoder_a_o,
    output reg  [3:0] encoder_b_o,
    output reg  [15:0] vl53_distance_mm_o,
    output reg         vl53_valid_o,
    output reg  [15:0] ultrasonic_distance_mm_o,
    output reg         ultrasonic_valid_o,
    output wire        ultrasonic_timeout_o,
    output reg  [3:0]  ir_active_o,
    output reg  signed [15:0] roll_deg_o,
    output reg  signed [15:0] pitch_deg_o,
    output reg  signed [15:0] yaw_deg_o,
    output reg         attitude_valid_o,
    output reg  [6:0]  pass_mask_o
);
    reg [11:0] pwm_energy_q [0:3];
    reg [1:0] phase_q [0:3];
    reg [7:0] leg1_samples_q;
    integer i;

    function [1:0] phase_forward;
        input [1:0] phase;
        begin case(phase) 2'b00:phase_forward=2'b01;2'b01:phase_forward=2'b11;
              2'b11:phase_forward=2'b10;default:phase_forward=2'b00;endcase end
    endfunction
    function [1:0] phase_reverse;
        input [1:0] phase;
        begin case(phase) 2'b00:phase_reverse=2'b10;2'b10:phase_reverse=2'b11;
              2'b11:phase_reverse=2'b01;default:phase_reverse=2'b00;endcase end
    endfunction

    assign ultrasonic_timeout_o=1'b0;

    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin
            encoder_a_o<=0;encoder_b_o<=0;vl53_distance_mm_o<=16'd800;
            vl53_valid_o<=0;ultrasonic_distance_mm_o<=16'd820;
            ultrasonic_valid_o<=0;ir_active_o<=0;roll_deg_o<=0;
            pitch_deg_o<=0;yaw_deg_o<=0;attitude_valid_o<=0;
            pass_mask_o<=0;leg1_samples_q<=0;
            for(i=0;i<4;i=i+1)begin pwm_energy_q[i]<=0;phase_q[i]<=0;end
        end else begin
            vl53_valid_o<=0;ultrasonic_valid_o<=0;attitude_valid_o<=0;
            if(video_frame_start_i)pass_mask_o[0]<=1;
            for(i=0;i<4;i=i+1)begin
                if(motor_pwm_i[i] && (motor_in1_i[i]^motor_in2_i[i]))begin
                    if(pwm_energy_q[i]>=ENERGY_PER_STEP-1)begin
                        pwm_energy_q[i]<=0;
                        phase_q[i]<=motor_in1_i[i]?phase_forward(phase_q[i]):phase_reverse(phase_q[i]);
                        pass_mask_o[4]<=1;
                    end else pwm_energy_q[i]<=pwm_energy_q[i]+1'b1;
                end
                encoder_a_o[i]<=phase_q[i][1];
                encoder_b_o[i]<=phase_q[i][0];
            end
            if(sample_tick_i)begin
                vl53_valid_o<=1;ultrasonic_valid_o<=1;attitude_valid_o<=1;
                pass_mask_o[1]<=1;
                roll_deg_o<=roll_deg_o>=15? -16'sd15:roll_deg_o+1'b1;
                pitch_deg_o<=pitch_deg_o<=-10?16'sd10:pitch_deg_o-1'b1;
                if(nav_state_i==4'd2||nav_state_i==4'd5)yaw_deg_o<=yaw_deg_o+16'sd3;
                if(nav_state_i==4'd1)begin
                    leg1_samples_q<=leg1_samples_q+1'b1;
                    if(leg1_samples_q>=OBSTACLE_SLOW_START && leg1_samples_q<OBSTACLE_BLOCK_START)begin
                        vl53_distance_mm_o<=16'd350;ultrasonic_distance_mm_o<=16'd370;
                        pass_mask_o[2]<=1;
                    end else if(leg1_samples_q>=OBSTACLE_BLOCK_START && leg1_samples_q<OBSTACLE_BLOCK_END)begin
                        vl53_distance_mm_o<=16'd180;ultrasonic_distance_mm_o<=16'd190;
                        pass_mask_o[3]<=1;
                    end else begin
                        vl53_distance_mm_o<=16'd800;ultrasonic_distance_mm_o<=16'd820;
                    end
                end else begin
                    vl53_distance_mm_o<=16'd800;ultrasonic_distance_mm_o<=16'd820;
                end
                if(nav_state_i==4'd4||nav_state_i==4'd5)pass_mask_o[5]<=1;
                if(route_done_i)pass_mask_o[6]<=1;
            end
        end
    end
endmodule
