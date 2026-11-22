// ==================== motor_closed_loop_config.sv ====================
`timescale 1ns/1ps
// 两版 OpenCTR 的 R20-ABOT 麦轮参数；内部轮序 FL/FR/RL/RR。
package motor_closed_loop_config;
    localparam integer CLK_HZ = 50_000_000;
    localparam integer BAUD = 115200;
    localparam integer SAMPLE_HZ = 50;
    localparam integer WHEEL_DIAMETER_MM = 98;
    localparam integer AXLE_MM = 160;
    localparam integer TRACK_MM = 196;
    localparam integer GEOMETRY_MM = (AXLE_MM + TRACK_MM) / 2;
    // 用户要求以原 OpenCTR 的 1024 线参数实测；30:1、AB 四倍频，输出轴 122880 计数/圈。
    localparam integer ENCODER_PPR = 1024;
    localparam integer GEAR_RATIO = 30;
    localparam integer COUNTS_PER_REV = ENCODER_PPR * GEAR_RATIO * 4;
    // round(pi * WHEEL_DIAMETER_MM / COUNTS_PER_REV * 65536)，毫米/计数 Q16。
    localparam signed [31:0] MM_PER_TICK_Q16 =
        (64'd355 * WHEEL_DIAMETER_MM * 65536 + (113 * COUNTS_PER_REV)/2) /
        (113 * COUNTS_PER_REV);
    localparam integer MAX_WHEEL_MM_S = 800;
    // BX72活动导航角速留在当前未deskew CSM的0.35rad/s门限以内。
    localparam integer NAV_MAX_YAW_MRAD_S = 300;
    localparam integer STEP_MM_S = 10;
    localparam integer PWM_PERIOD = 2500;
    localparam signed [31:0] KP = 600, KD = 800;
    localparam integer PWM_BRAKE_MODE = 1;
    // 用户逐轮确认：MOTOR1=B 反转、MOTOR2=C 正转、MOTOR3=A 反转、MOTOR4=D 反转。
    // 逻辑轮序 A/B/C/D = 左上/右上/左下/右下；物理通道从 0 开始。
    localparam [1:0] MOTOR_A=2'd2, MOTOR_B=2'd0, MOTOR_C=2'd1, MOTOR_D=2'd3;
    localparam [7:0] MOTOR_CHANNEL_MAP = {MOTOR_D,MOTOR_C,MOTOR_B,MOTOR_A};
    localparam [3:0] MOTOR_REVERSE = 4'b1011;
    // 80% 悬空单轮实测：MOTOR1..4 分别只改变编码器 CH0..3。
    // 结合轮位映射，逻辑轮序 A/B/C/D 对应编码器 CH2/CH0/CH1/CH3。
    localparam [7:0] ENCODER_CHANNEL_MAP = {2'd3,2'd1,2'd0,2'd2};
    // 正常前进方向计数为正：B、D 需要反向修正。
    localparam [3:0] ENCODER_REVERSE = 4'b1010;
    localparam integer JOG_MM_S = 100;
    localparam integer JOG_MRAD_S = 300;
    localparam integer JOG_CYCLES = CLK_HZ / 1000 * 300;
    // 用户已要求 80% 点动；每条物理通道命令最多驱动 1 秒，0 随时停止。
    localparam integer CALIBRATION_CYCLES = CLK_HZ;
    localparam integer CALIBRATION_PWM_PERCENT = 80;
    localparam integer REPORT_CYCLES = CLK_HZ / 10;
    localparam integer STARTUP_CYCLES = CLK_HZ / 100;
endpackage

// ==================== mecanum_motor_control.sv ====================
`timescale 1ns/1ps
// 单时钟四轮闭环；轮序 FL/FR/RL/RR，X 型麦轮，x前/y左/wz逆时针。
// 输入须已在 clk_i 域同步；本模块不承担通信、仲裁或超时保护。
module mecanum_motor_control #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer SAMPLE_HZ = 50,
    parameter integer GEOMETRY_MM = 178, // 半轴距 + 半轮距
    parameter integer MAX_WHEEL_MM_S = motor_closed_loop_config::MAX_WHEEL_MM_S,
    parameter integer STEP_MM_S = 10, // 每采样周期各轮最大目标变化，0禁用斜坡
    parameter signed [31:0] MM_PER_TICK_Q16 = 164, // 示例，必须标定
    parameter integer PWM_PERIOD = 2500,
    parameter signed [31:0] KP = 600, KD = 800,
    parameter integer PWM_BRAKE_MODE = 1,
    parameter [3:0] ENCODER_REVERSE = 0,
    parameter [3:0] MOTOR_REVERSE = 0
)(
    input wire clk_i, rst_n_i, enable_i,
    input wire signed [31:0] vx_mm_s_i, vy_mm_s_i, wz_mrad_s_i,
    input wire [3:0] encoder_a_i, encoder_b_i,
    output wire [3:0] in1_o, in2_o,
    output wire [127:0] position_ticks_o, measured_mm_s_o,
    output wire [127:0] target_mm_s_o,
    output wire [3:0] encoder_invalid_o,
    output reg sample_valid_o
);
    localparam integer SAMPLE_CYCLES = CLK_HZ / SAMPLE_HZ;
    // 定时器只到 SAMPLE_CYCLES-1；不改变采样周期及使能暂停行为。
    localparam integer SAMPLE_TIMER_W = SAMPLE_CYCLES > 0 ?
        (SAMPLE_CYCLES <= 1 ? 1 : $clog2(SAMPLE_CYCLES)) : 32;
    reg [SAMPLE_TIMER_W-1:0] timer;
    wire sample_due = timer == SAMPLE_CYCLES-1;
    wire sample_tick;
    reg signed [31:0] target [0:3];
    wire signed [31:0] velocity [0:3];
    wire signed [31:0] encoder_count [0:3];
    wire [3:0] velocity_valid;
    wire [15:0] group_duty [0:3];
    wire [3:0] group_direction;
    generate begin: g_serial_divider
        // 按完整有符号32位命令推导边界，不局限于UART点动范围。
        localparam [63:0] RAW_BOUND = 64'd4294967296 +
            (64'd2147483648 * GEOMETRY_MM) / 1000;
        localparam integer RAW_W = $clog2(RAW_BOUND + 1) + 1;
        localparam integer ROT_PRODUCT_W = 32 + $clog2(64'd1 + GEOMETRY_MM);
        localparam integer LIMIT_PRODUCT_W = RAW_W + $clog2(64'd1 + MAX_WHEEL_MM_S);
        localparam integer SLOPE_PRODUCT_W = 33 + $clog2(64'd1 + STEP_MM_S);
        localparam integer PRODUCT_W = LIMIT_PRODUCT_W > SLOPE_PRODUCT_W ?
            LIMIT_PRODUCT_W : SLOPE_PRODUCT_W;
        // 大参数乘积保留原有模2^64回绕语义。
        localparam integer DIV_W = PRODUCT_W > 64 ? 64 :
            (PRODUCT_W > ROT_PRODUCT_W ? PRODUCT_W : ROT_PRODUCT_W);
        localparam integer DEN_W = RAW_W-1 > 32 ? RAW_W-1 : 32;
        localparam [4:0] ST_IDLE = 5'd0;
        localparam [4:0] ST_ROT_MUL = 5'd1;
        localparam [4:0] ST_ROT_PREP = 5'd2;
        localparam [4:0] ST_RAW_BASE = 5'd3;
        localparam [4:0] ST_RAW_ROT = 5'd4;
        localparam [4:0] ST_ABS_PREP = 5'd5;
        localparam [4:0] ST_PEAK_UPDATE = 5'd6;
        localparam [4:0] ST_LIMIT_PREP = 5'd7;
        localparam [4:0] ST_DIVIDE = 5'd8;
        localparam [4:0] ST_DELTA_SUB = 5'd9;
        localparam [4:0] ST_DELTA_ABS = 5'd10;
        localparam [4:0] ST_DELTA_MAX = 5'd11;
        localparam [4:0] ST_SLOPE_DECIDE = 5'd12;
        localparam [4:0] ST_SLOPE_NUM = 5'd13;
        localparam [4:0] ST_SLOPE_PREP = 5'd14;
        localparam [1:0] DIV_LIMIT = 2'd0;
        localparam [1:0] DIV_SLOPE = 2'd1;
        localparam [1:0] DIV_ROTATION = 2'd2;

        localparam [4:0] ST_LIMIT_MUL = 5'd15;
        localparam [4:0] ST_LIMIT_LOAD = 5'd16;
        localparam [4:0] ST_DIV_SIGN = 5'd17;
        localparam [4:0] ST_DIV_RESULT = 5'd18;
        localparam [4:0] ST_DIV_WRITE = 5'd19;
        reg [4:0] state;
        reg candidate_valid;
        reg signed [31:0] job_vx, job_vy, job_wz;
        reg signed [31:0] job_target [0:3];
        reg signed [RAW_W-1:0] job_raw [0:3];
        reg [RAW_W-2:0] job_peak;
        reg signed [ROT_PRODUCT_W-1:0] rotation_product;
        reg signed [RAW_W-1:0] rotation_value;
        reg signed [32:0] raw_partial;
        reg [RAW_W-2:0] job_abs [0:3];
        reg signed [31:0] work_limited [0:3];
        reg signed [31:0] candidate_target [0:3];
        reg signed [32:0] job_delta [0:3];
        reg [31:0] job_max_delta;
        reg [31:0] job_delta_abs [0:3];
        reg signed [DIV_W-1:0] slope_numerator;
        reg signed [RAW_W-1:0] limit_raw_q;
        reg signed [DIV_W-1:0] limit_numerator_q;
        reg signed [DIV_W-1:0] div_signed_q, div_result_q;
        reg [2:0] wheel_index;

        // restoring division, most-significant dividend bit first
        reg [DIV_W-1:0] div_dividend, div_quotient;
        reg [DEN_W-1:0] div_divisor;
        reg [DEN_W-1:0] div_remainder;
        reg [6:0] div_bits_left;
        reg div_negative;
        reg [1:0] div_operation;
        reg signed [DIV_W-1:0] numerator_temp;
        wire [DEN_W:0] remainder_trial = {div_remainder, div_dividend[DIV_W-1]};
        // 用减法借位完成比较，避免比较器与减法器串接。
        wire [DEN_W+1:0] remainder_difference = {1'b0, remainder_trial} - {2'b0, div_divisor};
        wire div_quotient_bit = !remainder_difference[DEN_W+1];
        wire [DEN_W-1:0] remainder_next = div_quotient_bit ?
            remainder_difference[DEN_W-1:0] : remainder_trial[DEN_W-1:0];
        wire [DIV_W-1:0] quotient_next = {div_quotient[DIV_W-2:0], div_quotient_bit};

        wire job_matches_live = job_vx == vx_mm_s_i && job_vy == vy_mm_s_i &&
            job_wz == wz_mrad_s_i && job_target[0] == target[0] &&
            job_target[1] == target[1] && job_target[2] == target[2] &&
            job_target[3] == target[3];
        wire candidate_ready = candidate_valid && job_matches_live;
        assign sample_tick = sample_due && (!enable_i || candidate_ready);

        integer j;
        always @(posedge clk_i or negedge rst_n_i) begin
            if (!rst_n_i) begin
                state <= ST_IDLE;
                candidate_valid <= 1'b0;
                job_vx <= 0; job_vy <= 0; job_wz <= 0; job_peak <= 0;
                job_max_delta <= 0; slope_numerator <= 0;
                limit_raw_q <= 0; limit_numerator_q <= 0;
                div_signed_q <= 0; div_result_q <= 0;
                rotation_product <= 0; rotation_value <= 0; raw_partial <= 0;
                wheel_index <= 0;
                div_dividend <= 0; div_divisor <= 1; div_quotient <= 0;
                div_remainder <= 0; div_bits_left <= 0;
                div_negative <= 0; div_operation <= DIV_LIMIT; numerator_temp <= 0;
                for (j=0; j<4; j=j+1) begin
                    target[j] <= 0; job_target[j] <= 0; job_raw[j] <= 0;
                    job_abs[j] <= 0; work_limited[j] <= 0; candidate_target[j] <= 0;
                    job_delta[j] <= 0; job_delta_abs[j] <= 0;
                end
            end else if (!enable_i) begin
                state <= ST_IDLE;
                candidate_valid <= 1'b0;
                for (j=0; j<4; j=j+1) target[j] <= 0;
            end else begin
                if (sample_tick) begin
                    for (j=0; j<4; j=j+1)
                        target[j] <= candidate_target[j][31:0];
                    candidate_valid <= 1'b0;
                end

                // 输入或当前目标改变时丢弃过期计算，并从新快照重新开始。
                if (state != ST_IDLE && !job_matches_live) begin
                    job_vx <= vx_mm_s_i; job_vy <= vy_mm_s_i; job_wz <= wz_mrad_s_i;
                    job_peak <= 0; job_max_delta <= 0;
                    for (j=0; j<4; j=j+1) begin
                        job_target[j] <= target[j];
                    end
                    wheel_index <= 0; candidate_valid <= 1'b0;
                    state <= ST_ROT_MUL;
                end else if (state == ST_IDLE && (!candidate_valid || !job_matches_live)) begin
                    job_vx <= vx_mm_s_i; job_vy <= vy_mm_s_i; job_wz <= wz_mrad_s_i;
                    job_peak <= 0; job_max_delta <= 0;
                    for (j=0; j<4; j=j+1) begin
                        job_target[j] <= target[j];
                    end
                    wheel_index <= 0; candidate_valid <= 1'b0;
                    state <= ST_ROT_MUL;
                end else begin
                    case (state)
                        ST_IDLE: begin end

                        // 按寄存器阶段分解 command 到 raw/peak 的宽算术路径。
                        ST_ROT_MUL: begin
                            rotation_product <= $signed(job_wz) * GEOMETRY_MM;
                            state <= ST_ROT_PREP;
                        end

                        ST_ROT_PREP: begin
                            numerator_temp = rotation_product;
                            div_negative <= numerator_temp < 0;
                            div_dividend <= numerator_temp < 0 ? -numerator_temp : numerator_temp;
                            div_divisor <= 64'd1000;
                            div_quotient <= 0; div_remainder <= 0; div_bits_left <= DIV_W-1;
                            div_operation <= DIV_ROTATION; state <= ST_DIVIDE;
                        end

                        ST_RAW_BASE: begin
                            case (wheel_index)
                                0, 3: raw_partial <=
                                    {job_vx[31], job_vx} - {job_vy[31], job_vy};
                                1, 2: raw_partial <=
                                    {job_vx[31], job_vx} + {job_vy[31], job_vy};
                                default: raw_partial <= 0;
                            endcase
                            state <= ST_RAW_ROT;
                        end

                        ST_RAW_ROT: begin
                            case (wheel_index)
                                0, 2: job_raw[wheel_index] <= raw_partial - rotation_value;
                                1, 3: job_raw[wheel_index] <= raw_partial + rotation_value;
                                default: job_raw[wheel_index] <= 0;
                            endcase
                            if (wheel_index == 3) begin
                                wheel_index <= 0; state <= ST_ABS_PREP;
                            end else begin
                                wheel_index <= wheel_index + 1'b1; state <= ST_RAW_BASE;
                            end
                        end

                        ST_ABS_PREP: begin
                            job_abs[wheel_index] <= (RAW_W-1)'(job_raw[wheel_index] < 0 ?
                                -job_raw[wheel_index] : job_raw[wheel_index]);
                            state <= ST_PEAK_UPDATE;
                        end

                        ST_PEAK_UPDATE: begin
                            if (job_abs[wheel_index] > job_peak)
                                job_peak <= job_abs[wheel_index];
                            if (wheel_index == 3) begin
                                wheel_index <= 0; state <= ST_LIMIT_PREP;
                            end else begin
                                wheel_index <= wheel_index + 1'b1; state <= ST_ABS_PREP;
                            end
                        end

                        ST_LIMIT_PREP: begin
                            if (job_peak > MAX_WHEEL_MM_S) begin
                                limit_raw_q <= job_raw[wheel_index];
                                state <= ST_LIMIT_MUL;
                            end else begin
                                work_limited[wheel_index] <= 32'(job_raw[wheel_index]);
                                if (wheel_index == 3) begin
                                    wheel_index <= 0; state <= ST_DELTA_SUB;
                                end else wheel_index <= wheel_index + 1'b1;
                            end
                        end

                        ST_LIMIT_MUL: begin
                            limit_numerator_q <= limit_raw_q * MAX_WHEEL_MM_S;
                            state <= ST_LIMIT_LOAD;
                        end

                        ST_LIMIT_LOAD: begin
                            div_negative <= limit_numerator_q < 0;
                            div_dividend <= limit_numerator_q < 0 ? -limit_numerator_q : limit_numerator_q;
                            div_divisor <= job_peak;
                            div_quotient <= 0; div_remainder <= 0; div_bits_left <= DIV_W-1;
                            div_operation <= DIV_LIMIT; state <= ST_DIVIDE;
                        end

                        ST_DIVIDE: begin
                            div_dividend <= {div_dividend[DIV_W-2:0], 1'b0};
                            div_quotient <= quotient_next;
                            div_remainder <= remainder_next;
                            if (div_bits_left == 0) begin
                                state <= ST_DIV_SIGN;
                            end else div_bits_left <= div_bits_left - 1'b1;
                        end

                        ST_DIV_SIGN: begin
                            div_signed_q <= div_negative ? -$signed(div_quotient) : $signed(div_quotient);
                            state <= ST_DIV_RESULT;
                        end

                        ST_DIV_RESULT: begin
                            div_result_q <= div_operation == DIV_SLOPE ?
                                $signed(job_target[wheel_index]) + div_signed_q : div_signed_q;
                            state <= ST_DIV_WRITE;
                        end

                        ST_DIV_WRITE: begin
                                if (div_operation == DIV_ROTATION) begin
                                    rotation_value <= RAW_W'(div_result_q);
                                    wheel_index <= 0; state <= ST_RAW_BASE;
                                end else if (div_operation == DIV_SLOPE) begin
                                    candidate_target[wheel_index] <= 32'(div_result_q);
                                    if (wheel_index == 3) begin
                                        candidate_valid <= 1'b1; state <= ST_IDLE;
                                    end else begin
                                        wheel_index <= wheel_index + 1'b1; state <= ST_SLOPE_NUM;
                                    end
                                end else begin
                                    work_limited[wheel_index] <= 32'(div_result_q);
                                    if (wheel_index == 3) begin
                                        wheel_index <= 0; state <= ST_DELTA_SUB;
                                    end else begin
                                        wheel_index <= wheel_index + 1'b1; state <= ST_LIMIT_PREP;
                                    end
                                end
                        end

                        // 将减法、绝对值、四轮最大误差比较和阈值判定分开寄存。
                        ST_DELTA_SUB: begin
                            job_delta[wheel_index] <=
                                $signed({work_limited[wheel_index][31],work_limited[wheel_index]}) -
                                $signed({job_target[wheel_index][31],job_target[wheel_index]});
                            state <= ST_DELTA_ABS;
                        end

                        ST_DELTA_ABS: begin
                            job_delta_abs[wheel_index] <= 32'(job_delta[wheel_index] < 0 ?
                                -job_delta[wheel_index] : job_delta[wheel_index]);
                            state <= ST_DELTA_MAX;
                        end

                        ST_DELTA_MAX: begin
                            if (job_delta_abs[wheel_index] > job_max_delta)
                                job_max_delta <= job_delta_abs[wheel_index];
                            if (wheel_index == 3) begin
                                wheel_index <= 0; state <= ST_SLOPE_DECIDE;
                            end else begin
                                wheel_index <= wheel_index + 1'b1; state <= ST_DELTA_SUB;
                            end
                        end

                        ST_SLOPE_DECIDE: begin
                            if (STEP_MM_S > 0 && job_max_delta > STEP_MM_S) begin
                                wheel_index <= 0; state <= ST_SLOPE_NUM;
                            end else begin
                                for (j=0; j<4; j=j+1)
                                    candidate_target[j] <= work_limited[j];
                                candidate_valid <= 1'b1; state <= ST_IDLE;
                            end
                        end

                        ST_SLOPE_NUM: begin
                            slope_numerator <= job_delta[wheel_index] * STEP_MM_S;
                            state <= ST_SLOPE_PREP;
                        end

                        ST_SLOPE_PREP: begin
                            numerator_temp = slope_numerator;
                            div_negative <= numerator_temp < 0;
                            div_dividend <= numerator_temp < 0 ? -numerator_temp : numerator_temp;
                            div_divisor <= job_max_delta;
                            div_quotient <= 0; div_remainder <= 0; div_bits_left <= DIV_W-1;
                            div_operation <= DIV_SLOPE; state <= ST_DIVIDE;
                        end

                        default: state <= ST_IDLE;
                    endcase
                end
            end
        end
    end endgenerate

    // 分母用当前目标的快照，计算赶不上采样点时保持计数器并等待结果。
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            timer <= 0;
        end else begin
            if (sample_tick) timer <= 0;
            else if (timer < SAMPLE_CYCLES-1) timer <= timer+1'b1;
        end
    end
    for (genvar g=0; g<4; g=g+1) begin: wheel
        wire signed [31:0] raw_count;
        wire signed [31:0] count = ENCODER_REVERSE[g] ? -raw_count : raw_count;
        wire [15:0] duty = group_duty[g];
        wire direction = group_direction[g];
        wire signed [15:0] effort = direction ? $signed(duty) : -$signed(duty);
        wire signed [15:0] motor_effort = MOTOR_REVERSE[g] ? -effort : effort;
        assign position_ticks_o[g*32+:32] = count;
        assign measured_mm_s_o[g*32+:32] = velocity[g];
        assign target_mm_s_o[g*32+:32] = target[g];
        quadrature_decoder encoder(.clk_i(clk_i), .rst_n_i(rst_n_i),
            .encoder_a_i(encoder_a_i[g]), .encoder_b_i(encoder_b_i[g]),
            .position_ticks_o(raw_count), .step_valid_o(), .direction_o(),
            .invalid_transition_o(encoder_invalid_o[g]));
        assign encoder_count[g] = count;
        at8236_motor_channel #(.PWM_PERIOD(PWM_PERIOD), .BRAKE_PWM(PWM_BRAKE_MODE)) driver(.clk_i(clk_i), .rst_i(!rst_n_i),
            .speed_i(motor_effort), .enable_i(enable_i && rst_n_i),
            .in1_o(in1_o[g]), .in2_o(in2_o[g]), .duty_o());
    end
    // 两组测速保持并行；四轮共用增量环，采样快照和输出按四轮原子提交。
    for (genvar group_index=0; group_index<2; group_index=group_index+1) begin: speed_group
        encoder_velocity_pair #(.MM_PER_TICK_Q16(MM_PER_TICK_Q16), .SAMPLE_HZ(SAMPLE_HZ)) estimator(
            .clk_i(clk_i), .rst_n_i(rst_n_i), .sample_tick_i(sample_tick),
            .position0_i(encoder_count[group_index*2]), .position1_i(encoder_count[group_index*2+1]),
            .velocity0_o(velocity[group_index*2]), .velocity1_o(velocity[group_index*2+1]),
            .valid_o(velocity_valid[group_index*2]));
        assign velocity_valid[group_index*2+1] = velocity_valid[group_index*2];
    end
    openctr_speed_group #(.KP(KP), .KD(KD), .PWM_MAX(PWM_PERIOD), .WHEELS(4)) group_controller(
        .clk_i(clk_i), .rst_n_i(rst_n_i), .enable_i(enable_i),
        .sample_tick_i(&velocity_valid),
        .target_i({target[3],target[2],target[1],target[0]}),
        .measured_i({velocity[3],velocity[2],velocity[1],velocity[0]}),
        .duty_o({group_duty[3],group_duty[2],group_duty[1],group_duty[0]}),
        .direction_o(group_direction));
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            sample_valid_o <= 0;
        end else begin
            sample_valid_o <= &velocity_valid;
        end
    end
    // 仿真配置检查，不推断任何板级引脚或实车标定值。
    initial begin
        if (SAMPLE_HZ <= 0 || SAMPLE_CYCLES < 512 || CLK_HZ % SAMPLE_HZ != 0 ||
            GEOMETRY_MM < 1 || GEOMETRY_MM > 100000 || MAX_WHEEL_MM_S < 1 ||
            PWM_PERIOD < 1 || PWM_PERIOD > 32767 || STEP_MM_S < 0 ||
            MM_PER_TICK_Q16 <= 0 || MM_PER_TICK_Q16 > 2147483647/SAMPLE_HZ ||
            KP < 0 || KD < 0 || (PWM_BRAKE_MODE != 0 && PWM_BRAKE_MODE != 1))
            $fatal(1, "invalid mecanum parameters");
    end
endmodule

// Integrated motor support: quadrature_decoder.v
`timescale 1ns/1ps

// AB 相输入同步与四倍频正交解码。MM_PER_TICK 等物理标定由上层速度估计器持有。
module quadrature_decoder (
    input  wire               clk_i,
    input  wire               rst_n_i,
    input  wire               encoder_a_i,
    input  wire               encoder_b_i,
    output reg signed [31:0]  position_ticks_o,
    output reg                step_valid_o,
    output reg                direction_o,
    output reg                invalid_transition_o
);
    reg [1:0] sync_a_q;
    reg [1:0] sync_b_q;
    reg [1:0] previous_ab_q;
    wire [1:0] current_ab = {sync_a_q[1], sync_b_q[1]};
    // 分字节并行产生进位和借位，保持 32 位自然回绕并缩短计数器路径。
    wire [31:0] position_ticks_increment = {
        position_ticks_o[31:24] + {7'd0, (&position_ticks_o[23:0])},
        position_ticks_o[23:16] + {7'd0, (&position_ticks_o[15:0])},
        position_ticks_o[15:8]  + {7'd0, (&position_ticks_o[7:0])},
        position_ticks_o[7:0]   + 8'd1
    };
    wire [31:0] position_ticks_decrement = {
        position_ticks_o[31:24] - {7'd0, (~|position_ticks_o[23:0])},
        position_ticks_o[23:16] - {7'd0, (~|position_ticks_o[15:0])},
        position_ticks_o[15:8]  - {7'd0, (~|position_ticks_o[7:0])},
        position_ticks_o[7:0]   - 8'd1
    };

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            sync_a_q <= 2'b00;
            sync_b_q <= 2'b00;
            previous_ab_q <= 2'b00;
            position_ticks_o <= 32'sd0;
            step_valid_o <= 1'b0;
            direction_o <= 1'b0;
            invalid_transition_o <= 1'b0;
        end else begin
            sync_a_q <= {sync_a_q[0], encoder_a_i};
            sync_b_q <= {sync_b_q[0], encoder_b_i};
            step_valid_o <= 1'b0;
            invalid_transition_o <= 1'b0;

            if (current_ab != previous_ab_q) begin
                case ({previous_ab_q, current_ab})
                    4'b0001, 4'b0111, 4'b1110, 4'b1000: begin
                        position_ticks_o <= position_ticks_increment;
                        step_valid_o <= 1'b1;
                        direction_o <= 1'b1;
                    end
                    4'b0010, 4'b1011, 4'b1101, 4'b0100: begin
                        position_ticks_o <= position_ticks_decrement;
                        step_valid_o <= 1'b1;
                        direction_o <= 1'b0;
                    end
                    default: invalid_transition_o <= 1'b1;
                endcase
                previous_ab_q <= current_ab;
            end
        end
    end
endmodule

`timescale 1ns/1ps

// 在固定 sample_tick_i 周期内用编码器增量估计轮速。
// MM_PER_TICK_Q16 和 SAMPLE_HZ 是实车必须重新标定的物理参数。
// 两轮同时采样，乘法器分两拍复用，结果同时提交。
// 采样间隔至少 512 拍，允许完整完成本次计算。
module encoder_velocity_pair #(
    parameter signed [31:0] MM_PER_TICK_Q16 = 32'sd6554,
    parameter integer SAMPLE_HZ = 100
)(
    input wire clk_i, rst_n_i, sample_tick_i,
    input wire signed [31:0] position0_i, position1_i,
    output reg signed [31:0] velocity0_o, velocity1_o,
    output reg valid_o
);
    localparam signed [31:0] SCALE = MM_PER_TICK_Q16 * SAMPLE_HZ;
    localparam signed [63:0] SCALE_EXT = SCALE;
    localparam [63:0] SCALE_MAG = SCALE_EXT < 0 ? -SCALE_EXT : SCALE_EXT;
    localparam integer SCALED_W = 32 + $clog2(SCALE_MAG + 1);
    reg signed [31:0] previous0, previous1, delta0, delta1;
    reg signed [SCALED_W-1:0] scaled;
    reg signed [31:0] result0;
    reg stage1, stage2, stage3;
    wire signed [63:0] extended = scaled;
    wire signed [63:0] toward_zero = extended < 0 ? extended + 64'sd65535 : extended;
    wire signed [31:0] result = scaled > 64'sh0000_7fff_ffff_0000 ? 32'sh7fff_ffff :
        scaled < 64'shffff_8000_0000_0000 ? 32'sh8000_0000 : $signed(toward_zero[47:16]);
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            previous0 <= 0; previous1 <= 0; delta0 <= 0; delta1 <= 0;
            scaled <= 0; result0 <= 0; velocity0_o <= 0; velocity1_o <= 0;
            stage1 <= 0; stage2 <= 0; stage3 <= 0; valid_o <= 0;
        end else begin
            stage1 <= sample_tick_i; stage2 <= stage1; stage3 <= stage2;
            valid_o <= stage3;
            if (sample_tick_i) begin
                delta0 <= position0_i - previous0; delta1 <= position1_i - previous1;
                previous0 <= position0_i; previous1 <= position1_i;
            end
            if (stage1) delta0 <= delta1;
            if (stage1 || stage2) scaled <= delta0 * SCALE;
            if (stage2) result0 <= result;
            if (stage3) begin velocity0_o <= result0; velocity1_o <= result; end
        end
    end
endmodule

// Integrated motor support: wheel_speed_controller.v
`timescale 1ns/1ps

// 目标/反馈同时锁存，全部轮算完后原子提交结果，历史按 channel 分开。
module openctr_speed_group #(
    parameter signed [31:0] KP=600, KD=800,
    parameter integer PWM_MAX=2500,
    parameter integer WHEELS=2
)(
    input wire clk_i, rst_n_i, enable_i, sample_tick_i,
    input wire [WHEELS*32-1:0] target_i, measured_i,
    output reg [WHEELS*16-1:0] duty_o,
    output reg [WHEELS-1:0] direction_o
);
    localparam integer CHANNEL_W = WHEELS <= 1 ? 1 : $clog2(WHEELS);
    localparam [1:0] IDLE=0, ISSUE=1, WAIT_RESULT=2;
    reg [1:0] state;
    reg [CHANNEL_W-1:0] channel;
    reg [WHEELS*32-1:0] target_q, measured_q;
    reg [WHEELS*16-1:0] candidate_duty;
    reg [WHEELS-1:0] candidate_direction;
    reg [WHEELS*16-1:0] completed_duty;
    reg [WHEELS-1:0] completed_direction;
    wire launch = state==ISSUE;
    wire [15:0] engine_duty;
    wire engine_direction, done;
    wheel_speed_controller #(.KP(KP),.KD(KD),.PWM_MAX(PWM_MAX),.CHANNELS(WHEELS)) engine(
        .clk_i(clk_i),.rst_n_i(rst_n_i),.enable_i(enable_i),.sample_tick_i(launch),
        .channel_i(channel),.target_velocity_mm_s_i($signed(target_q[channel*32+:32])),
        .measured_velocity_mm_s_i($signed(measured_q[channel*32+:32])),
        .pwm_duty_o(engine_duty),.direction_o(engine_direction),.result_valid_o(done));
    always @* begin
        completed_duty=candidate_duty;
        completed_direction=candidate_direction;
        completed_duty[channel*16+:16]=engine_duty;
        completed_direction[channel]=engine_direction;
    end
    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin
            state<=IDLE; channel<=0; target_q<=0; measured_q<=0; candidate_duty<=0;
            candidate_direction<='1; duty_o<=0; direction_o<='1;
        end else if(!enable_i) begin
            state<=IDLE; channel<=0; duty_o<=0; direction_o<='1;
        end else begin
            case(state)
                IDLE: if(sample_tick_i) begin
                    target_q<=target_i; measured_q<=measured_i; channel<=0; state<=ISSUE;
                end
                ISSUE: state<=WAIT_RESULT;
                WAIT_RESULT: if(done) begin
                    candidate_duty<=completed_duty; candidate_direction<=completed_direction;
                    if(channel==WHEELS-1) begin
                        duty_o<=completed_duty; direction_o<=completed_direction; state<=IDLE;
                    end else begin channel<=channel+1'b1; state<=ISSUE; end
                end
                default: state<=IDLE;
            endcase
        end
    end
endmodule

// OpenCTR ax_speed.c 增量环：u += 600*e + 800*(e-e_prev)，e 的单位为 m/s。
// 输入为 mm/s，内部保留原始 ±4200 PWM 量纲；先更新并截断，再限幅。
// 与原 C 一样向零截断。两个除法复用逐拍计算，默认参数单轮 74 拍完成；禁用清历史。
module wheel_speed_controller #(
    parameter signed [31:0] KP = 600, KD = 800,
    parameter integer PWM_MAX = 2500,
    parameter integer CHANNELS = 1
) (
    input wire clk_i, rst_n_i, enable_i, sample_tick_i,
    input wire [(CHANNELS<=1 ? 1 : $clog2(CHANNELS))-1:0] channel_i,
    input wire signed [31:0] target_velocity_mm_s_i, measured_velocity_mm_s_i,
    output reg [15:0] pwm_duty_o,
    output reg direction_o,
    output reg result_valid_o
);
    localparam [3:0] IDLE=0, TERMS=1, SUM=2, PREP=3, DIVIDE=4,
                     MAP=5, MAP_ABS=6, MAP_SCALE=7, MAP_LOAD=8;
    reg [3:0] state;
    localparam signed [63:0] KP_EXT = KP, KD_EXT = KD;
    localparam [63:0] KP_MAG = KP_EXT < 0 ? -KP_EXT : KP_EXT;
    localparam [63:0] KD_MAG = KD_EXT < 0 ? -KD_EXT : KD_EXT;
    localparam integer P_BITS = 33 + $clog2(KP_MAG + 1);
    localparam integer D_BITS = 34 + $clog2(KD_MAG + 1);
    localparam integer P_W = P_BITS > 64 ? 64 : P_BITS;
    localparam integer D_W = D_BITS > 64 ? 64 : D_BITS;
    localparam integer TERM_W = P_W > D_W ? P_W : D_W;
    localparam integer NUM_W = TERM_W+2 > 64 || PWM_MAX < 1 || PWM_MAX > 32767 ? 64 : TERM_W+2;
    localparam integer MAP_BITS = PWM_MAX >= 1 && PWM_MAX <= 32767 ?
        $clog2(64'd4200 * PWM_MAX + 1) : NUM_W;
    reg signed [32:0] error_q;
    reg signed [32:0] previous_error_q [0:CHANNELS-1];
    reg [(CHANNELS<=1 ? 1 : $clog2(CHANNELS))-1:0] channel_q;
    reg signed [P_W-1:0] proportional_q;
    reg signed [D_W-1:0] differential_q;
    reg signed [23:0] accumulator_q;
    reg signed [NUM_W-1:0] numerator_q;
    wire signed [33:0] error_delta = $signed(error_q) - $signed(previous_error_q[channel_q]);
    reg signed [13:0] effort_q [0:CHANNELS-1];
    // Constant-index registers avoid inferring a RAM for asynchronously reset history.
    generate for (genvar previous_index=0; previous_index<CHANNELS; previous_index=previous_index+1) begin : g_previous_error_register
        always @(posedge clk_i or negedge rst_n_i) begin
            if (!rst_n_i) previous_error_q[previous_index] <= 0;
            else if (!enable_i) previous_error_q[previous_index] <= 0;
            else if (state == TERMS && channel_q == previous_index) previous_error_q[previous_index] <= error_q;
        end
    end endgenerate
    reg [NUM_W-1:0] dividend, quotient;
    reg [12:0] divisor, remainder;
    reg [6:0] bits_left;
    reg negative, output_division;
    reg signed [13:0] map_effort_q;
    reg [12:0] map_magnitude_q;
    reg [NUM_W-1:0] map_scaled_q;
    wire [13:0] trial = {remainder, dividend[NUM_W-1]};
    wire quotient_bit = trial >= {1'b0,divisor};
    wire [13:0] remainder_next = quotient_bit ? trial - {1'b0,divisor} : trial;
    wire [NUM_W-1:0] quotient_next = {quotient[NUM_W-2:0],quotient_bit};
    // 14 位余数减法器；NUM_W 拍算增量商，MAP_BITS 拍算占空比，均向零截断。
    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            state <= IDLE; error_q <= 0; channel_q <= 0; result_valid_o <= 0;
            for(integer ch=0;ch<CHANNELS;ch=ch+1) begin effort_q[ch]<=0; end
            proportional_q <= 0; differential_q <= 0; accumulator_q <= 0; numerator_q <= 0;
            dividend <= 0; quotient <= 0; divisor <= 1; remainder <= 0;
            bits_left <= 0; negative <= 0; output_division <= 0;
            map_effort_q <= 0; map_magnitude_q <= 0; map_scaled_q <= 0;
            pwm_duty_o <= 0; direction_o <= 1;
        end else if (!enable_i) begin
            state <= IDLE; result_valid_o <= 0;
            for(integer ch=0;ch<CHANNELS;ch=ch+1) begin effort_q[ch]<=0; end
            pwm_duty_o <= 0; direction_o <= 1;
        end else begin
            result_valid_o <= 0;
            case(state)
                IDLE: if(sample_tick_i) begin
                    channel_q <= CHANNELS==1 ? 1'b0 : channel_i;
                    error_q <= {target_velocity_mm_s_i[31],target_velocity_mm_s_i} -
                               {measured_velocity_mm_s_i[31],measured_velocity_mm_s_i};
                    state <= TERMS;
                end
                TERMS: begin
                    proportional_q <= KP*error_q;
                    differential_q <= KD*error_delta;
                    accumulator_q <= effort_q[channel_q]*24'sd1000;
                    state <= SUM;
                end
                SUM: begin
                    numerator_q <= accumulator_q + proportional_q + differential_q;
                    state <= PREP;
                end
                PREP: begin
                    negative <= numerator_q < 0;
                    dividend <= numerator_q < 0 ? -numerator_q : numerator_q;
                    divisor <= 1000; quotient <= 0; remainder <= 0;
                    bits_left <= NUM_W-1; output_division <= 0; state <= DIVIDE;
                end
                DIVIDE: begin
                    dividend <= {dividend[NUM_W-2:0],1'b0};
                    quotient <= quotient_next;
                    remainder <= remainder_next[12:0];
                    if(bits_left==0) begin
                        if(output_division) begin
                            pwm_duty_o <= quotient_next[15:0];
                            result_valid_o <= 1;
                            state <= IDLE;
                        end else begin
                            if(quotient_next > 4200) effort_q[channel_q] <= negative ? -14'sd4200 : 14'sd4200;
                            else effort_q[channel_q] <= negative ? -$signed(quotient_next[13:0]) : $signed(quotient_next[13:0]);
                            state <= MAP;
                        end
                    end else bits_left <= bits_left-1'b1;
                end
                MAP: begin
                    map_effort_q <= effort_q[channel_q];
                    state <= MAP_ABS;
                end
                MAP_ABS: begin
                    direction_o <= !map_effort_q[13];
                    map_magnitude_q <= 13'(map_effort_q[13] ? -map_effort_q : map_effort_q);
                    state <= MAP_SCALE;
                end
                MAP_SCALE: begin
                    map_scaled_q <= NUM_W'(map_magnitude_q) * NUM_W'(PWM_MAX);
                    state <= MAP_LOAD;
                end
                MAP_LOAD: begin
                    dividend <= NUM_W'(map_scaled_q << (NUM_W-MAP_BITS));
                    divisor <= 4200; quotient <= 0; remainder <= 0;
                    bits_left <= MAP_BITS-1; output_division <= 1; state <= DIVIDE;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule

// Integrated motor support: at8236_motor_channel.sv
`timescale 1ns/1ps

// One AT8236 board/channel.
// speed_i is signed in the range -1000..+1000.
// BRAKE_PWM=1 与 OpenCTR 相同：正向 IN1=1 / IN2=互补PWM，反向互换。
// BRAKE_PWM=0 使用单边 PWM；禁用或复位始终输出 00。
module at8236_motor_channel #(
    parameter integer PWM_PERIOD = 1000,
    parameter integer BRAKE_PWM = 1
) (
    input  wire              clk_i,
    input  wire              rst_i,
    input  wire signed [15:0] speed_i,
    input  wire               enable_i,
    output reg                in1_o,
    output reg                in2_o,
    output reg [15:0]         duty_o
);
    reg [15:0] pwm_count;
    reg [15:0] magnitude;

    always @* begin
        if (speed_i[15])
            magnitude = (~speed_i[15:0]) + 16'd1;
        else
            magnitude = speed_i[15:0];

        if (magnitude > PWM_PERIOD)
            duty_o = PWM_PERIOD;
        else
            duty_o = magnitude;

        in1_o = 1'b0;
        in2_o = 1'b0;
        if (enable_i && !rst_i) begin
            if (BRAKE_PWM) begin
                if (speed_i[15]) begin
                    in2_o = 1'b1; in1_o = !(pwm_count < duty_o);
                end else begin
                    in1_o = 1'b1; in2_o = !(pwm_count < duty_o);
                end
            end else if (speed_i != 0) begin
                if (speed_i[15]) in2_o = (pwm_count < duty_o);
                else in1_o = (pwm_count < duty_o);
            end
        end
    end

    always @(posedge clk_i) begin
        if (rst_i) begin
            pwm_count <= 16'd0;
        end else if (pwm_count >= PWM_PERIOD - 1) begin
            pwm_count <= 16'd0;
        end else begin
            pwm_count <= pwm_count + 1'b1;
        end
    end
endmodule
