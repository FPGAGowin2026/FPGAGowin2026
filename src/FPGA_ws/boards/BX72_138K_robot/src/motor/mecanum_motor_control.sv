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
    // 图片的 13 线规格与历史原始计数不一致，最终物理标定仍须输出轴整圈核验。
    localparam integer ENCODER_PPR = 1024;
    localparam integer GEAR_RATIO = 30;
    localparam integer COUNTS_PER_REV = ENCODER_PPR * GEAR_RATIO * 4;
    // round(pi * WHEEL_DIAMETER_MM / COUNTS_PER_REV * 65536)，毫米/计数 Q16。
    localparam signed [31:0] MM_PER_TICK_Q16 =
        (64'd355 * WHEEL_DIAMETER_MM * 65536 + (113 * COUNTS_PER_REV)/2) /
        (113 * COUNTS_PER_REV);
    localparam integer MAX_WHEEL_MM_S = 500;
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
    parameter integer MAX_WHEEL_MM_S = 500,
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
            for(integer ch=0;ch<CHANNELS;ch=ch+1) begin previous_error_q[ch]<=0; effort_q[ch]<=0; end
            proportional_q <= 0; differential_q <= 0; accumulator_q <= 0; numerator_q <= 0;
            dividend <= 0; quotient <= 0; divisor <= 1; remainder <= 0;
            bits_left <= 0; negative <= 0; output_division <= 0;
            map_effort_q <= 0; map_magnitude_q <= 0; map_scaled_q <= 0;
            pwm_duty_o <= 0; direction_o <= 1;
        end else if (!enable_i) begin
            state <= IDLE; result_valid_o <= 0;
            for(integer ch=0;ch<CHANNELS;ch=ch+1) begin previous_error_q[ch]<=0; effort_q[ch]<=0; end
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
                    previous_error_q[channel_q] <= error_q;
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

// ==================== mecanum_motor_closed_loop_core.sv ====================
`timescale 1ns/1ps
// 端口通道顺序由参数映射；闭环内部 bit0..3 固定为 FL/FR/RL/RR。
module mecanum_motor_closed_loop_core #(
    parameter integer ENABLE_DEBUG_UART = 1,
    // 仅供已在同一时钟域完成范围、时效与近障保护的板内调用。
    parameter integer TRUSTED_GUARDED_INPUT = 0,
    parameter integer CLK_HZ = motor_closed_loop_config::CLK_HZ,
    parameter integer BAUD = motor_closed_loop_config::BAUD,
    parameter integer SAMPLE_HZ = motor_closed_loop_config::SAMPLE_HZ,
    parameter integer PWM_PERIOD = motor_closed_loop_config::PWM_PERIOD,
    parameter integer JOG_CYCLES = CLK_HZ / 1000 * 300,
    parameter integer REMOTE_TIMEOUT_CYCLES = CLK_HZ / 1000 * 300,
    parameter integer REPORT_CYCLES = CLK_HZ / 10,
    parameter integer STARTUP_CYCLES = CLK_HZ / 100,
    parameter integer CALIBRATION_CYCLES = CLK_HZ,
    parameter signed [31:0] MM_PER_TICK_Q16 = motor_closed_loop_config::MM_PER_TICK_Q16,
    parameter [7:0] MOTOR_CHANNEL_MAP = motor_closed_loop_config::MOTOR_CHANNEL_MAP,
    parameter [7:0] ENCODER_CHANNEL_MAP = motor_closed_loop_config::ENCODER_CHANNEL_MAP,
    parameter [3:0] MOTOR_REVERSE = motor_closed_loop_config::MOTOR_REVERSE,
    parameter [3:0] ENCODER_REVERSE = motor_closed_loop_config::ENCODER_REVERSE
)(
    input wire clk_50m_i, user_reset_i, debug_uart_rx_i,
    input wire [3:0] encoder_a_i, encoder_b_i,
    input wire remote_valid_i, remote_link_down_i,
    input wire [47:0] remote_velocity_i,
    output wire debug_uart_tx_o,
    output wire [3:0] motor_in1_o, motor_in2_o,
    // 只导出已有测速，不改变速度环或电机串口协议。
    output wire odometry_valid_o,
    output wire signed [31:0] odometry_vx_o,odometry_vy_o,odometry_wz_o,
    output wire [127:0] odometry_ticks_o,
    output wire [3:0] odometry_invalid_o,
    output reg telemetry_valid_o,
    output reg [703:0] telemetry_payload_o
);
    reg [31:0] startup = 0;
    wire por = startup < STARTUP_CYCLES;
    always @(posedge clk_50m_i) if (por) startup <= startup + 1;
    (* ASYNC_REG="TRUE" *) reg [1:0] reset_sync = 2'b11;
    always @(posedge clk_50m_i or posedge user_reset_i)
        if (user_reset_i) reset_sync <= 2'b11;
        else reset_sync <= {reset_sync[0], por};
    wire rst = reset_sync[1];
    wire [7:0] rx_data;
    wire rx_valid;
    reg physical_report_mode;
    generate if(ENABLE_DEBUG_UART)begin : gen_debug_rx
    (* ASYNC_REG="TRUE" *) reg [1:0] rx_sync = 2'b11;
    always @(posedge clk_50m_i) rx_sync <= {rx_sync[0], debug_uart_rx_i};
    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) rx(
        .clk_i(clk_50m_i), .rst_i(rst), .rx_i(rx_sync[1]), .data_o(rx_data), .valid_o(rx_valid));
    end else begin : gen_no_debug_rx
        assign rx_data=8'd0;
        assign rx_valid=1'b0;
    end endgenerate

    reg [31:0] remaining;
    // 按 remaining 的同一覆盖顺序同步保持非零状态，缩短控制使能扇出路径。
    reg remaining_active_q;
    reg [7:0] command;
    reg remote_mode, remote_armed;
    reg signed [31:0] remote_vx,remote_vy,remote_wz;
    // 再锁存保护器的速度/valid，范围检查独占下一拍，避免串到租约寄存器。
    // 零速旁路保留立即停车与重新解锁的语义。
    reg remote_pending_valid;
    reg [47:0] remote_pending_velocity;
    reg remote_checked_valid;
    reg [47:0] remote_checked_velocity;
    (* ASYNC_REG="TRUE" *) reg [1:0] remote_link_sync;
    always @(posedge clk_50m_i) remote_link_sync <= {remote_link_sync[0],remote_link_down_i};
    wire remote_zero_now=remote_velocity_i==48'd0;
    wire signed [15:0] pending_vx=remote_pending_velocity[47:32];
    wire signed [15:0] pending_vy=remote_pending_velocity[31:16];
    wire signed [15:0] pending_wz=remote_pending_velocity[15:0];
    wire pending_bounds=pending_vx>=-400 && pending_vx<=400 && pending_vy>=-400 && pending_vy<=400 &&
        pending_wz>=-1000 && pending_wz<=1000;
    wire signed [15:0] checked_vx=remote_checked_velocity[47:32];
    wire signed [15:0] checked_vy=remote_checked_velocity[31:16];
    wire signed [15:0] checked_wz=remote_checked_velocity[15:0];
    // 单字节 1/2=前后、3/4=左右、a/b=逆/顺时针；qwer/QWER 单独点动物理 CH0..3。
    // 0=立即停止。物理通道点动锁住原始计数报告模式，直到下一条整车运动命令。
    // 运动期间忽略新运动字符，不能靠连续发字符延长当前脉冲。
    wire motion_byte = rx_data == "1" || rx_data == "2" || rx_data == "3" ||
                       rx_data == "4" || rx_data == "a" || rx_data == "b";
    wire calibration_byte = rx_data == "q" || rx_data == "w" || rx_data == "e" || rx_data == "r" ||
                            rx_data == "Q" || rx_data == "W" || rx_data == "E" || rx_data == "R";
    generate if (TRUSTED_GUARDED_INPUT && !ENABLE_DEBUG_UART) begin: g_guarded_input
    // 板内命令已由 MMIO commit 与 guard 验收；仍保留独立租约和断链立即关断。
    // guard 持续重新检查当前有效命令，因此断链解除后不需要零速解锁或重放旧命令。
    always @(posedge clk_50m_i) begin
        if (rst) begin
            remaining<=0;remaining_active_q<=0;command<="0";physical_report_mode<=0;
            remote_mode<=1;remote_armed<=0;remote_vx<=0;remote_vy<=0;remote_wz<=0;
            remote_pending_valid<=0;remote_pending_velocity<=0;
            remote_checked_valid<=0;remote_checked_velocity<=0;
        end else begin
            if (remaining != 0) begin
                remaining<=remaining-1'b1;
                remaining_active_q<=remaining>1;
                if (remaining==1) begin command<="0";remote_armed<=0;end
            end
            if (remote_valid_i===1'b1) begin
                if (remote_zero_now) begin
                    remaining<=0;remaining_active_q<=0;command<="0";remote_armed<=1;
                    remote_vx<=0;remote_vy<=0;remote_wz<=0;
                end else if (!(remote_link_down_i===1'b1) && !(remote_link_sync[1]===1'b1)) begin
                    remaining<=REMOTE_TIMEOUT_CYCLES;remaining_active_q<=(REMOTE_TIMEOUT_CYCLES!=0);
                    command<="U";remote_armed<=1;
                    remote_vx<=$signed(remote_velocity_i[47:32]);
                    remote_vy<=$signed(remote_velocity_i[31:16]);
                    remote_wz<=$signed(remote_velocity_i[15:0]);
                end
            end
            if (remote_link_down_i===1'b1 || remote_link_sync[1]===1'b1) begin
                remaining<=0;remaining_active_q<=0;command<="0";remote_armed<=0;
                remote_vx<=0;remote_vy<=0;remote_wz<=0;
            end
        end
    end
    end else begin: g_external_input
    always @(posedge clk_50m_i) begin
        if (rst) begin
            remaining<=0;remaining_active_q<=1'b0;command<="0";physical_report_mode<=0;
            remote_mode<=0;remote_armed<=0;remote_vx<=0;remote_vy<=0;remote_wz<=0;
            remote_pending_valid<=1'b0;remote_pending_velocity<=48'd0;
            remote_checked_valid<=1'b0;remote_checked_velocity<=48'd0;
        end
        else begin
            remote_pending_valid <= (remote_valid_i===1'b1) && !remote_zero_now &&
                !(rx_valid && rx_data=="0") && !(remote_link_sync[1]===1'b1);
            if (remote_valid_i===1'b1 && !remote_zero_now)
                remote_pending_velocity <= remote_velocity_i;
            // 范围检查完成后，只有同批已检查命令能更新租约。
            remote_checked_valid <= remote_pending_valid && pending_bounds &&
                !(remote_valid_i===1'b1 && remote_zero_now) &&
                !(rx_valid && rx_data=="0") && !(remote_link_sync[1]===1'b1);
            if (remote_pending_valid) remote_checked_velocity <= remote_pending_velocity;
            if (remaining != 0) begin
                remaining <= remaining - 1;
                remaining_active_q <= remaining > 1;
            end
            if (rx_valid) begin
                if (rx_data == "0") begin remaining <= 0; remaining_active_q<=1'b0; command <= "0";remote_mode<=0;remote_armed<=0; end
                else if (!remote_mode && remaining == 0 && (motion_byte || calibration_byte)) begin
                    remaining <= calibration_byte ? CALIBRATION_CYCLES : JOG_CYCLES;
                    remaining_active_q <= calibration_byte ?
                        (CALIBRATION_CYCLES != 0) : (JOG_CYCLES != 0);
                    command <= rx_data;physical_report_mode<=calibration_byte;
                end
            end
            // 网络接管前先停车；续租可同时改变三轴速度。停机/超时后须先收到零速度重新解锁。
            if(remote_mode && remaining==1)begin remote_armed<=0;command<="0";end
            if(remote_valid_i===1'b1 && remote_zero_now && !(rx_valid && rx_data=="0")) begin
                // 独立保护器发出的故障/超时零速不延迟。
                remote_pending_valid<=1'b0;
                remote_checked_valid<=1'b0;
                remaining<=0;remaining_active_q<=1'b0;command<="0";remote_mode<=1;remote_armed<=1;physical_report_mode<=0;
                remote_vx<=0;remote_vy<=0;remote_wz<=0;
            end else if(remote_checked_valid && !(rx_valid && rx_data=="0") &&
                        !(remote_link_sync[1]===1'b1)) begin
                if(remote_armed)begin
                    remaining<=REMOTE_TIMEOUT_CYCLES;remaining_active_q<=(REMOTE_TIMEOUT_CYCLES != 0);command<="U";remote_mode<=1;remote_armed<=1;physical_report_mode<=0;
                    remote_vx<=checked_vx;remote_vy<=checked_vy;remote_wz<=checked_wz;
                end
            end
            if(remote_link_sync[1]===1'b1 && remote_mode)begin
                remote_pending_valid<=1'b0;remote_checked_valid<=1'b0;
                remaining<=0;remaining_active_q<=1'b0;command<="0";remote_armed<=0;
            end
        end
    end
    end endgenerate
    // 链路同步后的状态只进入控制运算；异步断链在最终驱动口直接切断输出。
    // 避免 PHY/PLL 状态经除法器使能的长组合路径传播到算术寄存器。
    wire enabled = remaining_active_q && !rst && !user_reset_i && !(remote_mode && remote_link_sync[1]===1'b1);
    wire calibration_active=enabled && (command=="q" || command=="w" || command=="e" || command=="r" ||
                                        command=="Q" || command=="W" || command=="E" || command=="R");
    wire controller_enabled=enabled && !calibration_active;
    reg signed [31:0] vx, vy, wz;
    always @* begin
        vx = 0; vy = 0; wz = 0;
        case (command)
            "1": vx = motor_closed_loop_config::JOG_MM_S;
            "2": vx = -motor_closed_loop_config::JOG_MM_S;
            "3": vy = motor_closed_loop_config::JOG_MM_S;
            "4": vy = -motor_closed_loop_config::JOG_MM_S;
            "a": wz = motor_closed_loop_config::JOG_MRAD_S;
            "b": wz = -motor_closed_loop_config::JOG_MRAD_S;
            default: begin end
        endcase
        if(remote_mode)begin vx=remote_vx;vy=remote_vy;wz=remote_wz;end
    end
    wire [3:0] wheel_a, wheel_b;
    wire [3:0] wheel_in1, wheel_in2, encoder_invalid;
    wire [3:0] mapped_motor_in1,mapped_motor_in2;
    reg [3:0] calibration_in1,calibration_in2;
    wire [127:0] position_ticks, measured_mm_s, target_mm_s;
    wire [127:0] physical_channel_ticks;
    reg [15:0] calibration_pwm_count;
    wire [15:0] calibration_duty=(PWM_PERIOD * motor_closed_loop_config::CALIBRATION_PWM_PERCENT) / 100;
    always @(posedge clk_50m_i) begin
        if(rst || calibration_pwm_count==PWM_PERIOD-1)calibration_pwm_count<=0;
        else calibration_pwm_count<=calibration_pwm_count+1'b1;
    end
    always @* begin
        calibration_in1=0;calibration_in2=0;
        if(calibration_active && calibration_pwm_count<calibration_duty) begin
            case(command)
                "q":calibration_in1[0]=1;"Q":calibration_in2[0]=1;
                "w":calibration_in1[1]=1;"W":calibration_in2[1]=1;
                "e":calibration_in1[2]=1;"E":calibration_in2[2]=1;
                "r":calibration_in1[3]=1;"R":calibration_in2[3]=1;
                default:begin end
            endcase
        end
    end
    wire force_output_stop=rst || user_reset_i ||
        ((TRUSTED_GUARDED_INPUT || remote_mode) && remote_link_down_i===1'b1);
    assign motor_in1_o=force_output_stop ? 4'd0 : calibration_active ? calibration_in1 : mapped_motor_in1;
    assign motor_in2_o=force_output_stop ? 4'd0 : calibration_active ? calibration_in2 : mapped_motor_in2;
    // 调试行在物理点动模式下始终按编码器物理 CH0..3 输出、不过软件方向反相。
    for(genvar report_wheel=0;report_wheel<4;report_wheel=report_wheel+1)begin: physical_report
        wire signed [31:0] logical_count=position_ticks[report_wheel*32+:32];
        wire signed [31:0] raw_count=ENCODER_REVERSE[report_wheel] ? -logical_count : logical_count;
        assign physical_channel_ticks[ENCODER_CHANNEL_MAP[report_wheel*2+:2]*32+:32]=raw_count;
    end
    wire body_valid;
    mecanum_motor_control #(
        .CLK_HZ(CLK_HZ), .SAMPLE_HZ(SAMPLE_HZ),
        .GEOMETRY_MM(motor_closed_loop_config::GEOMETRY_MM),
        .MAX_WHEEL_MM_S(motor_closed_loop_config::MAX_WHEEL_MM_S),
        .STEP_MM_S(motor_closed_loop_config::STEP_MM_S),
        .MM_PER_TICK_Q16(MM_PER_TICK_Q16), .PWM_PERIOD(PWM_PERIOD),
        .KP(motor_closed_loop_config::KP), .KD(motor_closed_loop_config::KD),
        .PWM_BRAKE_MODE(motor_closed_loop_config::PWM_BRAKE_MODE),
        .ENCODER_REVERSE(ENCODER_REVERSE),
        .MOTOR_REVERSE(MOTOR_REVERSE)
    ) control(
        .clk_i(clk_50m_i), .rst_n_i(!rst), .enable_i(controller_enabled),
        .vx_mm_s_i(vx), .vy_mm_s_i(vy), .wz_mrad_s_i(wz),
        .encoder_a_i(wheel_a), .encoder_b_i(wheel_b), .in1_o(wheel_in1), .in2_o(wheel_in2),
        .position_ticks_o(position_ticks), .measured_mm_s_o(measured_mm_s),
        .target_mm_s_o(target_mm_s), .encoder_invalid_o(encoder_invalid),
        .sample_valid_o(body_valid));
    reg [3:0] invalid_since_sample;
    reg [3:0] invalid_snapshot;
    assign odometry_invalid_o = invalid_snapshot;
    fusion_mecanum_odometry #(.GEOMETRY_MM(motor_closed_loop_config::GEOMETRY_MM)) fusion_odometry(
        .clk(clk_50m_i),.rst(rst),.sample_i(body_valid),.measured_i(measured_mm_s),
        .valid_o(odometry_valid_o),.vx_o(odometry_vx_o),.vy_o(odometry_vy_o),.wz_o(odometry_wz_o));
    // 将与本次测速相同采样时刻的四轮累计计数送给静止检测器。
    reg [127:0] odometry_ticks_snapshot;
    assign odometry_ticks_o = odometry_ticks_snapshot;
    always @(posedge clk_50m_i) begin
        if(rst) begin
            invalid_since_sample <= 0;invalid_snapshot<=0;odometry_ticks_snapshot<=0;
        end else if(body_valid) begin
            invalid_snapshot<=invalid_since_sample | encoder_invalid;
            invalid_since_sample <= 0;
            odometry_ticks_snapshot<=position_ticks;
        end else invalid_since_sample <= invalid_since_sample | encoder_invalid;
    end
    // 在测速发布拍锁存轮计数与命令；等待同组正运动学计算结束再发布遥测。
    reg [575:0] telemetry_wheel_snapshot;
    reg [3:0] active_physical_mask;
    integer active_wheel;
    always @* begin
        active_physical_mask=0;
        if(calibration_active) begin
            case(command)
                "q","Q":active_physical_mask=1;
                "w","W":active_physical_mask=2;
                "e","E":active_physical_mask=4;
                "r","R":active_physical_mask=8;
                default:active_physical_mask=0;
            endcase
        end else if(controller_enabled) begin
            for(active_wheel=0;active_wheel<4;active_wheel=active_wheel+1)
                if(target_mm_s[active_wheel*32+:32]!=0)
                    active_physical_mask[MOTOR_CHANNEL_MAP[active_wheel*2+:2]]=1;
        end
    end
    always @(posedge clk_50m_i) begin
        if(rst) begin
            telemetry_valid_o<=0;telemetry_payload_o<=0;telemetry_wheel_snapshot<=0;
        end else begin
            telemetry_valid_o<=0;
            if(body_valid) telemetry_wheel_snapshot<={
                MOTOR_CHANNEL_MAP,ENCODER_CHANNEL_MAP,4'd0,MOTOR_REVERSE,4'd0,ENCODER_REVERSE,
                (enabled ? command : 8'h30),
                (calibration_active ? 8'(motor_closed_loop_config::CALIBRATION_PWM_PERCENT) : 8'd0),
                4'd0,active_physical_mask,4'd0,(invalid_since_sample|encoder_invalid),
                position_ticks[0+:32],position_ticks[32+:32],position_ticks[64+:32],position_ticks[96+:32],
                physical_channel_ticks[0+:32],physical_channel_ticks[32+:32],
                physical_channel_ticks[64+:32],physical_channel_ticks[96+:32],
                measured_mm_s[0+:32],measured_mm_s[32+:32],measured_mm_s[64+:32],measured_mm_s[96+:32],
                target_mm_s[0+:32],target_mm_s[32+:32],target_mm_s[64+:32],target_mm_s[96+:32]};
            if(odometry_valid_o) begin
                telemetry_payload_o<={telemetry_wheel_snapshot,odometry_vx_o,odometry_vy_o,odometry_wz_o,32'd0};
                telemetry_valid_o<=1;
            end
        end
    end
    // 分别按集中配置重排编码器输入和驱动输出；位映射低两位对应 FL。
    genvar wheel_index;
    generate for (wheel_index=0; wheel_index<4; wheel_index=wheel_index+1) begin: channel_map
        assign wheel_a[wheel_index] = encoder_a_i[ENCODER_CHANNEL_MAP[wheel_index*2 +: 2]];
        assign wheel_b[wheel_index] = encoder_b_i[ENCODER_CHANNEL_MAP[wheel_index*2 +: 2]];
        assign mapped_motor_in1[MOTOR_CHANNEL_MAP[wheel_index*2 +: 2]] = wheel_in1[wheel_index];
        assign mapped_motor_in2[MOTOR_CHANNEL_MAP[wheel_index*2 +: 2]] = wheel_in2[wheel_index];
    end endgenerate

    function automatic [63:0] hex32(input [31:0] value);
        reg [3:0] nibble;
        begin
            for (integer k=0; k<8; k=k+1) begin
                nibble = value[31-k*4-:4];
                hex32[63-k*8-:8] = nibble < 10 ? 8'h30+nibble : 8'h41+nibble-10;
            end
        end
    endfunction
    // 输出39字节状态与四路32位计数帧，计数按 FL/FR/RL/RR 排列。
    wire [7:0] state_char = physical_report_mode ? (calibration_active ? command : "C") :
        (enabled ? command : "0");
    generate if(ENABLE_DEBUG_UART)begin : gen_debug_tx
    wire [127:0] report_ticks=physical_report_mode ? physical_channel_ticks : position_ticks;
    wire [311:0] frame = {state_char,8'h20,hex32(report_ticks[0+:32]),8'h20,
        hex32(report_ticks[32+:32]),8'h20,hex32(report_ticks[64+:32]),8'h20,
        hex32(report_ticks[96+:32]),16'h0d0a};
    reg [31:0] report_timer;
    wire frame_ready;
    wire send_frame = report_timer == REPORT_CYCLES-1 && frame_ready;
    always @(posedge clk_50m_i)
        if (rst || report_timer == REPORT_CYCLES-1) report_timer <= 0;
        else report_timer <= report_timer+1;
    wire [7:0] tx_data;
    wire tx_start, tx_busy;
    packet_uart_tx #(.PACKET_BYTES(39)) packet(.clk(clk_50m_i), .rst(rst),
        .packet_valid_i(send_frame), .packet_data_i(frame), .packet_ready_o(frame_ready),
        .uart_busy_i(tx_busy), .uart_data_o(tx_data), .uart_start_o(tx_start), .packet_pending_o());
    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) tx(.clk_i(clk_50m_i), .rst_i(rst),
        .data_i(tx_data), .start_i(tx_start), .tx_o(debug_uart_tx_o), .busy_o(tx_busy));
    end else begin : gen_no_debug_tx
        assign debug_uart_tx_o=1'b1;
    end endgenerate
endmodule

// Integrated UART support: uart_rx.sv
`timescale 1ns/1ps

// 电机输入已由调用端同步；保留同步高有效复位和逐位数据输出。
module uart_rx #(
    parameter integer CLK_HZ=50_000_000,
    parameter integer BAUD=115200
)(
    input wire clk_i,rst_i,rx_i,
    output wire [7:0] data_o,
    output wire valid_o
);
    localparam integer BIT_CYCLES=CLK_HZ/BAUD;
    localparam integer HALF_CYCLES=(BIT_CYCLES/2<1)?1:BIT_CYCLES/2;
    bx72_uart_rx #(.BIT_CYCLES(BIT_CYCLES),.HALF_CYCLES(HALF_CYCLES),
        .ASYNC_RESET(0),.DATA_ON_SAMPLE(1)) receiver(
        .clk(clk_i),.reset_n(!rst_i),.rx_i(rx_i),.data_o(data_o),
        .valid_o(valid_o),.framing_error_o());
endmodule

// Integrated UART support: uart_tx.sv
`timescale 1ns/1ps

module uart_tx #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD = 115200
) (
    input  wire       clk_i,
    input  wire       rst_i,
    input  wire [7:0] data_i,
    input  wire       start_i,
    output reg        tx_o,
    output reg        busy_o
);
    localparam integer CLKS_PER_BIT = CLK_HZ / BAUD;
    localparam [1:0] ST_IDLE = 2'd0;
    localparam [1:0] ST_START = 2'd1;
    localparam [1:0] ST_DATA = 2'd2;
    localparam [1:0] ST_STOP = 2'd3;

    reg [1:0] state;
    reg [31:0] count;
    reg [2:0] bit_index;
    reg [7:0] data_reg;

    always @(posedge clk_i) begin
        if (rst_i) begin
            state <= ST_IDLE;
            count <= 32'd0;
            bit_index <= 3'd0;
            data_reg <= 8'd0;
            tx_o <= 1'b1;
            busy_o <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    tx_o <= 1'b1;
                    busy_o <= 1'b0;
                    if (start_i) begin
                        data_reg <= data_i;
                        bit_index <= 3'd0;
                        count <= CLKS_PER_BIT - 1;
                        tx_o <= 1'b0;
                        busy_o <= 1'b1;
                        state <= ST_START;
                    end
                end

                ST_START: begin
                    busy_o <= 1'b1;
                    if (count == 0) begin
                        tx_o <= data_reg[0];
                        count <= CLKS_PER_BIT - 1;
                        state <= ST_DATA;
                    end else begin
                        count <= count - 1'b1;
                    end
                end

                ST_DATA: begin
                    busy_o <= 1'b1;
                    if (count == 0) begin
                        count <= CLKS_PER_BIT - 1;
                        if (bit_index == 3'd7) begin
                            tx_o <= 1'b1;
                            state <= ST_STOP;
                        end else begin
                            bit_index <= bit_index + 1'b1;
                            tx_o <= data_reg[bit_index + 1'b1];
                        end
                    end else begin
                        count <= count - 1'b1;
                    end
                end

                ST_STOP: begin
                    busy_o <= 1'b1;
                    if (count == 0) begin
                        tx_o <= 1'b1;
                        busy_o <= 1'b0;
                        state <= ST_IDLE;
                    end else begin
                        count <= count - 1'b1;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule

// Integrated UART support: packet_uart_tx.v
`timescale 1ns/1ps

// 固定长度报文串行发送器；锁存完整帧后逐字节输出，忙期间不接收新帧。
module packet_uart_tx #(
    parameter integer PACKET_BYTES = 32
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         packet_valid_i,
    input  wire [PACKET_BYTES*8-1:0] packet_data_i,
    output wire         packet_ready_o,
    input  wire         uart_busy_i,
    output reg  [7:0]   uart_data_o,
    output reg          uart_start_o,
    output wire         packet_pending_o
);
    reg [PACKET_BYTES*8-1:0] shift_reg;
    localparam integer INDEX_WIDTH = (PACKET_BYTES > 1) ? $clog2(PACKET_BYTES) : 1;
    reg [INDEX_WIDTH-1:0] byte_index;
    reg pending;
    reg active;

    assign packet_ready_o = !pending && !active;
    assign packet_pending_o = pending || active;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            shift_reg    <= {PACKET_BYTES*8{1'b0}};
            byte_index   <= {INDEX_WIDTH{1'b0}};
            pending      <= 1'b0;
            active       <= 1'b0;
            uart_data_o  <= 8'd0;
            uart_start_o <= 1'b0;
        end else begin
            uart_start_o <= 1'b0;
            if (!active) begin
                if (pending) begin
                    if (!uart_busy_i) begin
                        uart_data_o  <= shift_reg[PACKET_BYTES*8-1 -: 8];
                        uart_start_o <= 1'b1;
                        active       <= 1'b1;
                        pending      <= 1'b0;
                        byte_index   <= {INDEX_WIDTH{1'b0}};
                    end
                end else if (packet_valid_i) begin
                    shift_reg <= packet_data_i;
                    pending   <= 1'b1;
                end
            end else begin
                if (uart_start_o) begin
                    uart_start_o <= 1'b0;
                end else if (!uart_busy_i) begin
                    if (byte_index == PACKET_BYTES - 1) begin
                        active     <= 1'b0;
                        byte_index <= {INDEX_WIDTH{1'b0}};
                    end else begin
                        shift_reg    <= shift_reg << 8;
                        uart_data_o  <= (shift_reg << 8) >> (PACKET_BYTES*8-8);
                        uart_start_o <= 1'b1;
                        byte_index   <= byte_index + 1'b1;
                    end
                end
            end
        end
    end
endmodule

// ==================== bx72_command_control.sv ====================
`timescale 1ns/1ps
// 单一通信入口的手动/自动仲裁。模式切换先停车，自动租约超时后必须重新启用。
module bx72_command_control #(
    parameter integer LEASE_CYCLES=10000000,
    parameter integer ALLOW_LEGACY_AUTO=1
)(
    input wire clk,reset_n,
    input wire manual_valid_i,
    input wire [47:0] manual_velocity_i,
    input wire link_down_i,mode_valid_i,auto_enable_i,
    input wire planner_valid_i,planner_scan_valid_i,planner_fault_i,
    input wire [47:0] planner_velocity_i,
    input wire [63:0] now_us_i,
    input wire task_accepted_i,task_done_i,task_fault_i,
    input wire [7:0] task_mode_i,task_flags_i,
    input wire [63:0] task_expiry_us_i,
    input wire navigation_valid_i,input wire [47:0] navigation_velocity_i,
    input wire cancel_i,
    output wire navigation_cancel_o,
    output reg auto_mode_o,
    output wire valid_o,link_down_o,
    output wire [47:0] velocity_o
);
    localparam integer AGE_W=LEASE_CYCLES<2 ? 1 : $clog2(LEASE_CYCLES+1);
    reg [AGE_W-1:0] age;
    reg owned;
    reg task_owned;
    reg [7:0] task_mode,task_flags;
    reg [63:0] task_expiry;
    wire task_expired=task_owned && now_us_i>=task_expiry;
    wire task_link_stop=task_owned && link_down_i && task_mode!=3 && !task_flags[0];
    wire task_cancel=task_owned && (manual_valid_i || mode_valid_i || cancel_i===1'b1 ||
        task_fault_i===1'b1 || task_expired || task_link_stop);
    assign navigation_cancel_o=task_cancel;
    wire lease_expired=owned && !task_owned && age>=LEASE_CYCLES-1 && !manual_valid_i && !mode_valid_i;
    wire mode_change=mode_valid_i && (!auto_enable_i || !auto_mode_o);
    wire manual_takeover=manual_valid_i && auto_mode_o;
    wire stop_now=mode_change || manual_takeover || lease_expired || task_cancel || (task_owned && task_done_i===1'b1);
    assign link_down_o=task_owned ? (task_cancel || task_fault_i===1'b1) :
        (link_down_i || (auto_mode_o && (!planner_scan_valid_i || planner_fault_i || lease_expired)));
    assign valid_o=stop_now || (task_owned ? navigation_valid_i : (auto_mode_o ? planner_valid_i : manual_valid_i));
    assign velocity_o=(stop_now || link_down_o) ? 48'd0 :
        task_owned ? navigation_velocity_i : (auto_mode_o ? planner_velocity_i : manual_velocity_i);
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n)begin auto_mode_o<=0;owned<=0;age<=0;end
        else if(task_accepted_i===1'b1 || link_down_i || lease_expired)begin auto_mode_o<=0;owned<=0;age<=0;end
        else if(mode_valid_i)begin auto_mode_o<=auto_enable_i && ALLOW_LEGACY_AUTO!=0;owned<=auto_enable_i && ALLOW_LEGACY_AUTO!=0;age<=0;end
        else if(manual_valid_i)begin auto_mode_o<=0;owned<=1;age<=0;end
        else if(owned)age<=age+1'b1;
    end
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin task_owned<=0;task_mode<=0;task_flags<=0;task_expiry<=0;end
        else if(task_cancel)task_owned<=0;
        else if(task_accepted_i===1'b1)begin
            task_owned<=1;task_mode<=task_mode_i;task_flags<=task_flags_i;task_expiry<=task_expiry_us_i;
        end
        else if(task_done_i===1'b1)task_owned<=0;
    end
endmodule
