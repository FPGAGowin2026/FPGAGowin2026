`timescale 1ns/1ps

module tb_motor_control;
    reg clk_i = 1'b0;
    reg rst_n_i = 1'b0;
    reg encoder_a_i = 1'b0;
    reg encoder_b_i = 1'b0;
    wire signed [31:0] position_ticks_o;
    wire step_valid_o;
    wire direction_step_o;
    wire invalid_transition_o;
    reg sample_tick_i = 1'b0;
    wire signed [31:0] estimated_velocity_o;
    wire velocity_valid_o;

    reg enable_i = 1'b0;
    reg signed [31:0] linear_i = 32'sd0;
    reg signed [31:0] angular_i = 32'sd0;
    reg signed [31:0] left_measured_i = 32'sd0;
    reg signed [31:0] right_measured_i = 32'sd0;
    wire signed [31:0] left_target_o;
    wire signed [31:0] right_target_o;
    wire [15:0] left_pwm_o;
    wire [15:0] right_pwm_o;
    wire left_direction_o;
    wire right_direction_o;
    integer checks = 0;

    quadrature_decoder decoder (
        .clk_i(clk_i), .rst_n_i(rst_n_i),
        .encoder_a_i(encoder_a_i), .encoder_b_i(encoder_b_i),
        .position_ticks_o(position_ticks_o), .step_valid_o(step_valid_o),
        .direction_o(direction_step_o), .invalid_transition_o(invalid_transition_o)
    );

    encoder_velocity_estimator #(
        .MM_PER_TICK_Q16(32'sd65536), .SAMPLE_HZ(10)
    ) estimator (
        .clk_i(clk_i), .rst_n_i(rst_n_i), .sample_tick_i(sample_tick_i),
        .position_ticks_i(position_ticks_o),
        .velocity_mm_s_o(estimated_velocity_o), .velocity_valid_o(velocity_valid_o)
    );

    differential_drive_control #(
        .TRACK_WIDTH_MM(200), .MAX_WHEEL_SPEED_MM_S(1000),
        .KP(32'sd8), .KI(32'sd8), .GAIN_SHIFT(4), .PWM_MAX(1000)
    ) drive (
        .clk_i(clk_i), .rst_n_i(rst_n_i), .enable_i(enable_i),
        .sample_tick_i(sample_tick_i), .linear_mm_s_i(linear_i),
        .angular_mrad_s_i(angular_i),
        .left_velocity_mm_s_i(left_measured_i),
        .right_velocity_mm_s_i(right_measured_i),
        .left_target_mm_s_o(left_target_o), .right_target_mm_s_o(right_target_o),
        .left_pwm_duty_o(left_pwm_o), .left_direction_o(left_direction_o),
        .right_pwm_duty_o(right_pwm_o), .right_direction_o(right_direction_o)
    );

    always #5 clk_i = ~clk_i;

    task automatic set_ab;
        input a;
        input b;
        begin
            @(negedge clk_i); encoder_a_i = a; encoder_b_i = b;
            repeat (3) @(posedge clk_i);
        end
    endtask

    task automatic pulse_sample;
        begin
            @(negedge clk_i); sample_tick_i = 1'b1;
            @(negedge clk_i); sample_tick_i = 1'b0;
        end
    endtask

    initial begin
        repeat (3) @(posedge clk_i);
        rst_n_i = 1'b1;

        // 00->01->11->10->00 为一个正向四倍频周期。
        set_ab(0,1); set_ab(1,1); set_ab(1,0); set_ab(0,0);
        #1;
        checks = checks + 1;
        if (position_ticks_o !== 32'sd4)
            $fatal(1, "FAIL quadrature forward ticks=%0d", position_ticks_o);

        pulse_sample(); #1;
        checks = checks + 1;
        if (!velocity_valid_o || estimated_velocity_o !== 32'sd40)
            $fatal(1, "FAIL velocity estimate=%0d valid=%b", estimated_velocity_o, velocity_valid_o);

        // 500 mm/s + 1000 mrad/s 在 200 mm 轮距下应得到 400/600 mm/s。
        enable_i = 1'b1; linear_i = 32'sd500; angular_i = 32'sd1000;
        #1;
        checks = checks + 1;
        if (left_target_o !== 32'sd400 || right_target_o !== 32'sd600)
            $fatal(1, "FAIL kinematics left=%0d right=%0d", left_target_o, right_target_o);
        pulse_sample(); #1;
        checks = checks + 1;
        if (left_pwm_o !== 16'd400 || right_pwm_o !== 16'd600 ||
            !left_direction_o || !right_direction_o)
            $fatal(1, "FAIL PI output left=%0d right=%0d", left_pwm_o, right_pwm_o);

        // 原地负向旋转必须给出相反轮向；禁用时立即清零 PWM。
        linear_i = 32'sd0; angular_i = -32'sd2000;
        pulse_sample(); #1;
        checks = checks + 1;
        if (left_target_o !== 32'sd200 || right_target_o !== -32'sd200 ||
            !left_direction_o || right_direction_o)
            $fatal(1, "FAIL rotate targets/directions");

        // 有效零目标也必须立即清积分和 PWM，不能复用上一拍输出。
        linear_i = 32'sd0; angular_i = 32'sd0;
        @(posedge clk_i); #1;
        checks = checks + 1;
        if (left_pwm_o !== 16'd0 || right_pwm_o !== 16'd0)
            $fatal(1, "FAIL zero target PWM clear");

        // 大误差按 PWM_MAX 饱和；方向和 16 位幅值都必须确定。
        linear_i = 32'sd1000;
        left_measured_i = -32'sd100000;
        right_measured_i = -32'sd100000;
        pulse_sample(); #1;
        checks = checks + 1;
        if (left_pwm_o !== 16'd1000 || right_pwm_o !== 16'd1000 ||
            !left_direction_o || !right_direction_o)
            $fatal(1, "FAIL positive PWM saturation");

        // 换向后的小负目标清掉正向积分，负数绝对值不得发生宽度截断。
        linear_i = -32'sd3;
        left_measured_i = 32'sd0;
        right_measured_i = 32'sd0;
        pulse_sample(); #1;
        checks = checks + 1;
        if (left_pwm_o !== 16'd3 || right_pwm_o !== 16'd3 ||
            left_direction_o || right_direction_o)
            $fatal(1, "FAIL negative PWM magnitude");

        enable_i = 1'b0;
        @(posedge clk_i); #1;
        checks = checks + 1;
        if (left_pwm_o !== 16'd0 || right_pwm_o !== 16'd0)
            $fatal(1, "FAIL disable PWM");

        $display("PASS: motor control %0d checks", checks);
        $finish;
    end
endmodule
