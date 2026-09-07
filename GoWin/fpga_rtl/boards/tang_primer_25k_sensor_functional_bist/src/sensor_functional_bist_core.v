`timescale 1ns/1ps

// 17 项传感器核心的片上功能 BIST。
// 位图顺序见 README；pass_mask 只在被测核心产生有效事件且数据精确匹配时置位。
module sensor_functional_bist_core #(
    parameter integer CLK_FREQ_HZ = 50000000,
    parameter integer SENSOR_UART_BAUD_RATE = 3125000,
    parameter integer BIST_TIMEOUT_CYCLES = 50000
) (
    input  wire        clk,
    input  wire        rst_n,
    output reg  [16:0] pass_mask_o,
    output reg  [16:0] observed_mask_o,
    output reg  [16:0] activity_mask_o,
    output reg         bist_done_o
);
    wire digital_ir_n;
    wire [4:0] sensor_uart;
    wire hc_echo;
    wire itr_pulse;
    wire bumper_left_n;
    wire bumper_right_n;
    wire bumper_clear;
    wire camera_vsync;
    wire camera_href;
    wire [7:0] mt9v03x_data;
    wire [7:0] ov5640_data;
    wire [7:0] ov7725_data;
    wire [7:0] scc8660_data;
    wire [7:0] multi_ir_n;
    wire encoder_a;
    wire encoder_b;
    wire tcrt5000_n;
    wire [11:0] tsl_adc_data;
    wire tsl_adc_valid;
    wire [4:0] uart_channel_done;
    wire stimulus_timeline_done;

    wire hc_trigger;
    wire tsl_adc_start;
    wire [6:0] tsl_pixel_index;

    sensor_functional_stimulus #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .SENSOR_UART_BAUD_RATE(SENSOR_UART_BAUD_RATE)
    ) u_stimulus (
        .clk(clk), .rst_n(rst_n),
        .hc_trigger_i(hc_trigger),
        .tsl_adc_start_i(tsl_adc_start),
        .tsl_pixel_index_i(tsl_pixel_index),
        .digital_ir_n_o(digital_ir_n),
        .sensor_uart_o(sensor_uart),
        .hc_echo_o(hc_echo), .itr_pulse_o(itr_pulse),
        .bumper_left_n_o(bumper_left_n),
        .bumper_right_n_o(bumper_right_n),
        .bumper_clear_o(bumper_clear),
        .camera_vsync_o(camera_vsync), .camera_href_o(camera_href),
        .mt9v03x_data_o(mt9v03x_data), .ov5640_data_o(ov5640_data),
        .ov7725_data_o(ov7725_data), .scc8660_data_o(scc8660_data),
        .multi_ir_n_o(multi_ir_n), .encoder_a_o(encoder_a),
        .encoder_b_o(encoder_b), .tcrt5000_n_o(tcrt5000_n),
        .tsl_adc_data_o(tsl_adc_data), .tsl_adc_valid_o(tsl_adc_valid),
        .uart_channel_done_o(uart_channel_done),
        .timeline_done_o(stimulus_timeline_done)
    );

    // 0: digital_ir_proximity
    wire digital_object_near;
    wire digital_changed;
    digital_ir_proximity #(
        .ASSERT_SAMPLES(3), .RELEASE_SAMPLES(3)
    ) u_digital_ir (
        .clk_i(clk), .rst_n_i(rst_n), .sensor_n_i(digital_ir_n),
        .object_near_o(digital_object_near), .changed_o(digital_changed)
    );

    // 1: gnss_nmea_uart
    wire gnss_valid;
    wire [1:0] gnss_type;
    wire [7:0] gnss_length;
    wire [7:0] gnss_checksum;
    wire gnss_checksum_error;
    wire gnss_format_error;
    wire gnss_overflow_error;
    wire gnss_framing_error;
    gnss_nmea_uart #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(SENSOR_UART_BAUD_RATE),
        .MAX_BODY_CHARS(20)
    ) u_gnss (
        .clk(clk), .rst_n(rst_n), .uart_rx(sensor_uart[0]),
        .sentence_valid(gnss_valid), .sentence_type(gnss_type),
        .body_length(gnss_length), .checksum_value(gnss_checksum),
        .checksum_error(gnss_checksum_error),
        .format_error(gnss_format_error),
        .overflow_error(gnss_overflow_error),
        .framing_error(gnss_framing_error)
    );

    // 2: gy39_uart
    wire [31:0] gy39_lux;
    wire signed [15:0] gy39_temperature;
    wire [31:0] gy39_pressure;
    wire [15:0] gy39_humidity;
    wire signed [15:0] gy39_altitude;
    wire gy39_lux_valid;
    wire gy39_environment_valid;
    wire gy39_checksum_error;
    wire gy39_format_error;
    wire gy39_framing_error;
    gy39_uart #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(SENSOR_UART_BAUD_RATE)
    ) u_gy39 (
        .clk(clk), .rst_n(rst_n), .uart_rx(sensor_uart[1]),
        .lux_centi_lux(gy39_lux), .temperature_centi_c(gy39_temperature),
        .pressure_centi_pa(gy39_pressure),
        .humidity_centi_rh(gy39_humidity), .altitude_m(gy39_altitude),
        .lux_valid(gy39_lux_valid),
        .environment_valid(gy39_environment_valid),
        .checksum_error(gy39_checksum_error),
        .format_error(gy39_format_error),
        .framing_error(gy39_framing_error)
    );

    // 3: hc_sr04。BIST 将一个系统拍映射为一个逻辑微秒，以缩短测试时间。
    wire [15:0] hc_distance_mm;
    wire [15:0] hc_pulse_width_us;
    wire hc_sample_valid;
    wire hc_timeout_error;
    wire hc_busy;
    hc_sr04_controller #(
        .CLK_FREQ_HZ(1000000), .TRIGGER_US(3),
        .TRIGGER_PERIOD_US(16), .ECHO_TIMEOUT_US(100)
    ) u_hc_sr04 (
        .clk(clk), .rst_n(rst_n), .echo_in(hc_echo),
        .trigger_out(hc_trigger), .distance_mm(hc_distance_mm),
        .pulse_width_us(hc_pulse_width_us), .sample_valid(hc_sample_valid),
        .timeout_error(hc_timeout_error), .busy(hc_busy)
    );

    // 4: itr9606_pulse_speed
    wire [31:0] itr_count;
    wire [31:0] itr_period;
    wire itr_sample_valid;
    wire itr_period_valid;
    wire itr_overflow;
    itr9606_pulse_speed #(
        .WINDOW_CYCLES(128), .MIN_EDGE_GAP_CYCLES(4)
    ) u_itr9606 (
        .clk(clk), .rst_n(rst_n), .pulse_i(itr_pulse),
        .pulse_count_o(itr_count), .period_cycles_o(itr_period),
        .sample_valid_o(itr_sample_valid), .period_valid_o(itr_period_valid),
        .count_overflow_o(itr_overflow)
    );

    // 5: jy61p_uart
    wire signed [15:0] jy_roll;
    wire signed [15:0] jy_pitch;
    wire signed [15:0] jy_yaw;
    wire [15:0] jy_version;
    wire jy_valid;
    wire jy_checksum_error;
    wire jy_framing_error;
    jy61p_uart #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(SENSOR_UART_BAUD_RATE)
    ) u_jy61p (
        .clk(clk), .rst_n(rst_n), .uart_rx(sensor_uart[2]),
        .roll_raw(jy_roll), .pitch_raw(jy_pitch), .yaw_raw(jy_yaw),
        .version_raw(jy_version), .frame_valid(jy_valid),
        .checksum_error(jy_checksum_error),
        .framing_error(jy_framing_error)
    );

    // 6: maxsonar_uart
    wire [15:0] maxsonar_range;
    wire maxsonar_valid;
    wire maxsonar_format_error;
    wire maxsonar_framing_error;
    maxsonar_uart #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(SENSOR_UART_BAUD_RATE)
    ) u_maxsonar (
        .clk(clk), .rst_n(rst_n), .uart_rx(sensor_uart[3]),
        .range_raw(maxsonar_range), .range_valid(maxsonar_valid),
        .format_error(maxsonar_format_error),
        .framing_error(maxsonar_framing_error)
    );

    // 7: mechanical_bumper
    wire bumper_left_pressed;
    wire bumper_right_pressed;
    wire bumper_collision;
    wire bumper_collision_event;
    wire bumper_collision_latched;
    mechanical_bumper #(
        .ACTIVE_LOW(1), .DEBOUNCE_CYCLES(3)
    ) u_bumper (
        .clk(clk), .rst_n(rst_n),
        .bumper_left_in(bumper_left_n),
        .bumper_right_in(bumper_right_n), .clear_latched(bumper_clear),
        .left_pressed(bumper_left_pressed),
        .right_pressed(bumper_right_pressed), .collision(bumper_collision),
        .collision_event(bumper_collision_event),
        .collision_latched(bumper_collision_latched)
    );

    // 8: mt9v03x_dvp
    wire mt_frame_active;
    wire mt_pixel_valid;
    wire [7:0] mt_pixel;
    wire [15:0] mt_pixel_x;
    wire [15:0] mt_line_y;
    wire mt_frame_start;
    wire mt_frame_end;
    mt9v03x_dvp_capture u_mt9v03x (
        .pclk(clk), .rst_n(rst_n), .vsync(camera_vsync),
        .href(camera_href), .data_i(mt9v03x_data),
        .frame_active(mt_frame_active), .pixel_valid(mt_pixel_valid),
        .pixel_gray(mt_pixel), .pixel_x(mt_pixel_x), .line_y(mt_line_y),
        .frame_start(mt_frame_start), .frame_end(mt_frame_end)
    );

    // 9: multi_ir_array
    wire [7:0] multi_ir_mask;
    wire multi_ir_changed;
    multi_ir_array #(
        .CHANNELS(8), .DEBOUNCE_CYCLES(3), .ACTIVE_LOW(1'b1)
    ) u_multi_ir (
        .clk(clk), .rst_n(rst_n), .sensor_i(multi_ir_n),
        .active_mask_o(multi_ir_mask), .change_valid_o(multi_ir_changed)
    );

    // 10: ov5640_dvp_capture，按 4300h=6Fh 的低字节优先总线顺序。
    wire ov5640_frame_valid;
    wire ov5640_line_valid;
    wire ov5640_pixel_valid;
    wire [15:0] ov5640_pixel;
    ov5640_dvp_capture #(
        .VSYNC_ACTIVE_LEVEL(1'b1), .DVP_LITTLE_ENDIAN(1'b1)
    ) u_ov5640 (
        .pclk_i(clk), .rst_n_i(rst_n), .vsync_i(camera_vsync),
        .href_i(camera_href), .data_i(ov5640_data),
        .frame_valid_o(ov5640_frame_valid),
        .line_valid_o(ov5640_line_valid),
        .pixel_valid_o(ov5640_pixel_valid),
        .pixel_rgb565_o(ov5640_pixel)
    );

    // 11: ov7725_dvp_capture
    wire ov7725_frame_active;
    wire ov7725_pixel_valid;
    wire [15:0] ov7725_pixel;
    wire [15:0] ov7725_x;
    wire [15:0] ov7725_y;
    wire ov7725_frame_start;
    wire ov7725_frame_end;
    wire ov7725_partial_error;
    ov7725_dvp_capture u_ov7725 (
        .pclk(clk), .rst_n(rst_n), .vsync(camera_vsync),
        .href(camera_href), .data_i(ov7725_data),
        .frame_active(ov7725_frame_active), .pixel_valid(ov7725_pixel_valid),
        .pixel_rgb565(ov7725_pixel), .pixel_x(ov7725_x), .line_y(ov7725_y),
        .frame_start(ov7725_frame_start), .frame_end(ov7725_frame_end),
        .partial_pixel_error(ov7725_partial_error)
    );

    // 12: quadrature_encoder
    wire signed [31:0] encoder_position;
    wire encoder_step_valid;
    wire encoder_direction;
    wire encoder_invalid;
    quadrature_decoder u_encoder (
        .clk_i(clk), .rst_n_i(rst_n), .encoder_a_i(encoder_a),
        .encoder_b_i(encoder_b), .position_ticks_o(encoder_position),
        .step_valid_o(encoder_step_valid), .direction_o(encoder_direction),
        .invalid_transition_o(encoder_invalid)
    );

    // 13: scc8660_dvp_capture
    wire scc_frame_active;
    wire scc_pixel_valid;
    wire [15:0] scc_pixel;
    wire [15:0] scc_x;
    wire [15:0] scc_y;
    wire scc_frame_start;
    wire scc_frame_end;
    wire scc_partial_error;
    scc8660_dvp_capture u_scc8660 (
        .pclk(clk), .rst_n(rst_n), .vsync(camera_vsync),
        .href(camera_href), .data_i(scc8660_data),
        .frame_active(scc_frame_active), .pixel_valid(scc_pixel_valid),
        .pixel_rgb565(scc_pixel), .pixel_x(scc_x), .line_y(scc_y),
        .frame_start(scc_frame_start), .frame_end(scc_frame_end),
        .partial_pixel_error(scc_partial_error)
    );

    // 14: tcrt5000
    wire tcrt_line_detected;
    wire tcrt_line_rise;
    wire tcrt_line_fall;
    wire [31:0] tcrt_filter_level;
    tcrt5000_filter #(
        .ACTIVE_LOW(1), .FILTER_CYCLES(3)
    ) u_tcrt5000 (
        .clk(clk), .rst_n(rst_n), .sensor_in(tcrt5000_n),
        .line_detected(tcrt_line_detected), .line_rise(tcrt_line_rise),
        .line_fall(tcrt_line_fall), .filter_level(tcrt_filter_level)
    );

    // 15: tf_luna_uart
    wire [15:0] tf_distance;
    wire [15:0] tf_strength;
    wire [15:0] tf_temperature;
    wire tf_valid;
    wire tf_checksum_error;
    tf_luna_uart #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(SENSOR_UART_BAUD_RATE)
    ) u_tf_luna (
        .clk(clk), .rst_n(rst_n), .uart_rx(sensor_uart[4]),
        .distance_cm(tf_distance), .strength(tf_strength),
        .temperature_raw(tf_temperature), .frame_valid(tf_valid),
        .checksum_error(tf_checksum_error)
    );

    // 16: tsl1401_adc_frontend
    wire tsl_sensor_clk;
    wire tsl_sensor_si;
    wire [11:0] tsl_pixel_data;
    wire tsl_pixel_valid;
    wire tsl_frame_start;
    wire tsl_frame_done;
    wire tsl_frame_active;
    wire tsl_adc_timeout;
    tsl1401_adc_frontend #(
        .ADC_WIDTH(12), .CLK_HALF_PERIOD_CYCLES(1),
        .PIXEL_SETTLE_CYCLES(1), .ADC_TIMEOUT_CYCLES(8),
        .INTEGRATION_CYCLES(8), .CHARGE_TRANSFER_CYCLES(4)
    ) u_tsl1401 (
        .clk(clk), .reset_n(rst_n), .sensor_clk(tsl_sensor_clk),
        .sensor_si(tsl_sensor_si), .adc_start(tsl_adc_start),
        .adc_data(tsl_adc_data), .adc_valid(tsl_adc_valid),
        .pixel_index(tsl_pixel_index), .pixel_data(tsl_pixel_data),
        .pixel_valid(tsl_pixel_valid), .frame_start(tsl_frame_start),
        .frame_done(tsl_frame_done), .frame_active(tsl_frame_active),
        .adc_timeout(tsl_adc_timeout)
    );

    reg [31:0] timeout_count_q;
    reg [4:0] uart_low_seen_q;
    reg hc_trigger_seen_q;
    reg hc_echo_seen_q;
    reg itr_period_match_q;
    reg [3:0] encoder_step_count_q;
    reg encoder_invalid_seen_q;
    reg uart_error_seen_q;
    reg hc_error_seen_q;
    reg itr_error_seen_q;
    reg dvp_error_seen_q;

    reg tsl_sensor_clk_d_q;
    reg tsl_sensor_si_d_q;
    reg [8:0] tsl_line_clock_count_q;
    reg [8:0] tsl_pixel_count_q;
    reg tsl_first_pixel_match_q;
    reg tsl_last_pixel_match_q;

    wire all_pass_now = &pass_mask_o;
    wire all_observed_now = &observed_mask_o;
    wire all_active_now = &activity_mask_o;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pass_mask_o <= 17'd0;
            observed_mask_o <= 17'd0;
            activity_mask_o <= 17'd0;
            bist_done_o <= 1'b0;
            timeout_count_q <= 32'd0;
            uart_low_seen_q <= 5'd0;
            hc_trigger_seen_q <= 1'b0;
            hc_echo_seen_q <= 1'b0;
            itr_period_match_q <= 1'b0;
            encoder_step_count_q <= 4'd0;
            encoder_invalid_seen_q <= 1'b0;
            uart_error_seen_q <= 1'b0;
            hc_error_seen_q <= 1'b0;
            itr_error_seen_q <= 1'b0;
            dvp_error_seen_q <= 1'b0;
            tsl_sensor_clk_d_q <= 1'b0;
            tsl_sensor_si_d_q <= 1'b0;
            tsl_line_clock_count_q <= 9'd0;
            tsl_pixel_count_q <= 9'd0;
            tsl_first_pixel_match_q <= 1'b0;
            tsl_last_pixel_match_q <= 1'b0;
        end else begin
            if (!bist_done_o) begin
                if (all_pass_now && all_observed_now && all_active_now)
                    bist_done_o <= 1'b1;
                else if (timeout_count_q >= BIST_TIMEOUT_CYCLES - 1)
                    bist_done_o <= 1'b1;
                else
                    timeout_count_q <= timeout_count_q + 1'b1;
            end

            uart_low_seen_q <= uart_low_seen_q | ~sensor_uart;
            if (hc_trigger) hc_trigger_seen_q <= 1'b1;
            if (hc_echo) hc_echo_seen_q <= 1'b1;

            uart_error_seen_q <= uart_error_seen_q |
                gnss_checksum_error | gnss_format_error |
                gnss_overflow_error | gnss_framing_error |
                gy39_checksum_error | gy39_format_error |
                gy39_framing_error | jy_checksum_error |
                jy_framing_error | maxsonar_format_error |
                maxsonar_framing_error | tf_checksum_error;
            hc_error_seen_q <= hc_error_seen_q | hc_timeout_error;
            itr_error_seen_q <= itr_error_seen_q | itr_overflow;
            dvp_error_seen_q <= dvp_error_seen_q |
                                ov7725_partial_error | scc_partial_error;

            // 片上激励活动位：与被测核心输出观察位分离，防止常量 PASS。
            if (!digital_ir_n) activity_mask_o[0] <= 1'b1;
            if (!sensor_uart[0]) activity_mask_o[1] <= 1'b1;
            if (!sensor_uart[1]) activity_mask_o[2] <= 1'b1;
            if (hc_trigger_seen_q && hc_echo_seen_q)
                activity_mask_o[3] <= 1'b1;
            if (itr_pulse) activity_mask_o[4] <= 1'b1;
            if (!sensor_uart[2]) activity_mask_o[5] <= 1'b1;
            if (!sensor_uart[3]) activity_mask_o[6] <= 1'b1;
            if (!bumper_left_n) activity_mask_o[7] <= 1'b1;
            if (camera_href && mt9v03x_data != 0)
                activity_mask_o[8] <= 1'b1;
            if (multi_ir_n != 8'hff) activity_mask_o[9] <= 1'b1;
            if (camera_href && ov5640_data != 0)
                activity_mask_o[10] <= 1'b1;
            if (camera_href && ov7725_data != 0)
                activity_mask_o[11] <= 1'b1;
            if ({encoder_a, encoder_b} != 2'b00)
                activity_mask_o[12] <= 1'b1;
            if (camera_href && scc8660_data != 0)
                activity_mask_o[13] <= 1'b1;
            if (!tcrt5000_n) activity_mask_o[14] <= 1'b1;
            if (!sensor_uart[4]) activity_mask_o[15] <= 1'b1;
            if (tsl_sensor_si || tsl_sensor_clk || tsl_adc_start)
                activity_mask_o[16] <= 1'b1;

            // 实际被测核心事件观察位。
            if (digital_changed) observed_mask_o[0] <= 1'b1;
            if (gnss_valid) observed_mask_o[1] <= 1'b1;
            if (gy39_lux_valid || gy39_environment_valid)
                observed_mask_o[2] <= 1'b1;
            if (hc_sample_valid) observed_mask_o[3] <= 1'b1;
            if (itr_sample_valid || itr_period_valid)
                observed_mask_o[4] <= 1'b1;
            if (jy_valid) observed_mask_o[5] <= 1'b1;
            if (maxsonar_valid) observed_mask_o[6] <= 1'b1;
            if (bumper_collision_event) observed_mask_o[7] <= 1'b1;
            if (mt_pixel_valid) observed_mask_o[8] <= 1'b1;
            if (multi_ir_changed) observed_mask_o[9] <= 1'b1;
            if (ov5640_pixel_valid) observed_mask_o[10] <= 1'b1;
            if (ov7725_pixel_valid) observed_mask_o[11] <= 1'b1;
            if (encoder_step_valid || encoder_invalid)
                observed_mask_o[12] <= 1'b1;
            if (scc_pixel_valid) observed_mask_o[13] <= 1'b1;
            if (tcrt_line_rise || tcrt_line_fall)
                observed_mask_o[14] <= 1'b1;
            if (tf_valid) observed_mask_o[15] <= 1'b1;
            if (tsl_frame_done) observed_mask_o[16] <= 1'b1;

            // 精确黄金结果；错误脉冲一旦出现，该类测试不再允许 PASS。
            if (digital_changed && digital_object_near)
                pass_mask_o[0] <= 1'b1;
            if (gnss_valid && gnss_type == 2'd1 &&
                gnss_length == 8'd11 && gnss_checksum == 8'h4a &&
                !uart_error_seen_q)
                pass_mask_o[1] <= 1'b1;
            if (gy39_lux_valid && gy39_lux == 32'h00003039 &&
                !uart_error_seen_q)
                pass_mask_o[2] <= 1'b1;
            if (hc_sample_valid && hc_pulse_width_us == 16'd58 &&
                hc_distance_mm == 16'd10 && !hc_error_seen_q)
                pass_mask_o[3] <= 1'b1;

            if (itr_period_valid && itr_period >= 32'd10 &&
                itr_period <= 32'd20)
                itr_period_match_q <= 1'b1;
            if (itr_sample_valid && itr_count == 32'd2 &&
                itr_period_match_q && !itr_error_seen_q)
                pass_mask_o[4] <= 1'b1;

            if (jy_valid && jy_roll == 16'sh1234 &&
                jy_pitch == 16'sh8000 && jy_yaw == -16'sd2 &&
                jy_version == 16'h5678 && !uart_error_seen_q)
                pass_mask_o[5] <= 1'b1;
            if (maxsonar_valid && maxsonar_range == 16'd1234 &&
                !uart_error_seen_q)
                pass_mask_o[6] <= 1'b1;
            if (bumper_collision_event && bumper_left_pressed &&
                !bumper_right_pressed && bumper_collision &&
                bumper_collision_latched)
                pass_mask_o[7] <= 1'b1;
            if (mt_pixel_valid && mt_pixel == 8'h3c && mt_pixel_x == 16'd1)
                pass_mask_o[8] <= 1'b1;
            if (multi_ir_changed && multi_ir_mask == 8'h02)
                pass_mask_o[9] <= 1'b1;
            if (ov5640_pixel_valid && ov5640_pixel == 16'hf800)
                pass_mask_o[10] <= 1'b1;
            if (ov7725_pixel_valid && ov7725_pixel == 16'hf800 &&
                ov7725_x == 16'd1 && !dvp_error_seen_q)
                pass_mask_o[11] <= 1'b1;

            if (encoder_step_valid && encoder_step_count_q != 4'hf)
                encoder_step_count_q <= encoder_step_count_q + 1'b1;
            if (encoder_invalid) encoder_invalid_seen_q <= 1'b1;
            if (encoder_invalid && encoder_position == 32'sd4 &&
                encoder_step_count_q == 4'd4)
                pass_mask_o[12] <= 1'b1;

            if (scc_pixel_valid && scc_pixel == 16'h07e0 &&
                scc_x == 16'd1 && !dvp_error_seen_q)
                pass_mask_o[13] <= 1'b1;
            if (tcrt_line_rise && tcrt_line_detected &&
                tcrt_filter_level == 32'd3)
                pass_mask_o[14] <= 1'b1;
            if (tf_valid && tf_distance == 16'd1234 &&
                tf_strength == 16'd567 && tf_temperature == 16'd2104 &&
                !uart_error_seen_q)
                pass_mask_o[15] <= 1'b1;

            // TSL1401：独立统计 SI 后的 CLK 上升沿和 ADC 有效样本数。
            tsl_sensor_clk_d_q <= tsl_sensor_clk;
            tsl_sensor_si_d_q <= tsl_sensor_si;
            if (tsl_sensor_si && !tsl_sensor_si_d_q) begin
                tsl_line_clock_count_q <= 9'd0;
            end else if (tsl_sensor_clk && !tsl_sensor_clk_d_q) begin
                tsl_line_clock_count_q <= tsl_line_clock_count_q + 1'b1;
            end
            if (tsl_frame_start) begin
                tsl_pixel_count_q <= 9'd0;
                tsl_first_pixel_match_q <= 1'b0;
                tsl_last_pixel_match_q <= 1'b0;
            end
            if (tsl_pixel_valid) begin
                tsl_pixel_count_q <= tsl_pixel_count_q + 1'b1;
                if (tsl_pixel_index == 7'd0 && tsl_pixel_data == 12'ha00)
                    tsl_first_pixel_match_q <= 1'b1;
                if (tsl_pixel_index == 7'd127 && tsl_pixel_data == 12'ha7f)
                    tsl_last_pixel_match_q <= 1'b1;
            end
            if (tsl_frame_done && tsl_line_clock_count_q == 9'd129 &&
                tsl_pixel_count_q == 9'd128 &&
                tsl_first_pixel_match_q && tsl_last_pixel_match_q &&
                !tsl_adc_timeout)
                pass_mask_o[16] <= 1'b1;
        end
    end
endmodule
