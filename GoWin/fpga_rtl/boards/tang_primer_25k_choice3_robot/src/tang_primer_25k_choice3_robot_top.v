`timescale 1ns/1ps

// Tang Primer 25K 赛题三整机顶层。所有电机引脚在 raw 急停/复位下组合拉低。
module tang_primer_25k_choice3_robot_top (
    input wire clk_50m_i,input wire reset_i,
    input wire uart_rx_i,output wire uart_tx_o,
    output wire tmds_clk_p_o,output wire tmds_clk_n_o,
    output wire [2:0] tmds_data_p_o,output wire [2:0] tmds_data_n_o,
    input wire ov_pclk_i,input wire ov_href_i,input wire ov_vsync_i,
    input wire [7:0] ov_data_i,output wire ov_xclk_o,
    output wire ov_reset_n_o,output wire ov_pwdn_o,
    inout wire ov_sccb_scl_io,inout wire ov_sccb_sda_io,
    output wire [3:0] motor_pwm_o,output wire [3:0] motor_in1_o,
    output wire [3:0] motor_in2_o,output wire motor_stby_o,
    input wire [3:0] encoder_a_i,input wire [3:0] encoder_b_i,
    output wire icm_cs_n_o,output wire icm_sclk_o,output wire icm_mosi_o,
    input wire icm_miso_i,
    inout wire vl53_scl_io,inout wire vl53_sda_io,
    output wire hc_trigger_o,input wire hc_echo_i,
    input wire [3:0] ir_sensor_i,input wire estop_n_i,
    inout wire [3:0] reserve_io
);
    // GW5A 高速原语仿真/实现所需的全局置复位网，1 表示释放。
    GSR GSR(.GSRI(1'b1));
    wire arst_n=~reset_i;
    wire sys_rst_n,video_rst_n,serial_rst_n,cam_rst_n;
    wire serial_clk,pixel_clk,camera_cfg_clk,pll_locked;
    choice3_reset_sync u_sys_reset(.clk_i(clk_50m_i),.arst_n_i(arst_n),.rst_n_o(sys_rst_n));
    choice3_video_clocks u_clocks(.clk_50m_i(clk_50m_i),.rst_n_i(arst_n),
      .serial_125m_o(serial_clk),.pixel_25m_o(pixel_clk),
      .camera_24m_o(camera_cfg_clk),.pll_locked_o(pll_locked));
    choice3_reset_sync u_video_reset(.clk_i(pixel_clk),
      .arst_n_i(arst_n&&pll_locked),.rst_n_o(video_rst_n));
    choice3_reset_sync u_serial_reset(.clk_i(serial_clk),
      .arst_n_i(arst_n&&pll_locked),.rst_n_o(serial_rst_n));

    // OV5640 SCCB/QVGA RGB565 初始化运行在 system 域；XCLK 独立取 PLL 23.958MHz。
    // 这样 configured/error 无需跨域，且延时参数明确按 50MHz 换算。
    wire ov_configured,ov_identity_error,ov_bus_error;
    wire [15:0] ov_chip_id;
    wire ov_xclk_unused;
    assign ov_xclk_o=camera_cfg_clk;
    ov5640_sccb_init #(.SCCB_CLK_DIV(64),.POWERDOWN_HOLD_CYCLES(500_000),
      .RESET_HOLD_CYCLES(500_000),.STARTUP_DELAY_CYCLES(1_000_000),
      .SOFT_RESET_DELAY_CYCLES(250_000)) u_ov_init(
      .clk(clk_50m_i),.reset_n(sys_rst_n),.camera_xclk(ov_xclk_unused),
      .camera_reset_n(ov_reset_n_o),.camera_pwdn(ov_pwdn_o),
      .camera_scl(ov_sccb_scl_io),.camera_sda(ov_sccb_sda_io),
      .configured(ov_configured),.identity_error(ov_identity_error),
      .bus_error(ov_bus_error),.chip_id(ov_chip_id));
    choice3_reset_sync u_cam_reset(.clk_i(ov_pclk_i),
      .arst_n_i(arst_n&&ov_configured),.rst_n_o(cam_rst_n));
    wire cam_frame_valid,cam_line_valid,cam_pixel_valid;
    wire [15:0] cam_pixel_rgb565;
    ov5640_dvp_capture #(.DVP_LITTLE_ENDIAN(1'b1)) u_ov_capture(
      .pclk_i(ov_pclk_i),.rst_n_i(cam_rst_n),.vsync_i(ov_vsync_i),
      .href_i(ov_href_i),.data_i(ov_data_i),.frame_valid_o(cam_frame_valid),
      .line_valid_o(cam_line_valid),.pixel_valid_o(cam_pixel_valid),
      .pixel_rgb565_o(cam_pixel_rgb565));

    wire [14:0] framebuffer_offset;
    wire [11:0] framebuffer_pixel;
    wire display_frame_valid,video_frame_start,cam_commit,video_commit,display_bank;
    wire [15:0] dropped_frames;
    choice3_camera_double_buffer u_framebuffer(
      .cam_pclk_i(ov_pclk_i),.cam_rst_n_i(cam_rst_n),
      .cam_frame_valid_i(cam_frame_valid),.cam_line_valid_i(cam_line_valid),
      .cam_pixel_valid_i(cam_pixel_valid),.cam_pixel_rgb565_i(cam_pixel_rgb565),
      .video_clk_i(pixel_clk),.video_rst_n_i(video_rst_n),
      .video_frame_start_i(video_frame_start),.video_read_offset_i(framebuffer_offset),
       .video_pixel_rgb444_o(framebuffer_pixel),.video_frame_valid_o(display_frame_valid),
      .video_frame_commit_pulse_o(video_commit),.cam_frame_commit_pulse_o(cam_commit),
      .dropped_frame_count_o(dropped_frames),.display_bank_o(display_bank));

    // ICM-42688-P：50MHz 计数参数显式换算，SPI Mode0。
    wire icm_ok,icm_initialized,icm_sample_valid;
    wire signed [15:0] accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z;
    icm42688p_spi #(.CLK_DIV(8),.STARTUP_DELAY_CYCLES(500_000),
      .SENSOR_READY_DELAY_CYCLES(2_250_000),.SAMPLE_INTERVAL_CYCLES(500_000))
      u_icm(.clk(clk_50m_i),.reset_n(sys_rst_n),.spi_miso(icm_miso_i),
      .spi_cs_n(icm_cs_n_o),.spi_sclk(icm_sclk_o),.spi_mosi(icm_mosi_o),
      .sensor_ok(icm_ok),.initialized(icm_initialized),.accel_x(accel_x),
      .accel_y(accel_y),.accel_z(accel_z),.gyro_x(gyro_x),.gyro_y(gyro_y),
       .gyro_z(gyro_z),.sample_valid(icm_sample_valid));
    wire icm_good_sample,icm_sentinel_rejected;
    choice3_imu_sample_guard u_imu_guard(.sample_valid_i(icm_sample_valid),
      .accel_x_i(accel_x),.accel_y_i(accel_y),.accel_z_i(accel_z),
      .gyro_x_i(gyro_x),.gyro_y_i(gyro_y),.gyro_z_i(gyro_z),
      .good_sample_o(icm_good_sample),.sentinel_rejected_o(icm_sentinel_rejected));
    wire signed [9:0] roll_proxy_sys,pitch_proxy_sys;
    wire attitude_proxy_valid_sys;
    choice3_attitude_tilt_proxy u_attitude_proxy(.clk_i(clk_50m_i),.rst_n_i(sys_rst_n),
      .sample_valid_i(icm_good_sample),.accel_x_i(accel_x),.accel_y_i(accel_y),
      .accel_z_i(accel_z),.roll_proxy_px_o(roll_proxy_sys),
      .pitch_proxy_px_o(pitch_proxy_sys),.valid_o(attitude_proxy_valid_sys));
    wire signed [9:0] roll_proxy_video,pitch_proxy_video;
    wire attitude_proxy_valid_video;
    choice3_attitude_cdc u_attitude_cdc(.src_clk_i(clk_50m_i),.src_rst_n_i(sys_rst_n),
      .src_valid_i(icm_good_sample&&attitude_proxy_valid_sys),.src_roll_i(roll_proxy_sys),
      .src_pitch_i(pitch_proxy_sys),.dst_clk_i(pixel_clk),.dst_rst_n_i(video_rst_n),
      .dst_roll_o(roll_proxy_video),.dst_pitch_o(pitch_proxy_video),
      .dst_valid_o(attitude_proxy_valid_video));

    // VL53L0X：I2C约390.6kHz；raw status 仅诊断，不伪装成PAL质量枚举。
    wire vl_ok,vl_initialized,vl_identity_error,vl_bus_error,vl_timeout;
    wire [7:0] vl_model_id,vl_range_status_raw;wire [2:0] vl_interrupt_status;
    wire [15:0] vl_distance;wire vl_distance_valid;
    vl53l0x_i2c #(.I2C_CLK_DIV(32),.STARTUP_DELAY_CYCLES(500_000),
      .SAMPLE_INTERVAL_CYCLES(50_000)) u_vl53(.clk(clk_50m_i),.reset_n(sys_rst_n),
      .i2c_scl(vl53_scl_io),.i2c_sda(vl53_sda_io),.sensor_ok(vl_ok),
      .initialized(vl_initialized),.identity_error(vl_identity_error),
      .bus_error(vl_bus_error),.timeout_error(vl_timeout),.model_id(vl_model_id),
      .interrupt_status(vl_interrupt_status),.range_status_raw(vl_range_status_raw),
      .distance_mm(vl_distance),.distance_valid(vl_distance_valid));
    wire vl_good_sample=vl_distance_valid&&vl_distance>=16'd30&&
                        vl_distance<=16'd4000&&vl_distance!=16'hffff;

    wire [15:0] hc_distance,hc_pulse_us;wire hc_sample_valid,hc_timeout,hc_busy;
    hc_sr04_controller #(.CLK_FREQ_HZ(50_000_000)) u_hc(.clk(clk_50m_i),
      .rst_n(sys_rst_n),.echo_in(hc_echo_i),.trigger_out(hc_trigger_o),
      .distance_mm(hc_distance),.pulse_width_us(hc_pulse_us),
      .sample_valid(hc_sample_valid),.timeout_error(hc_timeout),.busy(hc_busy));
    wire hc_good_sample=hc_sample_valid&&hc_distance>=16'd20&&hc_distance<=16'd4000;

    wire [3:0] ir_active;wire ir_changed;
    multi_ir_array #(.CHANNELS(4),.DEBOUNCE_CYCLES(50_000),.ACTIVE_LOW(1'b1))
      u_ir(.clk(clk_50m_i),.rst_n(sys_rst_n),.sensor_i(ir_sensor_i),
      .active_mask_o(ir_active),.change_valid_o(ir_changed));

    // 各外设事件转 toggle 后统一在50MHz域做 freshness。
    reg cam_toggle_q,icm_toggle_q,vl_toggle_q,hc_toggle_q;
    always @(posedge ov_pclk_i or negedge cam_rst_n)begin
      if(!cam_rst_n)cam_toggle_q<=0;else if(cam_commit)cam_toggle_q<=~cam_toggle_q;end
    always @(posedge clk_50m_i or negedge sys_rst_n)begin
      if(!sys_rst_n)begin icm_toggle_q<=0;vl_toggle_q<=0;hc_toggle_q<=0;end
      else begin
        if(icm_good_sample)icm_toggle_q<=~icm_toggle_q;
        if(vl_good_sample)vl_toggle_q<=~vl_toggle_q;
        if(hc_good_sample)hc_toggle_q<=~hc_toggle_q;
      end
    end
    wire camera_health,imu_health,vl_health,hc_health;
    wire camera_event,imu_event,vl_event,hc_event;
    choice3_event_freshness #(.TIMEOUT_CYCLES(75_000_000)) u_cam_fresh(
      .clk_i(clk_50m_i),.rst_n_i(sys_rst_n),.event_toggle_async_i(cam_toggle_q),
      .source_ready_i(ov_configured&&!ov_identity_error&&!ov_bus_error),
      .health_o(camera_health),.event_pulse_o(camera_event));
    choice3_event_freshness #(.TIMEOUT_CYCLES(2_000_000)) u_imu_fresh(
      .clk_i(clk_50m_i),.rst_n_i(sys_rst_n),.event_toggle_async_i(icm_toggle_q),
      .source_ready_i(icm_ok&&icm_initialized&&attitude_proxy_valid_sys),
      .health_o(imu_health),.event_pulse_o(imu_event));
    choice3_event_freshness #(.TIMEOUT_CYCLES(25_000_000)) u_vl_fresh(
      .clk_i(clk_50m_i),.rst_n_i(sys_rst_n),.event_toggle_async_i(vl_toggle_q),
      .source_ready_i(vl_ok&&vl_initialized&&!vl_identity_error&&!vl_bus_error&&!vl_timeout),
      .health_o(vl_health),.event_pulse_o(vl_event));
    choice3_event_freshness #(.TIMEOUT_CYCLES(10_000_000)) u_hc_fresh(
      .clk_i(clk_50m_i),.rst_n_i(sys_rst_n),.event_toggle_async_i(hc_toggle_q),
      .source_ready_i(!hc_timeout),.health_o(hc_health),.event_pulse_o(hc_event));

    wire arm_cmd,disarm_cmd,clear_cmd,route_cmd,manual_cmd_valid;
    wire [2:0] manual_cmd;
    wire armed,motor_enable,arm_rejected;
    wire [7:0] health,fault_now,fault_latched;
    wire [3:0] nav_state;wire route_done;
    wire [3:0] pwm_internal,in1_internal,in2_internal,deadtime;
    wire stby_internal;wire [63:0] duty_flat;wire [15:0] fused_mm;
    wire fused_valid,front_blocked,sample_tick;
    choice3_uart_console u_console(.clk_i(clk_50m_i),.rst_n_i(sys_rst_n),
      .uart_rx_i(uart_rx_i),.uart_tx_o(uart_tx_o),.armed_i(armed),
      .drive_enabled_i(motor_stby_o),
      .health_i(health),.fault_i(fault_now|fault_latched),.nav_state_i(nav_state),
      .vl53_range_status_raw_i(vl_range_status_raw),.arm_rejected_pulse_i(arm_rejected),
      .arm_pulse_o(arm_cmd),.disarm_pulse_o(disarm_cmd),
      .clear_fault_pulse_o(clear_cmd),.route_start_pulse_o(route_cmd),
      .manual_command_valid_o(manual_cmd_valid),.manual_motion_o(manual_cmd));
    choice3_control_core #(.PWM_PERIOD(2500),.DEADTIME_CYCLES(5000))
      u_control(.clk_i(clk_50m_i),.rst_n_i(sys_rst_n),
      .estop_n_i(estop_n_i),.peripheral_health_i({hc_health,vl_health,imu_health,camera_health}),
      .arm_pulse_i(arm_cmd),.disarm_pulse_i(disarm_cmd),
      .clear_fault_pulse_i(clear_cmd),.route_start_pulse_i(route_cmd),
      .manual_command_valid_i(manual_cmd_valid),.manual_motion_i(manual_cmd),
      .vl53_distance_mm_i(vl_distance),.vl53_valid_i(vl_good_sample),
      .ultrasonic_distance_mm_i(hc_distance),.ultrasonic_valid_i(hc_good_sample),
      .ultrasonic_timeout_i(hc_timeout||(hc_sample_valid&&!hc_good_sample)),
      .ir_active_i(ir_active),.encoder_a_i(encoder_a_i),.encoder_b_i(encoder_b_i),
      .motor_pwm_o(pwm_internal),.motor_in1_o(in1_internal),.motor_in2_o(in2_internal),
      .motor_stby_o(stby_internal),.armed_o(armed),.motor_enable_o(motor_enable),
      .health_o(health),.fault_now_o(fault_now),.fault_latched_o(fault_latched),
      .arm_rejected_pulse_o(arm_rejected),.nav_state_o(nav_state),
      .route_done_o(route_done),.fused_front_mm_o(fused_mm),
      .fused_front_valid_o(fused_valid),.front_blocked_o(front_blocked),
      .pwm_duty_flat_o(duty_flat),.deadtime_active_o(deadtime),
      .manual_mode_o(),.manual_motion_o(),.sample_tick_o(sample_tick));

    // raw 急停/复位不等待任何时钟，直接覆盖全部功率级控制引脚。
    wire physical_drive_enable=arst_n&&estop_n_i&&motor_enable;
    assign motor_pwm_o=physical_drive_enable?pwm_internal:4'b0000;
    assign motor_in1_o=physical_drive_enable?in1_internal:4'b0000;
    assign motor_in2_o=physical_drive_enable?in2_internal:4'b0000;
    assign motor_stby_o=physical_drive_enable?stby_internal:1'b0;

    wire [7:0] video_r,video_g,video_b;wire video_hs,video_vs,video_de;
    wire [9:0] video_h_count,video_v_count;
    choice3_vga_640x480 u_vga(.pixel_clk_i(pixel_clk),.rst_n_i(video_rst_n),
      .framebuffer_pixel_i(framebuffer_pixel),.framebuffer_valid_i(display_frame_valid),
      .fault_mask_i(fault_now|fault_latched),.attitude_roll_proxy_i(roll_proxy_video),
      .attitude_pitch_proxy_i(pitch_proxy_video),.attitude_valid_i(attitude_proxy_valid_video),
      .framebuffer_offset_o(framebuffer_offset),.frame_start_o(video_frame_start),
      .red_o(video_r),.green_o(video_g),.blue_o(video_b),.hsync_o(video_hs),
      .vsync_o(video_vs),.de_o(video_de),.h_count_o(video_h_count),.v_count_o(video_v_count));
    choice3_hdmi_phy u_hdmi(.pixel_clk_i(pixel_clk),.serial_5x_clk_i(serial_clk),
      .pixel_rst_n_i(video_rst_n),.serial_rst_n_i(serial_rst_n),
      .red_i(video_r),.green_i(video_g),.blue_i(video_b),
      .hsync_i(video_hs),.vsync_i(video_vs),.de_i(video_de),
      .tmds_clk_p_o(tmds_clk_p_o),.tmds_clk_n_o(tmds_clk_n_o),
      .tmds_data_p_o(tmds_data_p_o),.tmds_data_n_o(tmds_data_n_o));

    assign reserve_io=4'bzzzz;
endmodule
