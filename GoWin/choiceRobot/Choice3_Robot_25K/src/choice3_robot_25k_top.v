`timescale 1ns/1ps

// 选题三 25K 正式顶层。ONCHIP_DEMO=0 接真实外设；板级无外设验收由 demo wrapper 置 1。
module choice3_robot_25k_top #(
    parameter integer ONCHIP_DEMO = 0
) (
    input wire clk_50m_i,input wire reset_i,
    input wire uart_rx_i,output wire uart_tx_o,
    output wire tmds_clk_p_o,output wire tmds_clk_n_o,
    output wire [2:0] tmds_data_p_o,output wire [2:0] tmds_data_n_o,
    input wire ov_pclk_i,input wire ov_href_i,input wire ov_vsync_i,
    input wire [7:0] ov_data_i,output wire ov_xclk_o,
    output wire ov_reset_n_o,output wire ov_pwdn_o,
    inout wire ov_sccb_scl_io,inout wire ov_sccb_sda_io,
    input wire imu_uart_rx_i,input wire range_uart_rx_i,
    input wire [3:0] ir_sensor_i,
    input wire [3:0] encoder_a_i,input wire [3:0] encoder_b_i,
    input wire estop_n_i,
    output wire [3:0] motor_pwm_o,output wire [3:0] motor_in1_o,
    output wire [3:0] motor_in2_o,output wire motor_stby_o,
    inout wire [3:0] reserve_io
);
    GSR u_gsr(.GSRI(1'b1));
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

    // ---------------- 摄像头与显示帧缓存 ----------------
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

    reg cam_toggle_q;
    always @(posedge ov_pclk_i or negedge cam_rst_n)begin
      if(!cam_rst_n)cam_toggle_q<=0;else if(cam_commit)cam_toggle_q<=~cam_toggle_q;end
    wire camera_health_real,camera_event;
    choice3_event_freshness #(.TIMEOUT_CYCLES(75_000_000)) u_cam_fresh(
      .clk_i(clk_50m_i),.rst_n_i(sys_rst_n),.event_toggle_async_i(cam_toggle_q),
      .source_ready_i(ov_configured&&!ov_identity_error&&!ov_bus_error),
      .health_o(camera_health_real),.event_pulse_o(camera_event));

    // ---------------- JY61P 姿态角与 TF-Luna 激光测距 ----------------
    wire signed [15:0] jy_roll_raw,jy_pitch_raw,jy_yaw_raw;
    wire [15:0] jy_version;
    wire jy_frame_valid,jy_checksum_error,jy_framing_error;
    jy61p_uart #(.CLK_FREQ_HZ(50_000_000),.BAUD_RATE(9600)) u_jy61p(
      .clk(clk_50m_i),.rst_n(sys_rst_n),.uart_rx(imu_uart_rx_i),
      .roll_raw(jy_roll_raw),.pitch_raw(jy_pitch_raw),.yaw_raw(jy_yaw_raw),
      .version_raw(jy_version),.frame_valid(jy_frame_valid),
      .checksum_error(jy_checksum_error),.framing_error(jy_framing_error));
    wire signed [15:0] real_roll_deg,real_pitch_deg,real_yaw_deg;
    wire real_angle_valid;
    choice3_attitude_degrees u_angle_convert(
      .clk_i(clk_50m_i),.rst_n_i(sys_rst_n),.sample_valid_i(jy_frame_valid),
      .roll_raw_i(jy_roll_raw),.pitch_raw_i(jy_pitch_raw),.yaw_raw_i(jy_yaw_raw),
      .roll_deg_o(real_roll_deg),.pitch_deg_o(real_pitch_deg),.yaw_deg_o(real_yaw_deg),
      .valid_o(real_angle_valid));
    reg imu_transport_ok_q,imu_toggle_q;
    always @(posedge clk_50m_i or negedge sys_rst_n)begin
      if(!sys_rst_n)begin imu_transport_ok_q<=0;imu_toggle_q<=0;end
      else begin
        if(jy_checksum_error||jy_framing_error)imu_transport_ok_q<=0;
        if(real_angle_valid)begin imu_transport_ok_q<=1;imu_toggle_q<=~imu_toggle_q;end
      end
    end
    wire imu_health_real,imu_event;
    choice3_event_freshness #(.TIMEOUT_CYCLES(5_000_000)) u_imu_fresh(
      .clk_i(clk_50m_i),.rst_n_i(sys_rst_n),.event_toggle_async_i(imu_toggle_q),
      .source_ready_i(imu_transport_ok_q),.health_o(imu_health_real),.event_pulse_o(imu_event));

    wire [15:0] tf_distance_cm,tf_strength,tf_temperature;
    wire tf_frame_valid,tf_checksum_error;
    tf_luna_uart #(.CLK_FREQ_HZ(50_000_000),.BAUD_RATE(115200)) u_tf_luna(
      .clk(clk_50m_i),.rst_n(sys_rst_n),.uart_rx(range_uart_rx_i),
      .distance_cm(tf_distance_cm),.strength(tf_strength),
      .temperature_raw(tf_temperature),.frame_valid(tf_frame_valid),
      .checksum_error(tf_checksum_error));
    wire [19:0] tf_distance_mm_w=tf_distance_cm*20'd10;
    wire tf_good_sample=tf_frame_valid&&tf_strength>=16'd20&&
                        tf_distance_cm>=16'd20&&tf_distance_cm<=16'd800;
    reg range_transport_ok_q,range_toggle_q;
    always @(posedge clk_50m_i or negedge sys_rst_n)begin
      if(!sys_rst_n)begin range_transport_ok_q<=0;range_toggle_q<=0;end
      else begin
        if(tf_checksum_error)range_transport_ok_q<=0;
        if(tf_good_sample)begin range_transport_ok_q<=1;range_toggle_q<=~range_toggle_q;end
      end
    end
    wire range_health_real,range_event;
    choice3_event_freshness #(.TIMEOUT_CYCLES(10_000_000)) u_range_fresh(
      .clk_i(clk_50m_i),.rst_n_i(sys_rst_n),.event_toggle_async_i(range_toggle_q),
      .source_ready_i(range_transport_ok_q),.health_o(range_health_real),
      .event_pulse_o(range_event));

    wire [3:0] ir_active_real;wire ir_changed;
    multi_ir_array #(.CHANNELS(4),.DEBOUNCE_CYCLES(50_000),.ACTIVE_LOW(1'b1))
      u_ir(.clk(clk_50m_i),.rst_n(sys_rst_n),.sensor_i(ir_sensor_i),
      .active_mask_o(ir_active_real),.change_valid_o(ir_changed));

    // ---------------- 控制、安全、片上闭环场景 ----------------
    wire arm_cmd,disarm_cmd,clear_cmd,route_cmd,manual_cmd_valid;
    wire [2:0] manual_cmd;
    wire armed,motor_enable,arm_rejected;
    wire [7:0] health,fault_now,fault_latched;
    wire [3:0] nav_state;wire route_done;
    wire [3:0] pwm_internal,in1_internal,in2_internal,deadtime;
    wire stby_internal;wire [63:0] duty_flat;wire [15:0] fused_mm;
    wire fused_valid,front_blocked,sample_tick;
    wire [3:0] demo_encoder_a,demo_encoder_b,demo_ir;
    wire [15:0] demo_primary_mm,demo_secondary_mm;
    wire demo_primary_valid,demo_secondary_valid,demo_secondary_timeout;
    wire signed [15:0] demo_roll_deg,demo_pitch_deg,demo_yaw_deg;
    wire demo_attitude_valid;
    wire [6:0] demo_pass_low;

    wire [3:0] encoder_a_mux=ONCHIP_DEMO?demo_encoder_a:encoder_a_i;
    wire [3:0] encoder_b_mux=ONCHIP_DEMO?demo_encoder_b:encoder_b_i;
    wire [3:0] ir_mux=ONCHIP_DEMO?demo_ir:ir_active_real;
    wire [15:0] primary_mm=ONCHIP_DEMO?demo_primary_mm:tf_distance_mm_w[15:0];
    wire primary_valid=ONCHIP_DEMO?demo_primary_valid:tf_good_sample;
    wire [15:0] secondary_mm=ONCHIP_DEMO?demo_secondary_mm:16'hffff;
    wire secondary_valid=ONCHIP_DEMO?demo_secondary_valid:1'b0;
    wire secondary_timeout=ONCHIP_DEMO?demo_secondary_timeout:1'b1;
    wire signed [15:0] roll_deg_sys=ONCHIP_DEMO?demo_roll_deg:real_roll_deg;
    wire signed [15:0] pitch_deg_sys=ONCHIP_DEMO?demo_pitch_deg:real_pitch_deg;
    wire signed [15:0] yaw_deg_sys=ONCHIP_DEMO?demo_yaw_deg:real_yaw_deg;
    wire attitude_sample=ONCHIP_DEMO?demo_attitude_valid:real_angle_valid;
    wire [3:0] peripheral_health=ONCHIP_DEMO?4'b1111:
      {1'b0,range_health_real,imu_health_real,camera_health_real};

    // ---------------- PicoRV32 服务面与独立失联看门狗 ----------------
    wire wb_stb,wb_we,wb_cyc,wb_ack;
    wire [31:0] wb_addr,wb_wdata,wb_rdata;wire [3:0] wb_sel;
    wire [31:0] cpu_heartbeat,cpu_scratch,cpu_txn_count,cpu_error_count;
    wire heartbeat_write,cpu_alive,cpu_watchdog_fault,cpu_allow_commit,cpu_allow_start;
    wire cpu_path_abort;
    Gowin_PicoRV32_Top u_picorv32(
      .ser_tx(uart_tx_o),.ser_rx(uart_rx_i),
      .slv_ext_stb_o(wb_stb),.slv_ext_we_o(wb_we),.slv_ext_cyc_o(wb_cyc),
      .slv_ext_ack_i(wb_ack),.slv_ext_adr_o(wb_addr),.slv_ext_wdata_o(wb_wdata),
      .slv_ext_rdata_i(wb_rdata),.slv_ext_sel_o(wb_sel),.irq_in(12'b0),
      .clk_in(clk_50m_i),.resetn_in(sys_rst_n));
    wire physical_outputs_zero=(motor_pwm_o==4'b0000)&&(motor_in1_o==4'b0000)&&
                               (motor_in2_o==4'b0000)&&!motor_stby_o;
    wire [7:0] demo_pass=ONCHIP_DEMO?{physical_outputs_zero,demo_pass_low}:8'd0;
    choice3_robot_service_regs u_service_regs(
      .wb_clk_i(clk_50m_i),.wb_rst_i(!sys_rst_n),.wb_cyc_i(wb_cyc),.wb_stb_i(wb_stb),
      .wb_we_i(wb_we),.wb_adr_i(wb_addr),.wb_dat_i(wb_wdata),.wb_sel_i(wb_sel),
      .wb_dat_o(wb_rdata),.wb_ack_o(wb_ack),.health_i(health),
      .fault_i(fault_now|fault_latched),.nav_state_i(nav_state),.route_done_i(route_done),
      .drive_enabled_i(stby_internal),.armed_i(armed),.demo_mode_i(ONCHIP_DEMO!=0),
      .cpu_watchdog_fault_i(cpu_watchdog_fault),.fused_range_mm_i(fused_mm),
      .fused_range_valid_i(fused_valid),.front_blocked_i(front_blocked),
      .roll_deg_i(roll_deg_sys),.pitch_deg_i(pitch_deg_sys),.yaw_deg_i(yaw_deg_sys),
      .demo_pass_i(demo_pass),.heartbeat_o(cpu_heartbeat),
      .heartbeat_write_o(heartbeat_write),.arm_pulse_o(arm_cmd),
      .disarm_pulse_o(disarm_cmd),.route_start_pulse_o(route_cmd),
      .clear_fault_pulse_o(clear_cmd),.manual_command_valid_o(manual_cmd_valid),
      .manual_motion_o(manual_cmd),.scratch_o(cpu_scratch),
      .transaction_count_o(cpu_txn_count),.error_count_o(cpu_error_count));
    softcore_service_watchdog #(.TIMEOUT_CYCLES(5_000_000)) u_cpu_watchdog(
      .clk_i(clk_50m_i),.safety_reset_n_i(sys_rst_n),
      .heartbeat_write_i(heartbeat_write),.alive_o(cpu_alive),.fault_o(cpu_watchdog_fault),
      .allow_commit_o(cpu_allow_commit),.allow_start_o(cpu_allow_start),
      .path_abort_o(cpu_path_abort));

    wire control_estop_n=cpu_alive&&(ONCHIP_DEMO?1'b1:estop_n_i);
    choice3_control_core #(.PWM_PERIOD(2500),.DEADTIME_CYCLES(5000),
      .REQUIRED_HEALTH_MASK(8'h17)) u_control(
      .clk_i(clk_50m_i),.rst_n_i(sys_rst_n),.estop_n_i(control_estop_n),
      .peripheral_health_i(peripheral_health),.arm_pulse_i(arm_cmd),
      .disarm_pulse_i(disarm_cmd||cpu_path_abort),.clear_fault_pulse_i(clear_cmd),
      .route_start_pulse_i(route_cmd&&cpu_allow_start),
      .manual_command_valid_i(manual_cmd_valid),.manual_motion_i(manual_cmd),
      .vl53_distance_mm_i(primary_mm),.vl53_valid_i(primary_valid),
      .ultrasonic_distance_mm_i(secondary_mm),.ultrasonic_valid_i(secondary_valid),
      .ultrasonic_timeout_i(secondary_timeout),.ir_active_i(ir_mux),
      .encoder_a_i(encoder_a_mux),.encoder_b_i(encoder_b_mux),
      .motor_pwm_o(pwm_internal),.motor_in1_o(in1_internal),.motor_in2_o(in2_internal),
      .motor_stby_o(stby_internal),.armed_o(armed),.motor_enable_o(motor_enable),
      .health_o(health),.fault_now_o(fault_now),.fault_latched_o(fault_latched),
      .arm_rejected_pulse_o(arm_rejected),.nav_state_o(nav_state),
      .route_done_o(route_done),.fused_front_mm_o(fused_mm),
      .fused_front_valid_o(fused_valid),.front_blocked_o(front_blocked),
      .pwm_duty_flat_o(duty_flat),.deadtime_active_o(deadtime),
      .manual_mode_o(),.manual_motion_o(),.sample_tick_o(sample_tick));

    choice3_demo_stimulus u_demo(
      .clk_i(clk_50m_i),.rst_n_i(sys_rst_n),.sample_tick_i(sample_tick),
      .motor_pwm_i(pwm_internal),.motor_in1_i(in1_internal),.motor_in2_i(in2_internal),
      .nav_state_i(nav_state),.route_done_i(route_done),.video_frame_start_i(video_frame_start),
      .encoder_a_o(demo_encoder_a),.encoder_b_o(demo_encoder_b),
      .vl53_distance_mm_o(demo_primary_mm),.vl53_valid_o(demo_primary_valid),
      .ultrasonic_distance_mm_o(demo_secondary_mm),
      .ultrasonic_valid_o(demo_secondary_valid),
      .ultrasonic_timeout_o(demo_secondary_timeout),.ir_active_o(demo_ir),
      .roll_deg_o(demo_roll_deg),.pitch_deg_o(demo_pitch_deg),.yaw_deg_o(demo_yaw_deg),
      .attitude_valid_o(demo_attitude_valid),.pass_mask_o(demo_pass_low));

    // DEMO 下真实电机输出无条件为 0；内部 PWM 只反馈给片上电机/编码器模型。
    wire physical_drive_enable=arst_n&&estop_n_i&&cpu_alive&&motor_enable&&!ONCHIP_DEMO;
    assign motor_pwm_o=physical_drive_enable?pwm_internal:4'b0000;
    assign motor_in1_o=physical_drive_enable?in1_internal:4'b0000;
    assign motor_in2_o=physical_drive_enable?in2_internal:4'b0000;
    assign motor_stby_o=physical_drive_enable?stby_internal:1'b0;

    // ---------------- 姿态快照 CDC、数字 OSD 与 DVI ----------------
    wire signed [15:0] roll_deg_video,pitch_deg_video,yaw_deg_video;
    wire attitude_valid_video;
    choice3_attitude_cdc u_attitude_cdc(
      .src_clk_i(clk_50m_i),.src_rst_n_i(sys_rst_n),.src_valid_i(attitude_sample),
      .src_roll_i(roll_deg_sys),.src_pitch_i(pitch_deg_sys),.src_yaw_i(yaw_deg_sys),
      .dst_clk_i(pixel_clk),.dst_rst_n_i(video_rst_n),.dst_roll_o(roll_deg_video),
      .dst_pitch_o(pitch_deg_video),.dst_yaw_o(yaw_deg_video),
      .dst_valid_o(attitude_valid_video));
    wire [7:0] video_r,video_g,video_b;wire video_hs,video_vs,video_de;
    wire [9:0] video_h_count,video_v_count;
    choice3_vga_640x480 u_vga(.pixel_clk_i(pixel_clk),.rst_n_i(video_rst_n),
      .framebuffer_pixel_i(framebuffer_pixel),.framebuffer_valid_i(display_frame_valid),
      .fault_mask_i(fault_now|fault_latched),.roll_deg_i(roll_deg_video),
      .pitch_deg_i(pitch_deg_video),.yaw_deg_i(yaw_deg_video),
      .attitude_valid_i(attitude_valid_video),.demo_pattern_i(ONCHIP_DEMO!=0),
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
