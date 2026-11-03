module robot_top(
    //System clock reset
    input       clk50m        , //系统时钟输入，50MHz
    input       reset_n       , //复位信号输入
    input       n10_uart_rx   ,
    output      imu_cs_n      ,
    input       imu_miso      ,
    output      imu_mosi      ,
    output      imu_sclk      ,
    input [3:0] encoder_a_i, encoder_b_i,
    output [3:0] motor_in1_o, motor_in2_o,

    // OV5647 CSI-2 两 lane 差分输入和 I2C 配置接口。
    output      camera_sclk       ,
    inout       camera_sdat       ,
    input       camera_mipi_clk_p ,
    input       camera_mipi_clk_n ,
    input       camera_mipi_lane0_p,
    input       camera_mipi_lane0_n,
    input       camera_mipi_lane1_p,
    input       camera_mipi_lane1_n,
    input [1:0] camera_mipi_lp_clk,
    input [1:0] camera_mipi_lp_lane0,
    input [1:0] camera_mipi_lp_lane1,
    //led
    output [1:0]led           ,

    //ddr
    output [14:0] O_ddr_addr        ,
    output [2:0] O_ddr_ba           ,
    output O_ddr_cs_n               ,
    output O_ddr_ras_n              ,
    output O_ddr_cas_n              ,
    output O_ddr_we_n               ,
    output O_ddr_clk                ,
    output O_ddr_clk_n              ,
    output O_ddr_cke                ,
    output O_ddr_odt                ,
    output O_ddr_reset_n            ,
    output [3:0] O_ddr_dqm          ,
    inout [31:0] IO_ddr_dq          ,
    inout [3:0] IO_ddr_dqs          ,
    inout [3:0] IO_ddr_dqs_n        ,

    // 千兆 RGMII 双向接口和 RTL8211F-CG 管理接口。
    input       rgmii_rx_clk_i,
    input [3:0] rgmii_rxd,
    input       rgmii_rxdv,
    output      rgmii_gtxc,
    output [3:0] rgmii_txd,
    output      rgmii_txen,
    output      eth_mdc,
    inout       eth_mdio,
    output      eth_rst_n

);
    // OV5647输入1920×1080 RAW10；轻量Bayer彩色管线输出640×360 YUYV422。
    parameter integer IMAGE_WIDTH = 640, IMAGE_HEIGHT = 360;
    // 定位、地图与规划由GoRV32 FP固件执行；外参由固件合同统一维护。
    // 融合外参需按实车标定；坐标 x 前、y 左、z 上。
    parameter integer FUSION_IMU_YAW_AXIS = 2, FUSION_IMU_YAW_SIGN = 1;
    parameter integer FUSION_LIDAR_CLOCKWISE = 0;
    parameter signed [31:0] FUSION_LIDAR_X_Q16 = 0, FUSION_LIDAR_Y_Q16 = 0, FUSION_LIDAR_YAW_Q16 = 0;
    parameter integer FUSION_STILL_SAMPLES = 25, FUSION_STILL_V_MM_S_LIMIT = 1, FUSION_STILL_W_MRAD_S_LIMIT = 1;
    parameter signed [31:0] FUSION_R_STILL_V_Q24 = 1678, FUSION_R_STILL_W_Q24 = 1678;
    wire write_buffer_sel,read_buffer_sel,capture_enable;
    wire [31:0] camera_frame_count_gray;
    wire eth_frame_start,eth_stream_valid,eth_stream_ready;
    wire [15:0] eth_stream_pixel;
    wire [31:0] eth_stream_timestamp;
    wire [31:0] video_dropped_frames_gray,video_skipped_frames_gray;
    wire [319:0] ddr_debug_bus;
    wire [95:0] video_reject_reason_gray;
    wire [31:0] video_autocal_status, video_autocal_bursts_gray;
    wire [31:0] video_camera_diagnostic, video_guard_frames_gray;
    wire [3:0] camera_failure_sticky_pixel;
    wire loc_clk50m,loc_clk200m,loc_clk33m,loc_clk125m,control_clk40m;
    wire init_calib_complete,pll_stop;
    wire eth_phy_link_up;
    wire [31:0] eth_packet_count_gray;
    wire [31:0] eth_video_packet_count_gray;
    wire eth_video_fifo_overflow;
    wire eth_video_source_underflow;
    wire eth_video_frame_marker_error;
    wire g_rst_p;
    wire pll_lock;
    wire ddr_pll_lock;
    wire video_pll_lock;
    wire camera_init_done;
    wire camera_dphy_rx_reset_n;
    wire camera_pixel_clk;
    wire camera_vsync;
    wire camera_frame_bad;
    wire fifo_wrreq;
    wire [15:0] fifo_wrdata;
    assign pll_lock=ddr_pll_lock && video_pll_lock;
    assign g_rst_p=!reset_n || !pll_lock;
    assign led={camera_init_done,init_calib_complete};
    // 等待时钟稳定后释放机器人功能。
    reg [15:0] por_count = 16'd0;
    always @(posedge loc_clk50m or negedge reset_n) begin
        if (!reset_n || !pll_lock)
            por_count <= 16'd0;
        else if (!(&por_count))
            por_count <= por_count + 1'b1;
    end
    wire system_reset_n = (&por_count) && pll_lock;
    (* ASYNC_REG="TRUE" *) reg [1:0] sensor_reset_sync;
    always @(posedge loc_clk33m or negedge system_reset_n) begin
        if (!system_reset_n) sensor_reset_sync <= 2'b00;
        else sensor_reset_sync <= {sensor_reset_sync[0], 1'b1};
    end
    wire sensor_reset_n = sensor_reset_sync[1];
    (* ASYNC_REG="TRUE" *) reg [1:0] camera_reset_sync,init_reset_sync;
    always @(posedge camera_pixel_clk or negedge system_reset_n)
        if(!system_reset_n)camera_reset_sync<=0;else camera_reset_sync<={camera_reset_sync[0],1'b1};
    always @(posedge loc_clk50m or negedge system_reset_n)
        if(!system_reset_n)init_reset_sync<=0;else init_reset_sync<={init_reset_sync[0],1'b1};

    ddr3_pll ddr3_pll(
        .lock(ddr_pll_lock), //output lock
        .clkout0(loc_clk50m), //output clkout0
        .clkout1(loc_clk200m), //output clkout1
        .clkout2(),
        .clkin(clk50m), //input clkin
        .reset(~reset_n), //input reset
        .enclk0(1'b1), //input enclk0
        .enclk1(pll_stop), // DDR IP 控制实际使用的 200 MHz 内存时钟。
        .enclk2(1'b1) // CLKOUT2 未使用。
    );

    Gowin_PLL Gowin_PLL(
        .lock(video_pll_lock),
        .clkout0(loc_clk125m), //125 MHz Ethernet RGMII TX clock
        .clkout1(control_clk40m), // 1000/25 MHz，真实GoRV32 FP专用时钟。
        .clkout2(), // 复杂ISP退出，100MHz输出不接负载。
        .clkout3(loc_clk33m), //output clkout3
        .clkin(clk50m), //input clkin
        .reset(~reset_n) //input reset
    );

    // UDP 读端由完整相机帧触发，并服从发送 FIFO 背压。
    wire [7:0] gmii_rx_data;
    wire gmii_rx_valid, gmii_rx_error;
    wire udp_reply_valid, udp_reply_ready;
    wire telemetry_valid, telemetry_start, telemetry_read_request;
    wire telemetry_length_nonzero;
    wire [7:0] telemetry_length_low;
    wire [15:0] telemetry_length = {8'd0, telemetry_length_low};
    wire [7:0] telemetry_data;
    wire [255:0] udp_reply_data;
    wire [127:0] udp_rx_diagnostics;
    wire [63:0] udp_raw_prefix;
    wire rx_good_toggle,rx_sampling_reset,rx_phase_ready;
    wire [3:0] rx_phase_index;
    wire [15:0] rx_phase_mask;
    (* ASYNC_REG="TRUE" *) reg [20:0] rx_phase_tx_meta;
    (* ASYNC_REG="TRUE" *) reg [19:0] rx_phase_tx;
    // 状态选择位仍是同一同步链的第二级，单独保留供高扇出的诊断选择使用。
    (* ASYNC_REG="TRUE", syn_keep=1, syn_preserve=1 *) reg rx_phase_status_ready;
    (* ASYNC_REG="TRUE" *) reg [1:0] rx_phase_reset_sync;
    always @(posedge loc_clk125m or posedge g_rst_p)
        if(g_rst_p)rx_phase_reset_sync<=2'b11;
        else rx_phase_reset_sync<={rx_phase_reset_sync[0],1'b0};
    always @(posedge loc_clk125m)
        if(rx_phase_reset_sync[1])begin rx_phase_tx_meta<=0;rx_phase_tx<=0;rx_phase_status_ready<=0;end
        else begin
            rx_phase_tx_meta<={rx_phase_ready,rx_phase_index,rx_phase_mask};
            rx_phase_tx<=rx_phase_tx_meta[19:0];
            rx_phase_status_ready<=rx_phase_tx_meta[20];
        end
    wire [127:0] udp_diag_snapshot=rx_phase_status_ready ? udp_rx_diagnostics :
        {udp_rx_diagnostics[127:96],12'd0,rx_phase_tx[19:16],rx_phase_tx[15:0],
         udp_raw_prefix[15:0],udp_raw_prefix[31:16],udp_raw_prefix[47:32],udp_raw_prefix[63:48]};
    wire udp_auto_valid,udp_auto_enable;
    wire udp_control_valid;
    wire [47:0] udp_control_velocity;
    wire udp_task_fragment_valid;
    wire [255:0] udp_task_fragment;
    wire [63:0] udp_control_token;
    wire udp_control_heartbeat;
    wire rx_sampling_clk,rx_sampling_lock;
    // 协商/断链期间 PHY 时钟可能停止或变频，重新连通后须重启 PLL 初始化。
    // PLL_INIT 和请求接收器分别同步释放复位；复位不依赖已经停止的 RX 时钟。
    wire rx_pll_reset_p=g_rst_p || !eth_phy_link_up;
    wire udp_rx_reset_p=rx_pll_reset_p || !rx_sampling_lock || rx_sampling_reset;
    (* ASYNC_REG="TRUE" *) reg [1:0] control_link_down_sync;
    always @(posedge control_clk40m or posedge udp_rx_reset_p)
        if(udp_rx_reset_p)control_link_down_sync<=2'b11;
        else control_link_down_sync<={control_link_down_sync[0],!rx_phase_ready};
    bx72_rx_pll rx_sampling_pll(.clkin(rgmii_rx_clk_i),.init_clk(clk50m),
        .reset_p(rx_pll_reset_p),.clkout0(rx_sampling_clk),.lock(rx_sampling_lock),
        .good_toggle_i(rx_good_toggle),.sampling_reset_o(rx_sampling_reset),
        .ready_o(rx_phase_ready),.phase_index_o(rx_phase_index),.good_mask_o(rx_phase_mask));
    bx72_rgmii_rx rgmii_receiver (
        .rgmii_rx_clk_i(rx_sampling_clk),.rgmii_rxd(rgmii_rxd),.rgmii_rxdv(rgmii_rxdv),
        .gmii_rxd(gmii_rx_data),.gmii_rxdv(gmii_rx_valid),.gmii_rxer(gmii_rx_error)
    );
    bx72_udp_request_rx #(.USE_CONTROL_BRIDGE(1)) udp_request_receiver (
        .reset_p(udp_rx_reset_p),.rx_clk(rx_sampling_clk),.gmii_rxd(gmii_rx_data),
        .gmii_rxdv(gmii_rx_valid),.gmii_rxer(gmii_rx_error),.tx_clk(loc_clk125m),
        // A validated probe is useful during phase training too. Consume an ACK
        // only when the real transmitter accepts it; never silently drain it.
        .reply_ready(udp_reply_ready),
        .reply_valid(udp_reply_valid),.reply_data(udp_reply_data),
        .diagnostics_tx_o(udp_rx_diagnostics),.control_clk(control_clk40m),
        .command_bus_clk_i(loc_clk50m),
        .control_valid_o(udp_control_valid),.control_velocity_o(udp_control_velocity),
        .task_fragment_valid_o(udp_task_fragment_valid),.task_fragment_o(udp_task_fragment),
        .control_token_o(udp_control_token),
        .control_heartbeat_o(udp_control_heartbeat),
        .auto_mode_valid_o(udp_auto_valid),.auto_enable_o(udp_auto_enable),
        .training_reset_p_i(rx_pll_reset_p),.good_toggle_o(rx_good_toggle),.raw_prefix_tx_o(udp_raw_prefix)
    );
    eth_udp_video_sender #(
        .VIDEO_RTP_PAYLOAD_TYPE(7'd97),
        .IMAGE_WIDTH(IMAGE_WIDTH),
        .IMAGE_HEIGHT(IMAGE_HEIGHT),
        .USE_EXTERNAL_TIMESTAMP(1),.RX_DIAGNOSTICS_ENABLE(1),.TELEMETRY_ENABLE(1)
    ) eth_udp_video_sender (
        .clk50m(loc_clk50m),
        .clk125m(loc_clk125m),
        .reset_p(g_rst_p),
        .cam_pclk(loc_clk33m),
        .camera_vsync(eth_frame_start),
        .camera_pixel_valid(eth_stream_valid),
        .camera_pixel(eth_stream_pixel),
        .camera_timestamp(eth_stream_timestamp),.camera_pixel_ready(eth_stream_ready),
        .reply_valid(udp_reply_valid),.reply_data(udp_reply_data),.reply_ready(udp_reply_ready),
        .rx_diagnostics_i(udp_diag_snapshot),.rx_diagnostics_raw_i(!rx_phase_status_ready),
        .telemetry_valid(telemetry_valid),.telemetry_length(telemetry_length),
        .telemetry_length_nonzero(telemetry_length_nonzero),
        .telemetry_data(telemetry_data),.telemetry_start(telemetry_start),
        .telemetry_read(),.telemetry_read_request(telemetry_read_request),
        .rgmii_gtxc(rgmii_gtxc), .rgmii_txd(rgmii_txd),
        .rgmii_txen(rgmii_txen), .eth_mdc(eth_mdc),
        .eth_mdio(eth_mdio), .eth_rst_n(eth_rst_n),
        .phy_configured(),
        .phy_link_up(eth_phy_link_up),
        .packet_count(), .packet_count_gray(eth_packet_count_gray),
        .video_packet_count(),
        .video_packet_count_gray(eth_video_packet_count_gray),
        .video_fifo_overflow(eth_video_fifo_overflow),
        .video_source_underflow(eth_video_source_underflow),
        .video_frame_marker_error(eth_video_frame_marker_error)
    );
    //摄像头
    camera_init
    #(
        .SYS_CLOCK      ( 50_000_000   ),//系统时钟采用50MHz
        .SCL_CLOCK      ( 400_000      ),//SCL总线时钟采用400kHz
        .LP11_RESET_PROBE( 1            ) // BX72 启动阶段先停流并等待 LP-11
    )camera_init
    (
        .Clk         (loc_clk50m       ),
        .Rst_n       (init_reset_sync[1]),
        .Init_Done   (camera_init_done ),
        // 摄像头座已由板上电阻固定 RESET/PWDN，无 GPIO 跳线。
        .camera_rst_n(),
        .camera_pwdn (),
        .i2c_sclk    (camera_sclk      ),
        .i2c_sdat    (camera_sdat      ),
        .lp_clk_state(camera_mipi_lp_clk),
        .dphy_rx_reset_n(camera_dphy_rx_reset_n),
        .lp11_probe_status()
    );
    

    // RAW Bayer解码和必要颜色转换均在像素域完成，帧缓冲与网络保持YUYV合同。
    ov5647_csi2_capture #(
        .BAYER_PATTERN(1),
        .WB_R_GAIN_UQ2_8(10'd400),
        .WB_G_GAIN_UQ2_8(10'd256),
        .WB_B_GAIN_UQ2_8(10'd308)
    ) ov5647_capture (
        .clk50m(clk50m),
        // DPHY 接收独立的相机时钟，不随 DDR PLL 失锁直接异步释放其高速复位。
        // 系统重新初始化仍通过 sensor_init_done 和 pixel_reset_n 复位后级管线。
        .reset_n(reset_n),
        .pixel_reset_n(camera_reset_sync[1]),
        .sensor_init_done(camera_init_done),
        .dphy_reset_n(camera_dphy_rx_reset_n),
        .mipi_clk_p(camera_mipi_clk_p),
        .mipi_clk_n(camera_mipi_clk_n),
        .mipi_lane0_p(camera_mipi_lane0_p),
        .mipi_lane0_n(camera_mipi_lane0_n),
        .mipi_lane1_p(camera_mipi_lane1_p),
        .mipi_lane1_n(camera_mipi_lane1_n),
        .mipi_lp_clk(camera_mipi_lp_clk),
        .mipi_lp_lane0(camera_mipi_lp_lane0),
        .mipi_lp_lane1(camera_mipi_lp_lane1),
        .pixel_clk(camera_pixel_clk),
        .camera_vsync(camera_vsync),
        .camera_frame_bad(camera_frame_bad),
        .camera_pixel_valid(fifo_wrreq),
        .camera_pixel(fifo_wrdata),
        .debug_status(video_autocal_status),
        .autocal_burst_total_gray(video_autocal_bursts_gray),
        .autocal_control_word(video_camera_diagnostic),
        .autocal_bad_burst_gray(),
        .guard_bad_good_gray16(video_guard_frames_gray),
        .guard_total_frame_gray(),
        .guard_failure_sticky(camera_failure_sticky_pixel)
    );

    udp_camera_ddr #(
        .FRAME_PIXELS(IMAGE_WIDTH*IMAGE_HEIGHT), .USE_CAMERA_FRAME_BAD(1)
    ) ddr3_ctrl_2port(
        .clk(loc_clk50m),.pll_stop(pll_stop),.pll_lock(ddr_pll_lock),.clk_200m(loc_clk200m),
        .sys_rst_n(reset_n),.init_calib_complete(init_calib_complete),
        .cam_clk(camera_pixel_clk),.camera_vsync(camera_vsync),
        .camera_frame_bad(camera_frame_bad),
        .camera_pixel_valid(fifo_wrreq),.camera_pixel(fifo_wrdata),
        .stream_clk(loc_clk33m),.stream_ready(eth_stream_ready),
        .stream_frame_start(eth_frame_start),.stream_valid(eth_stream_valid),.stream_pixel(eth_stream_pixel),
        .stream_timestamp(eth_stream_timestamp),.stream_busy(),.capture_active(capture_enable),
        .write_buffer(write_buffer_sel),.read_buffer(read_buffer_sel),.camera_edges_gray(camera_frame_count_gray),
        .debug_gray(ddr_debug_bus),.dropped_frames_gray(video_dropped_frames_gray),
        .skipped_frames_gray(video_skipped_frames_gray),
        .reject_reason_gray(video_reject_reason_gray),
        //DDR3   
        .ddr3_dq(IO_ddr_dq)             ,   //DDR3 数据
        .ddr3_dqs_n(IO_ddr_dqs_n)          ,   //DDR3 dqs负
        .ddr3_dqs_p(IO_ddr_dqs)          ,   //DDR3 dqs正  
        .ddr3_addr(O_ddr_addr)           ,   //DDR3 地址   
        .ddr3_ba(O_ddr_ba)             ,   //DDR3 banck 选择
        .ddr3_ras_n(O_ddr_ras_n)          ,   //DDR3 行选择
        .ddr3_cas_n(O_ddr_cas_n)          ,   //DDR3 列选择
        .ddr3_we_n(O_ddr_we_n)           ,   //DDR3 读写选择
        .ddr3_reset_n(O_ddr_reset_n)        ,   //DDR3 复位
        .ddr3_ck_p(O_ddr_clk)          ,   //DDR3 时钟正
        .ddr3_ck_n(O_ddr_clk_n)           ,   //DDR3 时钟负
        .ddr3_cke(O_ddr_cke)            ,   //DDR3 时钟使能
        .ddr3_cs_n(O_ddr_cs_n)           ,   //DDR3 片选
        .ddr3_dm(O_ddr_dqm)             ,   //DDR3_dm
        .ddr3_odt(O_ddr_odt)                //DDR3_odt   
    );
    
    reg ddr_calib_sync1;
    reg ddr_calib_sync2;
    always @(posedge loc_clk50m or negedge system_reset_n) begin
        if (!system_reset_n) begin
            ddr_calib_sync1 <= 1'b0;
            ddr_calib_sync2 <= 1'b0;
        end else begin
            ddr_calib_sync1 <= init_calib_complete;
            ddr_calib_sync2 <= ddr_calib_sync1;
        end
    end

    (* ASYNC_REG="TRUE" *) reg [31:0] eth_count_gray_sync1,eth_count_gray_sync2;
    (* ASYNC_REG="TRUE" *) reg [31:0] eth_video_count_gray_sync1,eth_video_count_gray_sync2;
    reg [31:0] eth_count_binary_q,eth_video_count_binary_q;
    (* ASYNC_REG="TRUE" *) reg [2:0] eth_video_status_sync1, eth_video_status_sync2;
    (* ASYNC_REG="TRUE" *) reg [31:0] video_drop_meta,video_drop_sync;
    reg [31:0] video_drop_binary,video_skip_binary;
    (* ASYNC_REG="TRUE" *) reg [31:0] video_skip_meta,video_skip_sync;
    (* ASYNC_REG="TRUE" *) reg [31:0] ddr_debug_meta [0:9];
    (* ASYNC_REG="TRUE" *) reg [31:0] ddr_debug_sync [0:9];
    (* ASYNC_REG="TRUE" *) reg [95:0] video_reject_meta, video_reject_sync;
    (* ASYNC_REG="TRUE" *) reg [28:0] video_autocal_meta, video_autocal_sync;
    (* ASYNC_REG="TRUE" *) reg [31:0] camera_diag_meta,camera_diag_sync;
    (* ASYNC_REG="TRUE" *) reg [31:0] camera_guard_meta,camera_guard_sync;
    // 四个位各自只置位，直到像素域复位；目的域独立同步。
    (* ASYNC_REG="TRUE" *) reg [3:0] camera_failure_meta,camera_failure_sync;
    reg [23:0] video_reject_counts;
    wire [31:0] reject_bad_binary = gray_to_binary(video_reject_sync[31:0]);
    wire [31:0] reject_count_binary = gray_to_binary(video_reject_sync[63:32]);
    wire [31:0] reject_full_binary = gray_to_binary(video_reject_sync[95:64]);
    integer debug_index;
    // 相机计数先按Gray跨域，再在目的域判断非零，避免同步器前组合归约。
    wire [7:0] status_sources = {read_buffer_sel, 1'b0,
                             write_buffer_sel, capture_enable,
                             1'b0, ddr_calib_sync2,
                             camera_init_done, pll_lock};
    (* ASYNC_REG="TRUE" *) reg [7:0] status_meta, status_sync;
    (* ASYNC_REG="TRUE" *) reg [31:0] camera_count_meta, camera_count_sync;
    reg camera_seen_sync;
    wire [7:0] status_telemetry = {status_sync[7:4],camera_seen_sync,status_sync[2:0]};
    always @(posedge loc_clk33m or negedge sensor_reset_n) begin
        if (!sensor_reset_n) begin
            status_meta <= 8'd0; status_sync <= 8'd0;
            camera_count_meta <= 32'd0; camera_count_sync <= 32'd0;
            camera_seen_sync <= 1'b0;
            eth_count_gray_sync1 <= 32'd0;
            eth_count_gray_sync2 <= 32'd0;
            eth_video_count_gray_sync1 <= 32'd0;
            eth_video_count_gray_sync2 <= 32'd0;
            eth_video_status_sync1 <= 3'b000;
            eth_video_status_sync2 <= 3'b000;
            video_drop_meta<=0;video_drop_sync<=0;video_drop_binary<=0;
            video_skip_meta<=0;video_skip_sync<=0;video_skip_binary<=0;
            video_reject_meta<=0;video_reject_sync<=0;video_reject_counts<=0;
            video_autocal_meta<=0;video_autocal_sync<=0;
            camera_diag_meta<=0;camera_diag_sync<=0;camera_guard_meta<=0;camera_guard_sync<=0;
            camera_failure_meta<=0;camera_failure_sync<=0;
            for (debug_index = 0; debug_index < 10; debug_index = debug_index + 1) begin
                ddr_debug_meta[debug_index] <= 32'd0;
                ddr_debug_sync[debug_index] <= 32'd0;
            end
        end else begin
            status_meta <= status_sources; status_sync <= status_meta;
            camera_count_meta <= camera_frame_count_gray;
            camera_count_sync <= camera_count_meta;
            camera_seen_sync <= (camera_count_sync != 0);
            eth_count_gray_sync1 <= eth_packet_count_gray;
            eth_count_gray_sync2 <= eth_count_gray_sync1;
            eth_video_count_gray_sync1 <= eth_video_packet_count_gray;
            eth_video_count_gray_sync2 <= eth_video_count_gray_sync1;
            eth_video_status_sync1 <= {eth_video_frame_marker_error,
                                       eth_video_source_underflow,
                                       eth_video_fifo_overflow};
            eth_video_status_sync2 <= eth_video_status_sync1;
            video_drop_meta<=video_dropped_frames_gray;video_drop_sync<=video_drop_meta;
            video_drop_binary<=gray_to_binary(video_drop_sync);
            video_skip_meta<=video_skipped_frames_gray;video_skip_sync<=video_skip_meta;
            video_skip_binary<=gray_to_binary(video_skip_sync);
            video_reject_meta<=video_reject_reason_gray;video_reject_sync<=video_reject_meta;
            video_reject_counts<={reject_full_binary[7:0],reject_count_binary[7:0],reject_bad_binary[7:0]};
            // 状态字段只作阶段观测，跨状态边界的单样本不能视为原子快照。
            video_autocal_meta<=video_autocal_status[31:3];video_autocal_sync<=video_autocal_meta;
            // Diagnostic fields are interpreted only after a stable state.
            // The two frame counters remain independently Gray coded.
            camera_diag_meta<=video_camera_diagnostic;camera_diag_sync<=camera_diag_meta;
            camera_guard_meta<=video_guard_frames_gray;camera_guard_sync<=camera_guard_meta;
            camera_failure_meta<=camera_failure_sticky_pixel;camera_failure_sync<=camera_failure_meta;
            for (debug_index = 0; debug_index < 10; debug_index = debug_index + 1) begin
                // 槽3此前恒零；现在传校准完整burst计数，保持32位Gray跨域。
                ddr_debug_meta[debug_index] <= (debug_index == 3) ? video_autocal_bursts_gray : ddr_debug_bus[debug_index*32+:32];
                ddr_debug_sync[debug_index] <= ddr_debug_meta[debug_index];
            end
        end
    end
    function [31:0] gray_to_binary;
        input [31:0] gray;
        integer shift_amount;
        begin
            // 并行前缀异或缩短状态读取的组合路径。
            gray_to_binary = gray;
            for (shift_amount = 1; shift_amount < 32;
                 shift_amount = shift_amount * 2)
                gray_to_binary = gray_to_binary ^
                                 (gray_to_binary >> shift_amount);
        end
    endfunction
    always @(posedge loc_clk33m or negedge sensor_reset_n)
        if(!sensor_reset_n)begin eth_count_binary_q<=0;eth_video_count_binary_q<=0;end
        else begin eth_count_binary_q<=gray_to_binary(eth_count_gray_sync2);eth_video_count_binary_q<=gray_to_binary(eth_video_count_gray_sync2);end
    wire fusion_valid,imu_valid,motor_telemetry_valid,lidar_telemetry_valid;
    wire [255:0] imu_telemetry_payload;wire [703:0] motor_telemetry_payload;
    wire [1151:0] lidar_telemetry_payload;wire [831:0] fusion_telemetry_payload;
    wire [319:0] sensor_health;
    wire navigation_valid;
    wire [543:0] navigation_payload;
    wire local_plan_valid; wire [319:0] local_plan_payload;
    wire local_grid_valid,local_grid_ready; wire [1663:0] local_grid_payload;
    wire fp_ready,fp_fault,fp_busy,control_reset;
    wire [7:0] fp_fault_code;
    bx72_lite_robot_core robot(
        .control_clk(control_clk40m),.loc_clk33m(loc_clk33m),.reset_n(system_reset_n),
        .n10_uart_rx(n10_uart_rx),.imu_miso(imu_miso),.imu_cs_n(imu_cs_n),.imu_sclk(imu_sclk),.imu_mosi(imu_mosi),
        .encoder_a_i(encoder_a_i),.encoder_b_i(encoder_b_i),.motor_in1_o(motor_in1_o),.motor_in2_o(motor_in2_o),
        .command_valid_i(udp_control_valid),.command_velocity_i(udp_control_velocity),
        .task_fragment_valid_i(udp_task_fragment_valid),.task_fragment_i(udp_task_fragment),
        .command_token_i(udp_control_token),.navigation_valid_o(navigation_valid),.navigation_payload_o(navigation_payload),
        .local_plan_valid_o(local_plan_valid),.local_plan_payload_o(local_plan_payload),
        .local_grid_valid_o(local_grid_valid),.local_grid_payload_o(local_grid_payload),.local_grid_ready_i(local_grid_ready),
        .command_heartbeat_i(udp_control_heartbeat),
        .command_link_down_i(control_link_down_sync[1]),
        .auto_mode_valid_i(udp_auto_valid),.auto_enable_i(udp_auto_enable),.auto_mode_o(),
        .fusion_valid_o(fusion_valid),
        .imu_valid_o(imu_valid),.imu_payload_o(imu_telemetry_payload),
        .motor_valid_o(motor_telemetry_valid),.motor_payload_o(motor_telemetry_payload),
        .lidar_valid_o(lidar_telemetry_valid),.lidar_payload_o(lidar_telemetry_payload),
        .fusion_payload_o(fusion_telemetry_payload),.sensor_health_o(sensor_health),
        .cpu_ready_o(fp_ready),.cpu_fault_o(fp_fault),.cpu_busy_o(fp_busy),.cpu_fault_code_o(fp_fault_code),.control_reset_o(control_reset));
    // 诊断状态跨入遥测域；各bit独立采样，不用于控制。fault code在sticky fault后稳定。
    (* ASYNC_REG="TRUE" *) reg [11:0] fp_health_meta,fp_health_sync;
    always @(posedge loc_clk33m or negedge sensor_reset_n) begin
        if(!sensor_reset_n)begin fp_health_meta<=0;fp_health_sync<=0;end
        else begin
            // 保持遥测位布局；原周期算术诊断位保留为零，busy表示批量服务在途。
            fp_health_meta<={fp_fault_code,fp_busy,1'b0,fp_fault,fp_ready};
            fp_health_sync<=fp_health_meta;
        end
    end
    wire [895:0] health_telemetry_payload = {
        // 仅视频保留位增加诊断：flags高24位依次为full/count/bad的模256帧计数。
        video_reject_counts,status_telemetry,gray_to_binary(camera_count_sync),eth_count_binary_q,eth_video_count_binary_q,
        video_autocal_sync,eth_video_status_sync2,video_drop_binary,video_skip_binary,
        ddr_debug_sync[0],ddr_debug_sync[1],ddr_debug_sync[2],ddr_debug_sync[3],ddr_debug_sync[4],
        ddr_debug_sync[5],ddr_debug_sync[6],camera_diag_sync,camera_guard_sync,ddr_debug_sync[9],
        // 原CPU位2为保留位；置1声明此版本支持4位相机失败sticky。
        sensor_health,16'h4752,camera_failure_sync,fp_health_sync[11:3],1'b1,fp_health_sync[1:0]};
    reg [25:0] telemetry_health_count;
    reg health_telemetry_valid;
    always @(posedge loc_clk33m or negedge sensor_reset_n) begin
        if(!sensor_reset_n) begin
            telemetry_health_count<=0;health_telemetry_valid<=0;
        end else begin
            health_telemetry_valid<=0;
            if(telemetry_health_count==33333332) begin
                telemetry_health_count<=0;health_telemetry_valid<=1;
            end else telemetry_health_count<=telemetry_health_count+1'b1;
        end
    end
    // 两组生产者在原生域保存一次快照并字节化；50MHz公共包总线轮询，再以8位FIFO跨到125MHz。
    wire native_valid,native_nonzero,native_start,native_read;
    wire control_valid,control_nonzero,control_start,control_read;
    wire [7:0] native_length,native_data,control_length,control_data;
    wire telemetry_bus_reset=!init_reset_sync[1];
    wire bus_packet_valid,bus_packet_nonzero,bus_packet_start,bus_packet_read;
    wire [7:0] bus_packet_length,bus_packet_data;
    bx72_telemetry #(.CHANNEL_MASK(7'b0010101),.USE_EXTERNAL_SRC_RESET(1),.USE_EXTERNAL_TX_RESET(1)) telemetry_native_u(
        .src_clk(loc_clk33m),.tx_clk(loc_clk50m),.reset_n(system_reset_n),
        .src_reset_i(!sensor_reset_n),.tx_reset_i(telemetry_bus_reset),.tx_reset_o(),
        .imu_valid_i(imu_valid),.imu_payload_i(imu_telemetry_payload),
        .motor_valid_i(1'b0),.motor_payload_i(704'd0),
        .lidar_valid_i(lidar_telemetry_valid),.lidar_payload_i(lidar_telemetry_payload),
        .fusion_valid_i(1'b0),.fusion_payload_i(832'd0),
        .health_valid_i(health_telemetry_valid),.health_payload_i(health_telemetry_payload),
        .navigation_valid_i(1'b0),.navigation_payload_i(544'd0),
        .local_plan_valid_i(1'b0),.local_plan_payload_i(320'd0),
        .local_grid_valid_i(1'b0),.local_grid_payload_i(1664'd0),.local_grid_ready_o(),
        .tx_valid_o(native_valid),.tx_length_o(native_length),.tx_length_nonzero_o(native_nonzero),
        .tx_data_o(native_data),.tx_start_i(native_start),.tx_read_i(native_read),.dropped_o());
    // 电机、融合、导航、本地规划和栅格均在控制域采样。
    bx72_telemetry #(.SRC_CLK_HZ(40000000),.CHANNEL_MASK(8'b11101010),
        .USE_EXTERNAL_SRC_RESET(1),.USE_EXTERNAL_TX_RESET(1)) telemetry_control_u(
        .src_clk(control_clk40m),.tx_clk(loc_clk50m),.reset_n(system_reset_n),
        .src_reset_i(control_reset),.tx_reset_i(telemetry_bus_reset),.tx_reset_o(),
        .imu_valid_i(1'b0),.imu_payload_i(256'd0),
        .motor_valid_i(motor_telemetry_valid),.motor_payload_i(motor_telemetry_payload),
        .lidar_valid_i(1'b0),.lidar_payload_i(1152'd0),
        .fusion_valid_i(fusion_valid),.fusion_payload_i(fusion_telemetry_payload),
        .health_valid_i(1'b0),.health_payload_i(896'd0),
        .navigation_valid_i(navigation_valid),.navigation_payload_i(navigation_payload),
        .local_plan_valid_i(local_plan_valid),.local_plan_payload_i(local_plan_payload),
        .local_grid_valid_i(local_grid_valid),.local_grid_payload_i(local_grid_payload),.local_grid_ready_o(local_grid_ready),
        .tx_valid_o(control_valid),.tx_length_o(control_length),.tx_length_nonzero_o(control_nonzero),
        .tx_data_o(control_data),.tx_start_i(control_start),.tx_read_i(control_read),.dropped_o());
    bx72_telemetry_mux telemetry_mux_u(
        .clk(loc_clk50m),.reset_i(telemetry_bus_reset),
        .source0_valid_i(native_valid),.source0_length_nonzero_i(native_nonzero),
        .source0_length_i(native_length),.source0_data_i(native_data),
        .source0_start_o(native_start),.source0_read_o(native_read),
        .source1_valid_i(control_valid),.source1_length_nonzero_i(control_nonzero),
        .source1_length_i(control_length),.source1_data_i(control_data),
        .source1_start_o(control_start),.source1_read_o(control_read),
        .tx_valid_o(bus_packet_valid),.tx_length_nonzero_o(bus_packet_nonzero),
        .tx_length_o(bus_packet_length),.tx_data_o(bus_packet_data),
        .tx_start_i(bus_packet_start),.tx_read_i(bus_packet_read));

    bx72_telemetry_packet_cdc telemetry_packet_cdc_u(
        .src_clk(loc_clk50m),.src_reset_i(telemetry_bus_reset),
        .tx_clk(loc_clk125m),.reset_n(system_reset_n),
        .packet_valid_i(bus_packet_valid),.packet_length_nonzero_i(bus_packet_nonzero),
        .packet_length_i(bus_packet_length),.packet_data_i(bus_packet_data),
        .packet_start_o(bus_packet_start),.packet_read_o(bus_packet_read),
        .tx_valid_o(telemetry_valid),.tx_length_nonzero_o(telemetry_length_nonzero),
        .tx_length_o(telemetry_length_low),.tx_data_o(telemetry_data),
        .tx_start_i(telemetry_start),.tx_read_i(telemetry_read_request));
endmodule
