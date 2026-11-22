module camera_udp_top #(
    parameter [47:0] VIDEO_DEST_MAC = 48'h02_00_00_00_00_6F,
    parameter [31:0] VIDEO_DEST_IP = 32'hC0_00_02_6F,
    parameter [31:0] VIDEO_SOURCE_IP = 32'hC0_00_02_0A,
    parameter [47:0] SOURCE_MAC = 48'h02_00_00_00_07_20,
    parameter [15:0] TELEMETRY_SOURCE_PORT = 16'd5000,
    parameter [15:0] TELEMETRY_DEST_PORT = 16'd6102,
    parameter [15:0] CONTROL_PORT = 16'd6103,
    parameter [31:0] CONTROL_BROADCAST_IP = {VIDEO_SOURCE_IP[31:8], 8'hFF},
    parameter integer PACKET_GAP_CYCLES = 60_000
) (
    input wire clk50m, reset_n,
    input wire camera_pclk, camera_vsync, camera_href,
    input wire [7:0] camera_data,
    output wire camera_xclk, camera_sclk, camera_rst_n, camera_pwdn,
    inout wire camera_sdat,

    output wire [3:0] led,

    output wire imu_sclk, imu_mosi, imu_cs_n,
    input wire imu_miso,
    input wire lidar_rx,

    output wire eth_tx_clk, eth_tx_en, eth_reset_n, eth_mdc,
    output wire [3:0] eth_txd,
    inout wire eth_mdio,
    input wire eth_rx_clk, eth_rx_ctl,
    input wire [3:0] eth_rxd,
    output wire [17:0] servo_pwm,

    output wire [13:0] O_ddr_addr,
    output wire [2:0] O_ddr_ba,
    output wire O_ddr_cs_n, O_ddr_ras_n, O_ddr_cas_n, O_ddr_we_n,
    output wire O_ddr_clk, O_ddr_clk_n, O_ddr_cke, O_ddr_odt, O_ddr_reset_n,
    output wire [1:0] O_ddr_dqm,
    inout wire [15:0] IO_ddr_dq,
    inout wire [1:0] IO_ddr_dqs, IO_ddr_dqs_n
);
    wire video_locked, ddr_locked, ddr_ready, camera_configured, frame_valid;
    wire ddr_ref, ddr_mem_clk, ui_clk, pll_stop;
    wire camera_ref25, eth_clk125;
    wire [9:0] camera_delayed;
    wire [9:0] camera_inputs = {camera_vsync, camera_href, camera_data};
    wire camera_active, camera_valid;
    wire dvp_active, dvp_line, dvp_valid;
    wire [15:0] dvp_pixel;
    wire [15:0] camera_pixel;
    wire udp_request_toggle, udp_read, udp_empty;
    wire [15:0] udp_pixel;
    wire cmd_ready, write_ready, read_valid, cmd_enable, write_enable;
    wire [27:0] cmd_addr;
    wire [2:0] cmd;
    wire [127:0] read_data, write_data;

    // 延后 DVP 数据及行场同步的跳变，扩大采样上升沿之后的保持时间。
    genvar camera_pin;
    generate
        for (camera_pin = 0; camera_pin < 10; camera_pin = camera_pin + 1) begin : camera_input_delay
            IODELAY #(.C_STATIC_DLY(64)) delay_input (
                .DI(camera_inputs[camera_pin]), .DO(camera_delayed[camera_pin]), .DF(),
                .SDTAP(1'b0), .VALUE(1'b0), .DLYSTEP(8'd0)
            );
        end
    endgenerate

    Gowin_PLL video_pll (
        .lock(video_locked), .clkout0(eth_clk125), .clkout1(), .clkout2(camera_ref25),
        .clkout4(), .clkin(clk50m), .reset(!reset_n)
    );
    ddr3_pll memory_pll (
        .lock(ddr_locked), .clkout0(ddr_ref), .clkout1(ddr_mem_clk), .clkout2(),
        .clkin(clk50m), .reset(!reset_n), .enclk0(1'b1), .enclk1(1'b1), .enclk2(pll_stop)
    );

    // 各时钟域异步进入复位，两拍同步释放。
    wire video_reset_n = reset_n && video_locked;
    wire capture_reset_n = video_reset_n && camera_configured;
    wire memory_reset_n = video_reset_n && ddr_locked;
    (* ASYNC_REG = "TRUE" *) reg [1:0] config_reset, dvp_reset, ui_reset, eth_reset, pixel_reset;
    always @(posedge clk50m or negedge video_reset_n)
        if (!video_reset_n) config_reset <= 0;
        else config_reset <= {config_reset[0], 1'b1};
    always @(posedge camera_pclk or negedge capture_reset_n)
        if (!capture_reset_n) dvp_reset <= 0;
        else dvp_reset <= {dvp_reset[0], 1'b1};
    always @(posedge ui_clk or negedge memory_reset_n)
        if (!memory_reset_n) ui_reset <= 0;
        else ui_reset <= {ui_reset[0], 1'b1};
    always @(posedge eth_clk125 or negedge memory_reset_n)
        if (!memory_reset_n) eth_reset <= 0;
        else eth_reset <= {eth_reset[0], 1'b1};
    always @(posedge clk50m or negedge memory_reset_n)
        if (!memory_reset_n) pixel_reset <= 0;
        else pixel_reset <= {pixel_reset[0], 1'b1};

    wire [7:0] imu_who_am_i;
    wire imu_ready, imu_sample_valid;
    wire [95:0] imu_sample;
    imu963ra_spi imu (
        .clk(clk50m), .reset_n(config_reset[1]), .miso(imu_miso),
        .sclk(imu_sclk), .mosi(imu_mosi), .cs_n(imu_cs_n),
        .who_am_i(imu_who_am_i), .ready(imu_ready),
        .sample_valid(imu_sample_valid), .sample(imu_sample)
    );

    wire [23:0] motion_status, eth_motion_status, rx_debug, eth_rx_debug;
    (* ASYNC_REG = "TRUE" *) reg [23:0] debug_meta, debug_sync;
    reg [19:0] telemetry_publish_count;
    reg imu_have_sample, telemetry_toggle;
    reg [152:0] telemetry_snapshot;
    always @(posedge clk50m or negedge config_reset[1]) begin
        if (!config_reset[1]) begin
            telemetry_publish_count <= 0;
            imu_have_sample <= 0;
            telemetry_toggle <= 0;
            telemetry_snapshot <= 0;
        end else begin
            if (!imu_ready)
                imu_have_sample <= 0;
            else if (imu_sample_valid)
                imu_have_sample <= 1;

            if (telemetry_publish_count == 20'd999999) begin
                telemetry_publish_count <= 0;
                telemetry_snapshot <= {imu_ready && imu_have_sample, imu_who_am_i,
                                       imu_sample, motion_status, debug_sync};
                telemetry_toggle <= !telemetry_toggle;
            end else
                telemetry_publish_count <= telemetry_publish_count + 1'b1;
        end
    end

    wire eth_link;
    wire motion_valid;
    wire [3:0] motion_cmd;
    always @(posedge clk50m) begin
        debug_meta <= rx_debug;
        debug_sync <= debug_meta;
    end
    // The existing VIDEO_* address parameters describe the shared UDP link.
    hex1_udp_command_rx #(
        .LOCAL_MAC(SOURCE_MAC), .LOCAL_IP(VIDEO_SOURCE_IP),
        .BROADCAST_IP(CONTROL_BROADCAST_IP), .LOCAL_PORT(CONTROL_PORT)
    ) command_rx (
        .clk50(clk50m), .reset_n(config_reset[1]),
        .rgmii_rx_clk(eth_rx_clk), .rgmii_rx_ctl(eth_rx_ctl), .rgmii_rxd(eth_rxd),
        .cmd_valid(motion_valid), .cmd(motion_cmd), .rx_debug(rx_debug)
    );
    hexapod_gait gait_control (
        .clk(clk50m), .reset_n(config_reset[1]), .link_up(eth_link),
        .cmd_valid(motion_valid), .cmd(motion_cmd), .servo_pwm(servo_pwm),
        .status(motion_status)
    );

    wire [104:0] eth_imu_snapshot;
    wire eth_telemetry_updated;
    imu_snapshot_cdc #(.WIDTH(153)) imu_to_eth (
        .clk(eth_clk125), .reset_n(eth_reset[1]),
        .source_toggle(telemetry_toggle), .source_snapshot(telemetry_snapshot),
        .updated(eth_telemetry_updated), .snapshot({eth_imu_snapshot, eth_motion_status, eth_rx_debug})
    );

    camera_init #(.IMAGE_WIDTH(800), .IMAGE_HEIGHT(480)) configure_camera (
        .Clk(clk50m), .Rst_n(config_reset[1]), .Init_Done(camera_configured),
        .camera_rst_n(camera_rst_n), .camera_pwdn(camera_pwdn),
        .i2c_sclk(camera_sclk), .i2c_sdat(camera_sdat)
    );
    DVP_Capture capture_camera (
        .Rst_n(dvp_reset[1]), .PCLK(camera_pclk), .Vsync(camera_delayed[9]),
        .Href(camera_delayed[8]), .Data(camera_delayed[7:0]), .DataValid(dvp_valid),
        .DataPixel(dvp_pixel), .DataVs(dvp_active), .DataHs(dvp_line)
    );
    camera_line_align align_camera (
        .clk(camera_pclk), .reset_n(dvp_reset[1]),
        .frame_in(dvp_active), .line_in(dvp_line), .valid_in(dvp_valid), .pixel_in(dvp_pixel),
        .frame_out(camera_active), .line_out(), .valid_out(camera_valid),
        .pixel_out(camera_pixel)
    );

    wire gmii_clk, gmii_tx_en;
    wire [7:0] gmii_tx_data;
    wire lidar_packet_valid;
    wire [375:0] lidar_packet;
    // 串口接收与 UDP 调度共用网口字节时钟。
    d500_uart #(.CLK_HZ(125_000_000)) receive_lidar (
        .clk(eth_clk125), .reset_n(eth_reset[1]), .rx(lidar_rx),
        .packet_valid(lidar_packet_valid), .packet(lidar_packet)
    );
    phy_reg_config configure_phy (
        .Clk(clk50m), .Rst_n(config_reset[1]), .Phy_rst_n(eth_reset_n),
        .Rd_Data(), .Phy_init_done(), .Phy_link_up(eth_link),
        .mdc(eth_mdc), .mdio(eth_mdio)
    );
    camera_udp_stream #(
        .DEST_MAC(VIDEO_DEST_MAC), .DEST_IP(VIDEO_DEST_IP), .SOURCE_IP(VIDEO_SOURCE_IP),
        .SOURCE_MAC(SOURCE_MAC), .SOURCE_PORT(TELEMETRY_SOURCE_PORT), .DEST_PORT(TELEMETRY_DEST_PORT),
        .PACKET_GAP_CYCLES(PACKET_GAP_CYCLES)
    ) udp_video (
        .tx_clk(eth_clk125), .tx_reset_n(eth_reset[1]), .link_up(eth_link),
        .pixel_clk(clk50m), .pixel_reset_n(pixel_reset[1]),
        .imu_snapshot(eth_imu_snapshot), .imu_updated(eth_telemetry_updated),
        .motion_status(eth_motion_status), .motion_updated(eth_telemetry_updated), .rx_debug(eth_rx_debug),
        .lidar_packet(lidar_packet), .lidar_updated(lidar_packet_valid),
        .pixel_empty(udp_empty), .pixel(udp_pixel), .pixel_read(udp_read),
        .frame_request_toggle(udp_request_toggle),
        .gmii_tx_clk(gmii_clk), .gmii_tx_en(gmii_tx_en), .gmii_tx_data(gmii_tx_data)
    );
    gmii_to_rgmii transmit_phy (
        .gmii_tx_clk(gmii_clk),
        .gmii_txd(gmii_tx_data),
        .gmii_txen(gmii_tx_en), .gmii_txer(1'b0),
        .rgmii_tx_clk(eth_tx_clk), .rgmii_txd(eth_txd), .rgmii_txen(eth_tx_en)
    );

    camera_framebuffer framebuffer (
        .ui_clk(ui_clk), .reset_n(ui_reset[1]), .calibrated(ddr_ready),
        .camera_pclk(camera_pclk), .camera_active(camera_active),
        .camera_valid(camera_valid), .camera_pixel(camera_pixel),
        .frame_valid(frame_valid),
        .udp_clk(clk50m), .udp_request_toggle(udp_request_toggle), .udp_read(udp_read),
        .udp_empty(udp_empty), .udp_pixel(udp_pixel),
        .cmd_ready(cmd_ready), .write_ready(write_ready), .read_valid(read_valid),
        .read_data(read_data), .cmd_addr(cmd_addr), .cmd(cmd), .cmd_enable(cmd_enable),
        .write_enable(write_enable), .write_data(write_data)
    );
    DDR3_Memory_Interface_Top_60B memory (
        .clk(ddr_ref), .memory_clk(ddr_mem_clk), .pll_lock(ddr_locked),
        .pll_stop(pll_stop), .rst_n(reset_n), .clk_out(ui_clk), .ddr_rst(),
        .init_calib_complete(ddr_ready), .burst(1'b1),
        .cmd_ready(cmd_ready), .cmd(cmd), .cmd_en(cmd_enable), .addr(cmd_addr),
        .wr_data_rdy(write_ready), .wr_data(write_data), .wr_data_en(write_enable),
        .wr_data_end(write_enable), .wr_data_mask(16'd0),
        .rd_data(read_data), .rd_data_valid(read_valid), .rd_data_end(),
        .sr_req(1'b0), .ref_req(1'b0), .sr_ack(), .ref_ack(),
        .O_ddr_addr(O_ddr_addr), .O_ddr_ba(O_ddr_ba), .O_ddr_cs_n(O_ddr_cs_n),
        .O_ddr_ras_n(O_ddr_ras_n), .O_ddr_cas_n(O_ddr_cas_n), .O_ddr_we_n(O_ddr_we_n),
        .O_ddr_clk(O_ddr_clk), .O_ddr_clk_n(O_ddr_clk_n), .O_ddr_cke(O_ddr_cke),
        .O_ddr_odt(O_ddr_odt), .O_ddr_reset_n(O_ddr_reset_n), .O_ddr_dqm(O_ddr_dqm),
        .IO_ddr_dq(IO_ddr_dq), .IO_ddr_dqs(IO_ddr_dqs), .IO_ddr_dqs_n(IO_ddr_dqs_n)
    );

    assign camera_xclk = camera_ref25;
    assign led = ~{frame_valid, camera_configured, ddr_ready, video_locked};
endmodule
