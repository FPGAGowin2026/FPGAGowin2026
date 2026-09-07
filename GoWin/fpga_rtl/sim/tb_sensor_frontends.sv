`timescale 1ns/1ps

// 纯 RTL 自检：覆盖 DVP 正常组帧及半像素丢弃、IMU 原子提交及中途重启、
// 三路测距的校准、越界失效和陈旧数据超时。不模拟任何具体传感器总线。
module tb_sensor_frontends;

    reg pclk_i = 1'b0;
    reg clk_i = 1'b0;
    reg rst_n_i = 1'b0;

    reg       vsync_i = 1'b1;
    reg       href_i = 1'b0;
    reg [7:0] camera_data_i = 8'd0;
    wire      frame_valid_o;
    wire      line_valid_o;
    wire      pixel_valid_o;
    wire [15:0] pixel_rgb565_o;

    reg       imu_sample_start_i = 1'b0;
    reg       imu_byte_valid_i = 1'b0;
    reg [7:0] imu_byte_data_i = 8'd0;
    wire      imu_byte_ready_o;
    wire [95:0] imu_sample_data_o;
    wire      imu_sample_valid_o;
    wire      imu_sample_error_o;

    reg        front_sample_valid_i = 1'b0;
    reg [15:0] front_distance_mm_i = 16'd0;
    reg        left_sample_valid_i = 1'b0;
    reg [15:0] left_distance_mm_i = 16'd0;
    reg        right_sample_valid_i = 1'b0;
    reg [15:0] right_distance_mm_i = 16'd0;
    wire [15:0] front_distance_mm_o;
    wire        front_valid_o;
    wire        front_timeout_o;
    wire        front_invalid_sample_o;
    wire [15:0] left_distance_mm_o;
    wire        left_valid_o;
    wire        left_timeout_o;
    wire        left_invalid_sample_o;
    wire [15:0] right_distance_mm_o;
    wire        right_valid_o;
    wire        right_timeout_o;
    wire        right_invalid_sample_o;

    integer check_count = 0;

    always #4 pclk_i = ~pclk_i;
    always #5 clk_i = ~clk_i;

    ov5640_dvp_capture camera_dut (
        .pclk_i(pclk_i),
        .rst_n_i(rst_n_i),
        .vsync_i(vsync_i),
        .href_i(href_i),
        .data_i(camera_data_i),
        .frame_valid_o(frame_valid_o),
        .line_valid_o(line_valid_o),
        .pixel_valid_o(pixel_valid_o),
        .pixel_rgb565_o(pixel_rgb565_o)
    );

    imu_sample_commit #(
        .ACCEL_X_BIAS(16'sd1),
        .ACCEL_Y_BIAS(-16'sd2),
        .ACCEL_Z_BIAS(16'sd3),
        .GYRO_X_BIAS(16'sd0),
        .GYRO_Y_BIAS(16'sd10),
        .GYRO_Z_BIAS(-16'sd1)
    ) imu_dut (
        .clk_i(clk_i),
        .rst_n_i(rst_n_i),
        .sample_start_i(imu_sample_start_i),
        .byte_valid_i(imu_byte_valid_i),
        .byte_data_i(imu_byte_data_i),
        .byte_ready_o(imu_byte_ready_o),
        .sample_data_o(imu_sample_data_o),
        .sample_valid_o(imu_sample_valid_o),
        .sample_error_o(imu_sample_error_o)
    );

    range_validity_monitor #(
        .TIMEOUT_CYCLES(4),
        .MIN_DISTANCE_MM(16'd20),
        .MAX_DISTANCE_MM(16'd4000),
        .FRONT_OFFSET_MM(16'sd5),
        .LEFT_OFFSET_MM(-16'sd10),
        .RIGHT_OFFSET_MM(16'sd0)
    ) range_dut (
        .clk_i(clk_i),
        .rst_n_i(rst_n_i),
        .front_sample_valid_i(front_sample_valid_i),
        .front_distance_mm_i(front_distance_mm_i),
        .left_sample_valid_i(left_sample_valid_i),
        .left_distance_mm_i(left_distance_mm_i),
        .right_sample_valid_i(right_sample_valid_i),
        .right_distance_mm_i(right_distance_mm_i),
        .front_distance_mm_o(front_distance_mm_o),
        .front_valid_o(front_valid_o),
        .front_timeout_o(front_timeout_o),
        .front_invalid_sample_o(front_invalid_sample_o),
        .left_distance_mm_o(left_distance_mm_o),
        .left_valid_o(left_valid_o),
        .left_timeout_o(left_timeout_o),
        .left_invalid_sample_o(left_invalid_sample_o),
        .right_distance_mm_o(right_distance_mm_o),
        .right_valid_o(right_valid_o),
        .right_timeout_o(right_timeout_o),
        .right_invalid_sample_o(right_invalid_sample_o)
    );

    task check_true;
        input condition;
        input [8*64-1:0] description;
        begin
            check_count = check_count + 1;
            if (condition !== 1'b1) begin
                $display("FAIL check=%0d: %0s", check_count, description);
                $fatal(1);
            end
        end
    endtask

    task send_camera_byte;
        input frame_sync;
        input line_active;
        input [7:0] byte_value;
        begin
            @(negedge pclk_i);
            vsync_i = frame_sync;
            href_i = line_active;
            camera_data_i = byte_value;
            @(posedge pclk_i);
            #1;
        end
    endtask

    task start_imu_sample;
        begin
            @(negedge clk_i);
            imu_sample_start_i = 1'b1;
            @(posedge clk_i);
            #1;
            @(negedge clk_i);
            imu_sample_start_i = 1'b0;
        end
    endtask

    task send_imu_byte;
        input [7:0] byte_value;
        begin
            check_true(imu_byte_ready_o, "IMU receiver ready");
            imu_byte_data_i = byte_value;
            imu_byte_valid_i = 1'b1;
            @(posedge clk_i);
            #1;
            @(negedge clk_i);
            imu_byte_valid_i = 1'b0;
        end
    endtask

    initial begin
        // 两个时钟域共用测试复位；实板顶层仍需按真实时钟树处理复位释放。
        repeat (2) @(posedge clk_i);
        rst_n_i = 1'b1;

        // VSYNC 高电平期间无帧、无行、无像素提交。
        send_camera_byte(1'b1, 1'b0, 8'h00);
        check_true(!frame_valid_o && !line_valid_o && !pixel_valid_o,
                   "camera idle during VSYNC");

        // RGB565 高字节在前，只有第二个字节到达时提交像素。
        send_camera_byte(1'b0, 1'b1, 8'h12);
        check_true(frame_valid_o && line_valid_o && !pixel_valid_o,
                   "camera first byte held");
        send_camera_byte(1'b0, 1'b1, 8'h34);
        check_true(pixel_valid_o && pixel_rgb565_o == 16'h1234,
                   "camera RGB565 pixel commit");
        send_camera_byte(1'b0, 1'b1, 8'h56);
        send_camera_byte(1'b0, 1'b1, 8'h78);
        check_true(pixel_valid_o && pixel_rgb565_o == 16'h5678,
                   "camera consecutive pixel commit");

        // 半像素在 HREF 拉低时必须丢弃，下一行从新高字节重新开始。
        send_camera_byte(1'b0, 1'b1, 8'hab);
        send_camera_byte(1'b0, 1'b0, 8'h00);
        check_true(frame_valid_o && !line_valid_o && !pixel_valid_o,
                   "camera drops partial pixel at line end");
        send_camera_byte(1'b0, 1'b1, 8'hcd);
        check_true(!pixel_valid_o, "camera new line starts with high byte");
        send_camera_byte(1'b0, 1'b1, 8'hef);
        check_true(pixel_valid_o && pixel_rgb565_o == 16'hcdef,
                   "camera never joins bytes across lines");
        send_camera_byte(1'b1, 1'b0, 8'h00);
        check_true(!frame_valid_o && !line_valid_o && !pixel_valid_o,
                   "camera frame end invalidates outputs");

        // 未完整的 IMU 样本不得更新对外寄存器；重启给出错误脉冲。
        start_imu_sample();
        send_imu_byte(8'h12);
        send_imu_byte(8'h34);
        check_true(imu_sample_data_o == 96'd0 && !imu_sample_valid_o,
                   "IMU partial sample remains private");
        start_imu_sample();
        check_true(imu_sample_error_o && imu_sample_data_o == 96'd0,
                   "IMU restart discards partial sample");

        // 六轴原始量依次为 100、200、-100、50、-50、0，核对原子提交和偏置。
        send_imu_byte(8'h00); send_imu_byte(8'h64);
        send_imu_byte(8'h00); send_imu_byte(8'hc8);
        send_imu_byte(8'hff); send_imu_byte(8'h9c);
        send_imu_byte(8'h00); send_imu_byte(8'h32);
        send_imu_byte(8'hff); send_imu_byte(8'hce);
        send_imu_byte(8'h00); send_imu_byte(8'h00);
        check_true(imu_sample_valid_o, "IMU commits on twelfth byte only");
        check_true(imu_sample_data_o == 96'h0063_00ca_ff99_0032_ffc4_0001,
                   "IMU fixed register order and bias calibration");
        @(posedge clk_i);
        #1;
        check_true(!imu_sample_valid_o, "IMU valid is a one-cycle pulse");

        // 三路同时提交：前/左应用偏置，右侧越界样本立即无效。
        @(negedge clk_i);
        front_distance_mm_i = 16'd100;
        left_distance_mm_i = 16'd200;
        right_distance_mm_i = 16'd5000;
        front_sample_valid_i = 1'b1;
        left_sample_valid_i = 1'b1;
        right_sample_valid_i = 1'b1;
        @(posedge clk_i);
        #1;
        check_true(front_valid_o && front_distance_mm_o == 16'd105,
                   "front range calibration");
        check_true(left_valid_o && left_distance_mm_o == 16'd190,
                   "left range calibration");
        check_true(!right_valid_o && right_invalid_sample_o,
                   "right out-of-range sample rejected");
        @(negedge clk_i);
        front_sample_valid_i = 1'b0;
        left_sample_valid_i = 1'b0;
        right_sample_valid_i = 1'b0;

        // 右侧可从坏样本恢复；左侧负偏置饱和为零后仍按量程判为无效。
        // 同拍刷新前路，以便单独核对从最后一次有效样本开始的超时计数。
        front_distance_mm_i = 16'd100;
        front_sample_valid_i = 1'b1;
        right_distance_mm_i = 16'd300;
        right_sample_valid_i = 1'b1;
        left_distance_mm_i = 16'd5;
        left_sample_valid_i = 1'b1;
        @(posedge clk_i);
        #1;
        check_true(right_valid_o && right_distance_mm_o == 16'd300,
                   "right range recovers on valid sample");
        check_true(!left_valid_o && left_invalid_sample_o,
                   "left calibrated underflow rejected");
        @(negedge clk_i);
        front_sample_valid_i = 1'b0;
        right_sample_valid_i = 1'b0;
        left_sample_valid_i = 1'b0;

        // TIMEOUT_CYCLES=4：前路最后一个有效样本后恰好四拍失效并给出脉冲。
        repeat (3) begin
            @(posedge clk_i);
            #1;
            check_true(front_valid_o && !front_timeout_o,
                       "front range remains valid before timeout");
        end
        @(posedge clk_i);
        #1;
        check_true(!front_valid_o && front_timeout_o,
                   "front range timeout invalidates stale sample");
        @(posedge clk_i);
        #1;
        check_true(!front_timeout_o, "range timeout is a one-cycle pulse");

        $display("PASS: %0d sensor frontend checks", check_count);
        $finish;
    end

endmodule
