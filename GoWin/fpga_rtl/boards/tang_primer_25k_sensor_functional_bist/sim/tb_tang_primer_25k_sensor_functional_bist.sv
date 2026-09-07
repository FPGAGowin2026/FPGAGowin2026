`timescale 1ns/1ps

module tb_tang_primer_25k_sensor_functional_bist;
    localparam integer CLK_FREQ_HZ = 50000000;
    // 控制台在仿真中加速；片上传感器 UART 保持位流使用的 16 拍/位参数。
    localparam integer CONSOLE_BAUD = 5000000;
    localparam integer SENSOR_BAUD = 3125000;
    localparam integer BIT_NS = 200;

    reg clk_i = 1'b0;
    reg reset_i = 1'b1;
    reg uart_rx_i = 1'b1;
    wire uart_tx_o;

    reg [16:0] independent_activity = 17'd0;
    reg [16:0] independent_valid = 17'd0;
    reg [16:0] previous_pass_mask = 17'd0;
    reg [16:0] new_pass_bits;
    reg hc_trigger_seen = 1'b0;
    reg hc_echo_seen = 1'b0;
    reg tsl_clk_d = 1'b0;
    reg tsl_si_d = 1'b0;
    integer tsl_line_clocks = 0;
    integer tsl_pixels = 0;
    integer encoder_steps = 0;
    integer uart_index;
    reg [7:0] received_byte;

    localparam [31*8-1:0] EXPECT_BANNER =
        {"GW5A25 SENSOR FUNC BIST READY", 8'h0d, 8'h0a};
    localparam [22*8-1:0] EXPECT_PONG =
        {"PONG;FUNC_BIST=READY", 8'h0d, 8'h0a};
    localparam [51*8-1:0] EXPECT_BIST =
        {"FUNC_BIST=PASS;MODULES=17/17;MODE=ONCHIP_STIMULUS", 8'h0d, 8'h0a};
    localparam [45*8-1:0] EXPECT_DETAIL =
        {"PASS=0x1FFFF;OBS=0x1FFFF;ACT=0x1FFFF;DONE=1", 8'h0d, 8'h0a};
    localparam [12*8-1:0] EXPECT_HELP =
        {"CMDS:P,B,D", 8'h0d, 8'h0a};

    tang_primer_25k_sensor_functional_bist_top #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(CONSOLE_BAUD),
        .SENSOR_UART_BAUD_RATE(SENSOR_BAUD),
        .POR_CYCLES(8), .BIST_TIMEOUT_CYCLES(20000)
    ) dut (
        .clk_i(clk_i), .reset_i(reset_i),
        .uart_rx_i(uart_rx_i), .uart_tx_o(uart_tx_o)
    );

    always #10 clk_i = ~clk_i;

    // 独立于 DUT 的协议活动和黄金结果监视器。
    always @(posedge clk_i) begin
        if (!dut.rst_n) begin
            independent_activity <= 17'd0;
            independent_valid <= 17'd0;
            previous_pass_mask <= 17'd0;
            hc_trigger_seen <= 1'b0;
            hc_echo_seen <= 1'b0;
            tsl_clk_d <= 1'b0;
            tsl_si_d <= 1'b0;
            tsl_line_clocks <= 0;
            tsl_pixels <= 0;
            encoder_steps <= 0;
        end else begin
            // 真实片上激励活动。
            if (!dut.u_core.digital_ir_n) independent_activity[0] <= 1'b1;
            if (!dut.u_core.sensor_uart[0]) independent_activity[1] <= 1'b1;
            if (!dut.u_core.sensor_uart[1]) independent_activity[2] <= 1'b1;
            if (dut.u_core.hc_trigger) hc_trigger_seen <= 1'b1;
            if (dut.u_core.hc_echo) hc_echo_seen <= 1'b1;
            if (hc_trigger_seen && hc_echo_seen) independent_activity[3] <= 1'b1;
            if (dut.u_core.itr_pulse) independent_activity[4] <= 1'b1;
            if (!dut.u_core.sensor_uart[2]) independent_activity[5] <= 1'b1;
            if (!dut.u_core.sensor_uart[3]) independent_activity[6] <= 1'b1;
            if (!dut.u_core.bumper_left_n) independent_activity[7] <= 1'b1;
            if (dut.u_core.camera_href && dut.u_core.mt9v03x_data != 0)
                independent_activity[8] <= 1'b1;
            if (dut.u_core.multi_ir_n != 8'hff) independent_activity[9] <= 1'b1;
            if (dut.u_core.camera_href && dut.u_core.ov5640_data != 0)
                independent_activity[10] <= 1'b1;
            if (dut.u_core.camera_href && dut.u_core.ov7725_data != 0)
                independent_activity[11] <= 1'b1;
            if ({dut.u_core.encoder_a, dut.u_core.encoder_b} != 2'b00)
                independent_activity[12] <= 1'b1;
            if (dut.u_core.camera_href && dut.u_core.scc8660_data != 0)
                independent_activity[13] <= 1'b1;
            if (!dut.u_core.tcrt5000_n) independent_activity[14] <= 1'b1;
            if (!dut.u_core.sensor_uart[4]) independent_activity[15] <= 1'b1;
            if (dut.u_core.tsl_sensor_si || dut.u_core.tsl_sensor_clk ||
                dut.u_core.tsl_adc_start)
                independent_activity[16] <= 1'b1;

            // 每项有效脉冲必须携带预期数据，否则立即失败。
            if (dut.u_core.digital_changed) begin
                if (!dut.u_core.digital_object_near)
                    $fatal(1, "digital IR changed without object detection");
                independent_valid[0] <= 1'b1;
            end
            if (dut.u_core.gnss_valid) begin
                if (dut.u_core.gnss_type != 1 || dut.u_core.gnss_length != 11 ||
                    dut.u_core.gnss_checksum != 8'h4a)
                    $fatal(1, "GNSS golden sentence mismatch");
                independent_valid[1] <= 1'b1;
            end
            if (dut.u_core.gy39_lux_valid) begin
                if (dut.u_core.gy39_lux != 32'h00003039)
                    $fatal(1, "GY39 lux mismatch");
                independent_valid[2] <= 1'b1;
            end
            if (dut.u_core.hc_sample_valid) begin
                if (dut.u_core.hc_pulse_width_us != 58 ||
                    dut.u_core.hc_distance_mm != 10)
                    $fatal(1, "HC-SR04 result mismatch: width=%0d distance=%0d",
                           dut.u_core.hc_pulse_width_us,
                           dut.u_core.hc_distance_mm);
                independent_valid[3] <= 1'b1;
            end
            if (dut.u_core.itr_period_valid) begin
                if (dut.u_core.itr_period < 10 || dut.u_core.itr_period > 20)
                    $fatal(1, "ITR9606 period mismatch: %0d", dut.u_core.itr_period);
            end
            if (dut.u_core.itr_sample_valid && dut.u_core.itr_count == 2)
                independent_valid[4] <= 1'b1;
            if (dut.u_core.jy_valid) begin
                if (dut.u_core.jy_roll !== 16'sh1234 ||
                    dut.u_core.jy_pitch !== 16'sh8000 ||
                    dut.u_core.jy_yaw !== 16'shfffe ||
                    dut.u_core.jy_version !== 16'h5678)
                    $fatal(1, "JY61P payload mismatch");
                independent_valid[5] <= 1'b1;
            end
            if (dut.u_core.maxsonar_valid) begin
                if (dut.u_core.maxsonar_range != 1234)
                    $fatal(1, "MaxSonar range mismatch");
                independent_valid[6] <= 1'b1;
            end
            if (dut.u_core.bumper_collision_event) begin
                if (!dut.u_core.bumper_left_pressed ||
                    !dut.u_core.bumper_collision_latched)
                    $fatal(1, "bumper event/latch mismatch");
                independent_valid[7] <= 1'b1;
            end
            if (dut.u_core.mt_pixel_valid && dut.u_core.mt_pixel == 8'h3c)
                independent_valid[8] <= 1'b1;
            if (dut.u_core.multi_ir_changed && dut.u_core.multi_ir_mask == 8'h02)
                independent_valid[9] <= 1'b1;
            if (dut.u_core.ov5640_pixel_valid) begin
                if (dut.u_core.ov5640_pixel != 16'hf800)
                    $fatal(1, "OV5640 little-endian capture mismatch");
                independent_valid[10] <= 1'b1;
            end
            if (dut.u_core.ov7725_pixel_valid) begin
                if (dut.u_core.ov7725_pixel != 16'hf800)
                    $fatal(1, "OV7725 capture mismatch");
                independent_valid[11] <= 1'b1;
            end
            if (dut.u_core.encoder_step_valid) encoder_steps <= encoder_steps + 1;
            if (dut.u_core.encoder_invalid) begin
                if (encoder_steps != 4 || dut.u_core.encoder_position != 4)
                    $fatal(1, "quadrature valid/invalid path mismatch");
                independent_valid[12] <= 1'b1;
            end
            if (dut.u_core.scc_pixel_valid) begin
                if (dut.u_core.scc_pixel != 16'h07e0)
                    $fatal(1, "SCC8660 capture mismatch");
                independent_valid[13] <= 1'b1;
            end
            if (dut.u_core.tcrt_line_rise) begin
                if (!dut.u_core.tcrt_line_detected ||
                    dut.u_core.tcrt_filter_level != 3)
                    $fatal(1, "TCRT5000 filter mismatch");
                independent_valid[14] <= 1'b1;
            end
            if (dut.u_core.tf_valid) begin
                if (dut.u_core.tf_distance != 1234 ||
                    dut.u_core.tf_strength != 567 ||
                    dut.u_core.tf_temperature != 2104)
                    $fatal(1, "TF-Luna payload mismatch");
                independent_valid[15] <= 1'b1;
            end

            tsl_clk_d <= dut.u_core.tsl_sensor_clk;
            tsl_si_d <= dut.u_core.tsl_sensor_si;
            if (dut.u_core.tsl_sensor_si && !tsl_si_d)
                tsl_line_clocks <= 0;
            else if (dut.u_core.tsl_sensor_clk && !tsl_clk_d)
                tsl_line_clocks <= tsl_line_clocks + 1;
            if (dut.u_core.tsl_frame_start)
                tsl_pixels <= 0;
            if (dut.u_core.tsl_pixel_valid) begin
                if (dut.u_core.tsl_pixel_data !==
                    (12'ha00 | dut.u_core.tsl_pixel_index))
                    $fatal(1, "TSL1401 ADC payload mismatch at %0d",
                           dut.u_core.tsl_pixel_index);
                tsl_pixels <= tsl_pixels + 1;
            end
            if (dut.u_core.tsl_frame_done) begin
                if (tsl_line_clocks != 129 || tsl_pixels != 128 ||
                    dut.u_core.tsl_adc_timeout)
                    $fatal(1, "TSL1401 frame mismatch: clocks=%0d pixels=%0d",
                           tsl_line_clocks, tsl_pixels);
                independent_valid[16] <= 1'b1;
            end

            // 任一新 PASS 位都必须晚于独立观测到的激励与有效结果。
            new_pass_bits = dut.pass_mask & ~previous_pass_mask;
            if ((new_pass_bits & ~(independent_activity & independent_valid)) != 0)
                $fatal(1, "constant/early PASS detected: new=%h act=%h valid=%h",
                       new_pass_bits, independent_activity, independent_valid);
            previous_pass_mask <= dut.pass_mask;
        end
    end

    task send_uart_byte;
        input [7:0] value;
        integer bit_number;
        begin
            uart_rx_i = 1'b0;
            #(BIT_NS);
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
                uart_rx_i = value[bit_number];
                #(BIT_NS);
            end
            uart_rx_i = 1'b1;
            #(BIT_NS);
        end
    endtask

    task receive_uart_byte;
        output [7:0] value;
        integer bit_number;
        begin
            @(negedge uart_tx_o);
            #(BIT_NS + BIT_NS/2);
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
                value[bit_number] = uart_tx_o;
                #(BIT_NS);
            end
            if (!uart_tx_o) $fatal(1, "UART stop bit low");
            #(BIT_NS/2);
        end
    endtask

    task expect_message;
        input integer length;
        input [511:0] expected;
        begin
            for (uart_index = 0; uart_index < length; uart_index = uart_index + 1) begin
                receive_uart_byte(received_byte);
                if (received_byte !== expected[(length-1-uart_index)*8 +: 8])
                    $fatal(1, "UART message mismatch at byte %0d: got %02h expected %02h",
                           uart_index, received_byte,
                           expected[(length-1-uart_index)*8 +: 8]);
            end
        end
    endtask

    initial begin
        repeat (5) @(posedge clk_i);
        reset_i = 1'b0;
        fork
            expect_message(31, EXPECT_BANNER);
            begin
                wait (dut.rst_n);
                repeat (2) @(posedge clk_i);
                if (dut.pass_mask != 0)
                    $fatal(1, "PASS mask was nonzero before protocol events");
            end
        join

        wait (dut.bist_done);
        repeat (2) @(posedge clk_i);
        if (dut.pass_mask !== 17'h1ffff ||
            dut.observed_mask !== 17'h1ffff ||
            dut.activity_mask !== 17'h1ffff)
            $fatal(1, "BIST mask mismatch: pass=%h obs=%h act=%h",
                   dut.pass_mask, dut.observed_mask, dut.activity_mask);
        if (independent_activity !== 17'h1ffff ||
            independent_valid !== 17'h1ffff)
            $fatal(1, "independent monitor mismatch: act=%h valid=%h",
                   independent_activity, independent_valid);

        send_uart_byte("P");
        expect_message(22, EXPECT_PONG);
        send_uart_byte("B");
        expect_message(51, EXPECT_BIST);
        send_uart_byte("D");
        expect_message(45, EXPECT_DETAIL);
        send_uart_byte("?");
        expect_message(12, EXPECT_HELP);

        $display("PASS: Tang Primer 25K sensor functional BIST (17/17)");
        $display("PASS=0x%05h OBS=0x%05h ACT=0x%05h",
                 dut.pass_mask, dut.observed_mask, dut.activity_mask);
        $finish;
    end

    initial begin
        #10000000;
        $fatal(1, "simulation timeout");
    end
endmodule
