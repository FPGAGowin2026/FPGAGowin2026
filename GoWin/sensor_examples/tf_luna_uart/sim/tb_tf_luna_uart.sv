`timescale 1ns/1ps

module tb_tf_luna_uart;
    localparam integer CLK_FREQ_HZ = 10000000;
    localparam integer BAUD_RATE   = 1000000;
    localparam integer BIT_NS      = 1000;

    reg clk;
    reg rst_n;
    reg tf_luna_rx;
    wire [15:0] distance_cm;
    wire [15:0] strength;
    wire [15:0] temperature_raw;
    wire frame_valid;
    wire checksum_error;

    integer valid_count;
    integer error_count;
    reg [7:0] checksum;

    sensor_example_top #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUD_RATE(BAUD_RATE)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .tf_luna_rx(tf_luna_rx),
        .distance_cm(distance_cm),
        .strength(strength),
        .temperature_raw(temperature_raw),
        .frame_valid(frame_valid),
        .checksum_error(checksum_error)
    );

    always #50 clk = ~clk;

    always @(posedge clk) begin
        if (frame_valid)
            valid_count = valid_count + 1;
        if (checksum_error)
            error_count = error_count + 1;
    end

    task send_uart_byte;
        input [7:0] value;
        integer bit_number;
        begin
            tf_luna_rx = 1'b0;
            #(BIT_NS);
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
                tf_luna_rx = value[bit_number];
                #(BIT_NS);
            end
            tf_luna_rx = 1'b1;
            #(BIT_NS);
        end
    endtask

    task send_frame;
        input [15:0] frame_distance;
        input [15:0] frame_strength;
        input [15:0] frame_temperature;
        input integer make_bad_checksum;
        begin
            checksum = 8'h59 + 8'h59 +
                       frame_distance[7:0] + frame_distance[15:8] +
                       frame_strength[7:0] + frame_strength[15:8] +
                       frame_temperature[7:0] + frame_temperature[15:8];
            send_uart_byte(8'h59);
            send_uart_byte(8'h59);
            send_uart_byte(frame_distance[7:0]);
            send_uart_byte(frame_distance[15:8]);
            send_uart_byte(frame_strength[7:0]);
            send_uart_byte(frame_strength[15:8]);
            send_uart_byte(frame_temperature[7:0]);
            send_uart_byte(frame_temperature[15:8]);
            if (make_bad_checksum != 0)
                send_uart_byte(checksum ^ 8'h01);
            else
                send_uart_byte(checksum);
        end
    endtask

    initial begin
        clk          = 1'b0;
        rst_n        = 1'b0;
        tf_luna_rx   = 1'b1;
        valid_count  = 0;
        error_count  = 0;

        #500;
        rst_n = 1'b1;
        #1000;

        // Noise bytes must not create a frame.
        send_uart_byte(8'h00);
        send_uart_byte(8'h59);
        send_uart_byte(8'h12);
        send_frame(16'd1234, 16'd567, 16'd2104, 0);
        #(5 * BIT_NS);

        if (valid_count != 1 || error_count != 0)
            $fatal(1, "valid frame pulse counts are wrong");
        if (distance_cm !== 16'd1234 || strength !== 16'd567 ||
            temperature_raw !== 16'd2104)
            $fatal(1, "valid frame payload was decoded incorrectly");

        // A corrupt frame must be reported and must not overwrite data.
        send_frame(16'd88, 16'd99, 16'd2200, 1);
        #(5 * BIT_NS);
        if (valid_count != 1 || error_count != 1)
            $fatal(1, "bad checksum handling is wrong");
        if (distance_cm !== 16'd1234 || strength !== 16'd567 ||
            temperature_raw !== 16'd2104)
            $fatal(1, "bad frame overwrote the last valid sample");

        // Confirm recovery on the next valid frame.
        send_frame(16'd42, 16'd1000, 16'd2048, 0);
        #(5 * BIT_NS);
        if (valid_count != 2 || error_count != 1)
            $fatal(1, "parser did not recover after a checksum error");
        if (distance_cm !== 16'd42 || strength !== 16'd1000 ||
            temperature_raw !== 16'd2048)
            $fatal(1, "recovery frame payload is wrong");

        // 截断帧在超过一个字节时间后必须超时清除，不能与下一帧拼接。
        send_uart_byte(8'h59);
        send_uart_byte(8'h59);
        send_uart_byte(8'h12);
        #(15 * BIT_NS);
        send_frame(16'd77, 16'd88, 16'd99, 0);
        #(5 * BIT_NS);
        if (valid_count != 3 || error_count != 1 ||
            distance_cm !== 16'd77 || strength !== 16'd88 ||
            temperature_raw !== 16'd99)
            $fatal(1, "truncated-frame timeout recovery is wrong");

        // 合法校验字节可能正好等于 0x59；下一帧必须仍从完整双帧头开始。
        send_frame(16'd100, 16'd67, 16'd0, 0);
        send_frame(16'd7, 16'd8, 16'd9, 0);
        #(5 * BIT_NS);
        if (valid_count != 5 || error_count != 1)
            $fatal(1, "0x59 checksum caused parser misalignment");
        if (distance_cm !== 16'd7 || strength !== 16'd8 ||
            temperature_raw !== 16'd9)
            $fatal(1, "frame after 0x59 checksum decoded incorrectly");

        $display("PASS: TF-Luna UART frame parser");
        $finish;
    end

endmodule
