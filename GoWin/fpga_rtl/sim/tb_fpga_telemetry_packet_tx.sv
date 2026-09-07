`timescale 1ns/1ps

module tb_fpga_telemetry_packet_tx;
    reg clk_i = 1'b0;
    reg rst_n_i = 1'b0;
    reg start_i = 1'b0;
    reg ready_i = 1'b1;
    wire valid_o;
    wire [7:0] data_o;
    wire busy_o;
    wire done_o;
    reg [7:0] captured [0:71];
    integer count = 0;
    integer i;
    reg [15:0] crc;
    reg [7:0] held_data;

    fpga_telemetry_packet_tx dut (
        .clk_i(clk_i), .rst_n_i(rst_n_i), .start_i(start_i), .ready_i(ready_i),
        .sequence_i(32'h01020304), .timestamp_us_i(64'h0102030405060708),
        .left_ticks_i(-32'sd1), .right_ticks_i(32'sd2),
        .left_velocity_mm_s_i(-32'sd300), .right_velocity_mm_s_i(32'sd400),
        .accel_x_mm_s2_i(32'sd10), .accel_y_mm_s2_i(32'sd20), .accel_z_mm_s2_i(32'sd30),
        .gyro_x_mrad_s_i(-32'sd40), .gyro_y_mrad_s_i(32'sd50), .gyro_z_mrad_s_i(32'sd60),
        .range_front_mm_i(16'd700), .range_left_mm_i(16'd800), .range_right_mm_i(16'd900),
        .status_bits_i(32'h89abcdef),
        .valid_o(valid_o), .data_o(data_o), .busy_o(busy_o), .done_o(done_o)
    );

    always #5 clk_i = ~clk_i;

    function automatic [15:0] crc16_next;
        input [15:0] crc_in;
        input [7:0] data_in;
        integer bit_index;
        reg [15:0] work;
        begin
            work = crc_in ^ {data_in, 8'h00};
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1)
                work = work[15] ? ((work << 1) ^ 16'h1021) : (work << 1);
            crc16_next = work;
        end
    endfunction

    always @(posedge clk_i) begin
        if (valid_o && ready_i) begin
            captured[count] = data_o;
            count = count + 1;
        end
    end

    initial begin
        repeat (3) @(posedge clk_i);
        rst_n_i = 1'b1;
        @(negedge clk_i); start_i = 1'b1;
        @(negedge clk_i); start_i = 1'b0;

        // 下游背压期间 valid/data/busy 保持，恢复 ready 后从原字节继续。
        wait (count == 10);
        @(negedge clk_i); ready_i = 1'b0; held_data = data_o;
        repeat (4) begin
            @(posedge clk_i); #1;
            if (!valid_o || !busy_o || data_o !== held_data || count != 10)
                $fatal(1, "FAIL telemetry backpressure hold");
        end
        @(negedge clk_i); ready_i = 1'b1;

        wait (done_o);
        #1;
        if (count != 72)
            $fatal(1, "FAIL telemetry length=%0d", count);
        if ({captured[0],captured[1],captured[2],captured[3]} !== 32'h46504741 ||
            captured[4] !== 8'h01 || captured[5] !== 8'h02 ||
            {captured[6],captured[7]} !== 16'd50)
            $fatal(1, "FAIL telemetry header");
        if ({captured[20],captured[21],captured[22],captured[23]} !== 32'hffffffff ||
            {captured[60],captured[61]} !== 16'd700 ||
            {captured[66],captured[67],captured[68],captured[69]} !== 32'h89abcdef)
            $fatal(1, "FAIL telemetry payload");
        crc = 16'hffff;
        for (i = 0; i < 70; i = i + 1)
            crc = crc16_next(crc, captured[i]);
        if ({captured[70],captured[71]} !== crc)
            $fatal(1, "FAIL telemetry CRC got=%02x%02x expected=%04x",
                   captured[70], captured[71], crc);
        $display("PASS: fpga_telemetry_packet_tx 72-byte frame with backpressure");
        $finish;
    end
endmodule
