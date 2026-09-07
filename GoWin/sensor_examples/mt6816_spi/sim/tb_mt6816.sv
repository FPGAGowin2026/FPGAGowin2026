`timescale 1ns/1ps
module tb_mt6816;
    logic clk = 0, reset_n = 0, miso = 0;
    wire cs_n, sclk, mosi, initialized, sensor_ok, sample_valid, protocol_error;
    wire [13:0] angle_raw;
    wire no_magnet, over_speed;
    integer bit_no = 0, round_no = 0;
    logic [7:0] command = 0;
    logic error_seen = 0;
    logic [7:0] response;

    sensor_example_top #(.CLK_DIV(2), .STARTUP_DELAY_CYCLES(4),
        .SAMPLE_INTERVAL_CYCLES(8)) dut (.*,
        .encoder_miso(miso), .encoder_cs_n(cs_n), .encoder_sclk(sclk),
        .encoder_mosi(mosi));
    always #5 clk = ~clk;
    always @(posedge clk) if (protocol_error) error_seen <= 1;

    function automatic [7:0] mt_response(input [7:0] cmd, input integer rnd);
        logic [7:0] hi, lo;
        begin
            hi = 8'h12;
            lo = {6'h15, 1'b0, 1'b0};
            lo[0] = ^{hi, lo[7:1]};
            if (rnd == 0) lo[0] = ~lo[0];
            case (cmd)
                8'h83: mt_response = hi;
                8'h84: mt_response = lo;
                8'h85: mt_response = 8'h00;
                default: mt_response = 8'hff;
            endcase
        end
    endfunction

    always @(negedge cs_n) begin
        if (sclk !== 1'b1) $fatal(1, "MT6816 SCLK must idle high");
        bit_no = 0; command = 0; miso = 0;
    end
    always @(posedge sclk) if (!cs_n) begin
        if (bit_no < 8) command = {command[6:0], mosi};
        bit_no = bit_no + 1;
    end
    always @(negedge sclk) if (!cs_n && bit_no >= 8 && bit_no < 16) begin
        response = mt_response(command, round_no);
        miso = response[15-bit_no];
    end
    always @(posedge cs_n) if (command == 8'h85) round_no = round_no + 1;

    initial begin
        repeat (3) @(posedge clk); reset_n = 1;
        repeat (5000) begin
            @(posedge clk);
            if (sample_valid) begin
                if (!initialized || !sensor_ok || !error_seen)
                    $fatal(1, "parity error/recovery path not covered");
                if (angle_raw !== {8'h12, 6'h15}) $fatal(1, "angle byte order wrong");
                if (no_magnet || over_speed) $fatal(1, "unexpected warning flags");
                $display("PASS: MT6816 Mode-3 angle read rejects bad parity and recovers");
                $finish;
            end
        end
        $fatal(1, "timeout waiting for MT6816 sample");
    end
endmodule
