`timescale 1ns/1ps

module tb_tcrt5000;
    reg clk;
    reg rst_n;
    reg tcrt5000_digital;
    wire line_detected;
    wire line_rise;
    wire line_fall;
    wire [31:0] filter_level;

    integer rise_count;
    integer fall_count;
    integer index;

    sensor_example_top #(
        .ACTIVE_LOW(1),
        .FILTER_CYCLES(4)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .tcrt5000_digital(tcrt5000_digital),
        .line_detected(line_detected),
        .line_rise(line_rise),
        .line_fall(line_fall),
        .filter_level(filter_level)
    );

    always #50 clk = ~clk;

    always @(posedge clk) begin
        if (line_rise)
            rise_count = rise_count + 1;
        if (line_fall)
            fall_count = fall_count + 1;
    end

    task clock_cycles;
        input integer count;
        begin
            for (index = 0; index < count; index = index + 1)
                @(posedge clk);
            #1;
        end
    endtask

    initial begin
        clk               = 1'b0;
        rst_n             = 1'b0;
        tcrt5000_digital  = 1'b1;
        rise_count        = 0;
        fall_count        = 0;

        clock_cycles(4);
        rst_n = 1'b1;
        clock_cycles(3);

        // A two-cycle active-low glitch must be rejected.
        tcrt5000_digital = 1'b0;
        clock_cycles(2);
        tcrt5000_digital = 1'b1;
        clock_cycles(6);
        if (line_detected !== 1'b0 || rise_count != 0 || fall_count != 0)
            $fatal(1, "short optical glitch was not filtered");

        // A sustained active level must assert the line state once.
        tcrt5000_digital = 1'b0;
        clock_cycles(8);
        if (line_detected !== 1'b1 || rise_count != 1)
            $fatal(1, "stable line was not detected");

        // One inactive sample while detected must not clear the state.
        tcrt5000_digital = 1'b1;
        clock_cycles(1);
        tcrt5000_digital = 1'b0;
        clock_cycles(4);
        if (line_detected !== 1'b1 || fall_count != 0)
            $fatal(1, "hysteresis did not reject an inactive glitch");

        // A sustained inactive input must eventually clear the state once.
        tcrt5000_digital = 1'b1;
        clock_cycles(9);
        if (line_detected !== 1'b0 || rise_count != 1 || fall_count != 1)
            $fatal(1, "line release was not filtered correctly");

        $display("PASS: TCRT5000 digital filtering and edge pulses");
        $finish;
    end

endmodule
