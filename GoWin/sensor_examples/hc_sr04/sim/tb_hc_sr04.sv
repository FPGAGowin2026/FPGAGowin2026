`timescale 1ns/1ps

module tb_hc_sr04;
    localparam integer CLK_FREQ_HZ = 10000000;

    reg clk;
    reg rst_n;
    reg echo_in;
    wire trigger_out;
    wire [15:0] distance_mm;
    wire [15:0] pulse_width_us;
    wire sample_valid;
    wire timeout_error;
    wire busy;

    integer sample_count;
    integer timeout_count;
    time trigger_start;
    time trigger_end;

    reg frac_clk = 1'b0;
    reg frac_rst_n = 1'b0;
    reg frac_echo = 1'b0;
    wire frac_trigger;
    wire [15:0] frac_distance_mm;
    wire [15:0] frac_pulse_width_us;
    wire frac_sample_valid;
    wire frac_timeout_error;
    wire frac_busy;
    reg frac_done = 1'b0;

    sensor_example_top #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .TRIGGER_US(10),
        .TRIGGER_PERIOD_US(100),
        .ECHO_TIMEOUT_US(1000)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .echo_in(echo_in),
        .trigger_out(trigger_out),
        .distance_mm(distance_mm),
        .pulse_width_us(pulse_width_us),
        .sample_valid(sample_valid),
        .timeout_error(timeout_error),
        .busy(busy)
    );

    always #50 clk = ~clk;
    always #20.345052 frac_clk = ~frac_clk;

    sensor_example_top #(
        .CLK_FREQ_HZ(24576000),
        .TRIGGER_US(10),
        .TRIGGER_PERIOD_US(100),
        .ECHO_TIMEOUT_US(1000)
    ) fractional_clock_dut (
        .clk(frac_clk), .rst_n(frac_rst_n), .echo_in(frac_echo),
        .trigger_out(frac_trigger), .distance_mm(frac_distance_mm),
        .pulse_width_us(frac_pulse_width_us),
        .sample_valid(frac_sample_valid), .timeout_error(frac_timeout_error),
        .busy(frac_busy)
    );

    always @(posedge clk) begin
        if (sample_valid)
            sample_count = sample_count + 1;
        if (timeout_error)
            timeout_count = timeout_count + 1;
    end

    initial begin : fractional_clock_check
        time fractional_trigger_start;
        time fractional_trigger_end;
        #500 frac_rst_n = 1'b1;
        @(posedge frac_trigger);
        fractional_trigger_start = $time;
        @(negedge frac_trigger);
        fractional_trigger_end = $time;
        if ((fractional_trigger_end - fractional_trigger_start) < 10000)
            $fatal(1, "24.576 MHz trigger pulse became shorter than 10 us");
        #20050 frac_echo = 1'b1;
        #580500 frac_echo = 1'b0;
        @(posedge frac_sample_valid); #1;
        if (frac_pulse_width_us != 16'd581 ||
            frac_distance_mm < 16'd99 || frac_distance_mm > 16'd101)
            $fatal(1, "fractional-MHz echo conversion is wrong: %0d us, %0d mm",
                   frac_pulse_width_us, frac_distance_mm);
        frac_done = 1'b1;
    end

    initial begin
        clk           = 1'b0;
        rst_n         = 1'b0;
        echo_in       = 1'b0;
        sample_count  = 0;
        timeout_count = 0;

        #500;
        rst_n = 1'b1;

        // First transaction: 580.5 us tests half-microsecond round-up and
        // still corresponds to approximately 100 mm.
        @(posedge trigger_out);
        trigger_start = $time;
        @(negedge trigger_out);
        trigger_end = $time;
        if ((trigger_end - trigger_start) < 9900 ||
            (trigger_end - trigger_start) > 10100)
            $fatal(1, "trigger pulse is not 10 us wide");

        #20050;
        echo_in = 1'b1;
        #580500;
        echo_in = 1'b0;
        @(posedge sample_valid);
        @(posedge clk);
        #1;
        if (distance_mm < 16'd99 || distance_mm > 16'd101)
            $fatal(1, "distance conversion is outside tolerance: %0d mm", distance_mm);
        if (pulse_width_us != 16'd581)
            $fatal(1, "half-microsecond echo did not round up: %0d us", pulse_width_us);
        if (sample_count != 1 || timeout_count != 0)
            $fatal(1, "unexpected event pulse count after valid echo");

        // Second transaction: no echo must generate a timeout and no sample.
        @(posedge trigger_out);
        @(negedge trigger_out);
        @(posedge timeout_error);
        @(posedge clk);
        #1;
        if (sample_count != 1 || timeout_count != 1)
            $fatal(1, "missing or spurious timeout event");

        wait(frac_done);
        $display("PASS: HC-SR04 trigger, echo measurement, timeout, and fractional-MHz clock");
        $finish;
    end

endmodule
