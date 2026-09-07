`timescale 1ns/1ps

module tb_digital_ir_proximity;
    reg clk = 0;
    reg rst_n = 0;
    reg sensor_n = 1;
    wire object_near;
    wire changed;
    integer checks = 0;
    integer changes = 0;
    always #5 clk = ~clk;
    always @(posedge clk) if (changed) changes <= changes + 1;

    sensor_example_top #(.ASSERT_SAMPLES(3), .RELEASE_SAMPLES(3)) dut (
        .clk_i(clk), .rst_n_i(rst_n), .sensor_n_i(sensor_n),
        .object_near_o(object_near), .changed_o(changed)
    );

    task check_condition;
        input condition;
        input [255:0] message;
        begin
            checks = checks + 1;
            if (!condition) begin
                $display("FAIL: %0s", message);
                $fatal(1);
            end
        end
    endtask

    initial begin
        repeat (3) @(posedge clk);
        rst_n = 1;
        repeat (4) @(posedge clk);
        sensor_n = 0;
        @(posedge clk); sensor_n = 1;
        repeat (5) @(posedge clk); #1;
        check_condition(!object_near, "single-cycle noise rejected");
        sensor_n = 0;
        repeat (7) @(posedge clk); #1;
        check_condition(object_near, "stable low asserts proximity");
        sensor_n = 1;
        @(posedge clk); sensor_n = 0;
        repeat (5) @(posedge clk); #1;
        check_condition(object_near, "single-cycle release noise rejected");
        sensor_n = 1;
        repeat (7) @(posedge clk); #1;
        check_condition(!object_near, "stable high releases proximity");
        check_condition(changes == 2, "exactly two debounced changes");
        $display("PASS: digital_ir_proximity (%0d checks)", checks);
        $finish;
    end
endmodule
