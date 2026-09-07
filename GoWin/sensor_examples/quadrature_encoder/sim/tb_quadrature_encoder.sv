`timescale 1ns/1ps

module tb_quadrature_encoder;
    reg clk = 0;
    reg rst_n = 0;
    reg a = 0;
    reg b = 0;
    wire signed [31:0] ticks;
    wire step_valid;
    wire direction;
    wire invalid_transition;
    integer checks = 0;
    integer invalid_count = 0;

    always #5 clk = ~clk;
    always @(posedge clk) if (invalid_transition) invalid_count <= invalid_count + 1;

    sensor_example_top dut (
        .clk_i(clk), .rst_n_i(rst_n), .encoder_a_i(a), .encoder_b_i(b),
        .position_ticks_o(ticks), .step_valid_o(step_valid),
        .direction_o(direction), .invalid_transition_o(invalid_transition)
    );

    task check_condition;
        input condition;
        input [255:0] message;
        begin
            checks = checks + 1;
            if (!condition) begin
                $display("FAIL: %0s ticks=%0d", message, ticks);
                $fatal(1);
            end
        end
    endtask

    task set_ab;
        input [1:0] value;
        begin
            {a,b} = value;
            repeat (4) @(posedge clk);
            #1;
        end
    endtask

    initial begin
        {a,b} = 2'b10;
        repeat (3) @(posedge clk);
        rst_n = 1;
        repeat (7) @(posedge clk); #1;
        check_condition(ticks == 0 && invalid_count == 0,
                        "nonzero startup phase produces no false event");

        rst_n = 0;
        {a,b} = 2'b00;
        repeat (2) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);
        set_ab(2'b01); set_ab(2'b11); set_ab(2'b10); set_ab(2'b00);
        check_condition(ticks == 4, "forward cycle produces four ticks");
        check_condition(direction == 1'b1, "forward direction");
        set_ab(2'b10); set_ab(2'b11); set_ab(2'b01); set_ab(2'b00);
        check_condition(ticks == 0, "reverse cycle returns to zero");
        check_condition(direction == 1'b0, "reverse direction");
        set_ab(2'b11);
        check_condition(ticks == 0, "illegal two-bit transition changes no count");
        check_condition(invalid_count == 1, "illegal transition reported once");
        $display("PASS: quadrature_encoder (%0d checks)", checks);
        $finish;
    end
endmodule
