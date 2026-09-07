`timescale 1ns/1ps

module tb_mechanical_bumper;
    reg clk;
    reg rst_n;
    reg bumper_left_n;
    reg bumper_right_n;
    reg clear_latched;
    wire left_pressed;
    wire right_pressed;
    wire collision;
    wire collision_event;
    wire collision_latched;

    integer event_count;
    integer index;

    sensor_example_top #(
        .ACTIVE_LOW(1),
        .DEBOUNCE_CYCLES(4)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .bumper_left_n(bumper_left_n),
        .bumper_right_n(bumper_right_n),
        .clear_latched(clear_latched),
        .left_pressed(left_pressed),
        .right_pressed(right_pressed),
        .collision(collision),
        .collision_event(collision_event),
        .collision_latched(collision_latched)
    );

    always #50 clk = ~clk;

    always @(posedge clk) begin
        if (collision_event)
            event_count = event_count + 1;
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
        clk            = 1'b0;
        rst_n          = 1'b0;
        bumper_left_n  = 1'b1;
        bumper_right_n = 1'b1;
        clear_latched  = 1'b0;
        event_count    = 0;

        clock_cycles(4);
        rst_n = 1'b1;
        clock_cycles(3);

        // Short contact bounce must not create a collision.
        bumper_left_n = 1'b0;
        clock_cycles(2);
        bumper_left_n = 1'b1;
        clock_cycles(7);
        if (collision !== 1'b0 || collision_latched !== 1'b0 ||
            event_count != 0)
            $fatal(1, "short bumper bounce was not rejected");

        // A sustained left hit must debounce, pulse once, and latch.
        bumper_left_n = 1'b0;
        clock_cycles(9);
        if (left_pressed !== 1'b1 || right_pressed !== 1'b0 ||
            collision !== 1'b1 || collision_latched !== 1'b1 ||
            event_count != 1)
            $fatal(1, "left collision detection/latching is incorrect");

        // Clear is intentionally ignored while a bumper remains pressed.
        clear_latched = 1'b1;
        clock_cycles(2);
        clear_latched = 1'b0;
        if (collision_latched !== 1'b1)
            $fatal(1, "active collision was incorrectly cleared");

        bumper_left_n = 1'b1;
        clock_cycles(9);
        if (collision !== 1'b0 || collision_latched !== 1'b1)
            $fatal(1, "release or latch retention is incorrect");

        clear_latched = 1'b1;
        clock_cycles(2);
        clear_latched = 1'b0;
        clock_cycles(1);
        if (collision_latched !== 1'b0)
            $fatal(1, "released collision latch did not clear");

        // Confirm the independent right channel can create a new event.
        bumper_right_n = 1'b0;
        clock_cycles(9);
        if (right_pressed !== 1'b1 || collision_latched !== 1'b1 ||
            event_count != 2)
            $fatal(1, "right bumper channel is incorrect");

        $display("PASS: mechanical bumper debounce, event, and latch");
        $finish;
    end

endmodule
