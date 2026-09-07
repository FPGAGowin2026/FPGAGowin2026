`timescale 1ns/1ps

module tb_softcore_service_watchdog;

    localparam integer TIMEOUT_CYCLES = 4;

    reg  clk;
    reg  safety_reset_n;
    reg  heartbeat_write;
    wire alive;
    wire fault;
    wire allow_commit;
    wire allow_start;
    wire path_abort;

    integer checks;
    integer failures;
    integer abort_pulses;

    softcore_service_watchdog #(
        .TIMEOUT_CYCLES(TIMEOUT_CYCLES)
    ) dut (
        .clk_i(clk),
        .safety_reset_n_i(safety_reset_n),
        .heartbeat_write_i(heartbeat_write),
        .alive_o(alive),
        .fault_o(fault),
        .allow_commit_o(allow_commit),
        .allow_start_o(allow_start),
        .path_abort_o(path_abort)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    always @(posedge clk) begin
        #1;
        if (path_abort)
            abort_pulses = abort_pulses + 1;
    end

    task check_bit;
        input actual;
        input expected;
        input [8*80-1:0] description;
        begin
            checks = checks + 1;
            if (actual !== expected) begin
                failures = failures + 1;
                $display("FAIL: %0s actual=%b expected=%b", description,
                         actual, expected);
            end
        end
    endtask

    task check_count;
        input integer actual;
        input integer expected;
        input [8*80-1:0] description;
        begin
            checks = checks + 1;
            if (actual != expected) begin
                failures = failures + 1;
                $display("FAIL: %0s actual=%0d expected=%0d", description,
                         actual, expected);
            end
        end
    endtask

    task pulse_heartbeat;
        begin
            @(negedge clk);
            heartbeat_write = 1'b1;
            @(posedge clk);
            #1;
            @(negedge clk);
            heartbeat_write = 1'b0;
        end
    endtask

    task wait_missing_cycles;
        input integer count;
        integer i;
        begin
            for (i = 0; i < count; i = i + 1) begin
                @(posedge clk);
                #1;
            end
        end
    endtask

    initial begin
        checks = 0;
        failures = 0;
        abort_pulses = 0;
        safety_reset_n = 1'b0;
        heartbeat_write = 1'b0;

        repeat (3) @(posedge clk);
        #1;
        check_bit(alive, 1'b0, "reset keeps service offline");
        check_bit(fault, 1'b0, "reset clears fault");
        check_bit(allow_commit, 1'b0, "reset blocks commit");
        check_bit(allow_start, 1'b0, "reset blocks start");
        check_bit(path_abort, 1'b0, "reset does not pulse path abort");

        @(negedge clk);
        safety_reset_n = 1'b1;

        // 固件尚未启动时不授权动作，也不在启动等待期误锁存超时故障。
        wait_missing_cycles(TIMEOUT_CYCLES + 2);
        check_bit(alive, 1'b0, "no heartbeat remains offline");
        check_bit(fault, 1'b0, "no first heartbeat does not fault");
        check_bit(allow_commit, 1'b0, "offline blocks commit");
        check_bit(allow_start, 1'b0, "offline blocks start");
        check_count(abort_pulses, 0, "offline wait does not abort path");

        pulse_heartbeat();
        check_bit(alive, 1'b1, "first heartbeat marks service alive");
        check_bit(fault, 1'b0, "first heartbeat keeps fault clear");
        check_bit(allow_commit, 1'b1, "alive permits commit");
        check_bit(allow_start, 1'b1, "alive permits start");

        // 在最后一个允许周期前刷新，证明心跳会重新开始超时计数。
        wait_missing_cycles(TIMEOUT_CYCLES - 1);
        check_bit(alive, 1'b1, "service alive before timeout boundary");
        pulse_heartbeat();
        check_bit(alive, 1'b1, "heartbeat refreshes watchdog");
        check_bit(fault, 1'b0, "refresh prevents timeout");

        wait_missing_cycles(TIMEOUT_CYCLES - 1);
        check_bit(alive, 1'b1, "alive through timeout minus one cycles");
        check_bit(path_abort, 1'b0, "no early path abort");
        wait_missing_cycles(1);
        check_bit(alive, 1'b0, "timeout clears alive");
        check_bit(fault, 1'b1, "timeout latches fault");
        check_bit(allow_commit, 1'b0, "fault blocks commit");
        check_bit(allow_start, 1'b0, "fault blocks start");
        check_bit(path_abort, 1'b1, "timeout pulses path abort");
        check_count(abort_pulses, 1, "timeout creates one abort pulse");

        wait_missing_cycles(1);
        check_bit(path_abort, 1'b0, "path abort clears after one cycle");
        check_count(abort_pulses, 1, "fault does not repeat abort pulse");

        // 故障后的 CPU 心跳不能越过外部安全复位重新授权服务动作。
        pulse_heartbeat();
        check_bit(alive, 1'b0, "heartbeat cannot revive latched fault");
        check_bit(fault, 1'b1, "heartbeat cannot clear latched fault");
        check_bit(allow_commit, 1'b0, "latched fault still blocks commit");
        check_bit(allow_start, 1'b0, "latched fault still blocks start");
        check_count(abort_pulses, 1, "heartbeat after fault adds no abort pulse");

        // 外部安全复位是唯一清除 fault 的路径。
        @(negedge clk);
        safety_reset_n = 1'b0;
        #1;
        check_bit(alive, 1'b0, "safety reset keeps service offline");
        check_bit(fault, 1'b0, "safety reset clears fault");
        check_bit(allow_commit, 1'b0, "safety reset blocks commit");
        check_bit(allow_start, 1'b0, "safety reset blocks start");
        check_bit(path_abort, 1'b0, "safety reset clears abort output");

        @(negedge clk);
        safety_reset_n = 1'b1;
        pulse_heartbeat();
        check_bit(alive, 1'b1, "heartbeat re-enables after safety reset");
        check_bit(fault, 1'b0, "fault remains clear after safe restart");
        check_bit(allow_commit, 1'b1, "commit allowed after safe restart");
        check_bit(allow_start, 1'b1, "start allowed after safe restart");

        if (failures == 0) begin
            $display("PASS: softcore_service_watchdog (%0d checks)", checks);
            $finish;
        end else begin
            $display("FAIL: softcore_service_watchdog (%0d failures, %0d checks)",
                     failures, checks);
            $fatal(1);
        end
    end

endmodule
