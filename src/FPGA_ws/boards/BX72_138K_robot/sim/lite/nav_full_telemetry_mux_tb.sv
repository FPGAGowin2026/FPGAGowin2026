`timescale 1ns/1ps
// 检查双源整包锁定、逐字节消费、停顿保持、公平轮询与复位清空。
module nav_full_telemetry_mux_tb;
    reg clk = 1'b0;
    reg reset = 1'b1;
    reg source_enable = 1'b0;
    reg tx_start = 1'b0;
    reg tx_read = 1'b0;

    wire source0_start;
    wire source0_read;
    wire source1_start;
    wire source1_read;
    wire tx_valid;
    wire tx_length_nonzero;
    wire [7:0] tx_length;
    wire [7:0] tx_data;

    integer source0_packet = 0;
    integer source0_index = 0;
    integer source1_packet = 0;
    integer source1_index = 0;
    reg source0_started = 1'b0;
    reg source1_started = 1'b0;
    integer source0_starts = 0;
    integer source1_starts = 0;
    integer source0_reads = 0;
    integer source1_reads = 0;
    integer same_cycle_start_read = 0;
    integer separated_start = 0;
    integer last_start_source = -1;
    integer start_count = 0;
    integer i;

    always #4 clk = ~clk;

    function automatic [7:0] packet_length(input integer packet_id);
        begin
            case (packet_id % 7)
                0: packet_length = 8'd1;
                1: packet_length = 8'd48;
                2: packet_length = 8'd84;
                3: packet_length = 8'd104;
                4: packet_length = 8'd120;
                5: packet_length = 8'd128;
                default: packet_length = 8'd160;
            endcase
        end
    endfunction

    function automatic [7:0] packet_byte(input integer source_id,
                                          input integer packet_id,
                                          input integer byte_index);
        reg [31:0] mixed;
        begin
            mixed = (source_id * 32'h53) ^ (packet_id * 32'h35) ^
                    (byte_index * 32'h0d) ^ 32'h6b;
            packet_byte = mixed[7:0];
        end
    endfunction

    wire [7:0] source0_length = packet_length(source0_packet);
    // 两个源采用错开长度序列，避免错误地交叉选择长度仍然碰巧相同。
    wire [7:0] source1_length = packet_length(source1_packet + 1);
    wire [7:0] source0_data = packet_byte(0, source0_packet, source0_index);
    wire [7:0] source1_data = packet_byte(1, source1_packet, source1_index);

    bx72_telemetry_mux dut (
        .clk(clk), .reset_i(reset),
        .source0_valid_i(source_enable),
        .source0_length_nonzero_i(source_enable && source0_length != 0),
        .source0_length_i(source0_length), .source0_data_i(source0_data),
        .source0_start_o(source0_start), .source0_read_o(source0_read),
        .source1_valid_i(source_enable),
        .source1_length_nonzero_i(source_enable && source1_length != 0),
        .source1_length_i(source1_length), .source1_data_i(source1_data),
        .source1_start_o(source1_start), .source1_read_o(source1_read),
        .tx_valid_o(tx_valid), .tx_length_nonzero_o(tx_length_nonzero),
        .tx_length_o(tx_length), .tx_data_o(tx_data),
        .tx_start_i(tx_start), .tx_read_i(tx_read)
    );

    always @(posedge clk) begin
        if (reset) begin
            source0_packet = 0;
            source0_index = 0;
            source1_packet = 0;
            source1_index = 0;
            source0_started = 1'b0;
            source1_started = 1'b0;
            source0_starts = 0;
            source1_starts = 0;
            source0_reads = 0;
            source1_reads = 0;
            same_cycle_start_read = 0;
            separated_start = 0;
            last_start_source = -1;
            start_count = 0;
        end else begin
            if ((source0_start || source1_start) && !tx_start)
                $fatal(1, "source packet started without downstream start");
            if ((source0_read || source1_read) && !tx_read)
                $fatal(1, "source consumed a byte without downstream read");
            if (source0_start && source1_start)
                $fatal(1, "mux started both sources for one packet");
            if (source0_read && source1_read)
                $fatal(1, "mux consumed both sources on one byte read");
            if (source0_start || source1_start) begin
                integer selected;
                selected = source0_start ? 0 : 1;
                if ((selected == 0 && tx_length !== source0_length) ||
                    (selected == 1 && tx_length !== source1_length))
                    $fatal(1, "mux selected a length from the wrong source");
                if ((selected == 0 && source0_started) ||
                    (selected == 1 && source1_started))
                    $fatal(1, "duplicate source start before its packet finished");
                if (last_start_source == selected)
                    $fatal(1, "round-robin arbitration starved the other ready source");
                last_start_source = selected;
                start_count = start_count + 1;
                if (selected == 0) begin
                    source0_started = 1'b1;
                    source0_starts = source0_starts + 1;
                end else begin
                    source1_started = 1'b1;
                    source1_starts = source1_starts + 1;
                end
                if (tx_start && tx_read) same_cycle_start_read = same_cycle_start_read + 1;
                if (tx_start && !tx_read) separated_start = separated_start + 1;
            end

            if (tx_read && !(source0_read || source1_read))
                $fatal(1, "downstream byte read was not routed to a source");
            if (source0_read) begin
                if (!(source0_started || source0_start))
                    $fatal(1, "source0 byte consumed before start handshake");
                if (tx_data !== packet_byte(0, source0_packet, source0_index))
                    $fatal(1, "source0 mixed, duplicated, or reordered byte packet=%0d index=%0d got=%02x expected=%02x",
                           source0_packet, source0_index, tx_data,
                           packet_byte(0, source0_packet, source0_index));
                source0_reads = source0_reads + 1;
                if (source0_index + 1 >= packet_length(source0_packet)) begin
                    source0_packet = source0_packet + 1;
                    source0_index = 0;
                    source0_started = 1'b0;
                end else source0_index = source0_index + 1;
            end
            if (source1_read) begin
                if (!(source1_started || source1_start))
                    $fatal(1, "source1 byte consumed before start handshake");
                if (tx_data !== packet_byte(1, source1_packet, source1_index))
                    $fatal(1, "source1 mixed, duplicated, or reordered byte packet=%0d index=%0d got=%02x expected=%02x",
                           source1_packet, source1_index, tx_data,
                           packet_byte(1, source1_packet, source1_index));
                source1_reads = source1_reads + 1;
                if (source1_index + 1 >= packet_length(source1_packet + 1)) begin
                    source1_packet = source1_packet + 1;
                    source1_index = 0;
                    source1_started = 1'b0;
                end else source1_index = source1_index + 1;
            end
        end
    end

    task automatic wait_for_candidate;
        begin
            @(negedge clk);
            while (!tx_valid) @(negedge clk);
            if (!tx_length_nonzero || tx_length == 0)
                $fatal(1, "valid source did not provide a nonzero packet length");
        end
    endtask

    task automatic read_one_byte;
        begin
            @(negedge clk);
            tx_read = 1'b1;
            #1;
            if (!(source0_read || source1_read))
                $fatal(1, "locked byte read was not routed to its source");
            @(negedge clk);
            tx_read = 1'b0;
        end
    endtask

    task automatic send_packet(input integer style);
        integer length;
        integer first_byte;
        integer j;
        reg [7:0] held_data;
        reg [7:0] held_length;
        begin
            wait_for_candidate();
            length = tx_length;
            held_data = tx_data;
            held_length = tx_length;

            // 尚未start时下游停顿，候选首字节与包长必须稳定。
            repeat (2) begin
                @(negedge clk);
                if (!tx_valid || tx_data !== held_data || tx_length !== held_length)
                    $fatal(1, "candidate changed while downstream stalled before start");
            end

            @(negedge clk);
            tx_start = 1'b1;
            tx_read = (style == 0);
            #1;
            if (!(source0_start || source1_start))
                $fatal(1, "downstream start was not acknowledged by a source");
            if (style == 0 && !(source0_read || source1_read))
                $fatal(1, "same-cycle start/read did not consume the first byte");
            if (style == 0 && tx_data !== held_data)
                $fatal(1, "first byte changed on simultaneous start/read");
            @(negedge clk);
            tx_start = 1'b0;
            tx_read = 1'b0;

            if (style == 0) first_byte = 1;
            else begin
                first_byte = 0;
                held_data = tx_data;
                repeat (3) begin
                    @(negedge clk);
                    if (tx_valid || tx_data !== held_data || tx_length !== held_length)
                        $fatal(1, "locked packet changed while reads were stalled");
                end
            end

            for (j = first_byte; j < length; j = j + 1) begin
                if (style == 2 && (j % 11) == 4) begin
                    held_data = tx_data;
                    repeat (2) begin
                        @(negedge clk);
                        if (tx_valid || tx_data !== held_data)
                            $fatal(1, "head byte changed during downstream backpressure");
                    end
                end
                read_one_byte();
            end
            tx_start = 1'b0;
            tx_read = 1'b0;
        end
    endtask

    task automatic interrupt_packet_with_reset;
        integer j;
        begin
            wait_for_candidate();
            if (tx_length < 8'd4) $fatal(1, "reset interruption packet unexpectedly short");
            @(negedge clk);
            tx_start = 1'b1;
            tx_read = 1'b0;
            @(negedge clk);
            tx_start = 1'b0;
            for (j = 0; j < 3; j = j + 1) read_one_byte();
            @(negedge clk);
            reset = 1'b1;
            tx_start = 1'b0;
            tx_read = 1'b0;
            repeat (3) @(negedge clk);
            if (source0_start || source0_read || source1_start || source1_read)
                $fatal(1, "mux emitted a ghost source handshake during reset");
            reset = 1'b0;
            source_enable = 1'b1;
            repeat (2) @(negedge clk);
            if (source0_start || source0_read || source1_start || source1_read)
                $fatal(1, "mux consumed a byte without a new downstream start/read");
        end
    endtask

    initial begin
        repeat (4) @(negedge clk);
        reset = 1'b0;
        source_enable = 1'b1;

        // 每源连续提交至少七帧，覆盖全部合法长度；包含同拍、分离和长停顿。
        for (i = 0; i < 16; i = i + 1) begin
            if (i == 0) send_packet(0);
            else if (i == 1) send_packet(1);
            else send_packet(2);
        end
        if (source0_packet != 8 || source1_packet != 8 ||
            source0_starts != 8 || source1_starts != 8 ||
            source0_reads == 0 || source1_reads == 0 ||
            same_cycle_start_read == 0 || separated_start == 0)
            $fatal(1, "length coverage, exact packet completion, or handshake style missing");

        // 让两源进入48字节包后中断已消费的包；reset后源与mux必须从新epoch重新开始。
        send_packet(2);
        send_packet(2);
        interrupt_packet_with_reset();
        send_packet(0);
        send_packet(1);
        if (source0_packet != 1 || source1_packet != 1 ||
            source0_starts != 1 || source1_starts != 1)
            $fatal(1, "post-reset traffic did not begin from a clean packet epoch");

        $display("PASS: nav_full_telemetry_mux lengths=1/48/84/104/120/128/160 same/split start-read stalls fair two-source reset-clean");
        $finish;
    end

    initial begin
        #100_000_000;
        $fatal(1, "telemetry mux test timeout");
    end
endmodule
