`timescale 1ns/1ps

module tb_robot_service_regs;

    localparam [31:0] ADDR_SYS_ID       = 32'h2000_0000;
    localparam [31:0] ADDR_VERSION      = 32'h2000_0004;
    localparam [31:0] ADDR_CAPABILITIES = 32'h2000_0008;
    localparam [31:0] ADDR_HEARTBEAT    = 32'h2000_000c;
    localparam [31:0] ADDR_SCRATCH      = 32'h2000_f000;
    localparam [31:0] ADDR_TXN_COUNT    = 32'h2000_f004;
    localparam [31:0] ADDR_ERROR_COUNT  = 32'h2000_f008;
    localparam [31:0] INVALID_READ_DATA = 32'hdead_beef;

    reg         clk;
    reg         rst;
    reg         wb_cyc;
    reg         wb_stb;
    reg         wb_we;
    reg  [31:0] wb_adr;
    reg  [31:0] wb_wdata;
    reg  [3:0]  wb_sel;
    wire [31:0] wb_rdata;
    wire        wb_ack;
    wire [31:0] heartbeat;
    wire        heartbeat_write;
    wire [31:0] scratch;
    wire [31:0] transaction_count;
    wire [31:0] error_count;

    integer checks;
    integer failures;

    robot_service_regs dut (
        .wb_clk_i(clk),
        .wb_rst_i(rst),
        .wb_cyc_i(wb_cyc),
        .wb_stb_i(wb_stb),
        .wb_we_i(wb_we),
        .wb_adr_i(wb_adr),
        .wb_dat_i(wb_wdata),
        .wb_sel_i(wb_sel),
        .wb_dat_o(wb_rdata),
        .wb_ack_o(wb_ack),
        .heartbeat_o(heartbeat),
        .heartbeat_write_o(heartbeat_write),
        .scratch_o(scratch),
        .transaction_count_o(transaction_count),
        .error_count_o(error_count)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    task check_equal;
        input [31:0] actual;
        input [31:0] expected;
        input [8*80-1:0] description;
        begin
            checks = checks + 1;
            if (actual !== expected) begin
                failures = failures + 1;
                $display("FAIL: %0s actual=%08x expected=%08x", description,
                         actual, expected);
            end
        end
    endtask

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

    task wb_idle;
        begin
            @(negedge clk);
            wb_cyc   = 1'b0;
            wb_stb   = 1'b0;
            wb_we    = 1'b0;
            wb_adr   = 32'd0;
            wb_wdata = 32'd0;
            wb_sel   = 4'b0000;
        end
    endtask

    task wb_read;
        input  [31:0] address;
        output [31:0] value;
        integer timeout;
        begin
            @(negedge clk);
            wb_cyc   = 1'b1;
            wb_stb   = 1'b1;
            wb_we    = 1'b0;
            wb_adr   = address;
            wb_wdata = 32'd0;
            wb_sel   = 4'b1111;

            timeout = 0;
            while (!wb_ack && timeout < 5) begin
                @(posedge clk);
                #1;
                timeout = timeout + 1;
            end
            if (!wb_ack) begin
                failures = failures + 1;
                $display("FAIL: read timeout at %08x", address);
            end
            value = wb_rdata;
            wb_idle();
            @(posedge clk);
            #1;
        end
    endtask

    task wb_write;
        input [31:0] address;
        input [31:0] value;
        input [3:0]  select;
        integer timeout;
        begin
            @(negedge clk);
            wb_cyc   = 1'b1;
            wb_stb   = 1'b1;
            wb_we    = 1'b1;
            wb_adr   = address;
            wb_wdata = value;
            wb_sel   = select;

            timeout = 0;
            while (!wb_ack && timeout < 5) begin
                @(posedge clk);
                #1;
                timeout = timeout + 1;
            end
            if (!wb_ack) begin
                failures = failures + 1;
                $display("FAIL: write timeout at %08x", address);
            end
            if (address == ADDR_HEARTBEAT)
                check_bit(heartbeat_write, 1'b1,
                          "heartbeat write pulse asserted with ACK");
            wb_idle();
            @(posedge clk);
            #1;
        end
    endtask

    task wb_hold_request;
        input [31:0] address;
        integer ack_count;
        integer cycle_count;
        begin
            @(negedge clk);
            wb_cyc   = 1'b1;
            wb_stb   = 1'b1;
            wb_we    = 1'b0;
            wb_adr   = address;
            wb_wdata = 32'd0;
            wb_sel   = 4'b1111;
            ack_count = 0;

            for (cycle_count = 0; cycle_count < 4; cycle_count = cycle_count + 1) begin
                @(posedge clk);
                #1;
                if (wb_ack)
                    ack_count = ack_count + 1;
            end
            check_equal(ack_count, 1, "held request receives one ACK");
            wb_idle();
            @(posedge clk);
            #1;
        end
    endtask

    reg [31:0] value;
    reg [31:0] before_count;

    initial begin
        checks = 0;
        failures = 0;
        rst = 1'b1;
        wb_cyc = 1'b0;
        wb_stb = 1'b0;
        wb_we = 1'b0;
        wb_adr = 32'd0;
        wb_wdata = 32'd0;
        wb_sel = 4'b0000;

        repeat (3) @(posedge clk);
        @(negedge clk);
        rst = 1'b0;
        @(posedge clk);
        #1;

        check_equal(heartbeat, 32'd0, "heartbeat reset value");
        check_equal(scratch, 32'd0, "scratch reset value");
        check_equal(transaction_count, 32'd0, "transaction counter reset value");
        check_equal(error_count, 32'd0, "error counter reset value");

        wb_read(ADDR_SYS_ID, value);
        check_equal(value, 32'h5242_4f54, "system ID");
        wb_read(ADDR_VERSION, value);
        check_equal(value, 32'h0001_0000, "version");
        wb_read(ADDR_CAPABILITIES, value);
        check_equal(value, 32'h0000_0003, "capabilities");

        wb_write(ADDR_HEARTBEAT, 32'h1122_3344, 4'b1111);
        check_equal(heartbeat, 32'h1122_3344, "full heartbeat write");
        wb_write(ADDR_HEARTBEAT, 32'haabb_ccdd, 4'b0101);
        check_equal(heartbeat, 32'h11bb_33dd, "byte-select heartbeat write");
        wb_read(ADDR_HEARTBEAT, value);
        check_equal(value, 32'h11bb_33dd, "heartbeat readback");

        wb_write(ADDR_SCRATCH, 32'hdead_beef, 4'b1111);
        check_equal(scratch, 32'hdead_beef, "full scratch write");
        wb_write(ADDR_SCRATCH, 32'h1234_5678, 4'b1010);
        check_equal(scratch, 32'h12ad_56ef, "byte-select scratch write");
        wb_read(ADDR_SCRATCH, value);
        check_equal(value, 32'h12ad_56ef, "scratch readback");

        before_count = error_count;
        wb_read(32'h2000_1234, value);
        check_equal(value, INVALID_READ_DATA, "undefined read sentinel");
        check_equal(error_count, before_count + 1, "undefined read error count");

        before_count = error_count;
        wb_read(32'h2000_0001, value);
        check_equal(value, INVALID_READ_DATA, "unaligned read sentinel");
        check_equal(error_count, before_count + 1, "unaligned read error count");

        before_count = error_count;
        wb_write(ADDR_SYS_ID, 32'hffff_ffff, 4'b1111);
        check_equal(error_count, before_count + 1, "read-only write error count");
        wb_read(ADDR_SYS_ID, value);
        check_equal(value, 32'h5242_4f54, "read-only register unchanged");

        before_count = transaction_count;
        wb_hold_request(ADDR_VERSION);
        check_equal(transaction_count, before_count + 1,
                    "held request counted once");

        wb_read(ADDR_TXN_COUNT, value);
        check_equal(value, transaction_count - 1,
                    "transaction count read uses pre-access value");
        wb_read(ADDR_ERROR_COUNT, value);
        check_equal(value, error_count, "error count readback");

        // heartbeat_write_o 是单周期事件，不应在空闲周期保持。
        check_bit(heartbeat_write, 1'b0, "heartbeat write pulse clears");

        if (failures == 0) begin
            $display("PASS: robot_service_regs (%0d checks)", checks);
            $finish;
        end else begin
            $display("FAIL: robot_service_regs (%0d failures, %0d checks)",
                     failures, checks);
            $fatal(1);
        end
    end

endmodule
