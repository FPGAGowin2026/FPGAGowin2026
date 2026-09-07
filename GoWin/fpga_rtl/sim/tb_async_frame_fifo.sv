`timescale 1ns/1ps

module tb_async_frame_fifo;
    reg wr_clk = 1'b0;
    reg rd_clk = 1'b0;
    reg wr_rst_n = 1'b0;
    reg rd_rst_n = 1'b0;
    reg wr_valid = 1'b0;
    reg [15:0] wr_data = 16'd0;
    reg rd_ready = 1'b0;
    wire wr_full;
    wire wr_overflow;
    wire [15:0] rd_data;
    wire rd_valid;
    wire rd_empty;

    integer checks = 0;
    integer errors = 0;
    integer read_count = 0;
    reg [15:0] expected [0:7];
    reg [2:0] previous_wr_gray = 3'd0;
    reg [2:0] previous_rd_gray = 3'd0;

    always #3.5 wr_clk = ~wr_clk;
    initial begin
        #1.25;
        forever #5.5 rd_clk = ~rd_clk;
    end

    async_frame_fifo #(.DATA_WIDTH(16), .ADDR_WIDTH(2)) dut (
        .wr_clk_i(wr_clk), .wr_rst_n_i(wr_rst_n),
        .wr_valid_i(wr_valid), .wr_data_i(wr_data),
        .wr_full_o(wr_full), .wr_overflow_o(wr_overflow),
        .rd_clk_i(rd_clk), .rd_rst_n_i(rd_rst_n), .rd_ready_i(rd_ready),
        .rd_data_o(rd_data), .rd_valid_o(rd_valid), .rd_empty_o(rd_empty)
    );

    task automatic check_true;
        input condition;
        input [8*96-1:0] message;
        begin
            checks = checks + 1;
            if (!condition) begin
                errors = errors + 1;
                $display("[FAIL] %0s", message);
            end
        end
    endtask

    task automatic push;
        input [15:0] value;
        begin
            @(negedge wr_clk);
            wr_data <= value;
            wr_valid <= 1'b1;
            @(negedge wr_clk);
            wr_valid <= 1'b0;
        end
    endtask

    function automatic gray_one_bit;
        input [2:0] difference;
        begin
            gray_one_bit = difference == 3'b001 || difference == 3'b010 ||
                           difference == 3'b100;
        end
    endfunction

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n)
            previous_wr_gray <= 3'd0;
        else begin
            #1;
            if (dut.wr_gray_q != previous_wr_gray) begin
                if (!gray_one_bit(dut.wr_gray_q ^ previous_wr_gray))
                    $display("WR Gray jump: previous=%b current=%b", previous_wr_gray,
                             dut.wr_gray_q);
                check_true(gray_one_bit(dut.wr_gray_q ^ previous_wr_gray),
                           "write pointer changes one Gray bit per commit");
            end
            previous_wr_gray <= dut.wr_gray_q;
        end
    end

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            previous_rd_gray <= 3'd0;
        end else begin
            #1;
            if (dut.rd_gray_q != previous_rd_gray) begin
                if (!gray_one_bit(dut.rd_gray_q ^ previous_rd_gray))
                    $display("RD Gray jump: previous=%b current=%b", previous_rd_gray,
                             dut.rd_gray_q);
                check_true(gray_one_bit(dut.rd_gray_q ^ previous_rd_gray),
                           "read pointer changes one Gray bit per commit");
            end
            previous_rd_gray <= dut.rd_gray_q;
            if (rd_valid) begin
                check_true(rd_data == expected[read_count], "异步 FIFO 读出顺序和整记录数据正确");
                read_count = read_count + 1;
            end
        end
    end

    initial begin
        // 同时异步置位，两域以不同相位独立释放。
        #12 wr_rst_n = 1'b1;
        #9 rd_rst_n = 1'b1;
        repeat (3) @(posedge wr_clk);
        repeat (3) @(posedge rd_clk);
        check_true(rd_empty && !wr_full, "不同时刻释放复位后 FIFO 为空");

        expected[0] = 16'h1001;
        expected[1] = 16'h2002;
        expected[2] = 16'h3003;
        expected[3] = 16'h4004;
        push(expected[0]);
        push(expected[1]);
        push(expected[2]);
        push(expected[3]);
        repeat (3) @(posedge wr_clk);
        check_true(wr_full, "深度 4 的 FIFO 在四次原子提交后报满");

        push(16'hdead);
        #1 check_true(wr_overflow, "FIFO 满时拒绝额外整记录并给出溢出脉冲");
        repeat (4) @(posedge rd_clk);
        check_true(read_count == 0, "读侧背压期间未提前消费整记录");

        rd_ready = 1'b1;
        wait (read_count == 4);
        @(negedge rd_clk);
        rd_ready = 1'b0;
        repeat (3) @(posedge rd_clk);
        check_true(rd_empty && read_count == 4, "四条记录无污染地按序排空");

        expected[4] = 16'ha1a1;
        push(expected[4]);
        repeat (3) @(posedge rd_clk);
        #2 wr_rst_n = 1'b0;
        rd_rst_n = 1'b0;
        #5 wr_rst_n = 1'b1;
        #8 rd_rst_n = 1'b1;
        read_count = 5;
        repeat (4) @(posedge wr_clk);
        repeat (4) @(posedge rd_clk);
        check_true(rd_empty && !wr_full, "队列中复位后两域指针回到空状态");

        expected[5] = 16'h5a5a;
        push(expected[5]);
        rd_ready = 1'b1;
        wait (read_count == 6);
        @(negedge rd_clk);
        rd_ready = 1'b0;
        check_true(rd_data == 16'h5a5a, "复位后 FIFO 可重新建立并传输整记录");

        if (errors != 0)
            $fatal(1, "async_frame_fifo 自检失败：%0d/%0d", errors, checks);
        $display("PASS: async_frame_fifo %0d checks", checks);
        $finish;
    end
endmodule
