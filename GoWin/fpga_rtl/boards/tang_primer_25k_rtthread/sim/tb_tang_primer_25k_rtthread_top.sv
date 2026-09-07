`timescale 1ns/1ps

module tb_tang_primer_25k_rtthread_top;
    reg clk_i = 1'b0;
    reg reset_i = 1'b1;
    reg uart_rx_i = 1'b1;
    wire uart_tx_o;

    tri flash_qspi_csn;
    tri flash_qspi_miso;
    tri flash_qspi_mosi;
    tri flash_qspi_clk;
    tri flash_qspi_holdn;
    tri flash_qspi_wpn;

    integer cycles;

    always #10 clk_i = ~clk_i;

    tang_primer_25k_rtthread_top dut (
        .clk_i(clk_i),
        .reset_i(reset_i),
        .uart_rx_i(uart_rx_i),
        .uart_tx_o(uart_tx_o),
        .flash_qspi_csn(flash_qspi_csn),
        .flash_qspi_miso(flash_qspi_miso),
        .flash_qspi_mosi(flash_qspi_mosi),
        .flash_qspi_clk(flash_qspi_clk),
        .flash_qspi_holdn(flash_qspi_holdn),
        .flash_qspi_wpn(flash_qspi_wpn)
    );

    task automatic expect_por;
        input expected;
        begin
            if (dut.u_gorv32.POR_N !== expected) begin
                $display("FAIL: POR_N expected %0b, got %0b at cycle %0d",
                         expected, dut.u_gorv32.POR_N, cycles);
                $fatal(1);
            end
        end
    endtask

    initial begin
        repeat (4) begin
            @(posedge clk_i);
            cycles = cycles + 1;
        end
        reset_i = 1'b0;

        // 释放按键后，计数器完整经历 65536 个时钟，下一拍才释放 CPU。
        for (cycles = 0; cycles < 65536; cycles = cycles + 1) begin
            @(posedge clk_i);
            expect_por(1'b0);
        end
        @(posedge clk_i);
        #1;
        expect_por(1'b1);

        uart_rx_i = 1'b0;
        #1;
        if (uart_tx_o !== 1'b0) begin
            $display("FAIL: UART RX-to-TX wrapper connectivity is broken");
            $fatal(1);
        end
        uart_rx_i = 1'b1;
        #1;
        if (uart_tx_o !== 1'b1) begin
            $display("FAIL: UART idle level is not high");
            $fatal(1);
        end

        // 任意时刻按下复位都必须在一个时钟内重新拉低 POR_N。
        reset_i = 1'b1;
        @(posedge clk_i);
        #1;
        expect_por(1'b0);

        $display("PASS: Tang Primer 25K RT-Thread wrapper POR/UART connectivity");
        $finish;
    end

endmodule
