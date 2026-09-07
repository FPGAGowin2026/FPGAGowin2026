// Tang Primer 25K 上的最小 GoRV32 + RT-Thread 硬件封装。
// CPU 从板载 QSPI Flash 的 0x100000 偏移执行 XIP；仅 .data 搬入 64 KiB RAM。
module tang_primer_25k_rtthread_top (
    input  wire clk_i,
    input  wire reset_i,
    input  wire uart_rx_i,
    output wire uart_tx_o,

    inout  wire flash_qspi_csn,
    inout  wire flash_qspi_miso,
    inout  wire flash_qspi_mosi,
    inout  wire flash_qspi_clk,
    inout  wire flash_qspi_holdn,
    inout  wire flash_qspi_wpn
);

    // 外部按键为高有效。释放后再延迟约 1.31 ms，确保时钟和 Flash 稳定。
    reg [15:0] por_count = 16'd0;
    reg        core_por_n = 1'b0;

    always @(posedge clk_i) begin
        if (reset_i) begin
            por_count  <= 16'd0;
            core_por_n <= 1'b0;
        end else if (!core_por_n) begin
            if (&por_count)
                core_por_n <= 1'b1;
            else
                por_count <= por_count + 16'd1;
        end
    end

    // 本次目标只要求 UART shell；用户 JTAG 调试口保持空闲态，避免抢占配置 JTAG。
    Gowin_GoRV32_Top u_gorv32 (
        .HCLK(clk_i),
        .PCLK(clk_i),
        .POR_N(core_por_n),

        .UART1_CTSN(1'b0),
        .UART1_DCDN(1'b0),
        .UART1_DSRN(1'b0),
        .UART1_RIN(1'b0),
        .UART1_RXD(uart_rx_i),
        .UART1_DTRN(),
        .UART1_OUT1N(),
        .UART1_OUT2N(),
        .UART1_RTSN(),
        .UART1_TXD(uart_tx_o),

        .FLASH_QSPI_CSN(flash_qspi_csn),
        .FLASH_QSPI_MISO(flash_qspi_miso),
        .FLASH_QSPI_MOSI(flash_qspi_mosi),
        .FLASH_QSPI_CLK(flash_qspi_clk),
        .FLASH_QSPI_HOLDN(flash_qspi_holdn),
        .FLASH_QSPI_WPN(flash_qspi_wpn),

        .JTAG_TMS(1'b1),
        .JTAG_TDI(1'b0),
        .JTAG_TDO(),
        .JTAG_TCK(1'b0)
    );

endmodule
