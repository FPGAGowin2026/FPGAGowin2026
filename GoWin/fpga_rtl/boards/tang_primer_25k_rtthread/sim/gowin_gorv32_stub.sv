// 仅用于封装级仿真的 GoRV32 黑盒替身。
// 它不模拟 CPU，只暴露 POR_N 并把 UART RX 回环到 TX。
module Gowin_GoRV32_Top (
    input  wire HCLK,
    input  wire PCLK,
    input  wire POR_N,

    input  wire UART1_CTSN,
    input  wire UART1_DCDN,
    input  wire UART1_DSRN,
    input  wire UART1_RIN,
    input  wire UART1_RXD,
    output wire UART1_DTRN,
    output wire UART1_OUT1N,
    output wire UART1_OUT2N,
    output wire UART1_RTSN,
    output wire UART1_TXD,

    inout  wire FLASH_QSPI_CSN,
    inout  wire FLASH_QSPI_MISO,
    inout  wire FLASH_QSPI_MOSI,
    inout  wire FLASH_QSPI_CLK,
    inout  wire FLASH_QSPI_HOLDN,
    inout  wire FLASH_QSPI_WPN,

    input  wire JTAG_TMS,
    input  wire JTAG_TDI,
    output wire JTAG_TDO,
    input  wire JTAG_TCK
);

    assign UART1_TXD   = UART1_RXD;
    assign UART1_DTRN  = 1'b1;
    assign UART1_OUT1N = 1'b1;
    assign UART1_OUT2N = 1'b1;
    assign UART1_RTSN  = 1'b1;
    assign JTAG_TDO    = 1'b0;

    assign FLASH_QSPI_CSN   = 1'bz;
    assign FLASH_QSPI_MISO  = 1'bz;
    assign FLASH_QSPI_MOSI  = 1'bz;
    assign FLASH_QSPI_CLK   = 1'bz;
    assign FLASH_QSPI_HOLDN = 1'bz;
    assign FLASH_QSPI_WPN   = 1'bz;

endmodule
