`timescale 1ns/1ps

// Tang Primer 25K 最小板级顶层。只占用时钟、复位键和板载调试器 UART。
module tang_primer_25k_bringup_top #(
    parameter integer CLK_FREQ_HZ      = 50000000,
    parameter integer BAUD_RATE        = 115200,
    parameter integer HEARTBEAT_CYCLES = 50000000,
    parameter integer POR_CYCLES       = 65536
) (
    input  wire clk_i,
    input  wire reset_i,
    input  wire uart_rx_i,
    output wire uart_tx_o
);

    localparam integer SAFE_POR_CYCLES = (POR_CYCLES < 2) ? 2 : POR_CYCLES;

    reg [31:0] por_count;
    reg        por_done;
    reg        reset_meta;
    reg        reset_sync;
    wire       rst_n;

    // FPGA 配置完成后的寄存器初值用于产生不依赖外部按键的上电复位。
    initial begin
        por_count  = 32'd0;
        por_done   = 1'b0;
        reset_meta = 1'b0;
        reset_sync = 1'b0;
    end

    always @(posedge clk_i) begin
        reset_meta <= reset_i;
        reset_sync <= reset_meta;

        if (reset_sync) begin
            por_count <= 32'd0;
            por_done  <= 1'b0;
        end else if (!por_done) begin
            if (por_count >= (SAFE_POR_CYCLES - 1)) begin
                por_count <= 32'd0;
                por_done  <= 1'b1;
            end else begin
                por_count <= por_count + 1'b1;
            end
        end
    end

    assign rst_n = por_done && !reset_sync;

    bringup_console #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUD_RATE(BAUD_RATE),
        .HEARTBEAT_CYCLES(HEARTBEAT_CYCLES)
    ) u_console (
        .clk(clk_i),
        .rst_n(rst_n),
        .uart_rx_i(uart_rx_i),
        .uart_tx_o(uart_tx_o)
    );

endmodule
