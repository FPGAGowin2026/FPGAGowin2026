`timescale 1ns/1ps

// Tang Primer 25K 片上传感器功能 BIST 顶层。
// 仅使用板载 50 MHz、复位和 USB-UART；所有传感器激励均留在 FPGA 内部。
module tang_primer_25k_sensor_functional_bist_top #(
    parameter integer CLK_FREQ_HZ = 50000000,
    parameter integer BAUD_RATE = 115200,
    parameter integer SENSOR_UART_BAUD_RATE = 3125000,
    parameter integer POR_CYCLES = 65536,
    parameter integer BIST_TIMEOUT_CYCLES = 50000
) (
    input  wire clk_i,
    input  wire reset_i,
    input  wire uart_rx_i,
    output wire uart_tx_o
);
    localparam integer SAFE_POR_CYCLES = (POR_CYCLES < 2) ? 2 : POR_CYCLES;

    reg [31:0] por_count;
    reg por_done;
    reg reset_meta;
    reg reset_sync;
    wire rst_n = por_done && !reset_sync;

    initial begin
        por_count = 32'd0;
        por_done = 1'b0;
        reset_meta = 1'b0;
        reset_sync = 1'b0;
    end

    always @(posedge clk_i) begin
        reset_meta <= reset_i;
        reset_sync <= reset_meta;
        if (reset_sync) begin
            por_count <= 32'd0;
            por_done <= 1'b0;
        end else if (!por_done) begin
            if (por_count >= SAFE_POR_CYCLES - 1) begin
                por_count <= 32'd0;
                por_done <= 1'b1;
            end else begin
                por_count <= por_count + 1'b1;
            end
        end
    end

    wire [16:0] pass_mask;
    wire [16:0] observed_mask;
    wire [16:0] activity_mask;
    wire bist_done;

    sensor_functional_bist_core #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .SENSOR_UART_BAUD_RATE(SENSOR_UART_BAUD_RATE),
        .BIST_TIMEOUT_CYCLES(BIST_TIMEOUT_CYCLES)
    ) u_core (
        .clk(clk_i), .rst_n(rst_n), .pass_mask_o(pass_mask),
        .observed_mask_o(observed_mask),
        .activity_mask_o(activity_mask), .bist_done_o(bist_done)
    );

    sensor_functional_bist_console #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(BAUD_RATE)
    ) u_console (
        .clk(clk_i), .rst_n(rst_n),
        .uart_rx_i(uart_rx_i), .uart_tx_o(uart_tx_o),
        .pass_mask_i(pass_mask), .observed_mask_i(observed_mask),
        .activity_mask_i(activity_mask), .bist_done_i(bist_done)
    );
endmodule
