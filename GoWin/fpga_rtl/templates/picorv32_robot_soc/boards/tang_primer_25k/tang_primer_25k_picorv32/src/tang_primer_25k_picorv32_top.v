`timescale 1ns/1ps

// Tang Primer 25K 上的 PicoRV32 服务核首板验证顶层。
// 软核只连接调试 UART 与管理寄存器，不连接任何电机或安全执行输出。
module tang_primer_25k_picorv32_top #(
    parameter integer POR_CYCLES = 65536
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

    wire        wb_stb;
    wire        wb_we;
    wire        wb_cyc;
    wire        wb_ack;
    wire [31:0] wb_addr;
    wire [31:0] wb_wdata;
    wire [31:0] wb_rdata;
    wire [3:0]  wb_sel;

    wire [31:0] heartbeat;
    wire        heartbeat_write;
    wire [31:0] scratch;
    wire [31:0] transaction_count;
    wire [31:0] error_count;

    initial begin
        por_count  = 32'd0;
        por_done   = 1'b0;
        reset_meta = 1'b0;
        reset_sync = 1'b0;
    end

    // S1 高有效；双触发同步后与上电延时共同形成软核低有效复位。
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

    Gowin_PicoRV32_Top u_picorv32 (
        .ser_tx          (uart_tx_o),
        .ser_rx          (uart_rx_i),
        .slv_ext_stb_o   (wb_stb),
        .slv_ext_we_o    (wb_we),
        .slv_ext_cyc_o   (wb_cyc),
        .slv_ext_ack_i   (wb_ack),
        .slv_ext_adr_o   (wb_addr),
        .slv_ext_wdata_o (wb_wdata),
        .slv_ext_rdata_i (wb_rdata),
        .slv_ext_sel_o   (wb_sel),
        .irq_in          (12'b0),
        .clk_in          (clk_i),
        .resetn_in       (rst_n)
    );

    robot_service_regs #(
        .SYS_ID_VALUE       (32'h5242_4f54), // "RBOT"
        .VERSION_VALUE      (32'h0001_0000),
        .CAPABILITIES_VALUE (32'h0000_0003)
    ) u_service_regs (
        .wb_clk_i            (clk_i),
        .wb_rst_i            (!rst_n),
        .wb_cyc_i            (wb_cyc),
        .wb_stb_i            (wb_stb),
        .wb_we_i             (wb_we),
        .wb_adr_i            (wb_addr),
        .wb_dat_i            (wb_wdata),
        .wb_sel_i            (wb_sel),
        .wb_dat_o            (wb_rdata),
        .wb_ack_o            (wb_ack),
        .heartbeat_o         (heartbeat),
        .heartbeat_write_o   (heartbeat_write),
        .scratch_o           (scratch),
        .transaction_count_o (transaction_count),
        .error_count_o       (error_count)
    );

endmodule
