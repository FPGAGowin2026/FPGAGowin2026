`timescale 1ns/1ps

// 机器人服务软核的最小 Wishbone 寄存器组。
//
// 地址使用 CPU 看到的完整字节地址。所有请求都在有限时间内返回单周期 ACK：
// 合法读写正常完成；未定义地址、未对齐访问和对只读寄存器的写入同样 ACK，
// 但返回 0xDEADBEEF 并增加错误计数，避免软核因错误地址永久挂起。
module robot_service_regs #(
    parameter [31:0] SYS_ID_VALUE       = 32'h5242_4f54, // ASCII "RBOT"
    parameter [31:0] VERSION_VALUE      = 32'h0001_0000, // 1.0.0
    parameter [31:0] CAPABILITIES_VALUE = 32'h0000_0003  // bit0 heartbeat, bit1 selftest
) (
    input  wire        wb_clk_i,
    input  wire        wb_rst_i,
    input  wire        wb_cyc_i,
    input  wire        wb_stb_i,
    input  wire        wb_we_i,
    input  wire [31:0] wb_adr_i,
    input  wire [31:0] wb_dat_i,
    input  wire [3:0]  wb_sel_i,
    output reg  [31:0] wb_dat_o,
    output reg         wb_ack_o,

    output reg  [31:0] heartbeat_o,
    output reg         heartbeat_write_o,
    output reg  [31:0] scratch_o,
    output reg  [31:0] transaction_count_o,
    output reg  [31:0] error_count_o
);

    localparam [31:0] ADDR_SYS_ID       = 32'h2000_0000;
    localparam [31:0] ADDR_VERSION      = 32'h2000_0004;
    localparam [31:0] ADDR_CAPABILITIES = 32'h2000_0008;
    localparam [31:0] ADDR_HEARTBEAT    = 32'h2000_000c;

    localparam [31:0] ADDR_SCRATCH      = 32'h2000_f000;
    localparam [31:0] ADDR_TXN_COUNT    = 32'h2000_f004;
    localparam [31:0] ADDR_ERROR_COUNT  = 32'h2000_f008;

    localparam [31:0] INVALID_READ_DATA = 32'hdead_beef;

    reg request_seen_q;
    wire wb_request = wb_cyc_i && wb_stb_i;

    // 按 Wishbone SEL 合并被选中的字节；未选中的字节保持原值。
    function [31:0] merge_selected_bytes;
        input [31:0] old_value;
        input [31:0] new_value;
        input [3:0]  byte_select;
        begin
            merge_selected_bytes = old_value;
            if (byte_select[0])
                merge_selected_bytes[7:0] = new_value[7:0];
            if (byte_select[1])
                merge_selected_bytes[15:8] = new_value[15:8];
            if (byte_select[2])
                merge_selected_bytes[23:16] = new_value[23:16];
            if (byte_select[3])
                merge_selected_bytes[31:24] = new_value[31:24];
        end
    endfunction

    always @(posedge wb_clk_i) begin
        if (wb_rst_i) begin
            wb_dat_o           <= 32'd0;
            wb_ack_o           <= 1'b0;
            heartbeat_o        <= 32'd0;
            heartbeat_write_o  <= 1'b0;
            scratch_o          <= 32'd0;
            transaction_count_o <= 32'd0;
            error_count_o      <= 32'd0;
            request_seen_q     <= 1'b0;
        end else begin
            wb_ack_o          <= 1'b0;
            heartbeat_write_o <= 1'b0;

            // 一个连续的 CYC+STB 电平只接受一次，防止主设备等待 ACK 时重复计数。
            if (!wb_request) begin
                request_seen_q <= 1'b0;
            end else if (!request_seen_q) begin
                request_seen_q      <= 1'b1;
                wb_ack_o            <= 1'b1;
                transaction_count_o <= transaction_count_o + 1'b1;
                wb_dat_o            <= 32'd0;

                if (wb_adr_i[1:0] != 2'b00) begin
                    wb_dat_o      <= INVALID_READ_DATA;
                    error_count_o <= error_count_o + 1'b1;
                end else if (wb_we_i) begin
                    case (wb_adr_i)
                        ADDR_HEARTBEAT: begin
                            heartbeat_o <= merge_selected_bytes(
                                heartbeat_o, wb_dat_i, wb_sel_i
                            );
                            heartbeat_write_o <= 1'b1;
                        end
                        ADDR_SCRATCH: begin
                            scratch_o <= merge_selected_bytes(
                                scratch_o, wb_dat_i, wb_sel_i
                            );
                        end
                        default: begin
                            wb_dat_o      <= INVALID_READ_DATA;
                            error_count_o <= error_count_o + 1'b1;
                        end
                    endcase
                end else begin
                    case (wb_adr_i)
                        ADDR_SYS_ID:       wb_dat_o <= SYS_ID_VALUE;
                        ADDR_VERSION:      wb_dat_o <= VERSION_VALUE;
                        ADDR_CAPABILITIES: wb_dat_o <= CAPABILITIES_VALUE;
                        ADDR_HEARTBEAT:    wb_dat_o <= heartbeat_o;
                        ADDR_SCRATCH:      wb_dat_o <= scratch_o;
                        // 计数器读返回本次访问开始前的值，语义稳定且便于软件取样。
                        ADDR_TXN_COUNT:    wb_dat_o <= transaction_count_o;
                        ADDR_ERROR_COUNT:  wb_dat_o <= error_count_o;
                        default: begin
                            wb_dat_o      <= INVALID_READ_DATA;
                            error_count_o <= error_count_o + 1'b1;
                        end
                    endcase
                end
            end
        end
    end

endmodule
