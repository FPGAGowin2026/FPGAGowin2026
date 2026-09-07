`timescale 1ns/1ps

// 最小板级诊断控制台：上电横幅、周期心跳和单字符命令响应。
// 实时机器人数据通路不依赖本控制台；它只用于板卡连通性和调试。
module bringup_console #(
    parameter integer CLK_FREQ_HZ    = 50000000,
    parameter integer BAUD_RATE      = 115200,
    parameter integer HEARTBEAT_CYCLES = 50000000
) (
    input  wire clk,
    input  wire rst_n,
    input  wire uart_rx_i,
    output wire uart_tx_o
);

    localparam [2:0] MSG_BANNER = 3'd0;
    localparam [2:0] MSG_PONG   = 3'd1;
    localparam [2:0] MSG_ID     = 3'd2;
    localparam [2:0] MSG_STATUS = 3'd3;
    localparam [2:0] MSG_HELP   = 3'd4;
    localparam [2:0] MSG_HB     = 3'd5;

    localparam integer BANNER_LEN = 22;
    localparam integer PONG_LEN   = 6;
    localparam integer ID_LEN     = 19;
    localparam integer STATUS_LEN = 31;
    localparam integer HELP_LEN   = 14;
    localparam integer HB_LEN     = 4;
    localparam integer SAFE_HEARTBEAT_CYCLES =
        (HEARTBEAT_CYCLES < 2) ? 2 : HEARTBEAT_CYCLES;

    // Gowin 的 Verilog 前端不会把字符串中的 \r 解释为 0x0D，
    // 因此显式拼接 CR/LF，避免硬件端发送字面字符 "r"。
    localparam [BANNER_LEN*8-1:0] BANNER_TEXT =
        {"GW5A25 BRINGUP READY", 8'h0d, 8'h0a};
    localparam [PONG_LEN*8-1:0]   PONG_TEXT =
        {"PONG", 8'h0d, 8'h0a};
    localparam [ID_LEN*8-1:0]     ID_TEXT =
        {"ID=GW5A-LV25MG121", 8'h0d, 8'h0a};
    localparam [STATUS_LEN*8-1:0] STATUS_TEXT =
        {"STATUS=OK;CLK=50M;BAUD=115200", 8'h0d, 8'h0a};
    localparam [HELP_LEN*8-1:0]   HELP_TEXT =
        {"CMDS:P,I,S,?", 8'h0d, 8'h0a};
    localparam [HB_LEN*8-1:0]     HB_TEXT =
        {"HB", 8'h0d, 8'h0a};

    wire [7:0] rx_data;
    wire       rx_data_valid;
    wire       rx_frame_error;
    wire       tx_data_ready;
    wire       tx_busy;
    wire [7:0] tx_data;
    wire       tx_data_valid;

    reg [31:0] heartbeat_count;
    reg [31:0] received_count;
    reg [31:0] frame_error_count;
    reg [2:0]  pending_kind;
    reg        pending_valid;
    reg [2:0]  active_kind;
    reg [7:0]  active_index;
    reg        message_active;

    function [7:0] message_length;
        input [2:0] kind;
        begin
            case (kind)
                MSG_BANNER: message_length = BANNER_LEN;
                MSG_PONG:   message_length = PONG_LEN;
                MSG_ID:     message_length = ID_LEN;
                MSG_STATUS: message_length = STATUS_LEN;
                MSG_HELP:   message_length = HELP_LEN;
                MSG_HB:     message_length = HB_LEN;
                default:    message_length = HELP_LEN;
            endcase
        end
    endfunction

    function [7:0] message_byte;
        input [2:0] kind;
        input [7:0] index;
        begin
            case (kind)
                MSG_BANNER:
                    message_byte = BANNER_TEXT[(BANNER_LEN-1-index)*8 +: 8];
                MSG_PONG:
                    message_byte = PONG_TEXT[(PONG_LEN-1-index)*8 +: 8];
                MSG_ID:
                    message_byte = ID_TEXT[(ID_LEN-1-index)*8 +: 8];
                MSG_STATUS:
                    message_byte = STATUS_TEXT[(STATUS_LEN-1-index)*8 +: 8];
                MSG_HELP:
                    message_byte = HELP_TEXT[(HELP_LEN-1-index)*8 +: 8];
                MSG_HB:
                    message_byte = HB_TEXT[(HB_LEN-1-index)*8 +: 8];
                default:
                    message_byte = 8'h3f;
            endcase
        end
    endfunction

    assign tx_data       = message_byte(active_kind, active_index);
    assign tx_data_valid = message_active;

    bringup_uart_rx #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUD_RATE(BAUD_RATE)
    ) u_uart_rx (
        .clk(clk),
        .rst_n(rst_n),
        .rx_i(uart_rx_i),
        .data_o(rx_data),
        .data_valid_o(rx_data_valid),
        .frame_error_o(rx_frame_error)
    );

    bringup_uart_tx #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUD_RATE(BAUD_RATE)
    ) u_uart_tx (
        .clk(clk),
        .rst_n(rst_n),
        .data_i(tx_data),
        .data_valid_i(tx_data_valid),
        .data_ready_o(tx_data_ready),
        .tx_o(uart_tx_o),
        .busy_o(tx_busy)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            heartbeat_count  <= 32'd0;
            received_count   <= 32'd0;
            frame_error_count <= 32'd0;
            pending_kind     <= MSG_BANNER;
            pending_valid    <= 1'b1;
            active_kind      <= MSG_BANNER;
            active_index     <= 8'd0;
            message_active   <= 1'b0;
        end else begin
            if (heartbeat_count >= (SAFE_HEARTBEAT_CYCLES - 1)) begin
                heartbeat_count <= 32'd0;
                if (!pending_valid) begin
                    pending_kind  <= MSG_HB;
                    pending_valid <= 1'b1;
                end
            end else begin
                heartbeat_count <= heartbeat_count + 1'b1;
            end

            if (!message_active && pending_valid) begin
                active_kind    <= pending_kind;
                active_index   <= 8'd0;
                message_active <= 1'b1;
                pending_valid  <= 1'b0;
            end else if (message_active && tx_data_ready) begin
                if (active_index >= (message_length(active_kind) - 1)) begin
                    active_index   <= 8'd0;
                    message_active <= 1'b0;
                end else begin
                    active_index <= active_index + 1'b1;
                end
            end

            if (rx_frame_error)
                frame_error_count <= frame_error_count + 1'b1;

            // 新命令优先覆盖尚未开始的心跳，但不会打断正在发送的字节。
            if (rx_data_valid) begin
                received_count <= received_count + 1'b1;
                pending_valid  <= 1'b1;
                case (rx_data)
                    8'h50, 8'h70: pending_kind <= MSG_PONG;   // P/p
                    8'h49, 8'h69: pending_kind <= MSG_ID;     // I/i
                    8'h53, 8'h73: pending_kind <= MSG_STATUS; // S/s
                    default:      pending_kind <= MSG_HELP;
                endcase
            end
        end
    end

endmodule
