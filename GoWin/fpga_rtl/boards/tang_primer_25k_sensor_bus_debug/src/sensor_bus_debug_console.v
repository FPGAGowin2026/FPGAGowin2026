`timescale 1ns/1ps

// 传感器总线无设备诊断控制台。
// P：链路应答；B：汇总结果；D：逐类位图。位图最低位对应 README 中的 0 号核心。
module sensor_bus_debug_console #(
    parameter integer CLK_FREQ_HZ = 50000000,
    parameter integer BAUD_RATE   = 115200
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       uart_rx_i,
    output wire       uart_tx_o,
    input  wire [3:0] i2c_error_bitmap_i,
    input  wire [7:0] spi_transaction_bitmap_i,
    input  wire [7:0] spi_reject_bitmap_i
);
    localparam [2:0] MSG_BANNER = 3'd0;
    localparam [2:0] MSG_PONG   = 3'd1;
    localparam [2:0] MSG_BIST   = 3'd2;
    localparam [2:0] MSG_DETAIL = 3'd3;
    localparam [2:0] MSG_HELP   = 3'd4;

    localparam integer BANNER_LEN = 31;
    localparam integer PONG_LEN   = 22;
    localparam integer BIST_LEN   = 46;
    localparam integer DETAIL_LEN = 42;
    localparam integer HELP_LEN   = 12;

    localparam [BANNER_LEN*8-1:0] BANNER_TEXT =
        {"GW5A25 SENSOR BUS DEBUG READY", 8'h0d, 8'h0a};
    localparam [PONG_LEN*8-1:0] PONG_TEXT =
        {"PONG;BUS_DEBUG=READY", 8'h0d, 8'h0a};
    localparam [BIST_LEN*8-1:0] BIST_PASS_TEXT =
        {"BUS_BIST=PASS;I2C=4/4;SPI=8/8;MODE=NO_DEVICE", 8'h0d, 8'h0a};
    localparam [BIST_LEN*8-1:0] BIST_FAIL_TEXT =
        {"BUS_BIST=FAIL;I2C=0/4;SPI=0/8;MODE=NO_DEVICE", 8'h0d, 8'h0a};
    localparam [DETAIL_LEN*8-1:0] DETAIL_TEXT =
        {"I2C_ERR=0x0;SPI_TXN=0x00;SPI_REJECT=0x00", 8'h0d, 8'h0a};
    localparam [HELP_LEN*8-1:0] HELP_TEXT =
        {"CMDS:P,B,D", 8'h0d, 8'h0a};

    wire [7:0] rx_data;
    wire rx_data_valid;
    wire rx_frame_error;
    wire tx_data_ready;
    wire tx_busy;
    wire [7:0] tx_data;
    wire tx_data_valid;

    wire [2:0] i2c_error_count =
        {2'd0, i2c_error_bitmap_i[0]} + {2'd0, i2c_error_bitmap_i[1]} +
        {2'd0, i2c_error_bitmap_i[2]} + {2'd0, i2c_error_bitmap_i[3]};
    wire [3:0] spi_reject_count =
        {3'd0, spi_reject_bitmap_i[0]} + {3'd0, spi_reject_bitmap_i[1]} +
        {3'd0, spi_reject_bitmap_i[2]} + {3'd0, spi_reject_bitmap_i[3]} +
        {3'd0, spi_reject_bitmap_i[4]} + {3'd0, spi_reject_bitmap_i[5]} +
        {3'd0, spi_reject_bitmap_i[6]} + {3'd0, spi_reject_bitmap_i[7]};
    wire bist_pass = (&i2c_error_bitmap_i) &&
                     (&spi_transaction_bitmap_i) &&
                     (&spi_reject_bitmap_i);

    reg [2:0] pending_kind;
    reg pending_valid;
    reg [2:0] active_kind;
    reg [7:0] active_index;
    reg message_active;
    reg [31:0] frame_error_count;

    function [7:0] hex_char;
        input [3:0] value;
        begin
            hex_char = (value < 10) ? (8'h30 + value) : (8'h41 + value - 10);
        end
    endfunction

    function [7:0] message_length;
        input [2:0] kind;
        begin
            case (kind)
                MSG_BANNER: message_length = BANNER_LEN;
                MSG_PONG:   message_length = PONG_LEN;
                MSG_BIST:   message_length = BIST_LEN;
                MSG_DETAIL: message_length = DETAIL_LEN;
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
                MSG_BIST: begin
                    if (bist_pass)
                        message_byte = BIST_PASS_TEXT[(BIST_LEN-1-index)*8 +: 8];
                    else
                        message_byte = BIST_FAIL_TEXT[(BIST_LEN-1-index)*8 +: 8];
                    if (index == 8'd18)
                        message_byte = 8'h30 + i2c_error_count;
                    else if (index == 8'd26)
                        message_byte = 8'h30 + spi_reject_count;
                end
                MSG_DETAIL: begin
                    message_byte = DETAIL_TEXT[(DETAIL_LEN-1-index)*8 +: 8];
                    if (index == 8'd10)
                        message_byte = hex_char(i2c_error_bitmap_i);
                    else if (index == 8'd22)
                        message_byte = hex_char(spi_transaction_bitmap_i[7:4]);
                    else if (index == 8'd23)
                        message_byte = hex_char(spi_transaction_bitmap_i[3:0]);
                    else if (index == 8'd38)
                        message_byte = hex_char(spi_reject_bitmap_i[7:4]);
                    else if (index == 8'd39)
                        message_byte = hex_char(spi_reject_bitmap_i[3:0]);
                end
                default:
                    message_byte = HELP_TEXT[(HELP_LEN-1-index)*8 +: 8];
            endcase
        end
    endfunction

    assign tx_data = message_byte(active_kind, active_index);
    assign tx_data_valid = message_active;

    bus_debug_uart_rx #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(BAUD_RATE)
    ) u_uart_rx (
        .clk(clk), .rst_n(rst_n), .rx_i(uart_rx_i),
        .data_o(rx_data), .data_valid_o(rx_data_valid),
        .frame_error_o(rx_frame_error)
    );

    bus_debug_uart_tx #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(BAUD_RATE)
    ) u_uart_tx (
        .clk(clk), .rst_n(rst_n), .data_i(tx_data),
        .data_valid_i(tx_data_valid), .data_ready_o(tx_data_ready),
        .tx_o(uart_tx_o), .busy_o(tx_busy)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pending_kind <= MSG_BANNER;
            pending_valid <= 1'b1;
            active_kind <= MSG_BANNER;
            active_index <= 8'd0;
            message_active <= 1'b0;
            frame_error_count <= 32'd0;
        end else begin
            if (!message_active && pending_valid) begin
                active_kind <= pending_kind;
                active_index <= 8'd0;
                message_active <= 1'b1;
                pending_valid <= 1'b0;
            end else if (message_active && tx_data_ready) begin
                if (active_index >= message_length(active_kind) - 1) begin
                    active_index <= 8'd0;
                    message_active <= 1'b0;
                end else begin
                    active_index <= active_index + 1'b1;
                end
            end

            if (rx_frame_error)
                frame_error_count <= frame_error_count + 1'b1;

            if (rx_data_valid) begin
                pending_valid <= 1'b1;
                case (rx_data)
                    8'h50, 8'h70: pending_kind <= MSG_PONG;   // P/p
                    8'h42, 8'h62: pending_kind <= MSG_BIST;   // B/b
                    8'h44, 8'h64: pending_kind <= MSG_DETAIL; // D/d
                    default: pending_kind <= MSG_HELP;
                endcase
            end
        end
    end
endmodule
