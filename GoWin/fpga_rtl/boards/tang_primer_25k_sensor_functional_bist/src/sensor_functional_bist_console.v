`timescale 1ns/1ps

// P：链路应答；B：17 项汇总；D：PASS/观察/激励位图。
module sensor_functional_bist_console #(
    parameter integer CLK_FREQ_HZ = 50000000,
    parameter integer BAUD_RATE = 115200
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        uart_rx_i,
    output wire        uart_tx_o,
    input  wire [16:0] pass_mask_i,
    input  wire [16:0] observed_mask_i,
    input  wire [16:0] activity_mask_i,
    input  wire        bist_done_i
);
    localparam [2:0] MSG_BANNER = 3'd0;
    localparam [2:0] MSG_PONG   = 3'd1;
    localparam [2:0] MSG_BIST   = 3'd2;
    localparam [2:0] MSG_DETAIL = 3'd3;
    localparam [2:0] MSG_HELP   = 3'd4;

    localparam integer BANNER_LEN = 31;
    localparam integer PONG_LEN   = 22;
    localparam integer BIST_LEN   = 51;
    localparam integer DETAIL_LEN = 45;
    localparam integer HELP_LEN   = 12;

    localparam [BANNER_LEN*8-1:0] BANNER_TEXT =
        {"GW5A25 SENSOR FUNC BIST READY", 8'h0d, 8'h0a};
    localparam [PONG_LEN*8-1:0] PONG_TEXT =
        {"PONG;FUNC_BIST=READY", 8'h0d, 8'h0a};
    localparam [BIST_LEN*8-1:0] BIST_PASS_TEXT =
        {"FUNC_BIST=PASS;MODULES=17/17;MODE=ONCHIP_STIMULUS", 8'h0d, 8'h0a};
    localparam [BIST_LEN*8-1:0] BIST_FAIL_TEXT =
        {"FUNC_BIST=FAIL;MODULES=00/17;MODE=ONCHIP_STIMULUS", 8'h0d, 8'h0a};
    localparam [DETAIL_LEN*8-1:0] DETAIL_TEXT =
        {"PASS=0x00000;OBS=0x00000;ACT=0x00000;DONE=0", 8'h0d, 8'h0a};
    localparam [HELP_LEN*8-1:0] HELP_TEXT =
        {"CMDS:P,B,D", 8'h0d, 8'h0a};

    wire [7:0] rx_data;
    wire rx_data_valid;
    wire rx_frame_error;
    wire tx_data_ready;
    wire tx_busy;
    wire [7:0] tx_data;
    wire tx_data_valid;

    wire [5:0] pass_count =
        pass_mask_i[0] + pass_mask_i[1] + pass_mask_i[2] +
        pass_mask_i[3] + pass_mask_i[4] + pass_mask_i[5] +
        pass_mask_i[6] + pass_mask_i[7] + pass_mask_i[8] +
        pass_mask_i[9] + pass_mask_i[10] + pass_mask_i[11] +
        pass_mask_i[12] + pass_mask_i[13] + pass_mask_i[14] +
        pass_mask_i[15] + pass_mask_i[16];
    wire bist_pass = bist_done_i && (&pass_mask_i) &&
                     (&observed_mask_i) && (&activity_mask_i);

    reg [2:0] pending_kind;
    reg pending_valid;
    reg [2:0] active_kind;
    reg [7:0] active_index;
    reg message_active;
    reg [31:0] frame_error_count;

    function [7:0] hex_char;
        input [3:0] value;
        begin
            hex_char = (value < 10) ? (8'h30 + value) :
                                      (8'h41 + value - 10);
        end
    endfunction

    function [7:0] bitmap_hex;
        input [16:0] bitmap;
        input [2:0] digit;
        begin
            case (digit)
                0: bitmap_hex = hex_char({3'd0, bitmap[16]});
                1: bitmap_hex = hex_char(bitmap[15:12]);
                2: bitmap_hex = hex_char(bitmap[11:8]);
                3: bitmap_hex = hex_char(bitmap[7:4]);
                default: bitmap_hex = hex_char(bitmap[3:0]);
            endcase
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
                    if (index == 8'd23)
                        message_byte = 8'h30 + (pass_count / 10);
                    else if (index == 8'd24)
                        message_byte = 8'h30 + (pass_count % 10);
                end
                MSG_DETAIL: begin
                    message_byte = DETAIL_TEXT[(DETAIL_LEN-1-index)*8 +: 8];
                    if (index >= 7 && index <= 11)
                        message_byte = bitmap_hex(pass_mask_i, index - 7);
                    else if (index >= 19 && index <= 23)
                        message_byte = bitmap_hex(observed_mask_i, index - 19);
                    else if (index >= 31 && index <= 35)
                        message_byte = bitmap_hex(activity_mask_i, index - 31);
                    else if (index == 42)
                        message_byte = bist_done_i ? "1" : "0";
                end
                default:
                    message_byte = HELP_TEXT[(HELP_LEN-1-index)*8 +: 8];
            endcase
        end
    endfunction

    assign tx_data = message_byte(active_kind, active_index);
    assign tx_data_valid = message_active;

    func_bist_uart_rx #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(BAUD_RATE)
    ) u_uart_rx (
        .clk(clk), .rst_n(rst_n), .rx_i(uart_rx_i),
        .data_o(rx_data), .data_valid_o(rx_data_valid),
        .frame_error_o(rx_frame_error)
    );

    func_bist_uart_tx #(
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
                    8'h50, 8'h70: pending_kind <= MSG_PONG;
                    8'h42, 8'h62: pending_kind <= MSG_BIST;
                    8'h44, 8'h64: pending_kind <= MSG_DETAIL;
                    default: pending_kind <= MSG_HELP;
                endcase
            end
        end
    end
endmodule
