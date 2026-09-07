`timescale 1ns/1ps

// TF-Luna UART receiver and 9-byte frame parser.
//
// The TF-Luna default binary frame is:
//   0: 8'h59, 1: 8'h59,
//   2: distance low,  3: distance high,
//   4: strength low,  5: strength high,
//   6: temperature low, 7: temperature high,
//   8: unsigned sum of bytes 0 through 7 (low eight bits).
//
// All outputs are updated atomically when a valid checksum is received.
module tf_luna_uart #(
    parameter integer CLK_FREQ_HZ = 27000000,
    parameter integer BAUD_RATE   = 115200
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        uart_rx,
    output reg  [15:0] distance_cm,
    output reg  [15:0] strength,
    output reg  [15:0] temperature_raw,
    output reg         frame_valid,
    output reg         checksum_error
);

    localparam integer SAFE_BAUD_RATE = (BAUD_RATE < 1) ? 1 : BAUD_RATE;
    localparam integer RAW_CLKS_PER_BIT = CLK_FREQ_HZ / SAFE_BAUD_RATE;
    localparam integer CLKS_PER_BIT = (RAW_CLKS_PER_BIT < 2) ? 2 : RAW_CLKS_PER_BIT;
    localparam integer HALF_BIT_CLKS = CLKS_PER_BIT / 2;
    localparam integer FRAME_TIMEOUT_CLKS = CLKS_PER_BIT * 12;

    localparam [2:0] RX_IDLE  = 3'd0;
    localparam [2:0] RX_START = 3'd1;
    localparam [2:0] RX_DATA  = 3'd2;
    localparam [2:0] RX_STOP  = 3'd3;

    (* ASYNC_REG = "TRUE" *) reg uart_rx_meta;
    (* ASYNC_REG = "TRUE" *) reg uart_rx_sync;
    reg [2:0] rx_state;
    reg [31:0] rx_clock_count;
    reg [2:0] rx_bit_index;
    reg [7:0] rx_shift;
    reg [7:0] rx_byte;
    reg rx_byte_valid;

    reg [3:0] parser_index;
    reg [7:0] checksum_sum;
    reg [7:0] distance_low;
    reg [7:0] distance_high;
    reg [7:0] strength_low;
    reg [7:0] strength_high;
    reg [7:0] temperature_low;
    reg [7:0] temperature_high;
    reg [31:0] parser_timeout_count;

    // Synchronize the asynchronous receive pin into the local clock domain.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_rx_meta <= 1'b1;
            uart_rx_sync <= 1'b1;
        end else begin
            uart_rx_meta <= uart_rx;
            uart_rx_sync <= uart_rx_meta;
        end
    end

    // 8-N-1 UART receiver. The start bit is checked in its center, followed by
    // one sample in the center of each data bit.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state       <= RX_IDLE;
            rx_clock_count <= 32'd0;
            rx_bit_index   <= 3'd0;
            rx_shift       <= 8'd0;
            rx_byte        <= 8'd0;
            rx_byte_valid  <= 1'b0;
        end else begin
            rx_byte_valid <= 1'b0;

            case (rx_state)
                RX_IDLE: begin
                    rx_clock_count <= 32'd0;
                    rx_bit_index   <= 3'd0;
                    if (!uart_rx_sync)
                        rx_state <= RX_START;
                end

                RX_START: begin
                    if (rx_clock_count >= (HALF_BIT_CLKS - 1)) begin
                        rx_clock_count <= 32'd0;
                        if (!uart_rx_sync)
                            rx_state <= RX_DATA;
                        else
                            rx_state <= RX_IDLE;
                    end else begin
                        rx_clock_count <= rx_clock_count + 1'b1;
                    end
                end

                RX_DATA: begin
                    if (rx_clock_count >= (CLKS_PER_BIT - 1)) begin
                        rx_clock_count          <= 32'd0;
                        rx_shift[rx_bit_index] <= uart_rx_sync;
                        if (rx_bit_index == 3'd7) begin
                            rx_bit_index <= 3'd0;
                            rx_state     <= RX_STOP;
                        end else begin
                            rx_bit_index <= rx_bit_index + 1'b1;
                        end
                    end else begin
                        rx_clock_count <= rx_clock_count + 1'b1;
                    end
                end

                RX_STOP: begin
                    if (rx_clock_count >= (CLKS_PER_BIT - 1)) begin
                        rx_clock_count <= 32'd0;
                        if (uart_rx_sync) begin
                            rx_byte       <= rx_shift;
                            rx_byte_valid <= 1'b1;
                        end
                        rx_state <= RX_IDLE;
                    end else begin
                        rx_clock_count <= rx_clock_count + 1'b1;
                    end
                end

                default: rx_state <= RX_IDLE;
            endcase
        end
    end

    // Frame parser. A checksum failure produces a one-cycle error pulse and
    // leaves the last valid measurements unchanged.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            parser_index       <= 4'd0;
            checksum_sum       <= 8'd0;
            distance_low       <= 8'd0;
            distance_high      <= 8'd0;
            strength_low       <= 8'd0;
            strength_high      <= 8'd0;
            temperature_low    <= 8'd0;
            temperature_high   <= 8'd0;
            parser_timeout_count <= 32'd0;
            distance_cm        <= 16'd0;
            strength           <= 16'd0;
            temperature_raw    <= 16'd0;
            frame_valid        <= 1'b0;
            checksum_error     <= 1'b0;
        end else begin
            frame_valid    <= 1'b0;
            checksum_error <= 1'b0;

            if (rx_byte_valid) begin
                parser_timeout_count <= 32'd0;
                case (parser_index)
                    4'd0: begin
                        if (rx_byte == 8'h59) begin
                            checksum_sum <= 8'h59;
                            parser_index <= 4'd1;
                        end
                    end

                    4'd1: begin
                        if (rx_byte == 8'h59) begin
                            checksum_sum <= checksum_sum + rx_byte;
                            parser_index <= 4'd2;
                        end else begin
                            parser_index <= 4'd0;
                        end
                    end

                    4'd2: begin
                        distance_low <= rx_byte;
                        checksum_sum <= checksum_sum + rx_byte;
                        parser_index <= 4'd3;
                    end
                    4'd3: begin
                        distance_high <= rx_byte;
                        checksum_sum  <= checksum_sum + rx_byte;
                        parser_index  <= 4'd4;
                    end
                    4'd4: begin
                        strength_low <= rx_byte;
                        checksum_sum <= checksum_sum + rx_byte;
                        parser_index <= 4'd5;
                    end
                    4'd5: begin
                        strength_high <= rx_byte;
                        checksum_sum  <= checksum_sum + rx_byte;
                        parser_index  <= 4'd6;
                    end
                    4'd6: begin
                        temperature_low <= rx_byte;
                        checksum_sum    <= checksum_sum + rx_byte;
                        parser_index    <= 4'd7;
                    end
                    4'd7: begin
                        temperature_high <= rx_byte;
                        checksum_sum     <= checksum_sum + rx_byte;
                        parser_index     <= 4'd8;
                    end
                    4'd8: begin
                        if (rx_byte == checksum_sum) begin
                            distance_cm     <= {distance_high, distance_low};
                            strength        <= {strength_high, strength_low};
                            temperature_raw <= {temperature_high, temperature_low};
                            frame_valid     <= 1'b1;
                            parser_index    <= 4'd0;
                        end else begin
                            checksum_error <= 1'b1;
                            // 仅在坏帧时把 0x59 复用为下一帧首个帧头；
                            // 合法帧的校验字节也可能恰好为 0x59，不能因此错位。
                            if (rx_byte == 8'h59) begin
                                checksum_sum <= 8'h59;
                                parser_index <= 4'd1;
                            end else begin
                                parser_index <= 4'd0;
                            end
                        end
                    end

                    default: parser_index <= 4'd0;
                endcase
            end else if (parser_index != 0) begin
                // 截断帧若长时间没有下一个字节则丢弃，避免跨帧拼接。
                if (parser_timeout_count >= (FRAME_TIMEOUT_CLKS - 1)) begin
                    parser_timeout_count <= 32'd0;
                    parser_index <= 4'd0;
                    checksum_sum <= 8'd0;
                end else begin
                    parser_timeout_count <= parser_timeout_count + 1'b1;
                end
            end else begin
                parser_timeout_count <= 32'd0;
            end
        end
    end

endmodule
