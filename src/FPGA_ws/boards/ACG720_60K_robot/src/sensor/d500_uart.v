`timescale 1ns/1ps

// STL-19P / LDROBOT 0x54 0x2c 串口帧接收器。
// UART 为 230400 baud、8N1；仅在 47 字节帧的 CRC-8（poly 0x4d）正确时发布。
module d500_uart #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD   = 230400
)(
    input  wire         clk,
    input  wire         reset_n,
    input  wire         rx,
    output reg          packet_valid,
    output reg  [375:0] packet
);

    function integer clog2;
        input integer value;
        integer work_value;
        begin
            work_value = value - 1;
            for (clog2 = 0; work_value > 0; clog2 = clog2 + 1)
                work_value = work_value >> 1;
        end
    endfunction

    // Round the bit period to the nearest clock: 543 clocks at 125 MHz.
    localparam integer CLKS_PER_BIT = (CLK_HZ + (BAUD / 2)) / BAUD;
    localparam integer HALF_BIT_CLKS = (CLKS_PER_BIT + 1) / 2;
    localparam integer TIMER_WIDTH = (CLKS_PER_BIT <= 1) ? 1 : clog2(CLKS_PER_BIT);

    localparam [2:0] RX_IDLE      = 3'd0;
    localparam [2:0] RX_START     = 3'd1;
    localparam [2:0] RX_DATA      = 3'd2;
    localparam [2:0] RX_STOP      = 3'd3;
    localparam [2:0] RX_WAIT_HIGH = 3'd4;

    localparam [1:0] PKT_SYNC_54 = 2'd0;
    localparam [1:0] PKT_SYNC_2C = 2'd1;
    localparam [1:0] PKT_BODY    = 2'd2;

    reg rx_meta;
    reg rx_sync;
    reg [2:0] rx_state;
    reg [TIMER_WIDTH-1:0] rx_timer;
    reg [2:0] rx_bit_index;
    reg [7:0] rx_shift;
    reg [7:0] rx_data;
    reg       rx_byte_valid;
    reg [7:0] parser_data, parser_crc;
    reg parser_valid, is_sync54, is_sync2c, crc_match;

    reg [1:0] parser_state;
    reg [5:0] byte_index;
    reg [375:0] packet_work;
    reg [7:0] crc_state;

    function [7:0] crc8_update;
        input [7:0] crc_in;
        input [7:0] data_in;
        integer bit_no;
        reg [7:0] crc_value;
        begin
            crc_value = crc_in ^ data_in;
            for (bit_no = 0; bit_no < 8; bit_no = bit_no + 1) begin
                if (crc_value[7])
                    crc_value = {crc_value[6:0], 1'b0} ^ 8'h4d;
                else
                    crc_value = {crc_value[6:0], 1'b0};
            end
            crc8_update = crc_value;
        end
    endfunction

    // 对异步 UART 输入进行双触发器同步。
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;
        end
    end

    // LSB 优先的 UART 8N1 接收器，在数据位和停止位中心采样。
    // 停止位错误时丢弃该字节，并等待 RX 恢复为高电平。
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            rx_state      <= RX_IDLE;
            rx_timer      <= {TIMER_WIDTH{1'b0}};
            rx_bit_index  <= 3'd0;
            rx_shift      <= 8'd0;
            rx_data       <= 8'd0;
            rx_byte_valid <= 1'b0;
        end else begin
            rx_byte_valid <= 1'b0;
            case (rx_state)
                RX_IDLE: begin
                    rx_timer <= {TIMER_WIDTH{1'b0}};
                    if (!rx_sync) begin
                        rx_state <= RX_START;
                        rx_timer <= {TIMER_WIDTH{1'b0}};
                    end
                end

                RX_START: begin
                    if (rx_timer == HALF_BIT_CLKS - 1) begin
                        rx_timer <= {TIMER_WIDTH{1'b0}};
                        if (!rx_sync) begin
                            rx_state     <= RX_DATA;
                            rx_bit_index <= 3'd0;
                        end else begin
                            rx_state <= RX_IDLE;
                        end
                    end else begin
                        rx_timer <= rx_timer + 1'b1;
                    end
                end

                RX_DATA: begin
                    if (rx_timer == CLKS_PER_BIT - 1) begin
                        rx_timer <= {TIMER_WIDTH{1'b0}};
                        rx_shift[rx_bit_index] <= rx_sync;
                        if (rx_bit_index == 3'd7)
                            rx_state <= RX_STOP;
                        else
                            rx_bit_index <= rx_bit_index + 1'b1;
                    end else begin
                        rx_timer <= rx_timer + 1'b1;
                    end
                end

                RX_STOP: begin
                    if (rx_timer == CLKS_PER_BIT - 1) begin
                        rx_timer <= {TIMER_WIDTH{1'b0}};
                        if (rx_sync) begin
                            rx_data       <= rx_shift;
                            rx_byte_valid <= 1'b1;
                            rx_state      <= RX_IDLE;
                        end else begin
                            rx_state <= RX_WAIT_HIGH;
                        end
                    end else begin
                        rx_timer <= rx_timer + 1'b1;
                    end
                end

                RX_WAIT_HIGH: begin
                    rx_timer <= {TIMER_WIDTH{1'b0}};
                    if (rx_sync)
                        rx_state <= RX_IDLE;
                end

                default: begin
                    rx_state <= RX_IDLE;
                    rx_timer <= {TIMER_WIDTH{1'b0}};
                end
            endcase
        end
    end

    // UART bytes are hundreds of clocks apart. Decode and CRC get a separate
    // stage so that the 125 MHz packet-valid path starts at local registers.
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            parser_data <= 0; parser_crc <= 0; parser_valid <= 0;
            is_sync54 <= 0; is_sync2c <= 0; crc_match <= 0;
        end else begin
            parser_valid <= rx_byte_valid;
            if (rx_byte_valid) begin
                parser_data <= rx_data;
                parser_crc <= crc8_update(crc_state, rx_data);
                is_sync54 <= rx_data == 8'h54;
                is_sync2c <= rx_data == 8'h2c;
                crc_match <= rx_data == crc_state;
            end
        end
    end

    // 滑动搜索两字节帧头，收集固定的 47 字节并检查 CRC。
    // 按线缆字节顺序存储：首个接收字节位于 [375:368]。
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            parser_state <= PKT_SYNC_54;
            byte_index   <= 6'd0;
            packet_work  <= 376'd0;
            crc_state    <= 8'd0;
            packet_valid <= 1'b0;
            packet       <= 376'd0;
        end else begin
            packet_valid <= 1'b0;

            if (parser_valid) begin
                case (parser_state)
                    PKT_SYNC_54: begin
                        if (is_sync54) begin
                            packet_work[375:368] <= 8'h54;
                            crc_state <= crc8_update(8'd0, 8'h54);
                            parser_state <= PKT_SYNC_2C;
                        end
                    end

                    PKT_SYNC_2C: begin
                        if (is_sync2c) begin
                            packet_work[367:360] <= 8'h2c;
                            crc_state <= parser_crc;
                            byte_index <= 6'd2;
                            parser_state <= PKT_BODY;
                        end else if (is_sync54) begin
                            // 连续出现两个 0x54 时，保留后一个作为新帧头候选。
                            packet_work[375:368] <= 8'h54;
                            crc_state <= crc8_update(8'd0, 8'h54);
                            parser_state <= PKT_SYNC_2C;
                        end else begin
                            parser_state <= PKT_SYNC_54;
                        end
                    end

                    PKT_BODY: begin
                        if (byte_index < 6'd46) begin
                            packet_work[375 - (byte_index * 8) -: 8] <= parser_data;
                            crc_state <= parser_crc;
                            byte_index <= byte_index + 1'b1;
                        end else begin
                            if (crc_match) begin
                                packet <= {packet_work[375:8], parser_data};
                                packet_valid <= 1'b1;
                                parser_state <= PKT_SYNC_54;
                            end else if (is_sync54) begin
                                // CRC 错误时，若末字节为 0x54，则保留为新帧头候选。
                                packet_work[375:368] <= 8'h54;
                                crc_state <= crc8_update(8'd0, 8'h54);
                                parser_state <= PKT_SYNC_2C;
                            end else begin
                                parser_state <= PKT_SYNC_54;
                            end
                            byte_index <= 6'd0;
                        end
                    end

                    default: begin
                        parser_state <= PKT_SYNC_54;
                        byte_index <= 6'd0;
                    end
                endcase
            end
        end
    end

endmodule
