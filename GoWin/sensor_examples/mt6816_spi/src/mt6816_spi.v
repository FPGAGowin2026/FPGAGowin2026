`timescale 1ns/1ps

// MT6816 four-wire SPI angle reader. The public register map has no device-ID
// register, so integrity is established with the documented even-parity bit.
module mt6816_spi #(
    parameter integer CLK_DIV = 2,
    parameter integer STARTUP_DELAY_CYCLES = 432000,
    parameter integer SAMPLE_INTERVAL_CYCLES = 27000
) (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        spi_miso,
    output wire        spi_cs_n,
    output wire        spi_sclk,
    output wire        spi_mosi,
    output reg         initialized,
    output reg         sensor_ok,
    output reg  [13:0] angle_raw,
    output reg         no_magnet,
    output reg         over_speed,
    output reg         sample_valid,
    output reg         protocol_error
);
    localparam [3:0] ST_POWER = 4'd0, ST_H_START = 4'd1,
                     ST_H_WAIT = 4'd2, ST_L_START = 4'd3,
                     ST_L_WAIT = 4'd4, ST_S_START = 4'd5,
                     ST_S_WAIT = 4'd6, ST_CHECK = 4'd7,
                     ST_INTERVAL = 4'd8;

    reg [3:0] state;
    reg [31:0] delay_count;
    reg [7:0] angle_hi;
    reg [7:0] angle_lo;
    reg [7:0] status_byte;
    reg master_start;
    reg [119:0] master_tx;
    reg [7:0] master_bits;
    wire master_busy;
    wire master_done;
    wire [119:0] master_rx;

    spi_variable_master_mt6816 #(
        .CLK_DIV(CLK_DIV), .CPOL(1), .CPHA(1)
    ) u_spi (
        .clk(clk), .reset_n(reset_n), .start(master_start),
        .bit_length(master_bits), .tx_data(master_tx), .miso(spi_miso),
        .cs_n(spi_cs_n), .sclk(spi_sclk), .mosi(spi_mosi),
        .busy(master_busy), .done(master_done), .rx_data(master_rx)
    );

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= ST_POWER;
            delay_count <= 0;
            angle_hi <= 0;
            angle_lo <= 0;
            status_byte <= 0;
            master_start <= 0;
            master_tx <= 0;
            master_bits <= 0;
            initialized <= 0;
            sensor_ok <= 0;
            angle_raw <= 0;
            no_magnet <= 0;
            over_speed <= 0;
            sample_valid <= 0;
            protocol_error <= 0;
        end else begin
            master_start <= 0;
            sample_valid <= 0;
            protocol_error <= 0;
            case (state)
                ST_POWER: begin
                    if (delay_count + 1 >= STARTUP_DELAY_CYCLES) begin
                        delay_count <= 0;
                        initialized <= 1;
                        state <= ST_H_START;
                    end else delay_count <= delay_count + 1;
                end
                ST_H_START: if (!master_busy) begin
                    master_tx <= {8'h83, 8'h00, 104'd0};
                    master_bits <= 16;
                    master_start <= 1;
                    state <= ST_H_WAIT;
                end
                ST_H_WAIT: if (master_done) begin
                    angle_hi <= master_rx[7:0];
                    state <= ST_L_START;
                end
                ST_L_START: if (!master_busy) begin
                    master_tx <= {8'h84, 8'h00, 104'd0};
                    master_bits <= 16;
                    master_start <= 1;
                    state <= ST_L_WAIT;
                end
                ST_L_WAIT: if (master_done) begin
                    angle_lo <= master_rx[7:0];
                    state <= ST_S_START;
                end
                ST_S_START: if (!master_busy) begin
                    master_tx <= {8'h85, 8'h00, 104'd0};
                    master_bits <= 16;
                    master_start <= 1;
                    state <= ST_S_WAIT;
                end
                ST_S_WAIT: if (master_done) begin
                    status_byte <= master_rx[7:0];
                    state <= ST_CHECK;
                end
                ST_CHECK: begin
                    no_magnet <= angle_lo[1];
                    over_speed <= status_byte[3];
                    if ((^{angle_hi, angle_lo}) ||
                        ((angle_hi == 8'hff) && (angle_lo == 8'hff) &&
                         (status_byte == 8'hff))) begin
                        sensor_ok <= 0;
                        protocol_error <= 1;
                    end else begin
                        angle_raw <= {angle_hi, angle_lo[7:2]};
                        sensor_ok <= !angle_lo[1] && !status_byte[3];
                        sample_valid <= 1;
                    end
                    delay_count <= 0;
                    state <= ST_INTERVAL;
                end
                ST_INTERVAL: begin
                    if (delay_count + 1 >= SAMPLE_INTERVAL_CYCLES) begin
                        delay_count <= 0;
                        state <= ST_H_START;
                    end else delay_count <= delay_count + 1;
                end
                default: state <= ST_POWER;
            endcase
        end
    end
endmodule

module spi_variable_master_mt6816 #(
    parameter integer CLK_DIV = 2,
    parameter integer CPOL = 0,
    parameter integer CPHA = 0
) (
    input wire clk, input wire reset_n, input wire start,
    input wire [7:0] bit_length, input wire [119:0] tx_data,
    input wire miso, output reg cs_n, output reg sclk, output reg mosi,
    output reg busy, output reg done, output reg [119:0] rx_data
);
    localparam integer SAFE_DIV = (CLK_DIV < 1) ? 1 : CLK_DIV;
    reg [31:0] div_count;
    reg phase;
    reg [7:0] bits_left;
    reg [119:0] tx_shift;
    reg [119:0] rx_shift;
    wire [119:0] sampled_rx = {rx_shift[118:0], miso};

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            cs_n <= 1; sclk <= CPOL; mosi <= 0; busy <= 0; done <= 0;
            div_count <= 0; phase <= 0; bits_left <= 0;
            tx_shift <= 0; rx_shift <= 0; rx_data <= 0;
        end else begin
            done <= 0;
            if (!busy) begin
                sclk <= CPOL;
                if (start && bit_length != 0 && bit_length <= 120) begin
                    cs_n <= 0; busy <= 1; phase <= 0; div_count <= 0;
                    bits_left <= bit_length; tx_shift <= tx_data; rx_shift <= 0;
                    mosi <= CPHA ? 1'b0 : tx_data[119];
                end else cs_n <= 1;
            end else if (div_count + 1 >= SAFE_DIV) begin
                div_count <= 0;
                if (!phase) begin
                    sclk <= !CPOL; phase <= 1;
                    if (CPHA) mosi <= tx_shift[119];
                    else rx_shift <= sampled_rx;
                end else begin
                    sclk <= CPOL; phase <= 0;
                    if (CPHA) rx_shift <= sampled_rx;
                    if (bits_left == 1) begin
                        cs_n <= 1; busy <= 0; done <= 1;
                        rx_data <= CPHA ? sampled_rx : rx_shift;
                    end else begin
                        bits_left <= bits_left - 8'd1;
                        tx_shift <= {tx_shift[118:0], 1'b0};
                        if (!CPHA) mosi <= tx_shift[118];
                    end
                end
            end else div_count <= div_count + 1;
        end
    end
endmodule
