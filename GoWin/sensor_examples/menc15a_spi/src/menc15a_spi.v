`timescale 1ns/1ps
module menc15a_spi #(
    parameter integer CLK_DIV = 1,
    parameter integer STARTUP_DELAY_CYCLES = 27000,
    parameter integer SAMPLE_INTERVAL_CYCLES = 27000
) (
    input wire clk, input wire reset_n, input wire spi_miso,
    output wire spi_cs_n, output wire spi_sclk, output wire spi_mosi,
    output reg initialized, output reg sensor_ok,
    output reg [14:0] position_raw, output reg signed [15:0] speed_raw,
    output reg position_status, output reg speed_status,
    output reg sample_valid, output reg protocol_error
);
    localparam [3:0] ST_POWER=0, ST_POS_START=1, ST_POS_WAIT=2,
                     ST_SPEED_START=3, ST_SPEED_WAIT=4, ST_PUBLISH=5,
                     ST_INTERVAL=6;
    reg [3:0] state;
    reg [31:0] delay_count;
    reg [15:0] position_word, speed_word;
    reg master_start;
    reg [119:0] master_tx;
    wire master_busy, master_done;
    wire [119:0] master_rx;

    spi_variable_master_menc15a #(.CLK_DIV(CLK_DIV), .CPOL(0), .CPHA(1)) u_spi (
        .clk(clk), .reset_n(reset_n), .start(master_start), .bit_length(8'd32),
        .tx_data(master_tx), .miso(spi_miso), .cs_n(spi_cs_n), .sclk(spi_sclk),
        .mosi(spi_mosi), .busy(master_busy), .done(master_done), .rx_data(master_rx));

    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin
            state<=ST_POWER; delay_count<=0; position_word<=0; speed_word<=0;
            master_start<=0; master_tx<=0; initialized<=0; sensor_ok<=0;
            position_raw<=0; speed_raw<=0; position_status<=0; speed_status<=0;
            sample_valid<=0; protocol_error<=0;
        end else begin
            master_start<=0; sample_valid<=0; protocol_error<=0;
            case(state)
                ST_POWER: if(delay_count+1>=STARTUP_DELAY_CYCLES) begin
                    delay_count<=0; initialized<=1; state<=ST_POS_START;
                end else delay_count<=delay_count+1;
                ST_POS_START: if(!master_busy) begin
                    master_tx<={16'h8021,16'h0000,88'd0}; master_start<=1; state<=ST_POS_WAIT;
                end
                ST_POS_WAIT: if(master_done) begin
                    position_word<=master_rx[15:0];
                    if(master_rx[15:0]==16'hffff) begin
                        sensor_ok<=0; protocol_error<=1; delay_count<=0; state<=ST_INTERVAL;
                    end else state<=ST_SPEED_START;
                end
                ST_SPEED_START: if(!master_busy) begin
                    master_tx<={16'h8032,16'h0000,88'd0}; master_start<=1; state<=ST_SPEED_WAIT;
                end
                ST_SPEED_WAIT: if(master_done) begin
                    speed_word<=master_rx[15:0]; state<=ST_PUBLISH;
                end
                ST_PUBLISH: begin
                    if(speed_word==16'hffff) begin
                        sensor_ok<=0; protocol_error<=1;
                    end else begin
                        position_raw<=position_word[14:0];
                        position_status<=position_word[15]; speed_status<=speed_word[15];
                        if(speed_word[14:0] > 15'd16384)
                            speed_raw <= $signed({1'b0,speed_word[14:0]}) - 16'sd32768;
                        else speed_raw <= $signed({1'b0,speed_word[14:0]});
                        sensor_ok<=1; sample_valid<=1;
                    end
                    delay_count<=0; state<=ST_INTERVAL;
                end
                ST_INTERVAL: if(delay_count+1>=SAMPLE_INTERVAL_CYCLES) begin
                    delay_count<=0; state<=ST_POS_START;
                end else delay_count<=delay_count+1;
                default: state<=ST_POWER;
            endcase
        end
    end
endmodule

module spi_variable_master_menc15a #(
    parameter integer CLK_DIV=1, parameter integer CPOL=0, parameter integer CPHA=1
) (
    input wire clk,input wire reset_n,input wire start,input wire [7:0] bit_length,
    input wire [119:0] tx_data,input wire miso,output reg cs_n,output reg sclk,
    output reg mosi,output reg busy,output reg done,output reg [119:0] rx_data
);
    localparam integer SAFE_DIV=(CLK_DIV<1)?1:CLK_DIV;
    reg [31:0] div_count; reg phase; reg [7:0] bits_left;
    reg [119:0] tx_shift,rx_shift;
    wire [119:0] sampled_rx={rx_shift[118:0],miso};
    always @(posedge clk or negedge reset_n) begin
        if(!reset_n) begin cs_n<=1;sclk<=CPOL;mosi<=0;busy<=0;done<=0;div_count<=0;phase<=0;bits_left<=0;tx_shift<=0;rx_shift<=0;rx_data<=0; end
        else begin
            done<=0;
            if(!busy) begin sclk<=CPOL; if(start&&bit_length!=0&&bit_length<=120) begin cs_n<=0;busy<=1;phase<=0;div_count<=0;bits_left<=bit_length;tx_shift<=tx_data;rx_shift<=0;mosi<=CPHA?0:tx_data[119]; end else cs_n<=1; end
            else if(div_count+1>=SAFE_DIV) begin div_count<=0;
                if(!phase) begin sclk<=!CPOL;phase<=1;if(CPHA)mosi<=tx_shift[119];else rx_shift<=sampled_rx; end
                else begin sclk<=CPOL;phase<=0;if(CPHA)rx_shift<=sampled_rx;
                    if(bits_left==1) begin cs_n<=1;busy<=0;done<=1;rx_data<=CPHA?sampled_rx:rx_shift; end
                    else begin bits_left<=bits_left-8'd1;tx_shift<={tx_shift[118:0],1'b0};if(!CPHA)mosi<=tx_shift[118]; end
                end
            end else div_count<=div_count+1;
        end
    end
endmodule
