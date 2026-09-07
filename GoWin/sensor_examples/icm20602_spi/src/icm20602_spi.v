`timescale 1ns/1ps
module icm20602_spi #(
    parameter integer CLK_DIV=2,
    parameter integer STARTUP_DELAY_CYCLES=270000,
    parameter integer RESET_DELAY_CYCLES=270000,
    parameter integer SAMPLE_INTERVAL_CYCLES=27000,
    parameter integer ID_RETRY_LIMIT=32
) (
    input wire clk,input wire reset_n,input wire spi_miso,
    output wire spi_cs_n,output wire spi_sclk,output wire spi_mosi,
    output reg initialized,output reg sensor_ok,
    output reg signed [15:0] accel_x,accel_y,accel_z,
    output reg signed [15:0] gyro_x,gyro_y,gyro_z,
    output reg sample_valid,output reg protocol_error
);
    localparam [4:0] ST_POWER=0,ST_ID_START=1,ST_ID_WAIT=2,ST_RESET_START=3,
        ST_RESET_WAIT=4,ST_RESET_DELAY=5,ST_CFG_START=6,ST_CFG_WAIT=7,
        ST_STATUS_START=8,ST_STATUS_WAIT=9,ST_BURST_START=10,ST_BURST_WAIT=11,
        ST_PUBLISH=12,ST_INTERVAL=13,ST_FAULT=14;
    reg [4:0] state;
    reg [31:0] delay_count;
    reg [7:0] retry_count,cfg_index;
    reg master_start;reg [7:0] master_bits;reg [119:0] master_tx;
    wire master_busy,master_done;wire [119:0] master_rx;
    reg [111:0] frame;
    function [7:0] cfg_reg(input [7:0] n);begin case(n)
        0:cfg_reg=8'h6B;1:cfg_reg=8'h6C;2:cfg_reg=8'h1A;3:cfg_reg=8'h19;
        4:cfg_reg=8'h1B;5:cfg_reg=8'h1C;default:cfg_reg=8'h1D;endcase end endfunction
    function [7:0] cfg_val(input [7:0] n);begin case(n)
        0:cfg_val=8'h01;1:cfg_val=8'h00;2:cfg_val=8'h01;3:cfg_val=8'h07;
        4:cfg_val=8'h18;5:cfg_val=8'h10;default:cfg_val=8'h03;endcase end endfunction
    spi_variable_master_icm20602 #(.CLK_DIV(CLK_DIV),.CPOL(0),.CPHA(0)) u_spi(
        .clk(clk),.reset_n(reset_n),.start(master_start),.bit_length(master_bits),
        .tx_data(master_tx),.miso(spi_miso),.cs_n(spi_cs_n),.sclk(spi_sclk),
        .mosi(spi_mosi),.busy(master_busy),.done(master_done),.rx_data(master_rx));
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin state<=ST_POWER;delay_count<=0;retry_count<=0;cfg_index<=0;
            master_start<=0;master_bits<=0;master_tx<=0;frame<=0;initialized<=0;sensor_ok<=0;
            accel_x<=0;accel_y<=0;accel_z<=0;gyro_x<=0;gyro_y<=0;gyro_z<=0;
            sample_valid<=0;protocol_error<=0;end
        else begin master_start<=0;sample_valid<=0;protocol_error<=0;
            case(state)
                ST_POWER:if(delay_count+1>=STARTUP_DELAY_CYCLES)begin delay_count<=0;state<=ST_ID_START;end else delay_count<=delay_count+1;
                ST_ID_START:if(!master_busy)begin master_tx<={8'hF5,8'h00,104'd0};master_bits<=16;master_start<=1;state<=ST_ID_WAIT;end
                ST_ID_WAIT:if(master_done)begin if(master_rx[7:0]==8'h12)begin retry_count<=0;state<=ST_RESET_START;end
                    else begin protocol_error<=1;sensor_ok<=0;if(retry_count+8'd1>=ID_RETRY_LIMIT)state<=ST_FAULT;else begin retry_count<=retry_count+8'd1;state<=ST_ID_START;end end end
                ST_RESET_START:if(!master_busy)begin master_tx<={8'h6B,8'h80,104'd0};master_bits<=16;master_start<=1;state<=ST_RESET_WAIT;end
                ST_RESET_WAIT:if(master_done)begin delay_count<=0;state<=ST_RESET_DELAY;end
                ST_RESET_DELAY:if(delay_count+1>=RESET_DELAY_CYCLES)begin delay_count<=0;cfg_index<=0;state<=ST_CFG_START;end else delay_count<=delay_count+1;
                ST_CFG_START:if(!master_busy)begin master_tx<={cfg_reg(cfg_index),cfg_val(cfg_index),104'd0};master_bits<=16;master_start<=1;state<=ST_CFG_WAIT;end
                ST_CFG_WAIT:if(master_done)begin if(cfg_index==6)begin initialized<=1;sensor_ok<=1;state<=ST_STATUS_START;end else begin cfg_index<=cfg_index+8'd1;state<=ST_CFG_START;end end
                ST_STATUS_START:if(!master_busy)begin master_tx<={8'hBA,8'h00,104'd0};master_bits<=16;master_start<=1;state<=ST_STATUS_WAIT;end
                ST_STATUS_WAIT:if(master_done)begin if(master_rx[0])state<=ST_BURST_START;else begin delay_count<=0;state<=ST_INTERVAL;end end
                ST_BURST_START:if(!master_busy)begin master_tx<={8'hBB,112'd0};master_bits<=120;master_start<=1;state<=ST_BURST_WAIT;end
                ST_BURST_WAIT:if(master_done)begin frame<=master_rx[111:0];state<=ST_PUBLISH;end
                ST_PUBLISH:begin if(&frame)begin sensor_ok<=0;protocol_error<=1;end else begin
                        accel_x<=$signed(frame[111:96]);accel_y<=$signed(frame[95:80]);accel_z<=$signed(frame[79:64]);
                        gyro_x<=$signed(frame[47:32]);gyro_y<=$signed(frame[31:16]);gyro_z<=$signed(frame[15:0]);
                        sensor_ok<=1;sample_valid<=1;end delay_count<=0;state<=ST_INTERVAL;end
                ST_INTERVAL:if(delay_count+1>=SAMPLE_INTERVAL_CYCLES)begin delay_count<=0;state<=ST_STATUS_START;end else delay_count<=delay_count+1;
                ST_FAULT:begin initialized<=0;sensor_ok<=0;end
                default:state<=ST_POWER;
            endcase
        end
    end
endmodule

module spi_variable_master_icm20602 #(
    parameter integer CLK_DIV=2,parameter integer CPOL=0,parameter integer CPHA=0
)(input wire clk,input wire reset_n,input wire start,input wire [7:0] bit_length,
input wire [119:0] tx_data,input wire miso,output reg cs_n,output reg sclk,output reg mosi,
output reg busy,output reg done,output reg [119:0] rx_data);
    localparam integer SAFE_DIV=(CLK_DIV<1)?1:CLK_DIV;reg [31:0] div_count;reg phase;
    reg [7:0] bits_left;reg [119:0] tx_shift,rx_shift;wire [119:0] sampled_rx={rx_shift[118:0],miso};
    always @(posedge clk or negedge reset_n)begin
        if(!reset_n)begin cs_n<=1;sclk<=CPOL;mosi<=0;busy<=0;done<=0;div_count<=0;phase<=0;bits_left<=0;tx_shift<=0;rx_shift<=0;rx_data<=0;end
        else begin done<=0;if(!busy)begin sclk<=CPOL;if(start&&bit_length!=0&&bit_length<=120)begin cs_n<=0;busy<=1;phase<=0;div_count<=0;bits_left<=bit_length;tx_shift<=tx_data;rx_shift<=0;mosi<=CPHA?0:tx_data[119];end else cs_n<=1;end
        else if(div_count+1>=SAFE_DIV)begin div_count<=0;if(!phase)begin sclk<=!CPOL;phase<=1;if(CPHA)mosi<=tx_shift[119];else rx_shift<=sampled_rx;end
        else begin sclk<=CPOL;phase<=0;if(CPHA)rx_shift<=sampled_rx;if(bits_left==1)begin cs_n<=1;busy<=0;done<=1;rx_data<=CPHA?sampled_rx:rx_shift;end
        else begin bits_left<=bits_left-8'd1;tx_shift<={tx_shift[118:0],1'b0};if(!CPHA)mosi<=tx_shift[118];end end end else div_count<=div_count+1;end
    end
endmodule
