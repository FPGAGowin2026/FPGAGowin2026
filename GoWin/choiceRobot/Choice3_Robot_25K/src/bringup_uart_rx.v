`timescale 1ns/1ps

// 8-N-1 UART 接收器。异步输入先经过两级同步，停止位错误产生单拍脉冲。
module bringup_uart_rx #(
    parameter integer CLK_FREQ_HZ = 50000000,
    parameter integer BAUD_RATE   = 115200
) (
    input wire clk,input wire rst_n,input wire rx_i,
    output reg [7:0] data_o,output reg data_valid_o,output reg frame_error_o
);
    localparam integer SAFE_BAUD_RATE=(BAUD_RATE<1)?1:BAUD_RATE;
    localparam integer RAW_CLKS_PER_BIT=(CLK_FREQ_HZ+(SAFE_BAUD_RATE/2))/SAFE_BAUD_RATE;
    localparam integer CLKS_PER_BIT=(RAW_CLKS_PER_BIT<2)?2:RAW_CLKS_PER_BIT;
    localparam integer HALF_BIT_CLKS=CLKS_PER_BIT/2;
    localparam [2:0] ST_IDLE=0,ST_START=1,ST_DATA=2,ST_STOP=3;
    (* ASYNC_REG = "TRUE" *) reg rx_meta,rx_sync;
    reg [2:0] state,bit_index;reg [31:0] clock_count;reg [7:0] shift_data;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin rx_meta<=1;rx_sync<=1;end
        else begin rx_meta<=rx_i;rx_sync<=rx_meta;end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin state<=ST_IDLE;clock_count<=0;bit_index<=0;shift_data<=0;
          data_o<=0;data_valid_o<=0;frame_error_o<=0;end
        else begin
          data_valid_o<=0;frame_error_o<=0;
          case(state)
            ST_IDLE:begin clock_count<=0;bit_index<=0;if(!rx_sync)state<=ST_START;end
            ST_START:if(clock_count>=(HALF_BIT_CLKS-1))begin clock_count<=0;
              if(!rx_sync)state<=ST_DATA;else state<=ST_IDLE;end
              else clock_count<=clock_count+1'b1;
            ST_DATA:if(clock_count>=(CLKS_PER_BIT-1))begin clock_count<=0;
              shift_data[bit_index]<=rx_sync;if(bit_index==7)begin bit_index<=0;state<=ST_STOP;end
              else bit_index<=bit_index+1'b1;end else clock_count<=clock_count+1'b1;
            ST_STOP:if(clock_count>=(CLKS_PER_BIT-1))begin clock_count<=0;
              if(rx_sync)begin data_o<=shift_data;data_valid_o<=1;end else frame_error_o<=1;
              state<=ST_IDLE;end else clock_count<=clock_count+1'b1;
            default:state<=ST_IDLE;
          endcase
        end
    end
endmodule
