`timescale 1ns/1ps

// MT9V03X/MT9V034 module DVP front end: one 8-bit grayscale pixel on every
// PCLK rising edge while HREF is high, after a fresh active-high VSYNC pulse.
module mt9v03x_dvp_capture(
    input wire pclk,input wire rst_n,input wire vsync,input wire href,input wire[7:0]data_i,
    output reg frame_active,output reg pixel_valid,output reg[7:0]pixel_gray,
    output reg[15:0]pixel_x,output reg[15:0]line_y,output reg frame_start,output reg frame_end);
    reg armed_q,vsync_d_q,href_d_q;
    always@(posedge pclk or negedge rst_n)begin
        if(!rst_n)begin armed_q<=0;vsync_d_q<=1;href_d_q<=0;frame_active<=0;pixel_valid<=0;pixel_gray<=0;pixel_x<=0;line_y<=0;frame_start<=0;frame_end<=0;end
        else begin
            vsync_d_q<=vsync;href_d_q<=href;pixel_valid<=0;frame_start<=0;frame_end<=0;
            if(vsync)begin if(!vsync_d_q)begin frame_start<=1;if(frame_active)frame_end<=1;end armed_q<=1;frame_active<=0;pixel_x<=0;line_y<=0;end
            else if(!armed_q)begin frame_active<=0;pixel_x<=0;end
            else begin frame_active<=1;if(href)begin pixel_gray<=data_i;pixel_valid<=1;pixel_x<=pixel_x+1'b1;end else begin if(href_d_q)line_y<=line_y+1'b1;pixel_x<=0;end end
        end
    end
endmodule

// Local Seekfree drivers configure this camera with 4 UART bytes:
// A5, command, value[15:8], value[7:0], at a fixed 9600 baud.
module camera_uart_cmd_tx_mt9v03x #(parameter integer CLK_FREQ_HZ=27000000,parameter integer BAUD_RATE=9600)(
    input wire clk,input wire rst_n,input wire cfg_valid,input wire[7:0]cfg_command,input wire[15:0]cfg_value,
    output wire cfg_ready,output reg cfg_done,output reg tx_o);
    localparam integer C0=CLK_FREQ_HZ/BAUD_RATE;localparam integer C=(C0<1)?1:C0;
    reg busy_q;reg[31:0]count_q;reg[3:0]bit_q;reg[1:0]byte_q;reg[7:0]frame_q[0:3];
    assign cfg_ready=~busy_q;
    always@(posedge clk or negedge rst_n)begin
        if(!rst_n)begin busy_q<=0;count_q<=0;bit_q<=0;byte_q<=0;frame_q[0]<=0;frame_q[1]<=0;frame_q[2]<=0;frame_q[3]<=0;cfg_done<=0;tx_o<=1;end
        else begin cfg_done<=0;
            if(!busy_q)begin tx_o<=1;count_q<=0;if(cfg_valid)begin frame_q[0]<=8'hA5;frame_q[1]<=cfg_command;frame_q[2]<=cfg_value[15:8];frame_q[3]<=cfg_value[7:0];busy_q<=1;byte_q<=0;bit_q<=0;tx_o<=0;end end
            else if(count_q==C-1)begin count_q<=0;
                if(bit_q==0)begin bit_q<=1;tx_o<=frame_q[byte_q][0];end
                else if(bit_q<8)begin tx_o<=frame_q[byte_q][bit_q];bit_q<=bit_q+1'b1;end
                else if(bit_q==8)begin tx_o<=1;bit_q<=9;end
                else if(byte_q<3)begin byte_q<=byte_q+1'b1;bit_q<=0;tx_o<=0;end
                else begin busy_q<=0;tx_o<=1;cfg_done<=1;end
            end else count_q<=count_q+1'b1;
        end
    end
endmodule
