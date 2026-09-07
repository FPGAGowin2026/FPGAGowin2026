`timescale 1ns/1ps

// OV7725 RGB565 DVP stream collector. VSYNC high denotes frame sync/blanking,
// HREF high denotes valid bytes, and RGB565 arrives high byte first.
module ov7725_dvp_capture(
    input wire pclk,input wire rst_n,input wire vsync,input wire href,input wire[7:0]data_i,
    output reg frame_active,output reg pixel_valid,output reg[15:0]pixel_rgb565,
    output reg[15:0]pixel_x,output reg[15:0]line_y,
    output reg frame_start,output reg frame_end,output reg partial_pixel_error);
    reg armed_q,phase_q,vsync_d_q,href_d_q;reg[7:0]high_q;
    always@(posedge pclk or negedge rst_n)begin
        if(!rst_n)begin armed_q<=0;phase_q<=0;vsync_d_q<=1;href_d_q<=0;high_q<=0;frame_active<=0;pixel_valid<=0;pixel_rgb565<=0;pixel_x<=0;line_y<=0;frame_start<=0;frame_end<=0;partial_pixel_error<=0;end
        else begin
            vsync_d_q<=vsync;href_d_q<=href;pixel_valid<=0;frame_start<=0;frame_end<=0;partial_pixel_error<=0;
            if(vsync)begin
                if(!vsync_d_q)begin frame_start<=1;if(frame_active)frame_end<=1;end
                armed_q<=1;frame_active<=0;phase_q<=0;pixel_x<=0;line_y<=0;
            end else if(!armed_q)begin frame_active<=0;phase_q<=0;pixel_x<=0;end
            else begin
                frame_active<=1;
                if(href)begin
                    if(!phase_q)begin high_q<=data_i;phase_q<=1;end
                    else begin pixel_rgb565<={high_q,data_i};pixel_valid<=1;pixel_x<=pixel_x+1'b1;phase_q<=0;end
                end else begin
                    if(href_d_q)begin if(phase_q)partial_pixel_error<=1;line_y<=line_y+1'b1;end
                    phase_q<=0;pixel_x<=0;
                end
            end
        end
    end
endmodule

// Minimal OV7725 SCCB write master. Each request writes device address 42h,
// an 8-bit register address and one data byte. It does not include a power-up
// register table; the caller must issue the required sensor-specific sequence.
module ov7725_sccb_write #(
    parameter integer CLK_DIV=135
) (
    input wire clk,input wire rst_n,input wire cfg_valid,input wire[7:0]cfg_addr,input wire[7:0]cfg_data,
    output wire cfg_ready,output reg cfg_done,output reg cfg_error,
    output reg sioc,inout wire siod);
    localparam integer DIV=(CLK_DIV<1)?1:CLK_DIV;
    localparam[3:0]IDLE=0,START=1,BIT_LOW=2,BIT_HIGH=3,ACK_LOW=4,ACK_HIGH=5,STOP_LOW=6,STOP_HIGH=7,STOP_RELEASE=8;
    reg[3:0]state_q;reg[31:0]div_q;reg[1:0]byte_q;reg[2:0]bit_q;reg[7:0]shift_q,addr_q,data_q;reg sda_low_q;
    wire tick=(div_q==DIV-1);assign siod=sda_low_q?1'b0:1'bz;assign cfg_ready=(state_q==IDLE);
    always@(posedge clk or negedge rst_n)begin
        if(!rst_n)begin state_q<=IDLE;div_q<=0;byte_q<=0;bit_q<=7;shift_q<=0;addr_q<=0;data_q<=0;sda_low_q<=0;sioc<=1;cfg_done<=0;cfg_error<=0;end
        else begin
            cfg_done<=0;
            if(state_q==IDLE)begin div_q<=0;sioc<=1;sda_low_q<=0;if(cfg_valid)begin addr_q<=cfg_addr;data_q<=cfg_data;shift_q<=8'h42;byte_q<=0;bit_q<=7;cfg_error<=0;state_q<=START;end end
            else if(tick)begin
                div_q<=0;
                case(state_q)
                    START:begin sioc<=1;sda_low_q<=1;state_q<=BIT_LOW;end
                    BIT_LOW:begin sioc<=0;sda_low_q<=~shift_q[bit_q];state_q<=BIT_HIGH;end
                    BIT_HIGH:begin sioc<=1;if(bit_q==0)state_q<=ACK_LOW;else begin bit_q<=bit_q-1'b1;state_q<=BIT_LOW;end end
                    ACK_LOW:begin sioc<=0;sda_low_q<=0;state_q<=ACK_HIGH;end
                    ACK_HIGH:begin
                        sioc<=1;if(siod!==1'b0)cfg_error<=1;
                        if(byte_q==2)state_q<=STOP_LOW;
                        else begin byte_q<=byte_q+1'b1;bit_q<=7;if(byte_q==0)shift_q<=addr_q;else shift_q<=data_q;state_q<=BIT_LOW;end
                    end
                    STOP_LOW:begin sioc<=0;sda_low_q<=1;state_q<=STOP_HIGH;end
                    STOP_HIGH:begin sioc<=1;sda_low_q<=1;state_q<=STOP_RELEASE;end
                    STOP_RELEASE:begin sioc<=1;sda_low_q<=0;cfg_done<=1;state_q<=IDLE;end
                    default:state_q<=IDLE;
                endcase
            end else div_q<=div_q+1'b1;
        end
    end
endmodule
