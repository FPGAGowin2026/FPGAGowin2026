`timescale 1ns/1ps
module sensor_example_top #(parameter integer SCCB_CLK_DIV=135)(
    input wire clk,input wire rst_n,input wire camera_pclk,input wire camera_vsync,input wire camera_href,input wire[7:0]camera_data,
    input wire cfg_valid,input wire[7:0]cfg_addr,input wire[7:0]cfg_data,input wire[1:0]debug_select,
    output wire cfg_ready,cfg_done,cfg_error,output wire sccb_sioc,inout wire sccb_siod,
    output wire[4:0]capture_status,output reg[15:0]debug_data);
    wire frame_active,pixel_valid,frame_start,frame_end,partial_pixel_error;
    wire[15:0]pixel_rgb565,pixel_x,line_y;
    assign capture_status={partial_pixel_error,frame_end,frame_start,pixel_valid,frame_active};
    always @* begin
        case(debug_select)
            2'd0:debug_data=pixel_rgb565;
            2'd1:debug_data=pixel_x;
            2'd2:debug_data=line_y;
            default:debug_data=16'd0;
        endcase
    end
    ov7725_dvp_capture u_cap(.pclk(camera_pclk),.rst_n(rst_n),.vsync(camera_vsync),.href(camera_href),.data_i(camera_data),.frame_active(frame_active),.pixel_valid(pixel_valid),.pixel_rgb565(pixel_rgb565),.pixel_x(pixel_x),.line_y(line_y),.frame_start(frame_start),.frame_end(frame_end),.partial_pixel_error(partial_pixel_error));
    ov7725_sccb_write #(.CLK_DIV(SCCB_CLK_DIV))u_cfg(.clk(clk),.rst_n(rst_n),.cfg_valid(cfg_valid),.cfg_addr(cfg_addr),.cfg_data(cfg_data),.cfg_ready(cfg_ready),.cfg_done(cfg_done),.cfg_error(cfg_error),.sioc(sccb_sioc),.siod(sccb_siod));
endmodule
