`timescale 1ns/1ps
module sensor_example_top #(parameter integer CLK_FREQ_HZ=27000000,parameter integer CONFIG_BAUD_RATE=9600)(
    input wire clk,input wire rst_n,input wire camera_pclk,input wire camera_vsync,input wire camera_href,input wire[7:0]camera_data,
    input wire cfg_valid,input wire[7:0]cfg_command,input wire[15:0]cfg_value,input wire[1:0]debug_select,
    output wire cfg_ready,cfg_done,camera_cfg_tx,output wire[3:0]capture_status,output reg[15:0]debug_data);
    wire frame_active,pixel_valid,frame_start,frame_end;
    wire[7:0]pixel_gray;
    wire[15:0]pixel_x,line_y;
    assign capture_status={frame_end,frame_start,pixel_valid,frame_active};
    always @* begin
        case(debug_select)
            2'd0:debug_data={8'd0,pixel_gray};
            2'd1:debug_data=pixel_x;
            2'd2:debug_data=line_y;
            default:debug_data=16'd0;
        endcase
    end
    mt9v03x_dvp_capture u_cap(.pclk(camera_pclk),.rst_n(rst_n),.vsync(camera_vsync),.href(camera_href),.data_i(camera_data),.frame_active(frame_active),.pixel_valid(pixel_valid),.pixel_gray(pixel_gray),.pixel_x(pixel_x),.line_y(line_y),.frame_start(frame_start),.frame_end(frame_end));
    camera_uart_cmd_tx_mt9v03x #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.BAUD_RATE(CONFIG_BAUD_RATE))u_cfg(.clk(clk),.rst_n(rst_n),.cfg_valid(cfg_valid),.cfg_command(cfg_command),.cfg_value(cfg_value),.cfg_ready(cfg_ready),.cfg_done(cfg_done),.tx_o(camera_cfg_tx));
endmodule
