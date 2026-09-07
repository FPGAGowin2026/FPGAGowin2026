`timescale 1ns/1ps

// 320x240 RGB565 输入隔点降采样为 160x120，并以双时钟双缓冲送往显示域。
// 写入时量化为 RGB444；内存总量 38400x12=460800 bit，GW5A 目标约 36 个 BSRAM。
module choice3_camera_double_buffer (
    input  wire        cam_pclk_i,
    input  wire        cam_rst_n_i,
    input  wire        cam_frame_valid_i,
    input  wire        cam_line_valid_i,
    input  wire        cam_pixel_valid_i,
    input  wire [15:0] cam_pixel_rgb565_i,
    input  wire        video_clk_i,
    input  wire        video_rst_n_i,
    input  wire        video_frame_start_i,
    input  wire [14:0] video_read_offset_i,
    output reg  [11:0] video_pixel_rgb444_o,
    output reg         video_frame_valid_o,
    output reg         video_frame_commit_pulse_o,
    output reg         cam_frame_commit_pulse_o,
    output reg  [15:0] dropped_frame_count_o,
    output wire        display_bank_o
);
    localparam integer FRAME_PIXELS=19200;
    localparam integer TOTAL_PIXELS=38400;
    (* syn_ramstyle="block_ram", ram_style="block" *)
    reg [11:0] frame_mem [0:TOTAL_PIXELS-1];

    reg prev_frame_q,prev_line_q,capture_active_q,malformed_q,write_bank_q;
    reg [9:0] source_x_q;
    reg [8:0] source_y_q,line_count_q;
    reg [14:0] write_offset_q;
    reg [15:0] written_pixels_q;
    reg commit_toggle_q;
    (* ASYNC_REG="TRUE" *) reg ack_meta_q,ack_sync_q;
    reg [15:0] write_addr_q;
    reg [11:0] write_data_q;
    reg write_enable_q;

    (* ASYNC_REG="TRUE" *) reg commit_meta_q,commit_sync_q;
    reg display_bank_q,ack_toggle_q;
    wire [15:0] video_read_addr_w=(display_bank_q ? FRAME_PIXELS : 0)+video_read_offset_i;
    assign display_bank_o=display_bank_q;

    // 单写端口：Gowin 综合应把本 always 与下方读端口合并为 BSRAM。
    always @(posedge cam_pclk_i) begin
        if(write_enable_q) frame_mem[write_addr_q] <= write_data_q;
    end
    always @(posedge video_clk_i) begin
        video_pixel_rgb444_o <= frame_mem[video_read_addr_w];
    end

    always @(posedge cam_pclk_i or negedge cam_rst_n_i) begin
        if(!cam_rst_n_i) begin
            ack_meta_q<=0;ack_sync_q<=0;
        end else begin
            ack_meta_q<=ack_toggle_q;ack_sync_q<=ack_meta_q;
        end
    end

    always @(posedge cam_pclk_i or negedge cam_rst_n_i) begin
        if(!cam_rst_n_i) begin
            prev_frame_q<=0;prev_line_q<=0;capture_active_q<=0;malformed_q<=0;
            write_bank_q<=1;source_x_q<=0;source_y_q<=0;line_count_q<=0;
            write_offset_q<=0;written_pixels_q<=0;commit_toggle_q<=0;
            write_addr_q<=0;write_data_q<=0;write_enable_q<=0;
            cam_frame_commit_pulse_o<=0;dropped_frame_count_o<=0;
        end else begin
            prev_frame_q<=cam_frame_valid_i;prev_line_q<=cam_line_valid_i;
            write_enable_q<=0;cam_frame_commit_pulse_o<=0;

            if(cam_frame_valid_i && !prev_frame_q) begin
                // 上一提交尚未被显示域确认时整帧跳过，绝不覆盖 pending bank。
                capture_active_q <= (commit_toggle_q==ack_sync_q);
                malformed_q<=0;source_x_q<=0;source_y_q<=0;line_count_q<=0;
                write_offset_q<=0;written_pixels_q<=0;
            end

            if(cam_frame_valid_i && capture_active_q) begin
                if(cam_pixel_valid_i && cam_line_valid_i) begin
                    if(source_x_q>=320 || source_y_q>=240) malformed_q<=1;
                    else if(!source_x_q[0] && !source_y_q[0]) begin
                        write_addr_q <= (write_bank_q ? FRAME_PIXELS : 0)+write_offset_q;
                        // RGB565 -> RGB444：保留每个颜色分量的高 4 位。
                        write_data_q <= {cam_pixel_rgb565_i[15:12],
                                         cam_pixel_rgb565_i[10:7],
                                         cam_pixel_rgb565_i[4:1]};
                        write_enable_q<=1;
                        write_offset_q<=write_offset_q+1'b1;
                        written_pixels_q<=written_pixels_q+1'b1;
                    end
                    source_x_q<=source_x_q+1'b1;
                end
                if(prev_line_q && !cam_line_valid_i) begin
                    if(source_x_q!=320) malformed_q<=1;
                    source_x_q<=0;source_y_q<=source_y_q+1'b1;
                    line_count_q<=line_count_q+1'b1;
                end
            end

            if(prev_frame_q && !cam_frame_valid_i) begin
                capture_active_q<=0;
                if(capture_active_q && !malformed_q && line_count_q==240 &&
                   written_pixels_q==FRAME_PIXELS) begin
                    commit_toggle_q<=~commit_toggle_q;
                    // 两域严格交替 bank，不跨域传递 bank 旁带位。
                    write_bank_q<=~write_bank_q;
                    cam_frame_commit_pulse_o<=1;
                end else if(dropped_frame_count_o!=16'hffff)
                    dropped_frame_count_o<=dropped_frame_count_o+1'b1;
            end
        end
    end

    always @(posedge video_clk_i or negedge video_rst_n_i) begin
        if(!video_rst_n_i) begin
            commit_meta_q<=0;commit_sync_q<=0;
            display_bank_q<=0;ack_toggle_q<=0;
            video_frame_valid_o<=0;video_frame_commit_pulse_o<=0;
        end else begin
            commit_meta_q<=commit_toggle_q;commit_sync_q<=commit_meta_q;
            video_frame_commit_pulse_o<=0;
            if(video_frame_start_i && commit_sync_q!=ack_toggle_q) begin
                display_bank_q<=~display_bank_q;ack_toggle_q<=commit_sync_q;
                video_frame_valid_o<=1;
                video_frame_commit_pulse_o<=1;
            end
        end
    end
endmodule
