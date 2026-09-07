`timescale 1ns/1ps

// 640x480@约59.9 Hz（25.156 MHz 像素时钟）VGA/DVI 时序，源图放大4倍。
module choice3_vga_640x480 (
    input wire pixel_clk_i,input wire rst_n_i,
    input wire [11:0] framebuffer_pixel_i,input wire framebuffer_valid_i,
    input wire [7:0] fault_mask_i,
    input wire signed [15:0] roll_deg_i,
    input wire signed [15:0] pitch_deg_i,
    input wire signed [15:0] yaw_deg_i,input wire attitude_valid_i,
    input wire demo_pattern_i,
    output wire [14:0] framebuffer_offset_o,
    output wire frame_start_o,
    output reg [7:0] red_o,output reg [7:0] green_o,output reg [7:0] blue_o,
    output reg hsync_o,output reg vsync_o,output reg de_o,
    output reg [9:0] h_count_o,output reg [9:0] v_count_o
);
    wire active_w=(h_count_o<640)&&(v_count_o<480);
    wire hsync_w=~((h_count_o>=656)&&(h_count_o<752));
    wire vsync_w=~((v_count_o>=490)&&(v_count_o<492));
    wire [6:0] source_y=v_count_o[8:2];
    wire [7:0] source_x=h_count_o[9:2];
    wire [14:0] source_y_wide={8'd0,source_y};
    wire [14:0] source_x_wide={7'd0,source_x};
    wire [14:0] scaled_addr=(source_y_wide<<7)+(source_y_wide<<5)+source_x_wide;
    // RGB444 扩展到 8 位输出；复制半字节可覆盖 0..255 全量程。
    wire [7:0] fb_r={framebuffer_pixel_i[11:8],framebuffer_pixel_i[11:8]};
    wire [7:0] fb_g={framebuffer_pixel_i[7:4],framebuffer_pixel_i[7:4]};
    wire [7:0] fb_b={framebuffer_pixel_i[3:0],framebuffer_pixel_i[3:0]};
    assign framebuffer_offset_o=active_w?scaled_addr:15'd0;
    assign frame_start_o=(h_count_o==799)&&(v_count_o==524);
    reg [9:0] h_pipe_q,v_pipe_q;
    wire osd_hit;
    wire [7:0] osd_r,osd_g,osd_b;
    // 保留层级，便于在综合报告中独立审计 OSD 资源。
    (* syn_keep = 1, keep_hierarchy = "yes" *) choice3_attitude_osd u_attitude_osd(
      .clk_i(pixel_clk_i),.rst_n_i(rst_n_i),.x_i(h_count_o),.y_i(v_count_o),
      .roll_deg_i(roll_deg_i),
      .pitch_deg_i(pitch_deg_i),.yaw_deg_i(yaw_deg_i),.valid_i(attitude_valid_i),
      .hit_o(osd_hit),.red_o(osd_r),.green_o(osd_g),.blue_o(osd_b));

    always @(posedge pixel_clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin
            h_count_o<=0;v_count_o<=0;h_pipe_q<=0;v_pipe_q<=0;
            hsync_o<=1;vsync_o<=1;de_o<=0;
        end else begin
            if(h_count_o==799)begin h_count_o<=0;if(v_count_o==524)v_count_o<=0;
                else v_count_o<=v_count_o+1'b1;end
            else h_count_o<=h_count_o+1'b1;
            // 一拍控制延迟与同步 BSRAM 读延迟对齐。
            hsync_o<=hsync_w;vsync_o<=vsync_w;de_o<=active_w;
            h_pipe_q<=h_count_o;v_pipe_q<=v_count_o;
        end
    end

    // 数据不再增加额外寄存拍：同步 RAM 输出与 h/v/de 的一拍流水严格对齐。
    always @* begin
        red_o=0;green_o=0;blue_o=0;
        if(de_o) begin
            if(demo_pattern_i) begin
                // 片上色条/棋盘只用于无摄像头验证，状态寄存器会明确标记 DEMO。
                red_o={h_pipe_q[7:5],5'b0};
                green_o={v_pipe_q[7:5],5'b0};
                blue_o=(h_pipe_q[5]^v_pipe_q[5])?8'hd0:8'h30;
            end else if(framebuffer_valid_i) begin red_o=fb_r;green_o=fb_g;blue_o=fb_b;end
            else begin
                // 尚无完整摄像头帧时显示蓝/灰诊断棋盘，不冒充有效图像。
                red_o=h_pipe_q[6]^v_pipe_q[6]?8'h20:8'h60;
                green_o=h_pipe_q[6]^v_pipe_q[6]?8'h20:8'h60;blue_o=8'hb0;
            end
            if(fault_mask_i!=0 && h_pipe_q<128 && v_pipe_q<16) begin
                red_o=8'hff;green_o=0;blue_o=0;
            end
            if(osd_hit)begin red_o=osd_r;green_o=osd_g;blue_o=osd_b;end
        end
    end
endmodule
