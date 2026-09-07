`timescale 1ns/1ps

module tb_choice3_video_pipeline;
    reg cam_clk=0,video_clk=0,cam_rst_n=0,video_rst_n=0;
    reg frame_valid=0,line_valid=0,pixel_valid=0;
    reg [15:0] pixel_in=0;
    reg video_frame_start=0;
    reg [14:0] read_offset=0;
    wire [11:0] read_pixel;
    wire display_valid,video_commit,cam_commit,display_bank;
    wire [15:0] dropped;
    integer errors=0,cam_commits=0,video_commits=0;

    // DVP 字节序独立探针。
    reg raw_vsync=0,raw_href=0;reg [7:0] raw_data=0;
    wire cap_frame,cap_line,cap_pixel_valid;wire [15:0] cap_pixel;

    // VGA 与同步 RAM 对齐模型。
    wire [14:0] vga_offset;wire vga_frame_start;
    reg [11:0] model_fb_pixel;
    wire [7:0] red,green,blue;wire hs,vs,de;wire [9:0] hc,vc;
    reg [7:0] fault_mask=0;
    integer de_count=0,hs_low_count=0,vs_low_count=0;

    // TMDS 编码探针。
    reg tmds_de=0;reg [1:0] tmds_ctrl=0;reg [7:0] tmds_data=0;
    wire [9:0] tmds_code;

    always #7 cam_clk=~cam_clk;
    always #5 video_clk=~video_clk;
    always @(posedge cam_clk)if(cam_commit)cam_commits=cam_commits+1;
    always @(posedge video_clk)if(video_commit)video_commits=video_commits+1;

    choice3_camera_double_buffer u_buffer(
      .cam_pclk_i(cam_clk),.cam_rst_n_i(cam_rst_n),.cam_frame_valid_i(frame_valid),
      .cam_line_valid_i(line_valid),.cam_pixel_valid_i(pixel_valid),
      .cam_pixel_rgb565_i(pixel_in),.video_clk_i(video_clk),
      .video_rst_n_i(video_rst_n),.video_frame_start_i(video_frame_start),
      .video_read_offset_i(read_offset),.video_pixel_rgb444_o(read_pixel),
      .video_frame_valid_o(display_valid),.video_frame_commit_pulse_o(video_commit),
      .cam_frame_commit_pulse_o(cam_commit),.dropped_frame_count_o(dropped),
      .display_bank_o(display_bank));
    ov5640_dvp_capture #(.DVP_LITTLE_ENDIAN(1'b1)) u_capture(
      .pclk_i(cam_clk),.rst_n_i(cam_rst_n),.vsync_i(raw_vsync),.href_i(raw_href),
      .data_i(raw_data),.frame_valid_o(cap_frame),.line_valid_o(cap_line),
      .pixel_valid_o(cap_pixel_valid),.pixel_rgb565_o(cap_pixel));

    function automatic [15:0] frame_pixel(input integer frame_no,input integer x,input integer y);
        begin
            if(x==0&&y==0)frame_pixel=(frame_no==1)?16'h1234:16'habcd;
            else if(x==318&&y==238)frame_pixel=(frame_no==1)?16'hbeef:16'hdcba;
            else frame_pixel={frame_no[1:0],y[4:0],x[8:0]};
        end
    endfunction
    function automatic [11:0] quantize_rgb565(input [15:0] p);
        begin quantize_rgb565={p[15:12],p[10:7],p[4:1]};end
    endfunction
    function automatic [11:0] model_pixel(input [14:0] address);
        begin model_pixel=address^12'ha5a;end
    endfunction
    function automatic [7:0] expand_r(input [11:0] p);
        begin expand_r={p[11:8],p[11:8]};end
    endfunction
    function automatic [7:0] expand_g(input [11:0] p);
        begin expand_g={p[7:4],p[7:4]};end
    endfunction
    function automatic [7:0] expand_b(input [11:0] p);
        begin expand_b={p[3:0],p[3:0]};end
    endfunction
    task automatic check(input bit condition,input string message);
        if(!condition)begin $display("FAIL: %s",message);errors=errors+1;end
    endtask
    task automatic send_frame(input integer frame_no);
        integer x,y;
        begin
            @(negedge cam_clk);frame_valid=1;line_valid=0;pixel_valid=0;
            repeat(2)@(negedge cam_clk);
            for(y=0;y<240;y=y+1)begin
                line_valid=1;
                for(x=0;x<320;x=x+1)begin
                    pixel_valid=1;pixel_in=frame_pixel(frame_no,x,y);@(negedge cam_clk);
                end
                pixel_valid=0;line_valid=0;repeat(2)@(negedge cam_clk);
            end
            frame_valid=0;repeat(4)@(negedge cam_clk);
        end
    endtask
    task automatic pulse_video_boundary;
        begin @(negedge video_clk);video_frame_start=1;@(negedge video_clk);video_frame_start=0;
              repeat(3)@(posedge video_clk);#1;end
    endtask
    task automatic read_and_check(input [14:0] address,input [11:0] expected,input string label);
        begin read_offset=address;@(posedge video_clk);#1;check(read_pixel===expected,label);end
    endtask

    // VGA 使用同样的一拍同步读模型。
    always @(posedge video_clk) model_fb_pixel<=model_pixel(vga_offset);
    choice3_vga_640x480 u_vga(.pixel_clk_i(video_clk),.rst_n_i(video_rst_n),
      .framebuffer_pixel_i(model_fb_pixel),.framebuffer_valid_i(1'b1),
      .fault_mask_i(fault_mask),.roll_deg_i(16'sd0),
      .pitch_deg_i(16'sd0),.yaw_deg_i(16'sd0),.attitude_valid_i(1'b0),
      .demo_pattern_i(1'b0),
      .framebuffer_offset_o(vga_offset),
      .frame_start_o(vga_frame_start),.red_o(red),.green_o(green),.blue_o(blue),
      .hsync_o(hs),.vsync_o(vs),.de_o(de),.h_count_o(hc),.v_count_o(vc));
    choice3_tmds_encoder u_tmds(.pixel_clk_i(video_clk),.rst_n_i(video_rst_n),
      .de_i(tmds_de),.control_i(tmds_ctrl),.data_i(tmds_data),.code_o(tmds_code));

    integer n;
    reg [14:0] expected_addr;
    reg [11:0] expected_pixel;
    initial begin
        repeat(5)@(posedge cam_clk);cam_rst_n=1;
        repeat(5)@(posedge video_clk);video_rst_n=1;

        // OV5640 0x4300=0x6F 的低字节优先：34 后 12 必须发布 16'h1234。
        @(negedge cam_clk);raw_vsync=1;repeat(2)@(negedge cam_clk);raw_vsync=0;
        raw_href=1;raw_data=8'h34;@(negedge cam_clk);raw_data=8'h12;@(negedge cam_clk);
        #1;check(cap_pixel_valid&&cap_pixel==16'h1234,"OV5640 RGB565 小端字节序错误");
        raw_href=0;repeat(2)@(negedge cam_clk);

        send_frame(1);
        check(cam_commits==1&&dropped==0,"首个完整帧未恰好提交一次");
        // 显示域尚未确认首帧时，下一整帧必须跳过，不能覆写 pending bank。
        send_frame(9);
        check(cam_commits==1&&dropped==1,"未确认提交期间仍捕获/覆写了帧");
        repeat(5)@(posedge video_clk);#1;
        check(!display_valid&&display_bank==0,"显示端在帧边界前提前换 bank");
        pulse_video_boundary();
        check(display_valid&&display_bank==1&&video_commits==1,"显示端未在边界原子提交首帧");
        read_and_check(0,quantize_rgb565(16'h1234),"双缓冲首像素/RGB444量化错误");
        read_and_check(19199,quantize_rgb565(16'hbeef),"双缓冲末像素/地址边界错误");

        // 第二帧写入另一 bank；切换前旧首像素必须保持不变。
        fork
            send_frame(2);
            begin repeat(30000)@(posedge cam_clk);read_and_check(0,quantize_rgb565(16'h1234),
                  "摄像头写下一帧时污染当前显示 bank");end
        join
        check(cam_commits==2&&dropped==1&&display_bank==1,
              "第二帧提交握手或 bank 状态错误");
        read_and_check(0,quantize_rgb565(16'h1234),"显示边界前发生撕裂式切帧");
        pulse_video_boundary();
        check(display_bank==0&&video_commits==2,"第二帧未在边界切换");
        read_and_check(0,quantize_rgb565(16'habcd),"第二帧首像素错误");
        read_and_check(19199,quantize_rgb565(16'hdcba),"第二帧末像素错误");

        // 短帧必须丢弃，不能产生 commit。
        @(negedge cam_clk);frame_valid=1;line_valid=1;
        repeat(10)begin pixel_valid=1;pixel_in=16'hffff;@(negedge cam_clk);end
        pixel_valid=0;line_valid=0;repeat(2)@(negedge cam_clk);frame_valid=0;
        repeat(4)@(negedge cam_clk);
        check(cam_commits==2&&dropped==2,"短帧未被严格拒绝");

        // 完整检查一个 800x525 时序帧及逐拍数据/DE 对齐。
        de_count=0;hs_low_count=0;vs_low_count=0;
        for(n=0;n<800*525;n=n+1)begin
            @(posedge video_clk);#1;
            if(de)begin
                de_count=de_count+1;
                expected_addr=(({8'd0,u_vga.v_pipe_q[8:2]})<<7)+
                              (({8'd0,u_vga.v_pipe_q[8:2]})<<5)+
                              {7'd0,u_vga.h_pipe_q[9:2]};
                expected_pixel=model_pixel(expected_addr);
                check(red===expand_r(expected_pixel)&&green===expand_g(expected_pixel)&&
                      blue===expand_b(expected_pixel),"同步 RAM 像素与 DE/坐标未逐拍对齐");
                check(vga_offset<=19199,"活动区帧缓冲地址越界");
            end
            if(!hs)hs_low_count=hs_low_count+1;
            if(!vs)vs_low_count=vs_low_count+1;
        end
        check(de_count==640*480,"VGA 活动像素计数错误");
        check(hs_low_count==96*525,"VGA HSYNC 脉宽/周期错误");
        check(vs_low_count==2*800,"VGA VSYNC 脉宽/周期错误");

        // TMDS 控制码和值域。
        tmds_de=0;tmds_ctrl=0;@(posedge video_clk);#1;
        check(tmds_code==10'b1101010100,"TMDS control 00 错误");
        tmds_ctrl=3;@(posedge video_clk);#1;
        check(tmds_code==10'b1010101011,"TMDS control 11 错误");
        tmds_de=1;tmds_data=8'ha5;repeat(16)begin @(posedge video_clk);#1;
            check((^tmds_code)!==1'bx,"TMDS 数据编码出现 X/Z");
            check(u_tmds.disparity_q<=15&&u_tmds.disparity_q>=-15,"TMDS 游程差失控");
            tmds_data=tmds_data+8'h17;
        end

        if(errors==0)$display("PASS: tb_choice3_video_pipeline");
        else begin $display("FAIL: tb_choice3_video_pipeline errors=%0d",errors);$fatal(1);end
        $finish;
    end
    initial begin #30_000_000;$display("FAIL: video TB timeout");$fatal(1);end
endmodule
