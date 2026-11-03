`timescale 1ns/1ps
// 双缓冲所有权：相机 -> DDR 写完 -> UDP 读完；帧不完整时不发布。
// 相机和读取端通过 toggle 邮箱交接，数据保持到应答，不逐位同步像素总线。
module camera_udp_framebuffer #(
    parameter integer FRAME_PIXELS=230400,
    parameter integer UI_CLK_HZ=50000000,
    parameter integer USE_CAMERA_FRAME_BAD=0
)(
    input wire ui_clk, rst_n, init_calib_complete,
    input wire app_rdy, app_wdf_rdy, app_rd_data_valid,
    input wire [255:0] app_rd_data,
    output wire [27:0] app_addr,
    output wire app_en, app_wdf_wren, app_wdf_end,
    output wire [2:0] app_cmd,
    output wire [255:0] app_wdf_data,
    input wire cam_clk, camera_vsync, camera_pixel_valid,
    input wire camera_frame_bad,
    input wire [15:0] camera_pixel,
    input wire stream_clk, stream_ready,
    output reg stream_frame_start,
    output wire stream_valid,
    output wire [15:0] stream_pixel,
    output reg [31:0] stream_timestamp,
    output wire stream_busy,
    output wire capture_active, write_buffer, read_buffer,
    output reg [31:0] camera_edges_gray,
    output wire [319:0] debug_gray,
    output reg [31:0] dropped_frames_gray,
    // 相机域统计：无缓冲或等待交接时跳过的帧，与无效帧分开。
    output reg [31:0] skipped_frames_gray,
    // 按捕获帧记录拒帧原因；原因可重叠。低到高依次为相机坏帧、像素数错误、写 FIFO 满。
    output reg [95:0] reject_reason_gray
);
    localparam integer FRAME_BEATS=FRAME_PIXELS/16;
    localparam integer COUNT_W=$clog2(FRAME_BEATS+1);
    localparam integer PIXEL_W=$clog2(FRAME_PIXELS+1);
    (* ASYNC_REG="TRUE" *) reg [1:0] cam_reset_sync,stream_reset_sync,ui_reset_sync;
    always @(posedge cam_clk or negedge rst_n)
        if(!rst_n)cam_reset_sync<=0;else cam_reset_sync<={cam_reset_sync[0],1'b1};
    always @(posedge stream_clk or negedge rst_n)
        if(!rst_n)stream_reset_sync<=0;else stream_reset_sync<={stream_reset_sync[0],1'b1};
    always @(posedge ui_clk or negedge rst_n)
        if(!rst_n)ui_reset_sync<=0;else ui_reset_sync<={ui_reset_sync[0],1'b1};
    wire cam_rst_n=cam_reset_sync[1],stream_rst_n=stream_reset_sync[1],ui_rst_n=ui_reset_sync[1];
    // 每帧必须为完整的256位DDR用户字；640x360彩色YUYV可整帧对齐。
    reg [1:0] free_buffers;
    reg writer_busy, writer_buffer, end_pending, end_good;
    reg [COUNT_W-1:0] write_beats, read_beats;
    reg [27:0] write_addr, read_addr;
    reg reader_busy, reader_buffer, pending_valid, pending_buffer;
    reg [31:0] write_timestamp, pending_timestamp, read_timestamp;
    reg [31:0] time90k, time_phase;
    wire [32:0] phase_next={1'b0,time_phase}+33'd90000;
    wire [32:0] phase_reduced=phase_next-UI_CLK_HZ;
    reg last_was_write;
    reg [9:0] read_outstanding;
    reg read_window_ok;
    reg [4:0] write_reset_count;
    reg [4:0] read_reset_count;
    reg [1:0] write_cooldown;
    reg abort_pending;
    reg start_toggle, end_toggle, camera_good_held;
    reg camera_wait_ack, camera_capturing, camera_bad, vs_prev;
    reg [PIXEL_W-1:0] camera_pixels;
    reg [31:0] camera_edges;
    wire [15:0] camera_edges_low_next=camera_edges[15:0]+16'd1;
    wire [15:0] camera_edges_high_next=camera_edges[31:16]+(camera_edges[15:0]==16'hffff);
    reg [31:0] skipped_frames;
    reg camera_bad_flag_seen, camera_fifo_full_seen, camera_pixel_overflow_seen;
    reg [31:0] rejected_bad_flag_frames;
    reg [31:0] rejected_pixel_count_frames;
    reg [31:0] rejected_fifo_full_frames;
    // 每个字节独立预计算 +1，再由旧值的低位全 1 条件选择进位。
    // 保持同拍计数和 Gray 发布，避免 32 位贯穿进位；不要求跳帧事件之间有空拍。
    wire [7:0] skipped_byte0_inc=skipped_frames[7:0]+8'd1;
    wire [7:0] skipped_byte1_inc=skipped_frames[15:8]+8'd1;
    wire [7:0] skipped_byte2_inc=skipped_frames[23:16]+8'd1;
    wire [7:0] skipped_byte3_inc=skipped_frames[31:24]+8'd1;
    wire [31:0] skipped_frames_next={
        (&skipped_frames[23:0]) ? skipped_byte3_inc : skipped_frames[31:24],
        (&skipped_frames[15:0]) ? skipped_byte2_inc : skipped_frames[23:16],
        (&skipped_frames[7:0]) ? skipped_byte1_inc : skipped_frames[15:8],
        skipped_byte0_inc};
    reg end_ack, read_request, read_consumed, consumed_ack;
    (* ASYNC_REG="TRUE" *) reg [1:0] start_sync,end_sync,consumed_sync;
    (* ASYNC_REG="TRUE" *) reg [1:0] grant_sync;
    (* ASYNC_REG="TRUE" *) reg [1:0] ack_sync,request_sync;
    reg start_seen, request_seen;
    wire can_grant=init_calib_complete && !writer_busy && !abort_pending &&
        write_reset_count==0 && free_buffers!=0;
    // 编号保持到 start_event 返回；只同步控制位，避免 valid/编号独立同步。
    reg grant_valid,grant_buffer;
    wire start_event=start_sync[1]!=start_seen;
    // 两个计数只从零递增到帧末，复用帧末比较作为请求边界。
    wire write_frame_complete=(write_beats==FRAME_BEATS);
    wire read_frame_complete=(read_beats==FRAME_BEATS);
    wire finish_read=reader_busy && consumed_sync[1]!=consumed_ack &&
        read_frame_complete && read_outstanding==0;
    wire abort_write=writer_busy && end_pending && !end_good;
    wire finish_write=writer_busy && end_pending && end_good && write_frame_complete;
    // FIFO Reset 由 UI 寄存器产生，避免原始 DDR 复位直接驱动 RAM 时钟使能。
    wire fifo_reset=write_reset_count!=0;
    wire read_fifo_reset=read_reset_count!=0;
    wire wr_full,wr_empty,rd_full,rd_empty;
    wire [10:0] wr_count,rd_count;
    wire [11:0] read_reserved_beats={1'b0,rd_count}+{2'b0,read_outstanding};
    wire [255:0] wr_data;
    wire camera_write=camera_capturing && camera_pixel_valid &&
        camera_pixels<FRAME_PIXELS && !wr_full;
    // 宽度转换 FIFO 的计数/FWFT 输出在 pop 后需要更新，留两拍避免使用旧计数。
    // 坏帧结束的当拍允许排出一个已缓存beat，只写本帧自己的buffer；本沿撤销
    // writer_busy，复位FIFO后再交还所有权，且坏帧绝不发布。避免end_good穿过
    // 仲裁再串入生成FIFO的读指针加法/块RAM CE链。
    wire wr_request=writer_busy && !fifo_reset && write_cooldown==0 && !wr_empty &&
        wr_count!=0 && !write_frame_complete && app_wdf_rdy;
    // 预先寄存读余量，切断outstanding/rd_count经仲裁到FIFO CE的组合链。
    // 读命令每拍最多增加一beat，窗口寄存与FIFO计数可合计滞后两拍。
    // 阈值246覆盖最多三次边沿的额外命令，最坏248beat，保留8beat背压空间。
    wire rd_request=reader_busy && !read_frame_complete && !rd_full && read_window_ok;
    wire choose_write=wr_request && (!rd_request || !last_was_write);
    wire write_fire=init_calib_complete && app_rdy && choose_write;
    wire read_fire=init_calib_complete && app_rdy && rd_request && !choose_write;
    assign app_en=write_fire || read_fire;
    assign app_wdf_wren=write_fire;
    assign app_wdf_end=write_fire;
    assign app_cmd=choose_write ? 3'd0 : 3'd1;
    assign app_addr=choose_write ? write_addr : read_addr;
    assign app_wdf_data=wr_data;
    assign capture_active=camera_capturing;
    assign write_buffer=writer_buffer;
    assign read_buffer=reader_buffer;

    always @(posedge cam_clk or negedge cam_rst_n) begin
        if(!cam_rst_n) begin
            grant_sync<=0;ack_sync<=0;
            start_toggle<=0;end_toggle<=0;camera_good_held<=0;
            camera_wait_ack<=0;camera_capturing<=0;camera_bad<=0;camera_pixels<=0;
            vs_prev<=0;camera_edges<=0;camera_edges_gray<=0;
            skipped_frames<=0;skipped_frames_gray<=0;
            camera_bad_flag_seen<=0;camera_fifo_full_seen<=0;camera_pixel_overflow_seen<=0;
            rejected_bad_flag_frames<=0;rejected_pixel_count_frames<=0;
            rejected_fifo_full_frames<=0;reject_reason_gray<=0;
        end else begin
            grant_sync<={grant_sync[0],grant_valid};
            ack_sync<={ack_sync[0],end_ack};vs_prev<=camera_vsync;
            // 诊断 Gray 落后一拍，避免 32 位加法与 Gray 转换串联；不参与帧交接。
            camera_edges_gray<=camera_edges^(camera_edges>>1);
            if(camera_wait_ack && ack_sync[1]==end_toggle)camera_wait_ack<=0;
            if(camera_vsync && !vs_prev) begin
                camera_edges<={camera_edges_high_next,camera_edges_low_next};
                if(camera_capturing) begin
                    camera_capturing<=0;camera_wait_ack<=1;
                    camera_good_held<=!camera_bad && camera_pixels==FRAME_PIXELS &&
                        (!(USE_CAMERA_FRAME_BAD != 0) || !camera_frame_bad);
                    end_toggle<=~end_toggle;
                    // 原因计数在相机帧结束时采样；多个条件同时成立时各自计数。
                    if(USE_CAMERA_FRAME_BAD != 0 &&
                       (camera_bad_flag_seen || camera_frame_bad))
                        rejected_bad_flag_frames<=rejected_bad_flag_frames+1'b1;
                    if(camera_pixels!=FRAME_PIXELS || camera_pixel_overflow_seen ||
                       (camera_pixel_valid && camera_pixels>=FRAME_PIXELS))
                        rejected_pixel_count_frames<=rejected_pixel_count_frames+1'b1;
                    if(camera_fifo_full_seen || (camera_pixel_valid && wr_full))
                        rejected_fifo_full_frames<=rejected_fifo_full_frames+1'b1;
                end
            end
            if(!camera_vsync && vs_prev && !camera_wait_ack && !camera_capturing && grant_sync[1]) begin
                start_toggle<=~start_toggle;
                camera_capturing<=1;camera_bad<=0;camera_pixels<=0;
                camera_bad_flag_seen<=0;camera_fifo_full_seen<=0;camera_pixel_overflow_seen<=0;
            end
            if(!camera_vsync && vs_prev && !camera_capturing &&
               (camera_wait_ack || !grant_sync[1])) begin
                skipped_frames<=skipped_frames_next;
                skipped_frames_gray<=skipped_frames_next^(skipped_frames_next>>1);
            end
            if(camera_capturing && camera_pixel_valid) begin
                if(camera_pixels<FRAME_PIXELS)camera_pixels<=camera_pixels+1'b1;
                else begin camera_bad<=1;camera_pixel_overflow_seen<=1;end
                if(wr_full)begin camera_bad<=1;camera_fifo_full_seen<=1;end
            end
            if(camera_capturing && USE_CAMERA_FRAME_BAD != 0 && camera_frame_bad) begin
                camera_bad<=1'b1;
                camera_bad_flag_seen<=1'b1;
            end
            reject_reason_gray<={
                rejected_fifo_full_frames^(rejected_fifo_full_frames>>1),
                rejected_pixel_count_frames^(rejected_pixel_count_frames>>1),
                rejected_bad_flag_frames^(rejected_bad_flag_frames>>1)};
        end
    end

    reg stream_active;
    assign stream_busy=stream_active;
    reg [PIXEL_W-1:0] stream_pixels;
    reg [15:0] stream_stage_pixel;
    reg stream_stage_valid;
    wire [15:0] stream_fifo_pixel;
    // 首字直通 FIFO 提前填入一级寄存器；背压时保持数据，末像素不预取下一帧。
    wire stream_last=(stream_pixels==FRAME_PIXELS-1);
    wire stream_prefetch=stream_active && !stream_frame_start && !rd_empty &&
        (!stream_stage_valid || (stream_ready && !stream_last));
    assign stream_pixel=stream_stage_pixel;
    assign stream_valid=stream_active && !stream_frame_start && stream_stage_valid && stream_ready;
    always @(posedge stream_clk or negedge stream_rst_n) begin
        if(!stream_rst_n) begin
            request_sync<=0;request_seen<=0;read_consumed<=0;
            stream_active<=0;stream_pixels<=0;stream_frame_start<=0;stream_timestamp<=0;
            stream_stage_pixel<=0;stream_stage_valid<=0;
        end else begin
            request_sync<={request_sync[0],read_request};stream_frame_start<=0;
            if(request_sync[1]!=request_seen) begin
                request_seen<=request_sync[1];stream_active<=1;stream_pixels<=0;
                stream_frame_start<=1;stream_timestamp<=read_timestamp;
            end
            if(stream_prefetch) begin
                stream_stage_pixel<=stream_fifo_pixel;stream_stage_valid<=1;
            end else if(stream_valid)stream_stage_valid<=0;
            if(stream_valid) begin
                if(stream_last) begin
                    stream_active<=0;read_consumed<=~read_consumed;
                end else stream_pixels<=stream_pixels+1'b1;
            end
        end
    end

    reg [31:0] debug_count[0:9];
    reg [31:0] debug_gray_q[0:9];
    reg [31:0] dropped_frames;
    genvar g;
    generate for(g=0;g<10;g=g+1) begin: debug_output
        assign debug_gray[g*32+:32]=debug_gray_q[g];
    end endgenerate
    always @(posedge ui_clk or negedge ui_rst_n) begin
        if(!ui_rst_n) begin
            start_sync<=0;end_sync<=0;consumed_sync<=0;start_seen<=0;end_ack<=0;consumed_ack<=0;
            free_buffers<=2'b11;writer_busy<=0;writer_buffer<=0;end_pending<=0;end_good<=0;
            grant_valid<=0;grant_buffer<=0;
            write_beats<=0;write_addr<=0;write_timestamp<=0;
            reader_busy<=0;reader_buffer<=0;pending_valid<=0;pending_buffer<=0;
            read_beats<=0;read_addr<=0;read_timestamp<=0;pending_timestamp<=0;read_request<=0;
            write_reset_count<=16;read_reset_count<=16;write_cooldown<=0;abort_pending<=0;read_outstanding<=0;last_was_write<=0;
            read_window_ok<=1'b0;
            time90k<=0;time_phase<=0;dropped_frames<=0;dropped_frames_gray<=0;
            for(integer i=0;i<10;i=i+1)begin debug_count[i]<=0;debug_gray_q[i]<=0;end
        end else begin
            start_sync<={start_sync[0],start_toggle};end_sync<={end_sync[0],end_toggle};
            consumed_sync<={consumed_sync[0],read_consumed};
            read_window_ok<=read_reserved_beats<12'd246;
            if(start_event)grant_valid<=0;
            else if(!grant_valid && can_grant)begin
                grant_valid<=1;grant_buffer<=!free_buffers[0];
            end
            if(phase_next>=UI_CLK_HZ)begin time_phase<=phase_reduced[31:0];time90k<=time90k+1'b1;end
            else time_phase<=phase_next[31:0];
            if(write_reset_count!=0)write_reset_count<=write_reset_count-1'b1;
            if(read_reset_count!=0)read_reset_count<=read_reset_count-1'b1;
            if(write_cooldown!=0)write_cooldown<=write_cooldown-1'b1;
            if(abort_pending && write_reset_count==1)begin abort_pending<=0;end_ack<=end_sync[1];end
            free_buffers<=(free_buffers |
                (finish_read ? (2'b01<<reader_buffer) : 2'b00) |
                (abort_write ? (2'b01<<writer_buffer) : 2'b00)) &
                ~(start_event ? (2'b01<<grant_buffer) : 2'b00);
            if(start_event)begin
                // 编号由UI发放且保持到start应答，直接使用本域原件；不绕经相机域返回。
                start_seen<=start_sync[1];writer_busy<=1;writer_buffer<=grant_buffer;
                write_beats<=0;write_addr<=grant_buffer ? FRAME_PIXELS/2 : 0;
                end_pending<=0;write_timestamp<=time90k;
            end
            if(writer_busy && end_sync[1]!=end_ack)begin end_pending<=1;end_good<=camera_good_held;end
            if(write_fire)begin write_beats<=write_beats+1'b1;write_addr<=write_addr+28'd8;last_was_write<=1;write_cooldown<=2;end
            if(read_fire)begin read_beats<=read_beats+1'b1;read_addr<=read_addr+28'd8;last_was_write<=0;end
            case({read_fire,app_rd_data_valid})
                2'b10:read_outstanding<=read_outstanding+1'b1;
                2'b01:read_outstanding<=read_outstanding-1'b1;
                default:begin end
            endcase
            if(finish_write)begin
                writer_busy<=0;end_pending<=0;end_ack<=end_sync[1];
                pending_valid<=1;pending_buffer<=writer_buffer;pending_timestamp<=write_timestamp;
            end
            if(abort_write)begin
                writer_busy<=0;end_pending<=0;write_reset_count<=16;abort_pending<=1;
                dropped_frames<=dropped_frames+1'b1;
            end
            if(!reader_busy && pending_valid)begin
                reader_busy<=1;reader_buffer<=pending_buffer;pending_valid<=0;
                read_addr<=pending_buffer ? FRAME_PIXELS/2 : 0;read_beats<=0;
                read_timestamp<=pending_timestamp;read_request<=~read_request;
            end
            if(finish_read)begin reader_busy<=0;consumed_ack<=consumed_sync[1];end
            debug_count[0]<=debug_count[0]+1'b1;
            if(app_rdy)debug_count[1]<=debug_count[1]+1'b1;
            if(write_fire)debug_count[2]<=debug_count[2]+1'b1;
            if(read_fire)debug_count[4]<=debug_count[4]+1'b1;
            if(app_rd_data_valid)debug_count[5]<=debug_count[5]+1'b1;
            if(!app_en)debug_count[6]<=debug_count[6]+1'b1;
            if(writer_busy && end_pending)debug_count[7]<=debug_count[7]+1'b1;
            if(!writer_busy && !reader_busy)debug_count[8]<=debug_count[8]+1'b1;
            debug_count[9]<=dropped_frames;
            for(integer i=0;i<10;i=i+1)debug_gray_q[i]<=debug_count[i]^(debug_count[i]>>1);
            dropped_frames_gray<=dropped_frames^(dropped_frames>>1);
        end
    end

    wr_data_fifo camera_fifo(.Data(camera_pixel),.Reset(fifo_reset),.WrClk(cam_clk),.RdClk(ui_clk),
        .WrEn(camera_write),.RdEn(write_fire),.Wnum(),.Rnum(wr_count),.Almost_Empty(),.Almost_Full(),
        .Q(wr_data),.Empty(wr_empty),.Full(wr_full));
    rd_data_fifo stream_fifo(.Data(app_rd_data),.Reset(read_fifo_reset),.WrClk(ui_clk),.RdClk(stream_clk),
        .WrEn(app_rd_data_valid),.RdEn(stream_prefetch),.Wnum(rd_count),.Rnum(),.Almost_Empty(),.Almost_Full(),
        .Q(stream_fifo_pixel),.Empty(rd_empty),.Full(rd_full));
endmodule
