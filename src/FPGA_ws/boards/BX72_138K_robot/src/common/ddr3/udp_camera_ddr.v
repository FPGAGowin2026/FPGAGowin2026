// 相机与 UDP 专用 DDR3 包装，保留本板 32 位物理接口及供应商 IP。
module udp_camera_ddr #(
    parameter integer FRAME_PIXELS=384000,
    parameter integer USE_CAMERA_FRAME_BAD=0
)(
    input wire clk,pll_lock,clk_200m,sys_rst_n,
    output wire init_calib_complete,pll_stop,
    input wire cam_clk,camera_vsync,camera_pixel_valid,
    input wire camera_frame_bad,
    input wire [15:0] camera_pixel,
    input wire stream_clk,stream_ready,
    output wire stream_frame_start,stream_valid,
    output wire [15:0] stream_pixel,
    output wire [31:0] stream_timestamp,
    output wire stream_busy,
    output wire capture_active,write_buffer,read_buffer,
    output wire [31:0] camera_edges_gray,dropped_frames_gray,skipped_frames_gray,
    output wire [319:0] debug_gray,
    output wire [95:0] reject_reason_gray,
    //DDR3   
    inout   [31:0]     ddr3_dq             ,   //DDR3 数据
    inout   [3:0]      ddr3_dqs_n          ,   //DDR3 dqs负
    inout   [3:0]      ddr3_dqs_p          ,   //DDR3 dqs正  
    output  [14:0]     ddr3_addr           ,   //DDR3 地址   
    output  [2:0]      ddr3_ba             ,   //DDR3 banck 选择
    output             ddr3_ras_n          ,   //DDR3 行选择
    output             ddr3_cas_n          ,   //DDR3 列选择
    output             ddr3_we_n           ,   //DDR3 读写选择
    output             ddr3_reset_n        ,   //DDR3 复位
    output  [0:0]      ddr3_ck_p           ,   //DDR3 时钟正
    output  [0:0]      ddr3_ck_n           ,   //DDR3 时钟负
    output  [0:0]      ddr3_cke            ,   //DDR3 时钟使能
    output  [0:0]      ddr3_cs_n           ,   //DDR3 片选
    output  [3:0]      ddr3_dm             ,   //DDR3_dm
    output  [0:0]      ddr3_odt                //DDR3_odt   

);
    wire    ui_clk;
    wire    ui_clk_sync_rst;
    wire app_rdy, app_wdf_rdy, app_rd_data_valid;
    wire [255:0] app_rd_data, app_wdf_data;
    wire [27:0] app_addr;
    wire app_en, app_wdf_wren, app_wdf_end;
    wire [2:0] app_cmd;
    // DDR 用户指南要求 rst_n 在参考 clk 域同步释放。
    (* ASYNC_REG="TRUE" *) reg [1:0] ddr_reset_sync;
    always @(posedge clk or negedge sys_rst_n)
        if(!sys_rst_n)ddr_reset_sync<=2'b00;else ddr_reset_sync<={ddr_reset_sync[0],1'b1};
    wire ddr_reset_n=ddr_reset_sync[1];

    camera_udp_framebuffer #(
        .FRAME_PIXELS(FRAME_PIXELS),
        .USE_CAMERA_FRAME_BAD(USE_CAMERA_FRAME_BAD)
    ) framebuffer(
        .ui_clk(ui_clk),.rst_n(ddr_reset_n && !ui_clk_sync_rst),.init_calib_complete(init_calib_complete),
        .app_rdy(app_rdy),.app_wdf_rdy(app_wdf_rdy),.app_rd_data_valid(app_rd_data_valid),
        .app_rd_data(app_rd_data),.app_addr(app_addr),.app_en(app_en),.app_wdf_wren(app_wdf_wren),
        .app_wdf_end(app_wdf_end),.app_cmd(app_cmd),.app_wdf_data(app_wdf_data),
        .cam_clk(cam_clk),.camera_vsync(camera_vsync),.camera_pixel_valid(camera_pixel_valid),
        .camera_frame_bad(camera_frame_bad),
        .camera_pixel(camera_pixel),.stream_clk(stream_clk),.stream_ready(stream_ready),
        .stream_frame_start(stream_frame_start),.stream_valid(stream_valid),.stream_pixel(stream_pixel),
        .stream_timestamp(stream_timestamp),.stream_busy(stream_busy),.capture_active(capture_active),.write_buffer(write_buffer),
        .read_buffer(read_buffer),.camera_edges_gray(camera_edges_gray),.debug_gray(debug_gray),
        .dropped_frames_gray(dropped_frames_gray),.skipped_frames_gray(skipped_frames_gray),
        .reject_reason_gray(reject_reason_gray));

    // Camera framebuffer handles independent UI ready signals and pairs writes only
    // when command and write-data channels can both accept the same beat.

    //ddr3的IP核
    DDR3_Memory_Interface_Top DDR3_Memory_Interface_Top(
        .clk(clk), //input clk
		.pll_stop(pll_stop), //output pll_stop
        .memory_clk(clk_200m), //input memory_clk
        .pll_lock(pll_lock), //input pll_lock
        .rst_n(ddr_reset_n), //input rst_n
        .cmd_ready(app_rdy), //output cmd_ready
        .cmd(app_cmd), //input [2:0] cmd
        .cmd_en(app_en), //input cmd_en
        .addr({1'b0, app_addr}), //input [27:0] addr
        .wr_data_rdy(app_wdf_rdy), //output wr_data_rdy
        .wr_data(app_wdf_data), //input [255:0] wr_data
        .wr_data_en(app_wdf_wren), //input wr_data_en
        .wr_data_end(app_wdf_end), //input wr_data_end
        .wr_data_mask(32'd0), // camera writes complete 256-bit beats
        .rd_data(app_rd_data), //output [255:0] rd_data
        .rd_data_valid(app_rd_data_valid), //output rd_data_valid
        .rd_data_end(), //output rd_data_end
        .sr_req(1'b0), //input sr_req
        .ref_req(1'b0), //input ref_req
        .sr_ack(), //output sr_ack
        .ref_ack(), //output ref_ack
        .init_calib_complete(init_calib_complete), //output init_calib_complete
        .clk_out(ui_clk), //output clk_out
        .ddr_rst(ui_clk_sync_rst), //output ddr_rst
        .burst(1'b1), //input burst
        .O_ddr_addr(ddr3_addr), //output [13:0] O_ddr_addr
        .O_ddr_ba(ddr3_ba), //output [2:0] O_ddr_ba
        .O_ddr_cs_n(ddr3_cs_n), //output O_ddr_cs_n
        .O_ddr_ras_n(ddr3_ras_n), //output O_ddr_ras_n
        .O_ddr_cas_n(ddr3_cas_n), //output O_ddr_cas_n
        .O_ddr_we_n(ddr3_we_n), //output O_ddr_we_n
        .O_ddr_clk(ddr3_ck_p), //output O_ddr_clk
        .O_ddr_clk_n(ddr3_ck_n), //output O_ddr_clk_n
        .O_ddr_cke(ddr3_cke), //output O_ddr_cke
        .O_ddr_odt(ddr3_odt), //output O_ddr_odt
        .O_ddr_reset_n(ddr3_reset_n), //output O_ddr_reset_n
        .O_ddr_dqm(ddr3_dm), //output [1:0] O_ddr_dqm
        .IO_ddr_dq(ddr3_dq), //inout [15:0] IO_ddr_dq
        .IO_ddr_dqs(ddr3_dqs_p), //inout [1:0] IO_ddr_dqs
        .IO_ddr_dqs_n(ddr3_dqs_n) //inout [1:0] IO_ddr_dqs_n
    );

endmodule 
