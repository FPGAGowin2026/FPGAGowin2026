`timescale 1ns/1ps
// 用真实相机输入填满读预算，再解除背压；不强制 DUT 内部状态。
module nav_full_ddr_window_tb;
    localparam integer PIXELS=8192, BEATS=PIXELS/16;
    reg ui=0,cam=0,stream=0,rst_n=0,cal=0,vs=0,pv=0,ready=0;
    always #10 ui=~ui;
    always #15 cam=~cam;
    always #4 stream=~stream;
    reg [15:0] pix=0;
    wire [27:0] addr;
    wire en,we,wlast,start,valid,busy,capturing;
    wire [2:0] cmd;
    wire [255:0] wd;
    wire [15:0] pixel;
    reg rdy=0,rv=0;
    reg [255:0] rdata=0;
    camera_udp_framebuffer #(.FRAME_PIXELS(PIXELS),.UI_CLK_HZ(50000000)) fb(
        .ui_clk(ui),.rst_n(rst_n),.init_calib_complete(cal),
        .app_rdy(rdy),.app_wdf_rdy(1'b1),.app_rd_data_valid(rv),.app_rd_data(rdata),
        .app_addr(addr),.app_en(en),.app_wdf_wren(we),.app_wdf_end(wlast),.app_cmd(cmd),.app_wdf_data(wd),
        .cam_clk(cam),.camera_vsync(vs),.camera_pixel_valid(pv),.camera_frame_bad(1'b0),.camera_pixel(pix),
        .stream_clk(stream),.stream_ready(ready),.stream_frame_start(start),.stream_valid(valid),
        .stream_pixel(pixel),.stream_busy(busy),.capture_active(capturing));
    reg [255:0] memory[0:BEATS*2-1];
    reg [27:0] response_addr[0:BEATS-1];
    integer head=0,tail=0,ticks=0,writes=0,reads=0,responses=0;
    integer received=0,frames=0,max_reserved=0,stalled_reads=0;
    always @(negedge ui) begin
        ticks=ticks+1;
        rdy=rst_n && ticks%7!=0;
    end
    always @(posedge ui) if(rst_n) begin
        rv<=0;
        // 延迟返回并施加 DDR 响应背压，覆盖 outstanding 与 FIFO 占用的转移。
        if(head<tail && ticks%3==0) begin
            rdata<=memory[response_addr[head]/8];rv<=1;
            head=head+1;responses=responses+1;
        end
        if(en) begin
            if(!rdy || addr%8!=0 || addr>=PIXELS)
                $fatal(1,"invalid DDR command/address");
            if(cmd==0) begin
                if(!we || !wlast || writes>=BEATS) $fatal(1,"write beat count");
                memory[addr/8]<=wd;writes=writes+1;
            end else begin
                if(reads>=BEATS) $fatal(1,"read beyond frame");
                response_addr[tail]=addr;tail=tail+1;reads=reads+1;
            end
        end
        if(reads-received/16>max_reserved) max_reserved=reads-received/16;
        if(reads-received/16>248 || (rv && fb.rd_full))
            $fatal(1,"DDR read reservation overflow: %0d",reads-received/16);
    end
    always @(posedge stream) if(rst_n) begin
        if(start) frames=frames+1;
        if(valid) begin
            if(!ready || pixel!=(16'h2000+received))
                $fatal(1,"pixel sequence at %0d: %h",received,pixel);
            received=received+1;
        end
    end
    initial begin
        repeat(8)@(negedge ui);rst_n=1;cal=1;
        wait(fb.grant_sync[1] && !fb.camera_wait_ack);
        @(negedge cam);vs=1;repeat(5)@(negedge cam);vs=0;
        wait(capturing);repeat(2)@(negedge cam);
        for(integer i=0;i<PIXELS;i=i+1)begin @(negedge cam);pv=1;pix=16'h2000+i;end
        @(negedge cam);pv=0;repeat(3)@(negedge cam);vs=1;
        wait(reads>=240);
        repeat(2000)@(negedge ui);
        stalled_reads=reads;
        if(stalled_reads>248 || stalled_reads<240 || received!=0 || frames!=1)
            $fatal(1,"read budget not reached safely: reads=%0d",stalled_reads);
        repeat(2000)@(negedge ui);
        if(reads!=stalled_reads) $fatal(1,"read commands continued during full backpressure");
        ready=1;
        wait(received==PIXELS);wait(!fb.reader_busy);
        repeat(50)@(negedge ui);
        if(writes!=BEATS || reads!=BEATS || responses!=BEATS || frames!=1 || fb.dropped_frames!=0)
            $fatal(1,"frame count after resume");
        $display("PASS: DDR read-window backpressure, %0d pixels, %0d exact beats, max reserved=%0d",received,reads,max_reserved);
        $finish;
    end
    initial begin #2000000;$fatal(1,"timeout writes=%0d reads=%0d received=%0d",writes,reads,received);end
endmodule
