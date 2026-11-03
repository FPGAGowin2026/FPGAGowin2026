`timescale 1ns/1ps

// 初板轻量视频参考台：全帧走真实 1920x1080 RAW10 双像素/拍输入。
// 本文件仅供后续统一验证；RTL 改动线程不得自行运行仿真。
module c_tb_video;
    localparam integer RAW_WIDTH = 1920;
    localparam integer RAW_HEIGHT = 1080;
    localparam integer PAIRS_PER_LINE = RAW_WIDTH / 2;
    localparam integer IMAGE_WIDTH = 640;
    localparam integer IMAGE_HEIGHT = 360;
    localparam integer FRAME_PIXELS = IMAGE_WIDTH * IMAGE_HEIGHT;

    reg clk = 1'b0;
    reg reset_n = 1'b0;
    reg raw_frame_valid = 1'b0;
    reg raw_line_valid = 1'b0;
    reg [19:0] raw_pair = 20'd0;
    reg raw_vsync = 1'b1;
    reg raw_frame_bad = 1'b0;
    wire camera_vsync;
    wire camera_frame_bad;
    wire camera_pixel_valid;
    wire [15:0] camera_pixel;
    wire raw_overflow;

    integer test_phase = 0;
    integer sent_lines = 0;
    integer observed_pixels = 0;
    integer completed_frames = 0;
    integer active_phase = -1;
    reg camera_vsync_d = 1'b1;
    reg frame_active = 1'b0;

    always #11.4285 clk = ~clk; // 43.75MHz实际相机像素时钟。

    bx72_gowin_isp_pipeline #(
        .OUTPUT_WIDTH(IMAGE_WIDTH),
        .OUTPUT_HEIGHT(IMAGE_HEIGHT),
        .EOF_TIMEOUT(32)
    ) dut (
        .raw_clk(clk),
        .reset_n(reset_n),
        .raw_frame_valid(raw_frame_valid),
        .raw_line_valid(raw_line_valid),
        .raw_pair(raw_pair),
        .raw_vsync(raw_vsync),
        .raw_frame_bad(raw_frame_bad),
        .camera_vsync(camera_vsync),
        .camera_frame_bad(camera_frame_bad),
        .camera_pixel_valid(camera_pixel_valid),
        .camera_pixel(camera_pixel),
        .raw_overflow(raw_overflow)
    );

    function [9:0] texture_pixel;
        input integer x;
        input integer y;
        integer block_x;
        integer block_y;
        integer sample_index;
        integer block_base;
        integer sample_value;
        begin
            block_x = x / 3;
            block_y = y / 3;
            sample_index = (y % 3) * 3 + (x % 3);
            block_base = (block_x * 17 + block_y * 29) % 640;
            case (sample_index)
                0: sample_value = 0;
                1: sample_value = 16;
                2: sample_value = 32;
                3: sample_value = 48;
                4: sample_value = 64;
                5: sample_value = 80;
                6: sample_value = 96;
                7: sample_value = 112;
                default: sample_value = 128;
            endcase
            texture_pixel = block_base + sample_value;
        end
    endfunction

    function [7:0] oracle_gray;
        input integer output_index;
        integer out_x;
        integer out_y;
        integer dx;
        integer dy;
        integer box_sum;
        integer rounded_gray;
        begin
            out_x = output_index % IMAGE_WIDTH;
            out_y = output_index / IMAGE_WIDTH;
            box_sum = 0;
            for (dy = 0; dy < 3; dy = dy + 1)
                for (dx = 0; dx < 3; dx = dx + 1)
                    box_sum = box_sum + texture_pixel(out_x * 3 + dx, out_y * 3 + dy);

            // 10位 RAW10 九点和先精确除以36，按最接近整数舍入成8位亮度。
            rounded_gray = (box_sum + 18) / 36;
            if (rounded_gray > 255)
                oracle_gray = 8'hff;
            else
                oracle_gray = rounded_gray[7:0];
        end
    endfunction

    // 两个 RAW10 像素同拍输入，低10位对应先到的像素。
    task drive_frame;
        input integer line_count;
        input integer close_frame;
        input integer bad_at_end;
        integer y;
        integer pair_x;
        begin
            @(negedge clk);
            raw_vsync = 1'b0;
            raw_frame_valid = 1'b1;
            raw_line_valid = 1'b0;
            raw_frame_bad = 1'b0;
            raw_pair = 20'd0;
            @(negedge clk);

            for (y = 0; y < line_count; y = y + 1) begin
                raw_line_valid = 1'b1;
                for (pair_x = 0; pair_x < PAIRS_PER_LINE; pair_x = pair_x + 1) begin
                    raw_pair = {
                        texture_pixel(pair_x * 2 + 1, y),
                        texture_pixel(pair_x * 2, y)
                    };
                    @(negedge clk);
                end
                raw_line_valid = 1'b0;
                @(negedge clk);
            end
            sent_lines = line_count;

            if (close_frame != 0) begin
                raw_frame_valid = 1'b0;
                raw_frame_bad = (bad_at_end != 0);
                repeat (4) @(negedge clk);
                raw_vsync = 1'b1;
                repeat (48) @(negedge clk);
                raw_line_valid = 1'b0;
                raw_frame_valid = 1'b0;
                raw_frame_bad = 1'b0;
                raw_pair = 20'd0;
            end
        end
    endtask

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            camera_vsync_d <= 1'b1;
            frame_active <= 1'b0;
            active_phase <= -1;
            observed_pixels <= 0;
        end else begin
            camera_vsync_d <= camera_vsync;

            if (camera_vsync_d && !camera_vsync) begin
                frame_active <= 1'b1;
                active_phase <= test_phase;
                observed_pixels <= 0;
            end

            if (camera_pixel_valid) begin
                if (!frame_active)
                    $fatal(1, "像素有效信号出现在帧窗口之外");
                if (camera_pixel[7:0] !== 8'h80)
                    $fatal(1, "灰度 YUYV 色度不是中性值：%02x", camera_pixel[7:0]);
                if (active_phase == 0) begin
                    if (observed_pixels >= FRAME_PIXELS)
                        $fatal(1, "完整帧输出像素数超过640x360");
                    if (camera_pixel[15:8] !== oracle_gray(observed_pixels))
                        $fatal(1, "灰度不符 index=%0d got=%0d expected=%0d",
                               observed_pixels, camera_pixel[15:8], oracle_gray(observed_pixels));
                end
                observed_pixels <= observed_pixels + 1;
            end

            if (!camera_vsync_d && camera_vsync && frame_active) begin
                frame_active <= 1'b0;
                completed_frames <= completed_frames + 1;
                if (active_phase == 0) begin
                    if (observed_pixels != FRAME_PIXELS)
                        $fatal(1, "完整帧像素数错误：%0d，预期%0d", observed_pixels, FRAME_PIXELS);
                    if (camera_frame_bad)
                        $fatal(1, "完整输入帧被错误标记为坏帧");
                end else if (active_phase == 1) begin
                    if (!camera_frame_bad)
                        $fatal(1, "不完整/坏帧未被拒绝");
                    if (observed_pixels >= FRAME_PIXELS)
                        $fatal(1, "坏帧误输出为完整640x360帧");
                end
            end
        end
    end

    initial begin
        repeat (4) @(posedge clk);
        @(negedge clk);
        reset_n = 1'b1;
        repeat (4) @(posedge clk);

        // 完整 1080 行覆盖 360 个输出行；每个3x3窗口含九个不同 RAW10 值。
        test_phase = 0;
        drive_frame(RAW_HEIGHT, 1, 0);
        if (sent_lines != RAW_HEIGHT)
            $fatal(1, "测试源行数错误：%0d", sent_lines);
        wait (completed_frames == 1);

        // 三条有效源行只生成一条缩小图像行；EOF坏标志必须阻止该帧被发布。
        test_phase = 1;
        drive_frame(3, 1, 1);
        wait (completed_frames == 2);

        // 再次开始后在帧中途复位，复位后必须回到空闲坏帧状态且无残留像素。
        test_phase = 2;
        drive_frame(6, 0, 0);
        wait (frame_active && observed_pixels >= IMAGE_WIDTH * 2);
        @(negedge clk);
        reset_n = 1'b0;
        raw_frame_valid = 1'b0;
        raw_line_valid = 1'b0;
        raw_frame_bad = 1'b0;
        raw_vsync = 1'b1;
        raw_pair = 20'd0;
        repeat (3) @(posedge clk);
        if (camera_vsync !== 1'b1 || camera_pixel_valid !== 1'b0 || camera_frame_bad !== 1'b1)
            $fatal(1, "复位未中止帧并清除有效输出");
        @(negedge clk);
        reset_n = 1'b1;
        repeat (4) @(posedge clk);
        if (camera_vsync !== 1'b1 || camera_pixel_valid !== 1'b0)
            $fatal(1, "复位释放后仍残留帧像素");

        $display("PASS: 640x360 灰度YUYV完整帧、坏帧拒绝与复位中止");
        $finish;
    end

    initial begin
        #30000000;
        $fatal(1, "c_tb_video 超时");
    end
endmodule
