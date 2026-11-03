`timescale 1ns/1ps

// 完整RAW10 Bayer管线的颜色与几何测试源，同时覆盖两种像素对顺序。
// 本测试不能证明加密Gowin接收IP在实板上的像素对顺序或CFA相位。
module video_full_bayer_isp_tb #(parameter integer TEST_BAYER_PATTERN=1);
    localparam integer CFA_WIDTH = 24;
    localparam integer CFA_HEIGHT = 12;
    localparam integer OUT_WIDTH = 8;
    localparam integer OUT_HEIGHT = 4;

    reg clk = 1'b0;
    always #10 clk = ~clk;

    reg reset_n = 1'b0;
    reg frame_valid = 1'b0;
    reg line_valid = 1'b0;
    reg vsync = 1'b1;
    reg frame_bad = 1'b0;
    reg [19:0] pair_low_first = 20'd0;
    reg [19:0] pair_high_first = 20'd0;

    wire low_vsync, low_bad, low_valid, low_overflow;
    wire [15:0] low_pixel;
    wire high_vsync, high_bad, high_valid, high_overflow;
    wire [15:0] high_pixel;

    bx72_gowin_isp_pipeline #(
        .OUTPUT_WIDTH(OUT_WIDTH), .OUTPUT_HEIGHT(OUT_HEIGHT),
        .RAW_PAIR_LOW_FIRST(1), .BAYER_PATTERN(TEST_BAYER_PATTERN)
    ) dut_low_first (
        .raw_clk(clk), .reset_n(reset_n),
        .raw_frame_valid(frame_valid), .raw_line_valid(line_valid),
        .raw_pair(pair_low_first), .raw_vsync(vsync), .raw_frame_bad(frame_bad),
        .camera_vsync(low_vsync), .camera_frame_bad(low_bad),
        .camera_pixel_valid(low_valid), .camera_pixel(low_pixel),
        .raw_overflow(low_overflow)
    );

    bx72_gowin_isp_pipeline #(
        .OUTPUT_WIDTH(OUT_WIDTH), .OUTPUT_HEIGHT(OUT_HEIGHT),
        .RAW_PAIR_LOW_FIRST(0), .BAYER_PATTERN(TEST_BAYER_PATTERN)
    ) dut_high_first (
        .raw_clk(clk), .reset_n(reset_n),
        .raw_frame_valid(frame_valid), .raw_line_valid(line_valid),
        .raw_pair(pair_high_first), .raw_vsync(vsync), .raw_frame_bad(frame_bad),
        .camera_vsync(high_vsync), .camera_frame_bad(high_bad),
        .camera_pixel_valid(high_valid), .camera_pixel(high_pixel),
        .raw_overflow(high_overflow)
    );

    function [9:0] bggr_sample;
        input integer x;
        input integer y;
        begin
            if (TEST_BAYER_PATTERN == 1) begin
                // GBRG source: G B / R G, with independent R=800,G=400,B=200.
                if ((y & 1) == 0)
                    bggr_sample = ((x & 1) == 0) ? 10'd400 : 10'd200;
                else
                    bggr_sample = ((x & 1) == 0) ? 10'd800 : 10'd400;
            end else begin
                if ((y & 1) == 0)
                    bggr_sample = ((x & 1) == 0) ? 10'd200 : 10'd400;
                else
                    bggr_sample = ((x & 1) == 0) ? 10'd400 : 10'd800;
            end
        end
    endfunction

    integer low_count = 0;
    integer high_count = 0;
    integer low_index;
    integer high_index;
    reg low_active_seen = 1'b0;
    reg high_active_seen = 1'b0;
    reg low_frame_closed = 1'b0;
    reg high_frame_closed = 1'b0;
    always @(posedge clk) begin
        if (reset_n) begin
            if (!low_vsync) low_active_seen <= 1'b1;
            else if (low_active_seen) low_frame_closed <= 1'b1;
            if (!high_vsync) high_active_seen <= 1'b1;
            else if (high_active_seen) high_frame_closed <= 1'b1;
        end
        if (reset_n && low_valid) begin
            low_index = low_count;
            if ((low_index & 1) == 0 && low_pixel !== 16'ha163)
                $fatal(1, "low-first BGGR Y0/U mismatch at output word %0d: %04x", low_index, low_pixel);
            if ((low_index & 1) != 0 && low_pixel !== 16'ha1a0)
                $fatal(1, "low-first BGGR Y1/V mismatch at output word %0d: %04x", low_index, low_pixel);
            low_count = low_count + 1;
        end
        if (reset_n && high_valid) begin
            high_index = high_count;
            if ((high_index & 1) == 0 && high_pixel !== 16'ha163)
                $fatal(1, "high-first BGGR Y0/U mismatch at output word %0d: %04x", high_index, high_pixel);
            if ((high_index & 1) != 0 && high_pixel !== 16'ha1a0)
                $fatal(1, "high-first BGGR Y1/V mismatch at output word %0d: %04x", high_index, high_pixel);
            high_count = high_count + 1;
        end
    end

    task send_frame;
        integer y;
        integer pair_index;
        integer x;
        reg [9:0] p0;
        reg [9:0] p1;
        begin
            @(negedge clk);
            vsync = 1'b0;
            frame_valid = 1'b1;
            for (y = 0; y < CFA_HEIGHT; y = y + 1) begin
                @(negedge clk);
                line_valid = 1'b1;
                for (pair_index = 0; pair_index < CFA_WIDTH/2; pair_index = pair_index + 1) begin
                    x = pair_index * 2;
                    p0 = bggr_sample(x, y);
                    p1 = bggr_sample(x + 1, y);
                    pair_low_first = {p1, p0};
                    pair_high_first = {p0, p1};
                    @(negedge clk);
                end
                line_valid = 1'b0;
                @(negedge clk);
            end
            frame_valid = 1'b0;
            vsync = 1'b1;
            repeat (100) @(negedge clk);
        end
    endtask

    integer level, reference_code, oetf_error;
    real intensity, encoded;
    initial begin
        // Independent real-valued BT.709 source equation, not the RTL knots.
        for(level=0;level<256;level=level+1)begin
            intensity=level/255.0;
            encoded=intensity<0.018 ? 4.5*intensity : 1.099*(intensity**0.45)-0.099;
            reference_code=$rtoi(encoded*255.0+0.5);
            oetf_error=dut_low_first.video_oetf(level[7:0])-reference_code;
            if(oetf_error>2 || oetf_error < -2)$fatal(1,"OETF approximation %0d error=%0d",level,oetf_error);
        end
        repeat (5) @(negedge clk);
        reset_n = 1'b1;
        send_frame();
        wait (low_frame_closed && high_frame_closed);
        #1;
        if (low_bad || high_bad || low_overflow || high_overflow)
            $fatal(1, "valid BGGR frame was marked bad/overflow");
        if (low_count != OUT_WIDTH*OUT_HEIGHT || high_count != OUT_WIDTH*OUT_HEIGHT)
            $fatal(1, "wrong frame output count low=%0d high=%0d", low_count, high_count);
        $display("PASS: 3x3 BGGR Bayer RAW10 color, both RAW pair orders, 8x4 output");
        $finish;
    end
endmodule
