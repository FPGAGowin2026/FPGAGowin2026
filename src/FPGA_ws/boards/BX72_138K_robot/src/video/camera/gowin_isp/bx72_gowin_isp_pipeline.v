`timescale 1ns/1ps
// OV5647 1920x1080 RAW10 按3x3 CFA块降采样为640x360彩色YUYV422。
// 依据1080p初始化表采用BGGR；pair顺序及CFA相位保留参数，最终须用色卡确认。
// 源完整性由同一raw_clk域的入口guard检查，本模块固定输出YUYV且不跨时钟域。
module bx72_gowin_isp_pipeline #(
    parameter integer OUTPUT_WIDTH = 640,
    parameter integer OUTPUT_HEIGHT = 360,
    parameter integer EOF_TIMEOUT = 32,
    parameter [9:0] WB_R_GAIN_UQ2_8 = 10'd256,
    parameter [9:0] WB_G_GAIN_UQ2_8 = 10'd256,
    parameter [9:0] WB_B_GAIN_UQ2_8 = 10'd256,
    parameter integer DISPLAY_OETF_ENABLE = 1,
    parameter integer RAW_PAIR_LOW_FIRST = 1,
    // 0=BGGR, 1=GBRG, 2=GRBG, 3=RGGB。
    parameter integer BAYER_PATTERN = 0
)(
    input wire raw_clk,
    input wire reset_n,
    input wire raw_frame_valid,
    input wire raw_line_valid,
    input wire [19:0] raw_pair,
    input wire raw_vsync,
    input wire raw_frame_bad,
    output reg camera_vsync,
    output reg camera_frame_bad,
    output reg camera_pixel_valid,
    output reg [15:0] camera_pixel,
    output wire raw_overflow
);
    localparam integer FRAME_PIXELS = OUTPUT_WIDTH * OUTPUT_HEIGHT;
    localparam integer COUNT_WIDTH = (FRAME_PIXELS > 1) ? $clog2(FRAME_PIXELS + 1) : 1;
    localparam integer GROUP_INDEX_WIDTH = (OUTPUT_WIDTH > 1) ? $clog2(OUTPUT_WIDTH + 1) : 1;
    localparam [COUNT_WIDTH-1:0] FRAME_PIXELS_COUNT = FRAME_PIXELS;
    localparam [GROUP_INDEX_WIDTH-1:0] OUTPUT_WIDTH_COUNT = OUTPUT_WIDTH;

    reg fv_d, lv_d, vs_d;
    wire frame_start = raw_frame_valid && !fv_d;
    wire line_start = raw_line_valid && !lv_d;
    wire line_end = !raw_line_valid && lv_d;
    wire frame_end = raw_vsync && !vs_d;
    wire [9:0] raw_first = RAW_PAIR_LOW_FIRST ? raw_pair[9:0] : raw_pair[19:10];
    wire [9:0] raw_second = RAW_PAIR_LOW_FIRST ? raw_pair[19:10] : raw_pair[9:0];

    // 三像素水平块在双像素输入上跨两拍完成；分别累计x偶数和x奇数CFA样本。
    reg [1:0] pair_phase;
    reg [9:0] horizontal_even_partial, horizontal_odd_partial;
    reg [GROUP_INDEX_WIDTH-1:0] group_index;
    reg [1:0] line_mod3;
    reg input_line_parity;
    wire [1:0] pair_phase_now = line_start ? 2'd0 : pair_phase;
    wire [GROUP_INDEX_WIDTH-1:0] group_index_now =
        line_start ? {GROUP_INDEX_WIDTH{1'b0}} : group_index;
    wire [1:0] line_mod3_now = frame_start ? 2'd0 : line_mod3;
    wire input_line_parity_now = frame_start ? 1'b0 : input_line_parity;
    wire [10:0] horizontal_even_sum = (pair_phase_now == 2'd2) ?
        ({1'b0, horizontal_even_partial} + {1'b0, raw_first}) :
        (pair_phase_now == 2'd1) ? {1'b0, raw_first} : 11'd0;
    wire [10:0] horizontal_odd_sum = (pair_phase_now == 2'd2) ?
        // 当前pair第二像素是下一3像素块的首个奇数列样本，留给下个块。
        {1'b0, horizontal_odd_partial} :
        (pair_phase_now == 2'd1) ?
        ({1'b0, horizontal_odd_partial} + {1'b0, raw_second}) : 11'd0;
    wire group_complete = raw_line_valid && (pair_phase_now != 2'd0);

    // 两条行缓存各存一行每个3像素块的x偶/奇样本和，共28,160 bit。
    // 第三行到达时，三行同位置的CFA样本被分到R/G/B并分别归一化。
    reg [21:0] line0_sum_mem [0:OUTPUT_WIDTH-1];
    reg [21:0] line1_sum_mem [0:OUTPUT_WIDTH-1];
    reg [21:0] line0_sum_read_q, line1_sum_read_q;
    reg [10:0] row2_even_sum_q, row2_odd_sum_q;
    reg block_x_odd_q, block_y_odd_q;
    reg vertical_sum_valid_q;
    reg [9:0] raw_r_q, raw_g_q, raw_b_q;
    reg raw_rgb_valid_q;
    reg [7:0] rgb_r_q, rgb_g_q, rgb_b_q;
    reg color_rgb_valid_q;

    wire [11:0] row0_even = {1'b0, line0_sum_read_q[10:0]};
    wire [11:0] row0_odd  = {1'b0, line0_sum_read_q[21:11]};
    wire [11:0] row1_even = {1'b0, line1_sum_read_q[10:0]};
    wire [11:0] row1_odd  = {1'b0, line1_sum_read_q[21:11]};
    wire [11:0] row2_even = {1'b0, row2_even_sum_q};
    wire [11:0] row2_odd  = {1'b0, row2_odd_sum_q};
    wire [11:0] even_rows_even_col = block_y_odd_q ? row1_even : row0_even + row2_even;
    wire [11:0] even_rows_odd_col  = block_y_odd_q ? row1_odd  : row0_odd  + row2_odd;
    wire [11:0] odd_rows_even_col  = block_y_odd_q ? row0_even + row2_even : row1_even;
    wire [11:0] odd_rows_odd_col   = block_y_odd_q ? row0_odd  + row2_odd  : row1_odd;
    localparam integer RED_Y_PARITY = (BAYER_PATTERN == 0 || BAYER_PATTERN == 1) ? 1 : 0;
    localparam integer RED_X_PARITY = (BAYER_PATTERN == 0 || BAYER_PATTERN == 2) ? 1 : 0;
    localparam integer BLUE_Y_PARITY = 1 - RED_Y_PARITY;
    localparam integer BLUE_X_PARITY = 1 - RED_X_PARITY;
    wire [11:0] red_even_y = RED_X_PARITY ? even_rows_odd_col : even_rows_even_col;
    wire [11:0] red_odd_y = RED_X_PARITY ? odd_rows_odd_col : odd_rows_even_col;
    wire [11:0] blue_even_y = BLUE_X_PARITY ? even_rows_odd_col : even_rows_even_col;
    wire [11:0] blue_odd_y = BLUE_X_PARITY ? odd_rows_odd_col : odd_rows_even_col;
    wire [11:0] red_sum = RED_Y_PARITY ? red_odd_y : red_even_y;
    wire [11:0] blue_sum = BLUE_Y_PARITY ? blue_odd_y : blue_even_y;
    wire green_sites_off_diagonal = (BAYER_PATTERN == 0) || (BAYER_PATTERN == 3);
    wire green_sum_uses_evenrow_oddcol = green_sites_off_diagonal;
    wire [12:0] green_sum = green_sum_uses_evenrow_oddcol ?
        ({1'b0, even_rows_odd_col} + {1'b0, odd_rows_even_col}) :
        ({1'b0, even_rows_even_col} + {1'b0, odd_rows_odd_col});
    wire [1:0] x_even_count = block_x_odd_q ? 2'd1 : 2'd2;
    wire [1:0] x_odd_count  = block_x_odd_q ? 2'd2 : 2'd1;
    wire [1:0] y_even_count = block_y_odd_q ? 2'd1 : 2'd2;
    wire [1:0] y_odd_count  = block_y_odd_q ? 2'd2 : 2'd1;
    wire [1:0] red_y_count = RED_Y_PARITY ? y_odd_count : y_even_count;
    wire [1:0] red_x_count = RED_X_PARITY ? x_odd_count : x_even_count;
    wire [1:0] blue_y_count = BLUE_Y_PARITY ? y_odd_count : y_even_count;
    wire [1:0] blue_x_count = BLUE_X_PARITY ? x_odd_count : x_even_count;
    wire [3:0] red_count = {2'b0, red_y_count} * {2'b0, red_x_count};
    wire [3:0] blue_count = {2'b0, blue_y_count} * {2'b0, blue_x_count};
    wire [3:0] green_count_evenrow_oddcol =
        ({2'b0, y_even_count} * {2'b0, x_odd_count}) +
        ({2'b0, y_odd_count} * {2'b0, x_even_count});
    wire [3:0] green_count_evenrow_evencol =
        ({2'b0, y_even_count} * {2'b0, x_even_count}) +
        ({2'b0, y_odd_count} * {2'b0, x_odd_count});
    wire green_count_four = green_sum_uses_evenrow_oddcol ?
        (green_count_evenrow_oddcol == 4) : (green_count_evenrow_evencol == 4);

    function [9:0] average_rb;
        input [11:0] total;
        input [3:0] count;
        reg [13:0] rounded;
        begin
            rounded = {2'b0, total};
            case (count)
                4'd1: average_rb = total[9:0];
                4'd2: begin rounded = {2'b0, total} + 14'd1; average_rb = rounded[10:1]; end
                4'd4: begin rounded = {2'b0, total} + 14'd2; average_rb = rounded[11:2]; end
                default: average_rb = 10'd0;
            endcase
        end
    endfunction

    wire [14:0] green_four_rounded = {2'b0, green_sum} + 15'd2;
    wire [9:0] green_average_four = green_four_rounded[11:2];
    wire [21:0] green_five_product = ({9'd0, green_sum} + 22'd2) * 22'd205;
    wire [10:0] green_average_five = green_five_product[20:10];
    wire [9:0] green_average = green_count_four ? green_average_four :
        (green_average_five > 11'd1023 ? 10'd1023 : green_average_five[9:0]);

    wire [19:0] red_gain_product = {10'd0, raw_r_q} * {10'd0, WB_R_GAIN_UQ2_8};
    wire [19:0] green_gain_product = {10'd0, raw_g_q} * {10'd0, WB_G_GAIN_UQ2_8};
    wire [19:0] blue_gain_product = {10'd0, raw_b_q} * {10'd0, WB_B_GAIN_UQ2_8};
    wire [19:0] red_gain_rounded = red_gain_product + 20'd512;
    wire [19:0] green_gain_rounded = green_gain_product + 20'd512;
    wire [19:0] blue_gain_rounded = blue_gain_product + 20'd512;
    wire [9:0] red_gain_raw = red_gain_rounded[19:10];
    wire [9:0] green_gain_raw = green_gain_rounded[19:10];
    wire [9:0] blue_gain_raw = blue_gain_rounded[19:10];

    // ITU-R BT.709-6 OETF, approximated with linear segments in 8-bit RGB.
    // Source: https://www.itu.int/rec/R-REC-BT.709/en
    // This maps linear sensor intensity before the R'G'B' -> Y'CbCr matrix.
    function [7:0] video_oetf;
        input [7:0] linear_value;
        reg [7:0] low_value;
        reg [4:0] rise;
        reg [8:0] mapped, interpolation;
        begin
            low_value=0; rise=0; interpolation=0;
            if (linear_value < 4)
                mapped=({1'b0,linear_value}*9'd9+9'd1)>>1;
            else if (linear_value < 8)
                mapped=9'd18+(({1'b0,linear_value}-9'd4)<<2);
            else if (linear_value < 16)
                mapped=9'd34+(({1'b0,linear_value}-9'd8)*9'd21+9'd4)/9'd8;
            else begin
                case (linear_value[7:4])
                4'd1: begin low_value=8'd55; rise=5'd30; end
                4'd2: begin low_value=8'd85; rise=5'd22; end
                4'd3: begin low_value=8'd107; rise=5'd18; end
                4'd4: begin low_value=8'd125; rise=5'd16; end
                4'd5: begin low_value=8'd141; rise=5'd14; end
                4'd6: begin low_value=8'd155; rise=5'd13; end
                4'd7: begin low_value=8'd168; rise=5'd12; end
                4'd8: begin low_value=8'd180; rise=5'd11; end
                4'd9: begin low_value=8'd191; rise=5'd11; end
                4'd10: begin low_value=8'd202; rise=5'd10; end
                4'd11: begin low_value=8'd212; rise=5'd9; end
                4'd12: begin low_value=8'd221; rise=5'd9; end
                4'd13: begin low_value=8'd230; rise=5'd9; end
                4'd14: begin low_value=8'd239; rise=5'd8; end
                default: begin low_value=8'd247; rise=5'd8; end
                endcase
                interpolation=rise*linear_value[3:0]+9'd8;
                mapped={1'b0,low_value}+(interpolation>>4);
            end
            video_oetf=mapped[7:0];
        end
    endfunction
    function [7:0] rgb_to_y;
        input [7:0] red;
        input [7:0] green;
        input [7:0] blue;
        reg [17:0] r_term, g_term, b_term, weighted;
        reg [9:0] value;
        begin
            r_term = red * 18'd47;
            g_term = green * 18'd157;
            b_term = blue * 18'd16;
            weighted = r_term + g_term + b_term + 18'd128;
            value = (weighted >> 8) + 10'd16;
            if (value < 10'd16) rgb_to_y = 8'd16;
            else if (value > 10'd235) rgb_to_y = 8'd235;
            else rgb_to_y = value[7:0];
        end
    endfunction

    function [7:0] rgb_pair_to_u;
        input [8:0] red_sum_in;
        input [8:0] green_sum_in;
        input [8:0] blue_sum_in;
        reg signed [18:0] red_ext, green_ext, blue_ext;
        reg signed [18:0] weighted, value;
        begin
            red_ext = {10'd0, red_sum_in};
            green_ext = {10'd0, green_sum_in};
            blue_ext = {10'd0, blue_sum_in};
            weighted = -(red_ext * 19'sd26) - (green_ext * 19'sd87) +
                        (blue_ext * 19'sd112) + 19'sd256;
            value = (weighted >>> 9) + 19'sd128;
            if (value < 19'sd16) rgb_pair_to_u = 8'd16;
            else if (value > 19'sd240) rgb_pair_to_u = 8'd240;
            else rgb_pair_to_u = value[7:0];
        end
    endfunction

    function [7:0] rgb_pair_to_v;
        input [8:0] red_sum_in;
        input [8:0] green_sum_in;
        input [8:0] blue_sum_in;
        reg signed [18:0] red_ext, green_ext, blue_ext;
        reg signed [18:0] weighted, value;
        begin
            red_ext = {10'd0, red_sum_in};
            green_ext = {10'd0, green_sum_in};
            blue_ext = {10'd0, blue_sum_in};
            weighted = (red_ext * 19'sd112) - (green_ext * 19'sd102) -
                        (blue_ext * 19'sd10) + 19'sd256;
            value = (weighted >>> 9) + 19'sd128;
            if (value < 19'sd16) rgb_pair_to_v = 8'd16;
            else if (value > 19'sd240) rgb_pair_to_v = 8'd240;
            else rgb_pair_to_v = value[7:0];
        end
    endfunction

    reg sample_pair_half;
    reg [7:0] first_y, first_r, first_g, first_b;
    reg video_pair_valid_q;
    reg [15:0] video_word0_q, video_word1_q;
    reg second_word_pending;
    reg [COUNT_WIDTH-1:0] output_pixel_count;
    // 行缓存地址越界或输出碰撞是本地故障；输出计数另控制EOF排空。
    reg local_error_sticky, frame_output_error;
    reg [7:0] drain_ticks;
    reg eof_seen, close_pending, restart_pending;
    reg capture_vsync, capture_frame_bad;
    wire pipeline_empty = !vertical_sum_valid_q && !raw_rgb_valid_q &&
        !color_rgb_valid_q && !video_pair_valid_q && !second_word_pending && !sample_pair_half;

    assign raw_overflow = local_error_sticky | frame_output_error;

    always @(posedge raw_clk or negedge reset_n) begin
        if (!reset_n) begin
            fv_d <= 1'b0;
            lv_d <= 1'b0;
            vs_d <= 1'b1;
            pair_phase <= 2'd0;
            horizontal_even_partial <= 10'd0;
            horizontal_odd_partial <= 10'd0;
            group_index <= {GROUP_INDEX_WIDTH{1'b0}};
            line_mod3 <= 2'd0;
            input_line_parity <= 1'b0;
            line0_sum_read_q <= 22'd0;
            line1_sum_read_q <= 22'd0;
            row2_even_sum_q <= 11'd0;
            row2_odd_sum_q <= 11'd0;
            block_x_odd_q <= 1'b0;
            block_y_odd_q <= 1'b0;
            vertical_sum_valid_q <= 1'b0;
            raw_r_q <= 10'd0;
            raw_g_q <= 10'd0;
            raw_b_q <= 10'd0;
            raw_rgb_valid_q <= 1'b0;
            rgb_r_q <= 8'd0;
            rgb_g_q <= 8'd0;
            rgb_b_q <= 8'd0;
            color_rgb_valid_q <= 1'b0;
            sample_pair_half <= 1'b0;
            first_y <= 8'd16;
            first_r <= 8'd0;
            first_g <= 8'd0;
            first_b <= 8'd0;
            video_pair_valid_q <= 1'b0;
            video_word0_q <= 16'd0;
            video_word1_q <= 16'd0;
            second_word_pending <= 1'b0;
            output_pixel_count <= {COUNT_WIDTH{1'b0}};
            local_error_sticky <= 1'b0;
            frame_output_error <= 1'b0;
            drain_ticks <= 8'd0;
            eof_seen <= 1'b0;
            close_pending <= 1'b0;
            restart_pending <= 1'b0;
            capture_vsync <= 1'b1;
            capture_frame_bad <= 1'b1;
            camera_vsync <= 1'b1;
            camera_frame_bad <= 1'b1;
            camera_pixel_valid <= 1'b0;
            camera_pixel <= 16'd0;
        end else begin
            fv_d <= raw_frame_valid;
            lv_d <= raw_line_valid;
            vs_d <= raw_vsync;

            // 行/帧边沿建立3x3坐标；源行长和行数由入口guard检查。
            if (frame_start) begin
                pair_phase <= 2'd0;
                horizontal_even_partial <= 10'd0;
                horizontal_odd_partial <= 10'd0;
                group_index <= {GROUP_INDEX_WIDTH{1'b0}};
                line_mod3 <= 2'd0;
                input_line_parity <= 1'b0;
            end
            if (line_start) begin
                pair_phase <= 2'd0;
                horizontal_even_partial <= 10'd0;
                horizontal_odd_partial <= 10'd0;
                group_index <= {GROUP_INDEX_WIDTH{1'b0}};
            end
            if (raw_line_valid) begin
                case (pair_phase_now)
                    2'd0: begin
                        horizontal_even_partial <= raw_first;
                        horizontal_odd_partial <= raw_second;
                        pair_phase <= 2'd2;
                    end
                    2'd2: begin
                        horizontal_odd_partial <= raw_second;
                        pair_phase <= 2'd1;
                    end
                    default: begin
                        pair_phase <= 2'd0;
                    end
                endcase
            end else if (line_end) begin
                pair_phase <= 2'd0;
                horizontal_even_partial <= 10'd0;
                horizontal_odd_partial <= 10'd0;
                group_index <= {GROUP_INDEX_WIDTH{1'b0}};
                input_line_parity <= ~input_line_parity_now;
                if (line_mod3_now == 2'd2)
                    line_mod3 <= 2'd0;
                else
                    line_mod3 <= line_mod3_now + 1'b1;
            end

            vertical_sum_valid_q <= 1'b0;
            if (group_complete && (group_index_now < OUTPUT_WIDTH_COUNT)) begin
                group_index <= group_index_now + 1'b1;
                case (line_mod3_now)
                    2'd0: line0_sum_mem[group_index_now] <= {horizontal_odd_sum, horizontal_even_sum};
                    2'd1: line1_sum_mem[group_index_now] <= {horizontal_odd_sum, horizontal_even_sum};
                    default: begin
                        line0_sum_read_q <= line0_sum_mem[group_index_now];
                        line1_sum_read_q <= line1_sum_mem[group_index_now];
                        row2_even_sum_q <= horizontal_even_sum;
                        row2_odd_sum_q <= horizontal_odd_sum;
                        block_x_odd_q <= group_index_now[0];
                        block_y_odd_q <= input_line_parity_now;
                        vertical_sum_valid_q <= 1'b1;
                    end
                endcase
            end else if (group_complete) begin
                local_error_sticky <= 1'b1;
                capture_frame_bad <= 1'b1;
            end

            raw_rgb_valid_q <= vertical_sum_valid_q;
            if (vertical_sum_valid_q) begin
                raw_r_q <= average_rb(red_sum, red_count);
                raw_g_q <= green_average;
                raw_b_q <= average_rb(blue_sum, blue_count);
            end
            color_rgb_valid_q <= raw_rgb_valid_q;
            if (raw_rgb_valid_q) begin
                rgb_r_q <= (red_gain_raw > 10'd255) ? 8'hff :
                    DISPLAY_OETF_ENABLE ? video_oetf(red_gain_raw[7:0]) : red_gain_raw[7:0];
                rgb_g_q <= (green_gain_raw > 10'd255) ? 8'hff :
                    DISPLAY_OETF_ENABLE ? video_oetf(green_gain_raw[7:0]) : green_gain_raw[7:0];
                rgb_b_q <= (blue_gain_raw > 10'd255) ? 8'hff :
                    DISPLAY_OETF_ENABLE ? video_oetf(blue_gain_raw[7:0]) : blue_gain_raw[7:0];
            end

            video_pair_valid_q <= 1'b0;
            if (color_rgb_valid_q && !capture_vsync && !frame_start && !restart_pending) begin
                if (!sample_pair_half) begin
                    first_y <= rgb_to_y(rgb_r_q, rgb_g_q, rgb_b_q);
                    first_r <= rgb_r_q;
                    first_g <= rgb_g_q;
                    first_b <= rgb_b_q;
                    sample_pair_half <= 1'b1;
                end else begin
                    // 发送器每个16位像素先发高字节再发低字节，故打包为
                    // {Y0,U}/{Y1,V}后线上字节顺序为Y0,U,Y1,V（RTP PT97）。
                    video_word0_q <= {first_y, rgb_pair_to_u(
                        {1'b0, first_r} + {1'b0, rgb_r_q},
                        {1'b0, first_g} + {1'b0, rgb_g_q},
                        {1'b0, first_b} + {1'b0, rgb_b_q})};
                    video_word1_q <= {rgb_to_y(rgb_r_q, rgb_g_q, rgb_b_q), rgb_pair_to_v(
                        {1'b0, first_r} + {1'b0, rgb_r_q},
                        {1'b0, first_g} + {1'b0, rgb_g_q},
                        {1'b0, first_b} + {1'b0, rgb_b_q})};
                    video_pair_valid_q <= 1'b1;
                    sample_pair_half <= 1'b0;
                end
            end

            camera_vsync <= capture_vsync;
            camera_frame_bad <= capture_frame_bad;
            camera_pixel_valid <= 1'b0;
            if (!frame_start && !restart_pending && !capture_vsync) begin
                if (second_word_pending) begin
                    camera_pixel <= video_word1_q;
                    camera_pixel_valid <= 1'b1;
                    second_word_pending <= 1'b0;
                    if (video_pair_valid_q) begin
                        local_error_sticky <= 1'b1;
                        capture_frame_bad <= 1'b1;
                    end
                end else if (video_pair_valid_q) begin
                    camera_pixel <= video_word0_q;
                    camera_pixel_valid <= 1'b1;
                    second_word_pending <= 1'b1;
                end
                if (second_word_pending || video_pair_valid_q) begin
                    if (output_pixel_count < FRAME_PIXELS_COUNT)
                        output_pixel_count <= output_pixel_count + 1'b1;
                    else begin
                        frame_output_error <= 1'b1;
                        capture_frame_bad <= 1'b1;
                    end
                end
            end

            // 新SOF打断旧帧时丢弃旧流水并先发坏帧结束，再打开新帧。
            if (frame_start) begin
                vertical_sum_valid_q <= 1'b0;
                raw_rgb_valid_q <= 1'b0;
                color_rgb_valid_q <= 1'b0;
                video_pair_valid_q <= 1'b0;
                sample_pair_half <= 1'b0;
                second_word_pending <= 1'b0;
                camera_pixel_valid <= 1'b0;
                output_pixel_count <= {COUNT_WIDTH{1'b0}};
                frame_output_error <= 1'b0;
                eof_seen <= 1'b0;
                close_pending <= 1'b0;
                drain_ticks <= 8'd0;
                if (!capture_vsync) begin
                    capture_vsync <= 1'b1;
                    capture_frame_bad <= 1'b1;
                    restart_pending <= 1'b1;
                end else begin
                    capture_vsync <= 1'b0;
                    capture_frame_bad <= 1'b0;
                    restart_pending <= 1'b0;
                end
            end else if (restart_pending) begin
                capture_vsync <= 1'b0;
                capture_frame_bad <= 1'b0;
                restart_pending <= 1'b0;
                output_pixel_count <= {COUNT_WIDTH{1'b0}};
                frame_output_error <= 1'b0;
                eof_seen <= 1'b0;
                close_pending <= 1'b0;
                drain_ticks <= 8'd0;
            end else begin
                if (frame_end && !capture_vsync) begin
                    eof_seen <= 1'b1;
                    drain_ticks <= 8'd0;
                    if (raw_frame_bad) begin
                        capture_frame_bad <= 1'b1;
                    end
                end
                if (eof_seen && !close_pending) begin
                    if (pipeline_empty && output_pixel_count == FRAME_PIXELS_COUNT && !frame_output_error) begin
                        close_pending <= 1'b1;
                    end else if (drain_ticks >= EOF_TIMEOUT-1) begin
                        close_pending <= 1'b1;
                        capture_frame_bad <= 1'b1;
                    end else begin
                        drain_ticks <= drain_ticks + 1'b1;
                    end
                end
                if (close_pending) begin
                    capture_vsync <= 1'b1;
                    close_pending <= 1'b0;
                    eof_seen <= 1'b0;
                    if (output_pixel_count != FRAME_PIXELS_COUNT || frame_output_error)
                        capture_frame_bad <= 1'b1;
                end
            end
        end
    end
endmodule
