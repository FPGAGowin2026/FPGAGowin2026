module acg720_tutorial_25k_bist_top (
    input  wire clk_i,
    input  wire reset_i,
    input  wire uart_rx_i,
    output wire uart_tx_o
);
    wire reset_n = ~reset_i;

    localparam [5:0]
        ST_BOOT             = 6'd0,
        ST_BIN              = 6'd1,
        ST_PWM              = 6'd2,
        ST_RGB_START        = 6'd3,
        ST_RGB_WAIT         = 6'd4,
        ST_CHECKSUM_LOAD    = 6'd5,
        ST_DVP_FRAME_HIGH   = 6'd6,
        ST_DVP_FRAME_LOW    = 6'd7,
        ST_DVP_PREP         = 6'd8,
        ST_DVP_BYTE0        = 6'd9,
        ST_DVP_BYTE1        = 6'd10,
        ST_DVP_FLUSH        = 6'd11,
        ST_DVP_WAIT         = 6'd12,
        ST_UART_CHALLENGE   = 6'd13,
        ST_UART_WAIT        = 6'd14,
        ST_REPORT_SEND      = 6'd15,
        ST_REPORT_WAIT      = 6'd16,
        ST_REPORT_INTERVAL  = 6'd17,
        ST_CHECKSUM         = 6'd18;

    reg [5:0] state;
    reg [7:0] step_count;

    reg pass_bin;
    reg pass_pwm;
    reg pass_gray;
    reg pass_checksum;
    reg pass_dvp;
    reg pass_uart;
    wire pass_all = pass_bin & pass_pwm & pass_gray & pass_checksum & pass_dvp & pass_uart;

    // Chapter 5: parameter override keeps the exact tutorial counter logic but
    // makes its toggle observable in a few clocks.
    wire bin_led;
    reg bin_led_d;
    reg [2:0] bin_toggles;
    bin_counter #(.MCNT(3)) u_bin_counter (
        .clk(clk_i),
        .reset_n(reset_n),
        .led(bin_led)
    );

    // Chapter 19: observe both PWM levels over several short periods.
    reg pwm_enable;
    wire pwm_out;
    reg pwm_seen_high;
    reg pwm_seen_low;
    pwm_gen u_pwm_gen (
        .clk(clk_i),
        .reset_n(reset_n),
        .pwm_gen_en(pwm_enable),
        .counter_arr(32'd8),
        .counter_ccr(32'd3),
        .pwm_out(pwm_out)
    );

    // Chapter 39: formula path, red=255 -> floor(255*77/256)=76.
    reg rgb_valid;
    wire [7:0] gray_value;
    wire gray_valid;
    wire gray_hs;
    wire gray_vs;
    rgb2gray #(.PROC_METHOD("FORMULA")) u_rgb2gray (
        .clk(clk_i),
        .reset_p(reset_i),
        .rgb_valid(rgb_valid),
        .rgb_hs(1'b1),
        .rgb_vs(1'b1),
        .red_8b_i(8'hff),
        .green_8b_i(8'h00),
        .blue_8b_i(8'h00),
        .gray_8b_o(gray_value),
        .gray_valid(gray_valid),
        .gray_hs(gray_hs),
        .gray_vs(gray_vs)
    );

    // Chapter 53: RFC-style IPv4 header example; expected checksum 0xb1e6.
    wire [15:0] checksum_value;
    reg [15:0] checksum_total_len;
    // Tie one data bit to the real UART RX idle level so the checksum datapath
    // remains a runtime circuit rather than a compile-time constant. USB-UART
    // RX idles high, which selects the documented 0x003c test vector.
    wire [15:0] checksum_dynamic_total_len = checksum_total_len ^ {15'd0, ~uart_rx_i};
    (* syn_keep = 1, keep_hierarchy = "yes" *) checksum u_checksum (
        .ver(4'h4), .hdr_len(4'h5), .tos(8'h00), .total_len(checksum_dynamic_total_len),
        .id(16'h1c46), .offset(16'h4000), .ttl(8'h40), .protocol(8'h06),
        .src_ip(32'hac100a63), .dst_ip(32'hac100a0c),
        .checksum_result(checksum_value)
    );

    // Chapter 44: generate 11 frame markers (the tutorial intentionally drops
    // the first 10 frames), then feed the bytes 0x12,0x34 and expect RGB565 0x1234.
    reg dvp_vsync;
    reg dvp_href;
    reg [7:0] dvp_data;
    wire dvp_image_state;
    wire dvp_valid;
    wire [15:0] dvp_pixel;
    wire dvp_hs;
    wire dvp_vs;
    wire [11:0] dvp_x;
    wire [11:0] dvp_y;
    reg [3:0] dvp_frame_count;
    DVP_Capture u_dvp_capture (
        .Rst_p(reset_i), .PCLK(clk_i), .Vsync(dvp_vsync), .Href(dvp_href), .Data(dvp_data),
        .ImageState(dvp_image_state), .DataValid(dvp_valid), .DataPixel(dvp_pixel),
        .DataHs(dvp_hs), .DataVs(dvp_vs), .Xaddr(dvp_x), .Yaddr(dvp_y)
    );

    // Chapters 6 and 7: the tutorial transmitter drives the physical USB-UART
    // and is also looped back internally into the tutorial receiver.
    reg [7:0] uart_data;
    reg uart_send;
    wire uart_done;
    wire uart_busy;
    wire [7:0] loopback_data;
    wire loopback_done;
    reg challenge_tx_seen;
    reg challenge_rx_seen;
    reg uart_challenge_done;
    uart_byte_tx u_uart_tx (
        .clk(clk_i), .reset_n(reset_n), .data_byte(uart_data), .send_en(uart_send),
        .baud_set(3'd4), .uart_tx(uart_tx_o), .tx_done(uart_done), .uart_state(uart_busy)
    );
    uart_byte_rx u_uart_rx (
        .clk(clk_i), .reset_n(reset_n), .baud_set(3'd4), .uart_rx(uart_tx_o),
        .data_byte(loopback_data), .rx_done(loopback_done)
    );

    reg [5:0] report_index;
    reg [25:0] report_interval;
    localparam [5:0] REPORT_LAST = 6'd57;

    function [7:0] hex_char;
        input [3:0] nibble;
        begin
            hex_char = (nibble < 10) ? ("0" + nibble) : ("A" + nibble - 10);
        end
    endfunction

    function [7:0] report_byte;
        input [5:0] index;
        begin
            case (index)
                0:report_byte="A";  1:report_byte="C";  2:report_byte="G";  3:report_byte="7";  4:report_byte="2";  5:report_byte="0";
                6:report_byte="_";  7:report_byte="T";  8:report_byte="U";  9:report_byte="T"; 10:report_byte="O"; 11:report_byte="R";
               12:report_byte="I"; 13:report_byte="A"; 14:report_byte="L"; 15:report_byte="_"; 16:report_byte="2"; 17:report_byte="5";
               18:report_byte="K"; 19:report_byte=" "; 20:report_byte="B"; 21:report_byte="="; 22:report_byte = pass_bin      ? "1" : "0";
               23:report_byte=" "; 24:report_byte="P"; 25:report_byte="="; 26:report_byte = pass_pwm      ? "1" : "0";
               27:report_byte=" "; 28:report_byte="G"; 29:report_byte="="; 30:report_byte = pass_gray     ? "1" : "0";
               31:report_byte=" "; 32:report_byte="C"; 33:report_byte="="; 34:report_byte = pass_checksum ? "1" : "0";
               35:report_byte=" "; 36:report_byte="D"; 37:report_byte="="; 38:report_byte = pass_dvp      ? "1" : "0";
               39:report_byte=" "; 40:report_byte="U"; 41:report_byte="="; 42:report_byte = pass_uart     ? "1" : "0";
               43:report_byte=" "; 44:report_byte="A"; 45:report_byte="L"; 46:report_byte="L"; 47:report_byte="="; 48:report_byte = pass_all ? "1" : "0";
               49:report_byte=" "; 50:report_byte="S"; 51:report_byte="=";
               52:report_byte=hex_char(checksum_value[15:12]);
               53:report_byte=hex_char(checksum_value[11:8]);
               54:report_byte=hex_char(checksum_value[7:4]);
               55:report_byte=hex_char(checksum_value[3:0]);
               56:report_byte = 8'h0d;
               57:report_byte = 8'h0a;
               default: report_byte = "?";
            endcase
        end
    endfunction

    always @(posedge clk_i or negedge reset_n) begin
        if (!reset_n) begin
            state <= ST_BOOT;
            step_count <= 0;
            pass_bin <= 0;
            pass_pwm <= 0;
            pass_gray <= 0;
            pass_checksum <= 0;
            pass_dvp <= 0;
            pass_uart <= 0;
            checksum_total_len <= 16'h0000;
            bin_led_d <= 1;
            bin_toggles <= 0;
            pwm_enable <= 0;
            pwm_seen_high <= 0;
            pwm_seen_low <= 0;
            rgb_valid <= 0;
            dvp_vsync <= 0;
            dvp_href <= 0;
            dvp_data <= 0;
            dvp_frame_count <= 0;
            uart_data <= 8'h55;
            uart_send <= 0;
            challenge_tx_seen <= 0;
            challenge_rx_seen <= 0;
            uart_challenge_done <= 0;
            report_index <= 0;
            report_interval <= 0;
        end else begin
            uart_send <= 1'b0;
            case (state)
                ST_BOOT: begin
                    step_count <= 0;
                    bin_led_d <= bin_led;
                    bin_toggles <= 0;
                    state <= ST_BIN;
                end
                ST_BIN: begin
                    step_count <= step_count + 1'b1;
                    bin_led_d <= bin_led;
                    if (bin_led != bin_led_d)
                        bin_toggles <= bin_toggles + 1'b1;
                    if (step_count == 8'd15) begin
                        pass_bin <= (bin_toggles >= 3'd2) || ((bin_led != bin_led_d) && (bin_toggles >= 3'd1));
                        step_count <= 0;
                        pwm_enable <= 1;
                        state <= ST_PWM;
                    end
                end
                ST_PWM: begin
                    step_count <= step_count + 1'b1;
                    if (pwm_out) pwm_seen_high <= 1;
                    else pwm_seen_low <= 1;
                    if (step_count == 8'd24) begin
                        pass_pwm <= (pwm_seen_high || pwm_out) & (pwm_seen_low || !pwm_out);
                        pwm_enable <= 0;
                        rgb_valid <= 1;
                        state <= ST_RGB_START;
                    end
                end
                ST_RGB_START: begin
                    rgb_valid <= 0;
                    state <= ST_RGB_WAIT;
                end
                ST_RGB_WAIT: begin
                    if (gray_valid) begin
                        pass_gray <= (gray_value == 8'd76) && gray_hs && gray_vs;
                        state <= ST_CHECKSUM_LOAD;
                    end
                end
                ST_CHECKSUM_LOAD: begin
                    checksum_total_len <= 16'h003c;
                    state <= ST_CHECKSUM;
                end
                ST_CHECKSUM: begin
                    pass_checksum <= (checksum_value == 16'hb1e6);
                    dvp_frame_count <= 0;
                    state <= ST_DVP_FRAME_HIGH;
                end
                ST_DVP_FRAME_HIGH: begin
                    dvp_vsync <= 1;
                    state <= ST_DVP_FRAME_LOW;
                end
                ST_DVP_FRAME_LOW: begin
                    dvp_vsync <= 0;
                    if (dvp_frame_count == 4'd10) begin
                        step_count <= 0;
                        state <= ST_DVP_PREP;
                    end else begin
                        dvp_frame_count <= dvp_frame_count + 1'b1;
                        state <= ST_DVP_FRAME_HIGH;
                    end
                end
                ST_DVP_PREP: begin
                    step_count <= step_count + 1'b1;
                    if (step_count == 8'd3) begin
                        dvp_href <= 1;
                        dvp_data <= 8'h12;
                        state <= ST_DVP_BYTE0;
                    end
                end
                ST_DVP_BYTE0: begin
                    dvp_href <= 1;
                    dvp_data <= 8'h34;
                    state <= ST_DVP_BYTE1;
                end
                ST_DVP_BYTE1: begin
                    dvp_href <= 1;
                    dvp_data <= 8'h56;
                    state <= ST_DVP_FLUSH;
                end
                ST_DVP_FLUSH: begin
                    dvp_href <= 0;
                    dvp_data <= 0;
                    state <= ST_DVP_WAIT;
                end
                ST_DVP_WAIT: begin
                    if (dvp_valid) begin
                        pass_dvp <= (dvp_pixel == 16'h1234);
                        state <= ST_UART_CHALLENGE;
                    end
                end
                ST_UART_CHALLENGE: begin
                    uart_data <= 8'h55;
                    if (!uart_busy) begin
                        uart_send <= 1;
                        challenge_tx_seen <= 0;
                        challenge_rx_seen <= 0;
                        state <= ST_UART_WAIT;
                    end
                end
                ST_UART_WAIT: begin
                    if (uart_done) challenge_tx_seen <= 1;
                    if (loopback_done) begin
                        challenge_rx_seen <= 1;
                        pass_uart <= (loopback_data == 8'h55);
                    end
                    if ((challenge_tx_seen || uart_done) && (challenge_rx_seen || loopback_done)) begin
                        uart_challenge_done <= 1;
                        report_index <= 0;
                        state <= ST_REPORT_SEND;
                    end
                end
                ST_REPORT_SEND: begin
                    if (!uart_busy) begin
                        uart_data <= report_byte(report_index);
                        uart_send <= 1;
                        state <= ST_REPORT_WAIT;
                    end
                end
                ST_REPORT_WAIT: begin
                    if (uart_done) begin
                        if (report_index == REPORT_LAST) begin
                            report_interval <= 0;
                            state <= ST_REPORT_INTERVAL;
                        end else begin
                            report_index <= report_index + 1'b1;
                            state <= ST_REPORT_SEND;
                        end
                    end
                end
                ST_REPORT_INTERVAL: begin
                    if (report_interval == 26'd49_999_999) begin
                        report_interval <= 0;
                        report_index <= 0;
                        state <= ST_REPORT_SEND;
                    end else begin
                        report_interval <= report_interval + 1'b1;
                    end
                end
                default: state <= ST_BOOT;
            endcase
        end
    end
endmodule
