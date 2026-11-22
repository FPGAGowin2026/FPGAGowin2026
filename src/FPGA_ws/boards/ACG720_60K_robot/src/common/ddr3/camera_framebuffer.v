module camera_framebuffer #(
    parameter integer FRAME_BEATS = 48000,
    parameter integer BURST_BEATS = 100
) (
    input wire ui_clk, reset_n, calibrated,
    input wire camera_pclk, camera_active, camera_valid,
    input wire [15:0] camera_pixel,
    input wire udp_clk, udp_request_toggle, udp_read,
    output wire udp_empty,
    output wire [15:0] udp_pixel,
    output reg frame_valid,

    input wire cmd_ready, write_ready, read_valid,
    input wire [127:0] read_data,
    output wire [27:0] cmd_addr,
    output wire [2:0] cmd,
    output wire cmd_enable, write_enable,
    output wire [127:0] write_data
);
    localparam [2:0] IDLE = 0, WRITE = 1, READ = 2, DRAIN = 3;
    localparam [15:0] LAST_BEAT = FRAME_BEATS - 1;
    localparam [8:0] BURST_SIZE = BURST_BEATS;

    (* ASYNC_REG = "TRUE" *) reg [1:0] camera_sync, udp_request_sync;
    reg camera_prev;
    wire camera_start = camera_sync[1] && !camera_prev;
    wire camera_end = !camera_sync[1] && camera_prev;

    reg [2:0] state;
    reg [8:0] burst_left, reads_pending;
    reg [15:0] write_beat, udp_beat;
    reg writing;
    reg udp_request_seen, udp_active, udp_finished;
    reg [1:0] write_bank, latest_bank, udp_bank;
    // 三个帧槽分别容纳正在采集、最新完整和正在读取的帧。
    wire bank0_busy = (frame_valid && latest_bank == 0) || (udp_active && udp_bank == 0);
    wire bank1_busy = (frame_valid && latest_bank == 1) || (udp_active && udp_bank == 1);
    wire [1:0] free_bank = !bank0_busy ? 2'd0 : !bank1_busy ? 2'd1 : 2'd2;
    reg prefer_read;
    reg [4:0] write_reset_count;
    reg [8:0] camera_drain_count;
    wire camera_drain_timeout = camera_drain_count == 1;
    wire write_fifo_reset = !reset_n || write_reset_count != 0;
    wire [9:0] write_words, udp_words;
    wire write_fifo_full, write_fifo_empty, udp_fifo_full;
    wire write_request = writing && !camera_drain_timeout && write_reset_count == 0 &&
                         (write_words >= BURST_BEATS ||
                          (camera_drain_count != 0 && !write_fifo_empty));
    wire udp_read_request = udp_active && !udp_finished && udp_words <= BURST_BEATS;

    assign write_enable = state == WRITE && writing && !camera_drain_timeout &&
                          cmd_ready && write_ready && !write_fifo_empty;
    wire read_command = state == READ && cmd_ready;
    assign cmd_enable = write_enable || read_command;
    assign cmd = state == READ ? 3'd1 : 3'd0;
    // 每条命令传输 8 像素，每个槽按 0x80000 个像素地址对齐。
    assign cmd_addr = state == WRITE ? {7'd0, write_bank, write_beat, 3'b000} :
                                      {7'd0, udp_bank, udp_beat, 3'b000};

    always @(posedge ui_clk or negedge reset_n) begin
        if (!reset_n) begin
            camera_sync <= 0;
            udp_request_sync <= 0;
            camera_prev <= 0;
        end else begin
            camera_sync <= {camera_sync[0], camera_active};
            udp_request_sync <= {udp_request_sync[0], udp_request_toggle};
            camera_prev <= camera_sync[1];
        end
    end

    always @(posedge ui_clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= IDLE;
            burst_left <= 0;
            reads_pending <= 0;
            write_beat <= 0;
            udp_beat <= 0;
            writing <= 0;
            write_bank <= 0;
            latest_bank <= 0;
            udp_bank <= 0;
            udp_request_seen <= 0;
            udp_active <= 0;
            udp_finished <= 0;
            frame_valid <= 0;
            prefer_read <= 0;
            write_reset_count <= 0;
            camera_drain_count <= 0;
        end else begin
            if (write_reset_count != 0)
                write_reset_count <= write_reset_count - 1'b1;

            // VSYNC 到来时，最后一批像素可能仍在异步 FIFO 或等待 DDR 接收。
            // 留出 5.12 us 的消隐期排空窗口，再丢弃残帧并清 FIFO。
            if (camera_end)
                camera_drain_count <= 9'd256;
            else if (camera_drain_count != 0)
                camera_drain_count <= camera_drain_count - 1'b1;
            if (camera_drain_timeout) begin
                writing <= 0;
                write_reset_count <= 5'd16;
            end
            if (camera_start && calibrated && camera_drain_count == 0 && write_reset_count == 0) begin
                writing <= 1;
                write_bank <= free_bank;
                write_beat <= 0;
            end

            case ({read_command, read_valid})
                2'b10: reads_pending <= reads_pending + 1'b1;
                2'b01: if (reads_pending != 0)
                    reads_pending <= reads_pending - 1'b1;
                default: ;
            endcase

            case (state)
                IDLE: begin
                    if (frame_valid && !udp_active && udp_request_sync[1] != udp_request_seen) begin
                        // 锁定最新完整帧，直到该帧所有 DDR 读数据返回。
                        udp_request_seen <= udp_request_sync[1];
                        udp_bank <= latest_bank;
                        udp_beat <= 0;
                        udp_active <= 1;
                        udp_finished <= 0;
                    end else if (calibrated) begin
                        if (udp_read_request && (prefer_read || !write_request)) begin
                            state <= READ;
                            burst_left <= BURST_SIZE;
                            prefer_read <= 0;
                        end else if (write_request) begin
                            state <= WRITE;
                            burst_left <= BURST_SIZE;
                            prefer_read <= 1;
                        end
                    end
                end
                WRITE: begin
                    if (camera_drain_timeout)
                        state <= IDLE;
                    else if (write_enable) begin
                        write_beat <= write_beat + 1'b1;
                        burst_left <= burst_left - 1'b1;
                        if (burst_left == 1)
                            state <= IDLE;
                        // 最后一个写命令被接受后才发布新帧。
                        if (write_beat == LAST_BEAT) begin
                            frame_valid <= 1;
                            latest_bank <= write_bank;
                            writing <= 0;
                            state <= IDLE;
                        end
                    end
                end
                READ: if (read_command) begin
                    udp_beat <= udp_beat + 1'b1;
                    if (udp_beat == LAST_BEAT)
                        udp_finished <= 1;
                    burst_left <= burst_left - 1'b1;
                    if (burst_left == 1 || udp_beat == LAST_BEAT)
                        state <= DRAIN;
                end
                DRAIN: if (reads_pending == 0) begin
                    state <= IDLE;
                    if (udp_finished)
                        udp_active <= 0;
                end
                default: state <= IDLE;
            endcase
        end
    end

    wr_data_fifo camera_fifo (
        .Data(camera_pixel), .Reset(write_fifo_reset),
        .WrClk(camera_pclk), .RdClk(ui_clk),
        .WrEn(camera_valid && camera_active && !write_fifo_full), .RdEn(write_enable),
        .Rnum(write_words), .Wnum(), .Q(write_data),
        .Empty(write_fifo_empty), .Full(write_fifo_full), .Almost_Empty(), .Almost_Full()
    );
    rd_data_fifo udp_fifo (
        .Data(read_data), .Reset(!reset_n),
        .WrClk(ui_clk), .RdClk(udp_clk),
        .WrEn(read_valid && (state == READ || state == DRAIN) && !udp_fifo_full),
        .RdEn(udp_read), .Rnum(), .Wnum(udp_words), .Q(udp_pixel),
        .Empty(udp_empty), .Full(udp_fifo_full), .Almost_Empty(), .Almost_Full()
    );
endmodule
