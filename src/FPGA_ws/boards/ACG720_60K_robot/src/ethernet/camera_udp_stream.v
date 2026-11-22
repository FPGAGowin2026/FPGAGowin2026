module camera_udp_stream #(
    parameter [47:0] DEST_MAC = 48'h02_00_00_00_00_6F,
    parameter [31:0] DEST_IP = 32'hC0_00_02_6F,
    parameter [31:0] SOURCE_IP = 32'hC0_00_02_0A,
    parameter [47:0] SOURCE_MAC = 48'h02_00_00_00_07_20,
    parameter [15:0] SOURCE_PORT = 16'd5000,
    parameter [15:0] DEST_PORT = 16'd6102,
    parameter integer PACKET_GAP_CYCLES = 60_000
) (
    input wire tx_clk, tx_reset_n, link_up,
    input wire pixel_clk, pixel_reset_n,
    input wire [104:0] imu_snapshot,
    input wire imu_updated,
    input wire [23:0] motion_status,
    input wire motion_updated,
    input wire [23:0] rx_debug,
    input wire [375:0] lidar_packet,
    input wire lidar_updated,
    input wire pixel_empty,
    input wire [15:0] pixel,
    output wire pixel_read,
    output reg frame_request_toggle,
    output wire gmii_tx_clk, gmii_tx_en,
    output wire [7:0] gmii_tx_data
);
    localparam [9:0] FRAME_BLOCKS = 10'd600;
    localparam [9:0] BLOCK_PIXELS = 10'd640;
    // Pace the application to the Pi receiver; the RGMII link stays at 1 Gbps.
    localparam integer GAP_WIDTH = PACKET_GAP_CYCLES < 2 ? 1 : $clog2(PACKET_GAP_CYCLES + 1);
    localparam [GAP_WIDTH-1:0] PACKET_GAP = PACKET_GAP_CYCLES;
    localparam [1:0] VIDEO = 2'd0, IMU = 2'd1, LIDAR = 2'd2, MOTION = 2'd3;

    // 一个包发送时，另一个包从 DDR FIFO 预取，发送过程中无需等待像素。
    reg [15:0] pixels [0:2047];
    reg [1:0] filled, sent;
    reg fill_slot, send_slot, frame_active, prefetch_active;
    reg [9:0] fill_index, send_index, filled_blocks, block_index;
    reg [10:0] payload_index;
    reg [15:0] frame_id, send_pixel;
    reg [GAP_WIDTH-1:0] ifg_count;
    reg tx_start, tx_busy;
    reg imu_due, lidar_due, motion_due;
    reg [1:0] tx_kind;
    reg [31:0] imu_sequence;
    reg [407:0] telemetry_payload;
    reg [7:0] payload;
    reg start_video, start_imu, start_lidar, start_motion;
    wire tx_done, payload_req;
    (* ASYNC_REG = "TRUE" *) reg [1:0] link_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] filled_meta, filled_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] sent_meta, sent_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] request_sync;
    reg request_seen;
    wire dispatch_pending = start_video || start_imu || start_lidar || start_motion;

    assign pixel_read = prefetch_active &&
                        filled[fill_slot] == sent_sync[fill_slot] && !pixel_empty;

    // A filled slot stays immutable until the TX domain returns its sent toggle.
    // The packet RAM has independent 50 MHz write and 125 MHz read clocks.
    always @(posedge pixel_clk)
        if (pixel_read)
            pixels[{fill_slot, fill_index}] <= pixel;
    always @(posedge tx_clk) begin
        send_pixel <= pixels[{send_slot, send_index}];
    end

    always @(posedge pixel_clk or negedge pixel_reset_n) begin
        if (!pixel_reset_n) begin
            sent_meta <= 0;
            sent_sync <= 0;
            request_sync <= 0;
            request_seen <= 0;
            prefetch_active <= 0;
            filled <= 0;
            fill_slot <= 0;
            fill_index <= 0;
            filled_blocks <= 0;
        end else begin
            sent_meta <= sent;
            sent_sync <= sent_meta;
            request_sync <= {request_sync[0], frame_request_toggle};
            if (request_sync[1] != request_seen) begin
                request_seen <= request_sync[1];
                prefetch_active <= 1;
                fill_index <= 0;
                filled_blocks <= 0;
            end else if (pixel_read) begin
                if (fill_index == BLOCK_PIXELS - 1'b1) begin
                    filled[fill_slot] <= !filled[fill_slot];
                    fill_slot <= !fill_slot;
                    fill_index <= 0;
                    filled_blocks <= filled_blocks + 1'b1;
                    if (filled_blocks == FRAME_BLOCKS - 1'b1)
                        prefetch_active <= 0;
                end else
                    fill_index <= fill_index + 1'b1;
            end
        end
    end

    // Registered dispatch strobes keep gap-count and priority logic off the
    // 408-bit payload register's load/shift control path.
    always @(posedge tx_clk or negedge tx_reset_n) begin
        if (!tx_reset_n)
            telemetry_payload <= 0;
        else if (start_imu)
            telemetry_payload <= {32'h494d5531, imu_sequence, 7'd0, imu_snapshot, 232'd0};
        else if (start_lidar)
            telemetry_payload <= {32'h4c445231, lidar_packet};
        else if (start_motion)
            telemetry_payload <= {32'h48585331, motion_status, rx_debug, 328'd0};
        else if (payload_req && tx_kind != VIDEO)
            telemetry_payload <= {telemetry_payload[399:0], 8'd0};
    end

    always @(posedge tx_clk or negedge tx_reset_n) begin
        if (!tx_reset_n) begin
            link_sync <= 0;
            filled_meta <= 0;
            filled_sync <= 0;
            frame_request_toggle <= 0;
            frame_active <= 0;
            frame_id <= 0;
            sent <= 0;
            send_slot <= 0;
            send_index <= 0;
            block_index <= 0;
            payload_index <= 0;
            ifg_count <= 0;
            tx_start <= 0;
            tx_busy <= 0;
            imu_due <= 0;
            lidar_due <= 0;
            motion_due <= 0;
            tx_kind <= VIDEO;
            imu_sequence <= 0;
            start_video <= 0;
            start_imu <= 0;
            start_lidar <= 0;
            start_motion <= 0;
        end else begin
            link_sync <= {link_sync[0], link_up};
            filled_meta <= filled;
            filled_sync <= filled_meta;
            tx_start <= 0;
            start_video <= 0;
            start_imu <= 0;
            start_lidar <= 0;
            start_motion <= 0;
            if (ifg_count != 0)
                ifg_count <= ifg_count - 1'b1;

            if (!frame_active && link_sync[1]) begin
                frame_request_toggle <= !frame_request_toggle;
                frame_active <= 1;
                frame_id <= frame_id + 1'b1;
                block_index <= 0;
            end

            // 新包与调度同拍时，直接发送当前雷达数据。
            if (lidar_updated)
                lidar_due <= 1;
            if (!tx_busy && !dispatch_pending && link_sync[1] && ifg_count == 0) begin
                if (imu_due)
                    start_imu <= 1;
                else if (lidar_due)
                    start_lidar <= 1;
                else if (motion_due)
                    start_motion <= 1;
                else if (frame_active && filled_sync[send_slot] != sent[send_slot])
                    start_video <= 1;
            end
            if (dispatch_pending && link_sync[1]) begin
                tx_start <= 1;
                tx_busy <= 1;
                if (start_imu) begin
                    tx_kind <= IMU;
                    imu_due <= 0;
                end else if (start_lidar) begin
                    tx_kind <= LIDAR;
                    lidar_due <= 0;
                end else if (start_motion) begin
                    tx_kind <= MOTION;
                    motion_due <= 0;
                end else begin
                    tx_kind <= VIDEO;
                    send_index <= 0;
                    payload_index <= 0;
                end
            end else if (payload_req) begin
                if (tx_kind == VIDEO) begin
                    payload_index <= payload_index + 1'b1;
                    // 发送低字节时预取下一像素，本像素高字节仍在寄存器内。
                    if (payload_index >= 11'd8 && !payload_index[0])
                        send_index <= send_index + 1'b1;
                end
            end
            if (imu_updated)
                imu_due <= 1;
            if (motion_updated)
                motion_due <= 1;

            if (tx_done) begin
                tx_busy <= 0;
                ifg_count <= PACKET_GAP;
                if (tx_kind == IMU)
                    imu_sequence <= imu_sequence + 1'b1;
                else if (tx_kind == VIDEO) begin
                    sent[send_slot] <= filled_sync[send_slot];
                    send_slot <= !send_slot;
                    if (block_index == FRAME_BLOCKS - 1'b1)
                        frame_active <= 0;
                    else
                        block_index <= block_index + 1'b1;
                end
            end
        end
    end

    // ACV2 + 网络序帧号和块号 + 640 个连续小端 RGB565 像素。
    always @* begin
        case (payload_index)
            11'd0: payload = 8'h41;
            11'd1: payload = 8'h43;
            11'd2: payload = 8'h56;
            11'd3: payload = 8'h32;
            11'd4: payload = frame_id[15:8];
            11'd5: payload = frame_id[7:0];
            11'd6: payload = {6'd0, block_index[9:8]};
            11'd7: payload = block_index[7:0];
            default: payload = payload_index[0] ? send_pixel[15:8] : send_pixel[7:0];
        endcase
    end

    eth_udp_tx_gmii mac (
        .clk125m(tx_clk), .reset_p(!tx_reset_n), .tx_en_pulse(tx_start), .tx_done(tx_done),
        .dst_mac(DEST_MAC), .src_mac(SOURCE_MAC),
        .dst_ip(DEST_IP), .src_ip(SOURCE_IP), .dst_port(DEST_PORT), .src_port(SOURCE_PORT),
        .data_length(tx_kind == IMU ? 16'd22 : tx_kind == LIDAR ? 16'd51 :
                     tx_kind == MOTION ? 16'd10 : 16'd1288),
        .payload_req_o(payload_req), .payload_dat_i(tx_kind == VIDEO ? payload : telemetry_payload[407:400]),
        .gmii_tx_clk(gmii_tx_clk), .gmii_txen(gmii_tx_en), .gmii_txd(gmii_tx_data)
    );
endmodule
