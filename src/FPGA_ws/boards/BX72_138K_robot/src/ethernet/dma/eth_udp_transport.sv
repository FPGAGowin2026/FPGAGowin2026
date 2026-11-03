// ==================== async_pixel_fifo.v ====================
// Dual-clock first-word-fall-through FIFO for RGB565 pixels and frame markers.
// The read port has a one-entry output register so Gowin can infer dual-port
// block RAM instead of a large bank of asynchronous-read registers.
module async_pixel_fifo #(
    parameter integer ADDR_WIDTH = 11,
    parameter integer DATA_WIDTH = 18
) (
    input  wire                  arst,
    input  wire                  wr_clk,
    input  wire                  wr_en,
    input  wire [DATA_WIDTH-1:0] wr_data,
    output reg                   full,
    output reg                   overflow_sticky,
    input  wire                  rd_clk,
    input  wire                  rd_en,
    output reg  [DATA_WIDTH-1:0] rd_data,
    output reg                   rd_valid,
    output reg  [ADDR_WIDTH:0]   rd_count
);
    localparam integer DEPTH = (1 << ADDR_WIDTH);

    // 两个端口分别同步释放复位，指针和 RAM 读写不会在复位边界提前动作。
    (* ASYNC_REG="TRUE" *) reg [1:0] wr_reset_sync,rd_reset_sync;
    always @(posedge wr_clk or posedge arst)
        if(arst)wr_reset_sync<=2'b11;else wr_reset_sync<={wr_reset_sync[0],1'b0};
    always @(posedge rd_clk or posedge arst)
        if(arst)rd_reset_sync<=2'b11;else rd_reset_sync<={rd_reset_sync[0],1'b0};
    wire wr_reset=wr_reset_sync[1],rd_reset=rd_reset_sync[1];

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_WIDTH:0] wr_bin;
    reg [ADDR_WIDTH:0] wr_gray;
    reg [ADDR_WIDTH:0] rd_bin;
    reg [ADDR_WIDTH:0] rd_gray;

    (* ASYNC_REG = "TRUE" *) reg [ADDR_WIDTH:0] rd_gray_sync1;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_WIDTH:0] rd_gray_sync2;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_WIDTH:0] wr_gray_sync1;
    (* ASYNC_REG = "TRUE" *) reg [ADDR_WIDTH:0] wr_gray_sync2;

    function [ADDR_WIDTH:0] bin_to_gray;
        input [ADDR_WIDTH:0] value;
        begin
            bin_to_gray = (value >> 1) ^ value;
        end
    endfunction

    function [ADDR_WIDTH:0] gray_to_bin;
        input [ADDR_WIDTH:0] value;
        integer k;
        begin
            // Parallel-prefix XOR stages keep the Gray decode logarithmic in
            // depth. A descending recurrence creates a long XOR chain on the
            // FIFO occupancy-to-packet-launch path at the 125 MHz TX clock.
            gray_to_bin = value;
            for (k = 1; k <= ADDR_WIDTH; k = k * 2)
                gray_to_bin = gray_to_bin ^ (gray_to_bin >> k);
        end
    endfunction

    wire wr_accept = wr_en && !full;
    wire [ADDR_WIDTH:0] wr_bin_next = wr_bin + wr_accept;
    wire [ADDR_WIDTH:0] wr_gray_next = bin_to_gray(wr_bin_next);
    wire [ADDR_WIDTH:0] full_compare = {
        ~rd_gray_sync2[ADDR_WIDTH:ADDR_WIDTH-1],
         rd_gray_sync2[ADDR_WIDTH-2:0]
    };
    wire full_next = (wr_gray_next == full_compare);

    always @(posedge wr_clk or posedge wr_reset) begin
        if (wr_reset) begin
            wr_bin <= {(ADDR_WIDTH+1){1'b0}};
            wr_gray <= {(ADDR_WIDTH+1){1'b0}};
            rd_gray_sync1 <= {(ADDR_WIDTH+1){1'b0}};
            rd_gray_sync2 <= {(ADDR_WIDTH+1){1'b0}};
            full <= 1'b0;
            overflow_sticky <= 1'b0;
        end else begin
            rd_gray_sync1 <= rd_gray;
            rd_gray_sync2 <= rd_gray_sync1;
            if (wr_accept)
                mem[wr_bin[ADDR_WIDTH-1:0]] <= wr_data;
            else if (wr_en)
                overflow_sticky <= 1'b1;
            wr_bin <= wr_bin_next;
            wr_gray <= wr_gray_next;
            full <= full_next;
        end
    end

    // 空标志按下一读指针寄存，切断同步写指针到读地址进位的组合路径。
    reg rd_empty;
    wire rd_can_load = (!rd_valid || rd_en) && !rd_empty;
    // 常量递增先计算，读允许只控制寄存使能和一位空标志选择。
    wire [ADDR_WIDTH:0] rd_bin_inc = rd_bin + 1'b1;
    wire [ADDR_WIDTH:0] rd_gray_inc = bin_to_gray(rd_bin_inc);
    wire empty_after_load=(rd_gray_inc==wr_gray_sync2);
    wire empty_without_load=(rd_gray==wr_gray_sync2);
    reg [ADDR_WIDTH:0] wr_bin_sync;

    always @(posedge rd_clk or posedge rd_reset) begin
        if (rd_reset) begin
            rd_bin <= {(ADDR_WIDTH+1){1'b0}};
            rd_gray <= {(ADDR_WIDTH+1){1'b0}};
            wr_gray_sync1 <= {(ADDR_WIDTH+1){1'b0}};
            wr_gray_sync2 <= {(ADDR_WIDTH+1){1'b0}};
            wr_bin_sync <= {(ADDR_WIDTH+1){1'b0}};
            rd_count <= {(ADDR_WIDTH+1){1'b0}};
            rd_data <= {DATA_WIDTH{1'b0}};
            rd_valid <= 1'b0;
            rd_empty <= 1'b1;
        end else begin
            rd_empty <= rd_can_load ? empty_after_load : empty_without_load;
            wr_gray_sync1 <= wr_gray;
            wr_gray_sync2 <= wr_gray_sync1;
            // Decode and occupancy arithmetic are registered separately. The
            // extra reader-clock latency is conservative for packet launch:
            // writes may be counted late, but the FIFO is never read early.
            wr_bin_sync <= gray_to_bin(wr_gray_sync2);
            rd_count <= wr_bin_sync - rd_bin + rd_valid;
            if (rd_can_load) begin
                rd_data <= mem[rd_bin[ADDR_WIDTH-1:0]];
                rd_bin <= rd_bin_inc;
                rd_gray <= rd_gray_inc;
                rd_valid <= 1'b1;
            end else if (rd_en && rd_valid) begin
                rd_valid <= 1'b0;
            end
        end
    end
endmodule

// ==================== bx72_udp_request_rx.v ====================
`timescale 1ns/1ps

// 32 字节诊断请求：完整 Ethernet/IPv4/UDP 和应用 CRC 通过后才提交。
// RX 暂存不跨域；诊断与速度应答进入深度 4 FIFO，速度另经整帧 FIFO 到 50 MHz。
module bx72_udp_request_rx #(
    parameter [47:0] LOCAL_MAC = 48'h00_0A_35_01_FE_C0,
    parameter [31:0] LOCAL_IP = 32'hC0_A8_00_02,
    parameter [31:0] PEER_IP = 32'hC0_A8_00_04,
    parameter [15:0] LOCAL_PORT = 16'd5000,
    parameter [15:0] PEER_PORT = 16'd6103,
    parameter SPLIT_IP_CHECKSUM_RESET = 1'b1,
    parameter integer USE_CONTROL_BRIDGE = 0
) (
    input wire reset_p,
    input wire rx_clk,
    input wire [7:0] gmii_rxd,
    input wire gmii_rxdv,
    input wire gmii_rxer,
    input wire tx_clk,
    input wire reply_ready,
    output wire reply_valid,
    output wire [255:0] reply_data,
    output wire [127:0] diagnostics_tx_o,
    input wire control_clk,
    output wire control_valid_o,
    output wire [47:0] control_velocity_o,
    output wire auto_mode_valid_o,auto_enable_o,
    output wire task_fragment_valid_o,output wire [255:0] task_fragment_o,
    output wire [63:0] control_token_o,
    output wire control_heartbeat_o,
    input wire training_reset_p_i,
    output reg good_toggle_o,
    output reg [63:0] raw_prefix_tx_o,
    input wire command_bus_clk_i
);
    (* ASYNC_REG="TRUE" *) reg [1:0] rx_reset_sync, tx_reset_sync;
    always @(posedge rx_clk or posedge reset_p)
        if(reset_p)rx_reset_sync<=2'b11;else rx_reset_sync<={rx_reset_sync[0],1'b0};
    always @(posedge tx_clk or posedge reset_p)
        if(reset_p)tx_reset_sync<=2'b11;else tx_reset_sync<={tx_reset_sync[0],1'b0};
    wire rx_reset_p=rx_reset_sync[1], tx_reset_p=tx_reset_sync[1];
    // USE_CONTROL_BRIDGE=1 时 RX->50MHz 和 50->40MHz 两段各自使用真实时钟域。
    (* ASYNC_REG="TRUE" *) reg [1:0] command_bus_reset_sync;
    always @(posedge command_bus_clk_i or posedge reset_p)
        if(reset_p)command_bus_reset_sync<=2'b11;
        else command_bus_reset_sync<={command_bus_reset_sync[0],1'b0};
    wire payload_valid, pkt_done, pkt_error;
    wire [7:0] payload_byte;
    wire [15:0] payload_length, peer_port;
    wire [31:0] peer_ip;
    wire fifo_full;
    eth_udp_rx_gmii #(.SPLIT_IP_CHECKSUM_RESET(SPLIT_IP_CHECKSUM_RESET)) receiver (
        .reset_p(rx_reset_p), .local_mac(LOCAL_MAC), .local_ip(LOCAL_IP),
        .local_port(LOCAL_PORT), .clk125m_o(), .exter_mac(),
        .exter_ip(peer_ip), .exter_port(peer_port), .rx_data_length(payload_length),
        .data_overflow_i(fifo_full), .payload_valid_o(payload_valid),
        .payload_dat_o(payload_byte), .one_pkt_done(pkt_done), .pkt_error(pkt_error),
        .debug_crc_check(), .gmii_rx_clk(rx_clk), .gmii_rxdv(gmii_rxdv), .gmii_rxd(gmii_rxd),
        .rx_reset_stage0_i(rx_reset_sync[0]), .rx_async_reset_p_i(reset_p)
    );

    function [15:0] crc_byte;
        input [15:0] crc;
        input [7:0] value;
        reg [7:0] mix;
        reg m04, m15, m26, m37;
        begin
            // CRC-CCITT 逐位迭代改为等价并行余数，保留每拍一字节吞吐。
            mix=crc[15:8]^value;
            m04=mix[0]^mix[4];
            m15=mix[1]^mix[5];
            m26=mix[2]^mix[6];
            m37=mix[3]^mix[7];
            crc_byte[15:8]=crc[7:0]^{m37,m26,m15,m04^mix[7],mix[6],mix[5],mix[4],m37};
            crc_byte[7:0]={m26^mix[7],m15^mix[6],m04^mix[5],mix[4],m37,m26,m15,m04};
        end
    endfunction
    reg [255:0] staging;
    reg [5:0] count;
    reg [15:0] request_crc, reply_crc;
    reg rx_error_seen, rx_dv_d;
    // 1/2诊断、3/4速度、5/6自动模式、7/8任务分片。
    // 控制请求共用会话token和递增序号；任务重试仅补ACK，不重复执行。
    wire ack_kind_supported = payload_byte==8'd1 || payload_byte==8'd3 ||
        payload_byte==8'd5 || payload_byte==8'd7;
    wire [7:0] ack_kind_byte = ack_kind_supported ? payload_byte+8'd1 : 8'd2;
    wire [7:0] ack_byte=(count==6'd5) ? ack_kind_byte : payload_byte;
    wire motion_request=staging[215:208]==8'd3;
    wire auto_request=staging[215:208]==8'd5;
    wire task_request=staging[215:208]==8'd7;
    wire control_request=motion_request || auto_request || task_request;
    wire auto_fields_ok=staging[95:88]<=1 && staging[87:16]==0;
    // 新会话只能由合法type3零速包建立；全零任务/模式字段不能冒充启动握手。
    wire stop_request=motion_request && staging[95:48]==48'd0 && staging[159:96]!=0;
    wire signed [15:0] cmd_vx=staging[95:80],cmd_vy=staging[79:64],cmd_wz=staging[63:48];
    wire motion_fields_ok=cmd_vx>=-400 && cmd_vx<=400 && cmd_vy>=-400 && cmd_vy<=400 &&
        cmd_wz>=-1000 && cmd_wz<=1000 && staging[47:16]==0;
    reg application_magic_q,application_length_q;
    wire application_header_ok = application_magic_q && application_length_q;
    wire application_fields_ok = staging[215:208]==8'd1 ||
        (motion_request && motion_fields_ok) || (auto_request && auto_fields_ok) ||
        (task_request && staging[95:88]<8);
    // 负载末尾之后仍有 FCS/UDP 校验流水；提前注册应用检查以缩短 FIFO 写使能路径。
    // 源端点与 pkt_error 在 pkt_done 同拍才更新，仍在该裁决周期使用。
    reg application_header_q,application_fields_q,application_crc_q,application_ok_q;
    always @(posedge rx_clk or posedge rx_reset_p) begin
        if(rx_reset_p)begin application_magic_q<=0;application_length_q<=0;application_header_q<=0;application_fields_q<=0;application_crc_q<=0;application_ok_q<=0;end
        else begin
            // FCS四字节排空前完成两段检查，缩短16位长度比较到应用头判定的路径。
            application_magic_q<=staging[255:224]==32'h42583732 && staging[223:216]==8'd1;
            application_length_q<=count==32 && payload_length==32 && staging[207:192]==16'd32;
            application_header_q<=application_header_ok;
            application_fields_q<=application_fields_ok;
            application_crc_q<=staging[15:0]==request_crc;
            application_ok_q<=application_header_q && application_fields_q && application_crc_q;
        end
    end
    reg session_valid;
    reg [63:0] session_token;
    reg [31:0] last_sequence;
    // token 为网关启动时的递增纳秒时间；新会话只接受更大 token 的零速包。
    // 防止旧会话的延迟零速先回滚会话，再让旧非零包通过；RX 复位重新建立会话。
    // 比较与裁决分两拍，使用 32 位半字比较避免 64 位比较再串接 FIFO 提交长路径。
    // 序号同高位时低 31 位严格递增；高位不同则低 31 位严格变小（模 2^32 半量程窗口）。
    reg token_equal_q,token_hi_greater_q,token_hi_equal_q,token_lo_greater_q;
    reg sequence_same_half_q,sequence_greater_q,sequence_less_q,sequence_equal_q;
    always @(posedge rx_clk or posedge rx_reset_p) begin
        if(rx_reset_p)begin
            token_equal_q<=0;token_hi_greater_q<=0;token_hi_equal_q<=0;token_lo_greater_q<=0;
            sequence_same_half_q<=0;sequence_greater_q<=0;sequence_less_q<=0;sequence_equal_q<=0;
        end else begin
            token_equal_q<=staging[159:96]==session_token;
            token_hi_greater_q<=staging[159:128]>session_token[63:32];
            token_hi_equal_q<=staging[159:128]==session_token[63:32];
            token_lo_greater_q<=staging[127:96]>session_token[31:0];
            sequence_same_half_q<=staging[191]==last_sequence[31];
            sequence_greater_q<=staging[190:160]>last_sequence[30:0];
            sequence_less_q<=staging[190:160]<last_sequence[30:0];
            sequence_equal_q<=staging[191:160]==last_sequence;
        end
    end
    wire sequence_ok=!session_valid ? stop_request :
        (token_equal_q ? (sequence_same_half_q ? sequence_greater_q : sequence_less_q) :
         (token_hi_greater_q || (token_hi_equal_q && token_lo_greater_q)) && stop_request);
    reg sequence_ok_q;
    always @(posedge rx_clk or posedge rx_reset_p)
        if(rx_reset_p)sequence_ok_q<=0;else sequence_ok_q<=sequence_ok;
    wire control_full;
    wire request_valid=application_ok_q && !pkt_error && !rx_error_seen &&
        peer_ip==PEER_IP && peer_port==PEER_PORT;
    wire request_ok=request_valid && (!control_request || sequence_ok_q);
    // 接受裁决先锁存一拍，再驱动 FIFO 和会话状态，隔离报文检查与写使能扇出。
    // 相关负载字段也在同一拍锁存，保证 pkt_done 清空 staging 后仍提交原报文。
    wire [255:0] ack={staging[255:216],(motion_request ? 8'd4 : auto_request ? 8'd6 :
                      task_request ? 8'd8 : 8'd2),staging[207:16],reply_crc};
    // 只对最近提交的完整任务分片补ACK，不把重试再次送入控制域。
    // 分段注册比较，避免256位比较直接进入125MHz报文裁决路径。
    reg [255:0] last_task_ack;
    reg last_task_ack_valid;
    reg [7:0] task_ack_word_match_q;
    integer task_ack_word;
    always @(posedge rx_clk or posedge rx_reset_p) begin
        if(rx_reset_p)task_ack_word_match_q<=0;
        else for(task_ack_word=0;task_ack_word<8;task_ack_word=task_ack_word+1)
            task_ack_word_match_q[task_ack_word]<=
                ack[task_ack_word*32+:32]==last_task_ack[task_ack_word*32+:32];
    end
    wire duplicate_task=task_request && session_valid && last_task_ack_valid &&
        token_equal_q && sequence_equal_q && (&task_ack_word_match_q);
    wire deliver_request=control_request ||
        (staging[215:208]==8'd1 && session_valid && token_equal_q);
    wire commit_candidate=pkt_done && !fifo_full &&
        ((request_ok && (!deliver_request || !control_full)) || (request_valid && duplicate_task));
    reg commit_pipe_valid,commit_pipe_motion,commit_pipe_deliver;
    reg [255:0] commit_pipe_ack;
    wire commit=commit_pipe_valid;
    always @(posedge rx_clk or posedge rx_reset_p) begin
        if(rx_reset_p)commit_pipe_valid<=0;
        else begin
            commit_pipe_valid<=commit_candidate;
            // Data is sampled every cycle; valid alone controls when a
            // snapshot is consumed, avoiding a 256-bit packet-wide enable.
            commit_pipe_motion<=control_request && !duplicate_task;
            commit_pipe_deliver<=deliver_request && !duplicate_task;
            commit_pipe_ack<=ack;
        end
    end
    // 训练用事件不随采样路径的短复位清零，避免相位切换生成伪合格帧。
    (* ASYNC_REG="TRUE" *) reg [1:0] good_reset_sync;
    always @(posedge rx_clk or posedge training_reset_p_i)
        if(training_reset_p_i)good_reset_sync<=2'b11;
        else good_reset_sync<={good_reset_sync[0],1'b0};
    always @(posedge rx_clk or posedge good_reset_sync[1])
        if(good_reset_sync[1])good_toggle_o<=0;
        else if(commit)good_toggle_o<=~good_toggle_o;
    always @(posedge rx_clk or posedge rx_reset_p) begin
        if(rx_reset_p)begin
            session_valid<=0;session_token<=0;last_sequence<=0;
            last_task_ack_valid<=0;last_task_ack<=0;
        end
        else if(commit && commit_pipe_motion)begin
            session_valid<=1;session_token<=commit_pipe_ack[159:96];last_sequence<=commit_pipe_ack[191:160];
            last_task_ack_valid<=commit_pipe_ack[215:208]==8'd8;
            if(commit_pipe_ack[215:208]==8'd8)last_task_ack<=commit_pipe_ack;
        end
    end
    (* ASYNC_REG="TRUE" *) reg [1:0] control_reset_sync;
    always @(posedge control_clk or posedge reset_p)
        if(reset_p)control_reset_sync<=2'b11;else control_reset_sync<={control_reset_sync[0],1'b0};
    wire [255:0] command_data;
    wire command_valid;
    wire [255:0] rx_command_data;
    wire rx_command_valid,rx_command_empty;
    wire command_bridge_full,command_bridge_empty,command_bridge_valid;
    wire [255:0] command_bridge_data;
    wire command_bus_fetch,command_bridge_write;
    reg command_bus_pending;
    wire direct_command_valid;
    wire [255:0] direct_command_data;
    assign auto_mode_valid_o=command_valid && command_data[215:208]==8'd6;
    assign auto_enable_o=command_data[88];
    assign control_valid_o=command_valid && command_data[215:208]==8'd4;
    assign control_velocity_o=command_data[95:48];
    assign task_fragment_valid_o=command_valid && command_data[215:208]==8'd8;
    assign task_fragment_o=command_data;
    assign control_token_o=command_data[159:96];
    // type2仅更新链路时效，不进入人工运动入口，也不取消局部任务。
    assign control_heartbeat_o=command_valid;
    generate
        if(USE_CONTROL_BRIDGE!=0)begin:g_command_control_bridge
            // fetch 时保留一个桥 FIFO 槽；pending 确保只有一个读响应在途。
            // 本 FIFO 是该桥 FIFO 唯一写者，因此 fetch 时 not-full 可保证返回数据有槽。
            assign command_bus_fetch=!command_bus_pending && !command_bridge_full &&
                !rx_command_empty && !command_bus_reset_sync[1];
            assign command_bridge_write=rx_command_valid && command_bus_pending &&
                !command_bridge_full;
            assign command_data=command_bridge_data;
            assign command_valid=command_bridge_valid;

            async_frame_fifo #(.DATA_WIDTH(256),.ADDR_WIDTH(2)) commands (
                .wr_clk_i(rx_clk),.wr_rst_n_i(!rx_reset_p),.wr_valid_i(commit && commit_pipe_deliver),
                .wr_data_i(commit_pipe_ack),.wr_full_o(control_full),.wr_overflow_o(),
                .rd_clk_i(command_bus_clk_i),.rd_rst_n_i(!command_bus_reset_sync[1]),
                .rd_ready_i(command_bus_fetch),.rd_data_o(rx_command_data),
                .rd_valid_o(rx_command_valid),.rd_empty_o(rx_command_empty));

            async_frame_fifo #(.DATA_WIDTH(256),.ADDR_WIDTH(2)) command_to_control (
                .wr_clk_i(command_bus_clk_i),.wr_rst_n_i(!command_bus_reset_sync[1]),
                .wr_valid_i(command_bridge_write),.wr_data_i(rx_command_data),
                .wr_full_o(command_bridge_full),.wr_overflow_o(),
                .rd_clk_i(control_clk),.rd_rst_n_i(!control_reset_sync[1]),
                .rd_ready_i(1'b1),.rd_data_o(command_bridge_data),
                .rd_valid_o(command_bridge_valid),.rd_empty_o(command_bridge_empty));

            always @(posedge command_bus_clk_i or posedge command_bus_reset_sync[1])begin
                if(command_bus_reset_sync[1])command_bus_pending<=1'b0;
                else begin
                    if(command_bridge_write)command_bus_pending<=1'b0;
                    else if(command_bus_fetch)command_bus_pending<=1'b1;
                end
            end
        end else begin:g_direct_control_fifo
            assign command_bus_fetch=1'b0;
            assign command_bridge_write=1'b0;
            assign command_data=direct_command_data;
            assign command_valid=direct_command_valid;
            assign rx_command_data=256'd0;
            assign rx_command_valid=1'b0;
            assign rx_command_empty=1'b1;
            assign command_bridge_full=1'b0;
            assign command_bridge_empty=1'b1;
            assign command_bridge_valid=1'b0;
            async_frame_fifo #(.DATA_WIDTH(256),.ADDR_WIDTH(2)) commands (
                .wr_clk_i(rx_clk),.wr_rst_n_i(!rx_reset_p),.wr_valid_i(commit && commit_pipe_deliver),
                .wr_data_i(commit_pipe_ack),.wr_full_o(control_full),.wr_overflow_o(),
                .rd_clk_i(control_clk),.rd_rst_n_i(!control_reset_sync[1]),.rd_ready_i(1'b1),
                .rd_data_o(direct_command_data),.rd_valid_o(direct_command_valid),.rd_empty_o());
            always @(posedge command_bus_clk_i or posedge command_bus_reset_sync[1])
                if(command_bus_reset_sync[1])command_bus_pending<=1'b0;
                else command_bus_pending<=1'b0;
        end
    endgenerate
    // 只读接收诊断，八个 16 位计数各自注册 Gray 后同步到 TX 域。
    // 顺序：时钟、帧、SFD、负载帧、完成、入队、RX_ER 帧、网络错误。
    // 计数允许回绕，不参与报文接受或运动控制。
    reg [15:0] diag_clock,diag_frame,diag_sfd,diag_payload;
    reg diag_sfd_event;
    reg [7:0] diag_sfd_byte_q;
    reg diag_sfd_window_q;
    reg [15:0] diag_done,diag_commit,diag_rxer,diag_error;
    reg [3:0] diag_preamble_index;
    reg [127:0] diag_gray;
    wire [127:0] diag_binary={diag_error,diag_rxer,diag_commit,diag_done,
        diag_payload,diag_sfd,diag_frame,diag_clock};
    integer diag_index;
    always @(posedge rx_clk or posedge rx_reset_p) begin
        if(rx_reset_p) begin
            diag_clock<=0;diag_frame<=0;diag_sfd<=0;diag_payload<=0;
            diag_done<=0;diag_commit<=0;diag_rxer<=0;diag_error<=0;
            diag_preamble_index<=0;diag_gray<=0;diag_sfd_event<=0;
            diag_sfd_byte_q<=0;diag_sfd_window_q<=0;
        end else begin
            // 先锁存同拍字节和有效窗口，再做SFD比较；只读诊断比原链晚一拍。
            diag_sfd_byte_q<=gmii_rxd;
            diag_sfd_window_q<=gmii_rxdv && diag_preamble_index<8;
            diag_sfd_event<=diag_sfd_window_q && diag_sfd_byte_q==8'hd5;
            if(diag_sfd_event)diag_sfd<=diag_sfd+1'b1;
            diag_clock<=diag_clock+1'b1;
            if(gmii_rxdv && !rx_dv_d)diag_frame<=diag_frame+1'b1;
            if(!gmii_rxdv)diag_preamble_index<=0;
            else begin
                if(diag_preamble_index<15)diag_preamble_index<=diag_preamble_index+1'b1;
            end
            if(payload_valid && count==0)diag_payload<=diag_payload+1'b1;
            if(pkt_done)diag_done<=diag_done+1'b1;
            if(commit)diag_commit<=diag_commit+1'b1;
            if(!gmii_rxdv && rx_dv_d && rx_error_seen)diag_rxer<=diag_rxer+1'b1;
            if(pkt_done && pkt_error)diag_error<=diag_error+1'b1;
            for(diag_index=0;diag_index<8;diag_index=diag_index+1)
                diag_gray[diag_index*16+:16]<=diag_binary[diag_index*16+:16]^
                    (diag_binary[diag_index*16+:16]>>1);
        end
    end
    (* ASYNC_REG="TRUE" *) reg [127:0] diag_sync1,diag_sync2;
    reg [127:0] diag_local_q;
    reg [31:0] diag_group_parity_q;
    reg [127:0] diag_tx;
    // 分组解码 Gray 计数：先在四位组内做前缀，再跨组合并，避免 16 级串接 XOR。
    function automatic [15:0] diag_decode_local;
        input [15:0] gray;
        integer group_index;
        reg [3:0] nibble;
        begin
            for(group_index=0;group_index<4;group_index=group_index+1) begin
                nibble[3]=gray[group_index*4+3];
                nibble[2]=nibble[3]^gray[group_index*4+2];
                nibble[1]=nibble[2]^gray[group_index*4+1];
                nibble[0]=nibble[1]^gray[group_index*4];
                diag_decode_local[group_index*4+:4]=nibble;
            end
        end
    endfunction
    function automatic [3:0] diag_group_parity;
        input [15:0] gray;
        begin
            diag_group_parity[3]=^gray[15:12];
            diag_group_parity[2]=^gray[11:8];
            diag_group_parity[1]=^gray[7:4];
            diag_group_parity[0]=^gray[3:0];
        end
    endfunction
    function automatic [15:0] diag_decode_combine;
        input [15:0] local_binary;
        input [3:0] group_parity;
        begin
            diag_decode_combine[15:12]=local_binary[15:12];
            diag_decode_combine[11:8]=local_binary[11:8]^{4{group_parity[3]}};
            diag_decode_combine[7:4]=local_binary[7:4]^{4{group_parity[3]^group_parity[2]}};
            diag_decode_combine[3:0]=local_binary[3:0]^{4{group_parity[3]^group_parity[2]^group_parity[1]}};
        end
    endfunction
    integer diag_tx_index;
    always @(posedge tx_clk or posedge tx_reset_p) begin
        if(tx_reset_p)begin diag_sync1<=0;diag_sync2<=0;diag_local_q<=0;diag_group_parity_q<=0;diag_tx<=0;end
        else begin
            diag_sync1<=diag_gray;diag_sync2<=diag_sync1;
            for(diag_tx_index=0;diag_tx_index<8;diag_tx_index=diag_tx_index+1) begin
                diag_local_q[diag_tx_index*16+:16]<=diag_decode_local(diag_sync2[diag_tx_index*16+:16]);
                diag_group_parity_q[diag_tx_index*4+:4]<=diag_group_parity(diag_sync2[diag_tx_index*16+:16]);
                diag_tx[diag_tx_index*16+:16]<=diag_decode_combine(
                    diag_local_q[diag_tx_index*16+:16],diag_group_parity_q[diag_tx_index*4+:4]);
            end
        end
    end
    assign diagnostics_tx_o=diag_tx;
    // 诊断保留 RX_DV 后首 8 个原始 GMII 字节；只给诊断窗口增加一拍输入寄存。
    // 控制解析仍使用原始 GMII 输入，快照与采样字节统一使用延迟后的 DV/RXD。
    (* ASYNC_REG="TRUE" *) reg [1:0] raw_reset_sync;
    always @(posedge rx_clk or posedge rx_reset_p)
        if(rx_reset_p)raw_reset_sync<=2'b11;else raw_reset_sync<={raw_reset_sync[0],1'b0};
    wire raw_reset_p=raw_reset_sync[1];
    reg [7:0] raw_rxd_q;
    reg raw_rxdv_q, raw_rxdv_d;
    reg [3:0] raw_index;
    reg [63:0] raw_prefix;
    wire raw_write=raw_rxdv_q && raw_rxdv_d && raw_index==7;
    wire [63:0] raw_snapshot_data={raw_prefix[55:0],raw_rxd_q};
    always @(posedge rx_clk or posedge raw_reset_p)begin
        if(raw_reset_p)begin
            raw_rxd_q<=0;raw_rxdv_q<=0;raw_rxdv_d<=0;
            raw_index<=0;raw_prefix<=0;
        end else begin
            raw_rxd_q<=gmii_rxd;raw_rxdv_q<=gmii_rxdv;raw_rxdv_d<=raw_rxdv_q;
            if(raw_rxdv_q && !raw_rxdv_d)begin
                raw_index<=1;raw_prefix<={56'd0,raw_rxd_q};
            end else if(raw_rxdv_q && raw_index<8)begin
                raw_prefix<={raw_prefix[55:0],raw_rxd_q};raw_index<=raw_index+1'b1;
            end
        end
    end
    wire raw_valid;
    wire [63:0] raw_data;
    async_frame_fifo #(.DATA_WIDTH(64),.ADDR_WIDTH(2)) raw_snapshots(
        .wr_clk_i(rx_clk),.wr_rst_n_i(!raw_reset_p),.wr_valid_i(raw_write),
        .wr_data_i(raw_snapshot_data),.wr_full_o(),.wr_overflow_o(),
        .rd_clk_i(tx_clk),.rd_rst_n_i(!tx_reset_p),.rd_ready_i(1'b1),
        .rd_data_o(raw_data),.rd_valid_o(raw_valid),.rd_empty_o());
    always @(posedge tx_clk or posedge tx_reset_p)
        if(tx_reset_p)raw_prefix_tx_o<=0;else if(raw_valid)raw_prefix_tx_o<=raw_data;
    // 在 FIFO 写沿使用已锁定的请求和应答 CRC，避免提前暴露未经 FCS 校验的数据。
    always @(posedge rx_clk or posedge rx_reset_p) begin
        if(rx_reset_p) begin
            staging<=0;count<=0;request_crc<=16'hffff;reply_crc<=16'hffff;
            rx_error_seen<=0;rx_dv_d<=0;
        end else begin
            rx_dv_d<=gmii_rxdv;
            if(gmii_rxdv && !rx_dv_d)rx_error_seen<=gmii_rxer;
            else if(gmii_rxdv && gmii_rxer)rx_error_seen<=1;
            if(payload_valid) begin
                if(count<32)staging<={staging[247:0],payload_byte};
                if(count<33)count<=count+1'b1;
                if(count<30) begin
                    request_crc<=crc_byte(request_crc,payload_byte);
                    reply_crc<=crc_byte(reply_crc,ack_byte);
                end
            end
            // 错误帧和正确帧都清空暂存计数；下一帧不继承 CRC 或字节。
            if(pkt_done) begin
                count<=0;request_crc<=16'hffff;reply_crc<=16'hffff;
            end
        end
    end

    wire fifo_valid, fifo_empty;
    wire [255:0] fifo_data;
    reg [255:0] held_reply;
    reg held_valid, read_pending;
    // FIFO 的 rd_valid 是出队后一拍的脉冲；用持有寄存器转换为 ready/valid。
    wire read_ready=!held_valid && !read_pending;
    async_frame_fifo #(.DATA_WIDTH(256),.ADDR_WIDTH(2),.REGISTER_FULL(1)) replies (
        .wr_clk_i(rx_clk),.wr_rst_n_i(!rx_reset_p),.wr_valid_i(commit),.wr_data_i(commit_pipe_ack),
        .wr_full_o(fifo_full),.wr_overflow_o(),
        .rd_clk_i(tx_clk),.rd_rst_n_i(!tx_reset_p),.rd_ready_i(read_ready),
        .rd_data_o(fifo_data),.rd_valid_o(fifo_valid),.rd_empty_o(fifo_empty)
    );
    always @(posedge tx_clk or posedge tx_reset_p) begin
        if(tx_reset_p)begin held_reply<=0;held_valid<=0;read_pending<=0;end
        else begin
            if(read_ready && !fifo_empty)read_pending<=1;
            if(fifo_valid)begin held_reply<=fifo_data;held_valid<=1;read_pending<=0;end
            if(held_valid && reply_ready)held_valid<=0;
        end
    end
    assign reply_valid=held_valid;
    assign reply_data=held_reply;
endmodule

// 发送域共享包读取器：先确认整包跨域可见，再预取不超过当前包的字节。
// 长度为1..255字节；start不消耗数据，但允许start与首次read同拍。
module bx72_telemetry_packet_reader (
    input wire tx_clk, tx_reset,
    input wire descriptor_empty, descriptor_valid,
    input wire [7:0] descriptor_data,
    output wire descriptor_read,
    input wire fifo_byte_valid,
    input wire [7:0] fifo_byte_data,
    input wire [11:0] byte_count,
    output wire byte_prefetch,
    output reg tx_valid_o, tx_length_nonzero_o,
    output reg [7:0] tx_length_o,
    output wire [7:0] tx_data_o,
    input wire tx_start_i, tx_read_i
);
    localparam [2:0] TX_IDLE=3'd0, TX_DESCRIPTOR=3'd1, TX_BYTES=3'd2,
                     TX_PREFETCH=3'd3, TX_OFFER=3'd4, TX_ACTIVE=3'd5;

    reg [2:0] tx_state;
    reg [7:0] tx_remaining;
    reg [7:0] prefetch_remaining;
    reg [1:0] byte_buffer_count;
    reg [7:0] byte_head,byte_tail;
    // 提前一字节判断包尾，避免高扇出的125 MHz读请求穿过8位比较和状态译码。
    reg tx_last_byte;
    reg tx_accept_read;
    reg tx_count_compare_valid;
    reg tx_count_upper_nonzero,tx_count_nibble_gt,tx_count_nibble_equal;
    reg tx_count_low_nibble_ge;
    wire tx_count_reached_length=tx_count_upper_nonzero || tx_count_nibble_gt ||
        (tx_count_nibble_equal && tx_count_low_nibble_ge);
    assign descriptor_read=(tx_state==TX_IDLE) && !descriptor_empty && !tx_reset;
    wire byte_valid=(byte_buffer_count!=0);
    wire start_fire=tx_valid_o && tx_start_i && (tx_state==TX_OFFER) && !tx_reset;
    wire byte_read=tx_read_i && !tx_reset && (tx_accept_read || start_fire);
    assign byte_prefetch=((tx_state==TX_PREFETCH)||(tx_state==TX_ACTIVE)) &&
        (byte_buffer_count<2) && (prefetch_remaining!=0) &&
        fifo_byte_valid && !tx_reset;
    assign tx_data_o=byte_head;

    // 两字节本地缓冲隔离 FIFO 输出和发送器；不跨 descriptor 预取下一包。
    always @(posedge tx_clk or posedge tx_reset) begin
        if(tx_reset) begin
            byte_head<=0;
            byte_tail<=0;
            byte_buffer_count<=0;
        end else begin
            if(byte_prefetch && byte_buffer_count!=0)byte_tail<=fifo_byte_data;
            case({byte_prefetch,byte_read})
                2'b10: begin
                    if(byte_buffer_count==0)byte_head<=fifo_byte_data;
                    byte_buffer_count<=byte_buffer_count+1'b1;
                end
                2'b01: begin
                    byte_head<=byte_tail;
                    byte_buffer_count<=byte_buffer_count-1'b1;
                end
                2'b11: begin
                    byte_head<=fifo_byte_data;
                end
                default: begin end
            endcase
        end
    end

    always @(posedge tx_clk or posedge tx_reset) begin
        if(tx_reset) begin
            tx_state<=TX_IDLE;
            tx_valid_o<=0;
            tx_length_nonzero_o<=0;
            tx_length_o<=0;
            tx_remaining<=0;
            prefetch_remaining<=0;
            tx_last_byte<=0;
            tx_accept_read<=0;
            tx_count_compare_valid<=0;
            tx_count_upper_nonzero<=0;
            tx_count_nibble_gt<=0;
            tx_count_nibble_equal<=0;
            tx_count_low_nibble_ge<=0;
        end else begin
            case(tx_state)
                TX_IDLE: if(descriptor_read)tx_state<=TX_DESCRIPTOR;
                TX_DESCRIPTOR: if(descriptor_valid) begin
                    tx_length_o<=descriptor_data;
                    tx_length_nonzero_o<=(descriptor_data!=0);
                    tx_remaining<=descriptor_data;
                    tx_last_byte<=(descriptor_data==1);
                    tx_count_compare_valid<=0;
                    tx_state<=TX_BYTES;
                end
                // 等待整包字节跨域可见；长度比较分段寄存避免宽比较链。
                TX_BYTES: begin
                    tx_count_compare_valid<=1'b1;
                    tx_count_upper_nonzero<=|byte_count[11:8];
                    tx_count_nibble_gt<=byte_count[7:4]>tx_length_o[7:4];
                    tx_count_nibble_equal<=byte_count[7:4]==tx_length_o[7:4];
                    tx_count_low_nibble_ge<=byte_count[3:0]>=tx_length_o[3:0];
                    if(fifo_byte_valid && tx_count_compare_valid && tx_count_reached_length) begin
                        prefetch_remaining<=tx_length_o;
                        tx_state<=TX_PREFETCH;
                    end
                end
                TX_PREFETCH: begin
                    if(byte_prefetch)prefetch_remaining<=prefetch_remaining-1'b1;
                    if((byte_buffer_count>=2) ||
                       ((tx_length_o==1) && (byte_buffer_count==1))) begin
                        tx_valid_o<=1'b1;
                        tx_state<=TX_OFFER;
                    end
                end
                TX_OFFER: if(start_fire && byte_valid) begin
                    tx_valid_o<=1'b0;
                    tx_accept_read<=!(tx_read_i && tx_last_byte);
                    if(tx_read_i) begin
                        if(tx_last_byte)tx_state<=TX_IDLE;
                        else tx_state<=TX_ACTIVE;
                    end else begin
                        tx_state<=TX_ACTIVE;
                    end
                end
                TX_ACTIVE: if(tx_read_i) begin
                    if(tx_last_byte)begin
                        tx_accept_read<=1'b0;
                        tx_state<=TX_IDLE;
                    end
                end
                default: begin
                    tx_state<=TX_IDLE;
                    tx_valid_o<=0;
                    tx_length_nonzero_o<=0;
                end
            endcase
            if(byte_read) begin
                tx_remaining<=tx_remaining-1'b1;
                tx_last_byte<=(tx_remaining==2);
            end
            if(tx_state==TX_ACTIVE && byte_prefetch)
                prefetch_remaining<=prefetch_remaining-1'b1;
        end
    end
endmodule

// ==================== bx72_telemetry.sv ====================
`timescale 1ns/1ps

// 各输入 valid 表示一次完整快照，向量最高字节最先发送。
// 每类保留一个待发快照；队列堵塞时丢弃该类新快照并计数，序号仍递增。
// 源时钟域逐字节写 FIFO，整包提交后才发布长度，避免 TX 域的宽总线选通。
module bx72_telemetry #(
    parameter integer SRC_CLK_HZ = 33333333,
    parameter [7:0] CHANNEL_MASK = 8'b01111111,
    parameter integer USE_EXTERNAL_SRC_RESET = 0,
    parameter integer USE_EXTERNAL_TX_RESET = 0
) (
    input wire src_clk, tx_clk, reset_n,
    input wire imu_valid_i,
    input wire [255:0] imu_payload_i,
    input wire motor_valid_i,
    input wire [703:0] motor_payload_i,
    input wire lidar_valid_i,
    input wire [1151:0] lidar_payload_i,
    input wire fusion_valid_i,
    input wire [831:0] fusion_payload_i,
    input wire health_valid_i,
    input wire [895:0] health_payload_i,
    input wire navigation_valid_i,input wire [543:0] navigation_payload_i,
    // 本地规划与栅格都由控制时钟域快照；栅格通过ready保护整包一致性。
    input wire local_plan_valid_i,input wire [319:0] local_plan_payload_i,
    input wire local_grid_valid_i,input wire [1663:0] local_grid_payload_i,
    output wire local_grid_ready_o,
    output wire tx_valid_o,
    output wire [7:0] tx_length_o,
    // 与长度同拍发布，供发送仲裁直接使用，避免在 125 MHz 域重做归约。
    output wire tx_length_nonzero_o,
    output wire [7:0] tx_data_o,
    input wire tx_start_i, tx_read_i,
    // 源时钟域计数：从低到高依次为 IMU、电机、雷达、融合、健康状态。
    output wire [159:0] dropped_o,
    // 生产双时钟实例可复用同域复位，避免等价释放链被综合合并。
    input wire src_reset_i,
    input wire tx_reset_i,
    output wire tx_reset_o
);
    wire src_reset;
    wire tx_reset;
    generate
        if (USE_EXTERNAL_SRC_RESET != 0) begin : g_external_src_reset
            assign src_reset = src_reset_i;
        end else begin : g_internal_src_reset
            (* ASYNC_REG="TRUE" *) reg [1:0] src_reset_sync;
            always @(posedge src_clk or negedge reset_n)
                if (!reset_n) src_reset_sync <= 2'b11;
                else src_reset_sync <= {src_reset_sync[0], 1'b0};
            assign src_reset = src_reset_sync[1];
        end
        if (USE_EXTERNAL_TX_RESET != 0) begin : g_external_tx_reset
            assign tx_reset = tx_reset_i;
        end else begin : g_internal_tx_reset
            (* ASYNC_REG="TRUE" *) reg [1:0] tx_reset_sync;
            always @(posedge tx_clk or negedge reset_n)
                if (!reset_n) tx_reset_sync <= 2'b11;
                else tx_reset_sync <= {tx_reset_sync[0], 1'b0};
            assign tx_reset = tx_reset_sync[1];
        end
    endgenerate
    assign tx_reset_o = tx_reset;

    // 分数分频消除 33.333333 MHz 四舍五入为 33 MHz 所造成的时间累计误差。
    localparam integer PHASE_WIDTH = bx72_clock_contract::phase_width(SRC_CLK_HZ);
    reg [PHASE_WIDTH-1:0] us_phase;
    reg [31:0] time_us;
    wire [PHASE_WIDTH-1:0] next_us_phase;
    wire us_tick;
    bx72_us_phase #(.CLK_HZ(SRC_CLK_HZ)) source_time_phase(.phase_i(us_phase),.next_phase_o(next_us_phase),.tick_o(us_tick));
    always @(posedge src_clk or posedge src_reset) begin
        if (src_reset) begin us_phase <= 0; time_us <= 0; end
        else begin
            us_phase<=next_us_phase;
            if(us_tick)time_us<=time_us+1'b1;
        end
    end

    reg [7:0] pending;
    reg [255:0] imu_snapshot;
    reg [703:0] motor_snapshot;
    reg [1151:0] lidar_snapshot;
    reg [831:0] fusion_snapshot;
    reg [895:0] health_snapshot;
    reg [543:0] navigation_snapshot;
    reg [319:0] local_plan_snapshot;
    reg [1663:0] local_grid_snapshot;
    reg [31:0] sequence_next [0:7];
    reg [31:0] snapshot_sequence [0:7];
    reg [31:0] snapshot_time [0:7];
    reg [31:0] dropped [0:7];
    assign dropped_o = {dropped[4],dropped[3],dropped[2],dropped[1],dropped[0]};
    assign local_grid_ready_o=!pending[7] && !src_reset;
    wire [7:0] sample_valid = {
        (local_grid_valid_i===1'b1),(local_plan_valid_i===1'b1),
        (navigation_valid_i===1'b1),health_valid_i,fusion_valid_i,
        lidar_valid_i,motor_valid_i,imu_valid_i
    } & CHANNEL_MASK;

    reg [2:0] round_robin;
    reg selected_valid;
    reg [2:0] selected;
    integer scan, candidate;
    always @* begin
        selected_valid = 1'b0;
        selected = 0;
        // 反向扫描使 round_robin 指向的一类具有最高优先级。
        for (scan = 7; scan >= 0; scan = scan - 1) begin
            candidate = round_robin + scan;
            if (candidate >= 8) candidate = candidate - 8;
            if (pending[candidate]) begin
                selected_valid = 1'b1;
                selected = candidate[2:0];
            end
        end
    end

    localparam [7:0] PACKET_FIRST_BYTE = 8'h42;
    reg serializing;
    reg [2:0] active_channel;
    reg [7:0] byte_index;
    // 已暂存的字节由 serializing 表示有效；bytes_left 统计尚未暂存的字节。
    reg [7:0] serializer_stage_data;
    reg [7:0] bytes_left;
    reg [7:0] packet_length;
    reg [31:0] active_drop_snapshot;
    wire byte_full, fifo_byte_valid;
    wire [7:0] fifo_byte_data;
    wire byte_prefetch;
    wire [11:0] byte_count;
    wire descriptor_full, descriptor_empty, descriptor_valid;
    wire [7:0] descriptor_data;
    wire select_fire = !serializing && selected_valid && !descriptor_full;
    wire byte_write = serializing && !byte_full;
    wire descriptor_write = byte_write && (bytes_left == 0);
    wire descriptor_read;
    reg [31:0] active_sequence;
    reg [31:0] active_timestamp;

    function [7:0] header_byte_at;
        input [7:0] index;
        input [2:0] channel_id;
        input [7:0] length;
        input [31:0] sequence_number;
        input [31:0] timestamp_us;
        reg [7:0] packet_type;
        begin
            case (channel_id)
                0: packet_type=8'd3;
                1: packet_type=8'd4;
                2: packet_type=8'd5;
                3: packet_type=8'd6;
                4: packet_type=8'd7;
                5: packet_type=8'd8;
                6: packet_type=8'd9;
                7: packet_type=8'd10;
                default: packet_type=8'd0;
            endcase
            case (index)
                0: header_byte_at=PACKET_FIRST_BYTE;
                1: header_byte_at=8'h58;
                2: header_byte_at=8'h37;
                3: header_byte_at=8'h32;
                4: header_byte_at=8'd1;
                5: header_byte_at=packet_type;
                6: header_byte_at=8'd0;
                7: header_byte_at=length[7:0];
                8: header_byte_at=sequence_number[31:24];
                9: header_byte_at=sequence_number[23:16];
                10: header_byte_at=sequence_number[15:8];
                11: header_byte_at=sequence_number[7:0];
                12: header_byte_at=timestamp_us[31:24];
                13: header_byte_at=timestamp_us[23:16];
                14: header_byte_at=timestamp_us[15:8];
                15: header_byte_at=timestamp_us[7:0];
                default: header_byte_at=8'd0;
            endcase
        end
    endfunction

    function [7:0] payload_byte_at;
        input [2:0] channel_id;
        input [7:0] byte_offset;
        input [31:0] drop_count;
        reg [5:0] word_index;
        reg [7:0] drop_byte;
        begin
            word_index=byte_offset[7:2];
            drop_byte=drop_count[31-{byte_offset[1:0],3'b0} -: 8];
            case (channel_id)
                0: begin
                    if (word_index == 6'd7) payload_byte_at=drop_byte;
                    else if (word_index < 6'd7) payload_byte_at=imu_snapshot[255-{byte_offset,3'b0} -: 8];
                    else payload_byte_at=8'd0;
                end
                1: begin
                    if (word_index == 6'd21) payload_byte_at=drop_byte;
                    else if (word_index < 6'd22) payload_byte_at=motor_snapshot[703-{byte_offset,3'b0} -: 8];
                    else payload_byte_at=8'd0;
                end
                2: begin
                    if (word_index == 6'd3) payload_byte_at=drop_byte;
                    else if (word_index < 6'd36) payload_byte_at=lidar_snapshot[1151-{byte_offset,3'b0} -: 8];
                    else payload_byte_at=8'd0;
                end
                3: begin
                    if (word_index < 6'd26) payload_byte_at=fusion_snapshot[831-{byte_offset,3'b0} -: 8];
                    else payload_byte_at=8'd0;
                end
                4: begin
                    if (word_index < 6'd28) payload_byte_at=health_snapshot[895-{byte_offset,3'b0} -: 8];
                    else payload_byte_at=8'd0;
                end
                5: begin
                    if (word_index < 6'd17) payload_byte_at=navigation_snapshot[543-{byte_offset,3'b0} -: 8];
                    else payload_byte_at=8'd0;
                end
                6: begin
                    if (word_index < 6'd10) payload_byte_at=local_plan_snapshot[319-{byte_offset,3'b0} -: 8];
                    else payload_byte_at=8'd0;
                end
                7: begin
                    if (word_index < 6'd52) payload_byte_at=local_grid_snapshot[1663-{byte_offset,3'b0} -: 8];
                    else payload_byte_at=8'd0;
                end
                default: payload_byte_at=8'd0;
            endcase
        end
    endfunction

    // 包头使用 F0..FF，随后自然回绕到载荷索引 00，省去逐字节减 16。
    // 包长不超过 255 字节，载荷索引不会再次进入 F0..FF。
    reg [7:0] serializer_data;
    always @* begin
        if (byte_index[7:4] == 4'hf) begin
            serializer_data=header_byte_at({4'd0,byte_index[3:0]},active_channel,packet_length,active_sequence,active_timestamp);
        end else begin
            serializer_data=payload_byte_at(active_channel,byte_index,active_drop_snapshot);
        end
    end

    integer channel;
    always @(posedge src_clk or posedge src_reset) begin
        if (src_reset) begin
            pending <= 0;
            round_robin <= 0;
            serializing <= 0;
            active_channel <= 0;
            byte_index <= 0;
            serializer_stage_data <= 0;
            bytes_left <= 0;
            packet_length <= 0;
            active_drop_snapshot <= 0;
            active_sequence <= 0;
            active_timestamp <= 0;
            for (channel=0; channel<8; channel=channel+1) begin
                sequence_next[channel] <= 0;
                snapshot_sequence[channel] <= 0;
                snapshot_time[channel] <= 0;
                dropped[channel] <= 0;
            end
        end else begin
            if (select_fire) begin
                serializing <= 1'b1;
                active_channel <= selected;
                // 首字节固定，选包同拍装入，后续每次写出并行计算下一字节。
                serializer_stage_data <= PACKET_FIRST_BYTE;
                byte_index <= 8'hf1;
                active_drop_snapshot <= dropped[selected];
                active_sequence <= snapshot_sequence[selected];
                active_timestamp <= snapshot_time[selected];
                round_robin <= selected == 3'd7 ? 3'd0 : selected + 3'd1;
                // 各类型沿用固定长度和历史字段槽；规划及栅格载荷逐字保真。
                case (selected)
                    0: begin packet_length<=8'd48;  bytes_left<=8'd47;  end
                    1: begin packet_length<=8'd104; bytes_left<=8'd103; end
                    2: begin packet_length<=8'd160; bytes_left<=8'd159; end
                    3: begin packet_length<=8'd120; bytes_left<=8'd119; end
                    4: begin packet_length<=8'd128; bytes_left<=8'd127; end
                    5: begin packet_length<=8'd84; bytes_left<=8'd83; end
                    6: begin packet_length<=8'd56; bytes_left<=8'd55; end
                    7: begin packet_length<=8'd224; bytes_left<=8'd223; end
                    default: begin packet_length<=8'd0; bytes_left<=8'd0; end
                endcase
            end else if (byte_write) begin
                if (bytes_left == 0) begin
                    serializing <= 1'b0;
                    pending[active_channel] <= 1'b0;
                    byte_index <= 0;
                    bytes_left <= 0;
                end else begin
                    serializer_stage_data <= serializer_data;
                    byte_index <= byte_index + 1'b1;
                    bytes_left <= bytes_left - 1'b1;
                end
            end

            // pending覆盖候选、活动序列化及整包写入；同拍释放/选择也不覆写快照。
            for (channel=0; channel<8; channel=channel+1) begin
                if (sample_valid[channel]) begin
                    sequence_next[channel] <= sequence_next[channel] + 1'b1;
                    if (!pending[channel]) begin
                        pending[channel] <= 1'b1;
                        snapshot_sequence[channel] <= sequence_next[channel];
                        snapshot_time[channel] <= time_us;
                        case (channel)
                            0: imu_snapshot <= imu_payload_i;
                            1: motor_snapshot <= motor_payload_i;
                            2: lidar_snapshot <= lidar_payload_i;
                            3: fusion_snapshot <= fusion_payload_i;
                            4: health_snapshot <= health_payload_i;
                            5: navigation_snapshot <= navigation_payload_i;
                            6: local_plan_snapshot <= local_plan_payload_i;
                            7: local_grid_snapshot <= local_grid_payload_i;
                        endcase
                    end else dropped[channel] <= dropped[channel] + 1'b1;
                end
            end
        end
    end

    async_pixel_fifo #(.ADDR_WIDTH(11),.DATA_WIDTH(8)) byte_fifo (
        .arst(!reset_n),.wr_clk(src_clk),.wr_en(byte_write),.wr_data(serializer_stage_data),
        .full(byte_full),.overflow_sticky(),.rd_clk(tx_clk),.rd_en(byte_prefetch),
        .rd_data(fifo_byte_data),.rd_valid(fifo_byte_valid),.rd_count(byte_count)
    );
    async_frame_fifo #(.ADDR_WIDTH(4),.DATA_WIDTH(8)) length_fifo (
        .wr_clk_i(src_clk),.wr_rst_n_i(!src_reset),.wr_valid_i(descriptor_write),
        .wr_data_i(packet_length),.wr_full_o(descriptor_full),.wr_overflow_o(),
        .rd_clk_i(tx_clk),.rd_rst_n_i(!tx_reset),.rd_ready_i(descriptor_read),
        .rd_data_o(descriptor_data),.rd_valid_o(descriptor_valid),.rd_empty_o(descriptor_empty)
    );

    bx72_telemetry_packet_reader packet_reader (
        .tx_clk(tx_clk), .tx_reset(tx_reset),
        .descriptor_empty(descriptor_empty), .descriptor_valid(descriptor_valid),
        .descriptor_data(descriptor_data), .descriptor_read(descriptor_read),
        .fifo_byte_valid(fifo_byte_valid), .fifo_byte_data(fifo_byte_data),
        .byte_count(byte_count), .byte_prefetch(byte_prefetch),
        .tx_valid_o(tx_valid_o), .tx_length_nonzero_o(tx_length_nonzero_o),
        .tx_length_o(tx_length_o), .tx_data_o(tx_data_o),
        .tx_start_i(tx_start_i), .tx_read_i(tx_read_i)
    );
endmodule

// ==================== eth_udp_video_sender.v ====================
`timescale 1ns/1ps

// 16位视频字流；BX72活动格式为YUYV（PT97），历史RGB565为PT96。
// The caller supplies a pixel stream and frame VSYNC; an asynchronous FIFO
// carries that stream to the Ethernet TX clock domain.
module eth_udp_video_sender #(
    parameter integer TX_CLK_HZ = 125000000,
    parameter [6:0] VIDEO_RTP_PAYLOAD_TYPE = 7'd97,
    parameter integer STATUS_INTERVAL_CYCLES = TX_CLK_HZ,
    parameter integer IMAGE_WIDTH = 640,
    parameter integer IMAGE_HEIGHT = 360,
    parameter integer MAX_PIXELS_PER_PACKET = 730,
    parameter integer FIFO_ADDR_WIDTH = 11,
    parameter [47:0] DST_MAC = 48'hFF_FF_FF_FF_FF_FF,
    parameter [47:0] SRC_MAC = 48'h00_0A_35_01_FE_C0,
    parameter [31:0] DST_IP = 32'hC0_A8_00_04,
    parameter [31:0] SRC_IP = 32'hC0_A8_00_02,
    parameter [15:0] DST_PORT = 16'd6102,
    parameter [15:0] SRC_PORT = 16'd5000,
    parameter [31:0] RTP_TIMESTAMP_STEP = 32'd3000,
    parameter integer USE_EXTERNAL_TIMESTAMP = 0,
    parameter [15:0] REPLY_PORT = 16'd6103,
    parameter integer RX_DIAGNOSTICS_ENABLE = 0,
    parameter integer TELEMETRY_ENABLE = 0,
    parameter [15:0] TELEMETRY_PORT = 16'd6104
) (
    input  wire        clk50m,
    input  wire        clk125m,
    input  wire        reset_p,
    input  wire        cam_pclk,
    input  wire        camera_vsync,
    input  wire        camera_pixel_valid,
    input  wire [15:0] camera_pixel,
    input  wire [31:0] camera_timestamp,
    input  wire        reply_valid,
    input  wire [255:0] reply_data,
    input  wire [127:0] rx_diagnostics_i,
    input  wire rx_diagnostics_raw_i,
    // 完整遥测包已提交后才置 valid；start 后保持首字节，read 每次消费一字节。
    // 全部信号属于 clk125m，数据来自上游 FIFO 的寄存读口；包内不允许断流。
    input  wire        telemetry_valid,
    input  wire [15:0] telemetry_length,
    input  wire        telemetry_length_nonzero,
    input  wire [7:0]  telemetry_data,
    output reg         telemetry_start,
    output wire        telemetry_read,
    output wire        telemetry_read_request,
    output wire        reply_ready,
    output wire        camera_pixel_ready,
    output wire        rgmii_gtxc,
    output wire [3:0]  rgmii_txd,
    output wire        rgmii_txen,
    output wire        eth_mdc,
    inout  wire        eth_mdio,
    output wire        eth_rst_n,
    output wire        phy_configured,
    output wire        phy_link_up,
    output reg  [31:0] packet_count,
    output reg  [31:0] packet_count_gray,
    output reg  [31:0] video_packet_count,
    output reg  [31:0] video_packet_count_gray,
    output wire        video_fifo_overflow,
    output reg         video_source_underflow,
    output reg         video_frame_marker_error
);
    localparam [31:0] FRAME_PIXELS = IMAGE_WIDTH * IMAGE_HEIGHT;
    // 按图像尺寸缩窄计数器，减少 125 MHz 发包门限的比较路径。
    localparam integer PIXEL_COUNT_WIDTH =
        ($clog2(FRAME_PIXELS + 1) > 10) ? $clog2(FRAME_PIXELS + 1) : 10;
    localparam integer STATUS_COUNT_WIDTH =
        (STATUS_INTERVAL_CYCLES > 1) ? $clog2(STATUS_INTERVAL_CYCLES) : 1;
    localparam [9:0] MAX_PACKET_PIXELS = MAX_PIXELS_PER_PACKET;
    localparam [15:0] STATUS_PAYLOAD_LENGTH = 16'd32;
    localparam [10:0] RTP_HEADER_LENGTH = 11'd12;
    localparam [3:0]  ETHERNET_IFG_BYTES = 4'd12;
    localparam [1:0]  LAUNCH_REPLY=2'd0,LAUNCH_STATUS=2'd1,
                      LAUNCH_TELEMETRY=2'd2,LAUNCH_VIDEO=2'd3;

    // 各域异步置位、同步释放复位，避免 PLL 锁定信号直接释放状态机。
    (* ASYNC_REG="TRUE" *) reg [1:0] cam_reset_sync, tx_reset_sync, phy_reset_sync;
    always @(posedge cam_pclk or posedge reset_p)
        if(reset_p)cam_reset_sync<=2'b11;else cam_reset_sync<={cam_reset_sync[0],1'b0};
    always @(posedge clk125m or posedge reset_p)
        if(reset_p)tx_reset_sync<=2'b11;else tx_reset_sync<={tx_reset_sync[0],1'b0};
    always @(posedge clk50m or posedge reset_p)
        if(reset_p)phy_reset_sync<=2'b11;else phy_reset_sync<={phy_reset_sync[0],1'b0};
    wire cam_reset_p=cam_reset_sync[1],tx_reset_p=tx_reset_sync[1],phy_reset_p=phy_reset_sync[1];

    wire phy_init_done;
    wire phy_link_raw;
    wire [15:0] phy_rd_data;
    wire tx_done;
    wire payload_req, payload_req_telemetry;
    wire [7:0] payload_data;
    wire gmii_tx_clk;
    wire gmii_txen;
    wire [7:0] gmii_txd;
    reg tx_en_pulse;
    reg tx_busy;
    reg packet_is_video;
    reg packet_is_reply;
    reg packet_is_telemetry;
    reg telemetry_yield_video;
    reg packet_launch_pending;
    reg [1:0] packet_launch_kind;
    reg [15:0] launch_telemetry_length;
    reg [15:0] launch_rtp_sequence;
    reg [31:0] launch_rtp_timestamp;
    reg [15:0] launch_payload_length;
    reg [9:0] pending_launch_pixels,pending_launch_last_pixel_index;
    reg launch_starts_frame,launch_last_fragment;
    reg [255:0] packet_reply;
    reg [127:0] status_diagnostics_shift;
    reg packet_rx_diagnostics_raw;
    reg status_due;
    reg [STATUS_COUNT_WIDTH-1:0] status_interval_count;
    // 分组进位代替贯穿 27 位的进位链，保持状态包周期完全一致。
    reg status_carry16;
    wire [STATUS_COUNT_WIDTH-1:0] status_count_next;
    generate if(STATUS_COUNT_WIDTH>16) begin: g_status_split
        wire [15:0] low_next=status_interval_count[15:0]+16'd1;
        wire [STATUS_COUNT_WIDTH-17:0] high_next=status_interval_count[STATUS_COUNT_WIDTH-1:16]+status_carry16;
        assign status_count_next={high_next,low_next};
    end else begin: g_status_short
        assign status_count_next=status_interval_count+1'b1;
    end endgenerate
    reg [10:0] payload_index;
    reg pixel_payload_active;
    reg packet_starts_frame;
    reg [9:0] packet_last_pixel_index,ready_last_pixel_index;
    reg [15:0] ready_payload_length;
    reg ready_last_fragment;
    reg [15:0] packet_payload_length;
    reg [PIXEL_COUNT_WIDTH-1:0] video_pixels_remaining;
    reg [9:0] packet_pixels;
    reg packet_is_last_video_fragment;
    reg [9:0] packet_pixels_consumed;
    reg pixel_byte_phase;
    reg [15:0] rtp_sequence;
    reg [31:0] rtp_timestamp;
    reg [3:0] ifg_count;
    // 空闲资格与 IFG 计数分开寄存，避免仲裁反复比较计数器。
    reg ifg_ready;

    reg [PIXEL_COUNT_WIDTH-1:0] camera_frame_pixel_count;
    reg camera_vsync_d;
    reg video_capture_enable;
    reg video_packet_ready_q;
    reg [9:0] ready_launch_pixels;
    (* ASYNC_REG="TRUE" *) reg phy_link_cam_sync1;
    (* ASYNC_REG="TRUE" *) reg phy_link_cam_sync2;
    (* ASYNC_REG="TRUE" *) reg phy_init_tx_sync1;
    (* ASYNC_REG="TRUE" *) reg phy_init_tx_sync2;
    (* ASYNC_REG="TRUE" *) reg phy_link_tx_sync1;
    (* ASYNC_REG="TRUE" *) reg phy_link_tx_sync2;
    (* ASYNC_REG="TRUE" *) reg fifo_overflow_tx_sync1;
    (* ASYNC_REG="TRUE" *) reg fifo_overflow_tx_sync2;
    reg video_frame_active;

    wire timestamp_full,timestamp_valid,timestamp_overflow;
    wire [31:0] fifo_timestamp;
    wire timestamp_write_ready=!(USE_EXTERNAL_TIMESTAMP && camera_frame_pixel_count==0 && timestamp_full);
    wire fifo_wr_en = camera_pixel_valid && video_capture_enable && phy_link_cam_sync2 && timestamp_write_ready;
    // 帧标记与像素保持 18 位，时间戳单独排队，每帧仅写一次。
    localparam integer FIFO_DATA_WIDTH=18;
    wire [FIFO_DATA_WIDTH-1:0] fifo_wr_data = {
        (camera_frame_pixel_count == FRAME_PIXELS - 1),
        (camera_frame_pixel_count == 0),
        camera_pixel
    };
    wire fifo_full;
    wire fifo_overflow_cam;
    wire fifo_rd_valid;
    wire [FIFO_DATA_WIDTH-1:0] fifo_rd_data;
    wire [FIFO_ADDR_WIDTH:0] fifo_rd_count;
    wire fifo_rd_en;
    wire discard_until_sof = !tx_busy && !video_frame_active && fifo_rd_valid && !fifo_rd_data[16];
    // SOF 与时间戳就绪先寄存；SOF 队头在开始帧之前不会被弹出。
    // 保留当前 busy/active 门控，避免与刚启动的状态包争用发送器。
    reg frame_head_ready_q;
    wire begin_video_frame=frame_head_ready_q && !tx_busy && !video_frame_active;
    always @(posedge clk125m or posedge tx_reset_p)
        if(tx_reset_p)frame_head_ready_q<=0;
        else frame_head_ready_q<=!tx_busy && !video_frame_active &&
            fifo_rd_valid && fifo_rd_data[16] && timestamp_valid;
    wire [15:0] packet_low_next=packet_count[15:0]+16'd1;
    wire [15:0] video_low_next=video_packet_count[15:0]+16'd1;
    wire [15:0] packet_high_next=packet_count[31:16]+16'd1;
    wire [15:0] video_high_next=video_packet_count[31:16]+16'd1;
    wire [31:0] packet_count_next={(&packet_count[15:0]) ? packet_high_next : packet_count[31:16],packet_low_next};
    wire [31:0] video_count_next={(&video_packet_count[15:0]) ? video_high_next : video_packet_count[31:16],video_low_next};

    async_pixel_fifo #(
        .ADDR_WIDTH(FIFO_ADDR_WIDTH),
        .DATA_WIDTH(FIFO_DATA_WIDTH)
    ) camera_to_eth_fifo (
        .arst(reset_p),
        .wr_clk(cam_pclk),
        .wr_en(fifo_wr_en),
        .wr_data(fifo_wr_data),
        .full(fifo_full),
        .overflow_sticky(fifo_overflow_cam),
        .rd_clk(clk125m),
        .rd_en(fifo_rd_en),
        .rd_data(fifo_rd_data),
        .rd_valid(fifo_rd_valid),
        .rd_count(fifo_rd_count)
    );

    generate if(USE_EXTERNAL_TIMESTAMP) begin: g_timestamp_queue
        // 两路队列仅在首像素成功写入时同时提交，读端先锁存时间戳再启动发包。
        async_pixel_fifo #(.ADDR_WIDTH(2),.DATA_WIDTH(32)) frame_timestamp_fifo (
            .arst(reset_p),.wr_clk(cam_pclk),
            .wr_en(fifo_wr_en && !fifo_full && camera_frame_pixel_count==0),
            .wr_data(camera_timestamp),.full(timestamp_full),.overflow_sticky(timestamp_overflow),
            .rd_clk(clk125m),.rd_en(begin_video_frame),.rd_data(fifo_timestamp),
            .rd_valid(timestamp_valid),.rd_count());
    end else begin: g_legacy_timestamp
        assign timestamp_full=1'b0;
        assign timestamp_valid=1'b1;
        assign timestamp_overflow=1'b0;
        assign fifo_timestamp=32'd0;
    end endgenerate

    assign phy_configured = phy_init_done;
    assign phy_link_up = phy_link_raw;
    assign video_fifo_overflow = fifo_overflow_tx_sync2;
    // DDR 读端在 FIFO 满或链路断开时保持当前像素，不制造缺像素帧。
    assign camera_pixel_ready=!fifo_full && phy_link_cam_sync2 && video_capture_enable && timestamp_write_ready;

    phy_reg_config phy_config (
        .Clk(clk50m),
        .Rst_n(~phy_reset_p),
        .Phy_rst_n(eth_rst_n),
        .Rd_Data(phy_rd_data),
        .Phy_init_done(phy_init_done),
        .Phy_link_up(phy_link_raw),
        .mdc(eth_mdc),
        .mdio(eth_mdio)
    );

    always @(posedge cam_pclk or posedge cam_reset_p) begin
        if (cam_reset_p) begin
            camera_frame_pixel_count <= {PIXEL_COUNT_WIDTH{1'b0}};
            camera_vsync_d <= 1'b0;
            video_capture_enable <= 1'b0;
            phy_link_cam_sync1 <= 1'b0;
            phy_link_cam_sync2 <= 1'b0;
        end else begin
            phy_link_cam_sync1 <= phy_link_raw;
            phy_link_cam_sync2 <= phy_link_cam_sync1;
            camera_vsync_d <= camera_vsync;

            if (camera_vsync && !camera_vsync_d)
                video_capture_enable <= 1'b1;

            if (camera_vsync && !camera_vsync_d) begin
                camera_frame_pixel_count <= {PIXEL_COUNT_WIDTH{1'b0}};
            end else if (camera_pixel_valid && (!USE_EXTERNAL_TIMESTAMP || camera_pixel_ready)) begin
                if (camera_frame_pixel_count >= FRAME_PIXELS - 1)
                    camera_frame_pixel_count <= {PIXEL_COUNT_WIDTH{1'b0}};
                else
                    camera_frame_pixel_count <= camera_frame_pixel_count + 1'b1;
            end
        end
    end

    wire [PIXEL_COUNT_WIDTH-1:0] launch_pixels_wide =
        (video_pixels_remaining > MAX_PIXELS_PER_PACKET) ?
        MAX_PIXELS_PER_PACKET : video_pixels_remaining;
    wire [9:0] launch_pixels = launch_pixels_wide[9:0];
    wire [15:0] launch_video_payload_length =
        16'd12 + {5'd0, launch_pixels, 1'b0};
    wire video_packet_ready = video_frame_active &&
        (fifo_rd_count >= ready_launch_pixels) && phy_init_tx_sync2 && phy_link_tx_sync2;
    wire video_byte_request = payload_req && packet_is_video &&
        pixel_payload_active;
    wire fifo_pixel_pop = video_byte_request && pixel_byte_phase;
    assign fifo_rd_en = discard_until_sof || fifo_pixel_pop;

    // 前 16 字节为固定头；后 16 字节由诊断快照移位输出。
    reg [7:0] status_payload_data;
    reg [7:0] status_payload_next;
    reg [4:0] status_index_next;
    always @* begin
        if (status_index_next[4]) status_payload_next = status_diagnostics_shift[127:120];
        else case (status_index_next[3:0])
            4'd0:  status_payload_next = 8'h41; // A
            4'd1:  status_payload_next = 8'h43; // C
            4'd2:  status_payload_next = 8'h47; // G
            4'd3:  status_payload_next = 8'h36; // 6
            4'd4:  status_payload_next = 8'h01; // protocol version
            4'd5:  status_payload_next = packet_rx_diagnostics_raw ? 8'h02 : 8'h01;
            4'd6:  status_payload_next = packet_count[15:8];
            4'd7:  status_payload_next = packet_count[7:0];
            4'd8:  status_payload_next = 8'h55; // UDP-TEST
            4'd9:  status_payload_next = 8'h44;
            4'd10: status_payload_next = 8'h50;
            4'd11: status_payload_next = 8'h2D;
            4'd12: status_payload_next = 8'h54;
            4'd13: status_payload_next = 8'h45;
            4'd14: status_payload_next = 8'h53;
            4'd15: status_payload_next = 8'h54;
            default: status_payload_next = 8'h00;
        endcase
    end
    always @(posedge clk125m or posedge tx_reset_p)
        if(tx_reset_p)begin status_payload_data<=8'h41;status_index_next<=1;end
        else if(!tx_busy || packet_is_video || packet_is_reply || packet_is_telemetry) begin
            status_payload_data<=8'h41;status_index_next<=1;
        end else if(payload_req) begin
            status_payload_data<=status_payload_next;status_index_next<=status_index_next+1'b1;
        end

    // 发包启动时预装完整 RTP 头，逐字节移位避免热路径中的索引译码。
    reg [95:0] rtp_header_shift;
    wire [7:0] rtp_header_data = rtp_header_shift[95:88];

    wire tx_launch_ready=!tx_busy && ifg_ready && phy_init_tx_sync2 && phy_link_tx_sync2;
    // ACK待启动或发送期间冻结移位寄存器；仅用已寄存的pending/kind避开
    // RX有效与状态仲裁组合链，并保持ready握手取得的完整回复字节。
    wire reply_packet_active=(tx_busy && packet_is_reply) ||
        (packet_launch_pending && packet_launch_kind==LAUNCH_REPLY);
    // 状态包发送期间冻结诊断快照；其余时钟持续采样，避免RX相位/回复优先级
    // 组合链扇出到128位快照寄存器的CE。快照在状态包首字节前已稳定。
    wire status_packet_active=tx_busy && !packet_is_video && !packet_is_reply && !packet_is_telemetry;
    // 遥测发布保持到 start，注册可选状态不会漏包，并减少仲裁使能的级联。
    reg telemetry_offer_ready;
    always @(posedge clk125m or posedge tx_reset_p)
        if(tx_reset_p)telemetry_offer_ready<=0;
        else telemetry_offer_ready<=TELEMETRY_ENABLE && telemetry_valid && telemetry_length_nonzero;
    assign reply_ready = tx_launch_ready && !packet_launch_pending;
    assign telemetry_read = TELEMETRY_ENABLE && packet_is_telemetry && payload_req;
    // 原始请求交给遥测模块本地发送状态门控，避免包类型状态跨模块进入计数使能。
    assign telemetry_read_request = TELEMETRY_ENABLE && payload_req_telemetry;
    assign payload_data = packet_is_telemetry ? telemetry_data :
        packet_is_reply ? packet_reply[255:248] : packet_is_video ?
        ((!pixel_payload_active) ? rtp_header_data :
         (fifo_rd_valid ? (pixel_byte_phase ? fifo_rd_data[7:0] : fifo_rd_data[15:8]) : 8'h00)) :
        status_payload_data;

    always @(posedge clk125m or posedge tx_reset_p) begin
        if (tx_reset_p) begin
            phy_init_tx_sync1 <= 1'b0;
            phy_init_tx_sync2 <= 1'b0;
            phy_link_tx_sync1 <= 1'b0;
            phy_link_tx_sync2 <= 1'b0;
            fifo_overflow_tx_sync1 <= 1'b0;
            fifo_overflow_tx_sync2 <= 1'b0;
            status_interval_count <= 32'd0;status_carry16<=0;
            status_due <= 1'b0;
            tx_en_pulse <= 1'b0;
            tx_busy <= 1'b0;
            packet_is_video <= 1'b0;
            packet_is_reply <= 1'b0;
            packet_is_telemetry <= 1'b0;
            telemetry_yield_video <= 1'b0;
            packet_launch_pending<=1'b0;
            packet_launch_kind<=LAUNCH_REPLY;
            telemetry_start <= 1'b0;
            rtp_header_shift <= 96'd0;
            status_diagnostics_shift <= 128'd0;
            packet_rx_diagnostics_raw <= 1'b0;
            packet_payload_length <= 16'd0;
            payload_index <= 11'd0;
            pixel_payload_active<=0;packet_starts_frame<=0;packet_last_pixel_index<=0;ready_last_pixel_index<=0;ready_payload_length<=0;ready_last_fragment<=0;
            video_frame_active <= 1'b0;
            video_packet_ready_q <= 1'b0;
            ready_launch_pixels <= MAX_PACKET_PIXELS;
            video_pixels_remaining <= FRAME_PIXELS;
            packet_pixels <= 10'd0;
            packet_is_last_video_fragment <= 1'b0;
            packet_pixels_consumed <= 10'd0;
            pixel_byte_phase <= 1'b0;
            rtp_sequence <= 16'd0;
            rtp_timestamp <= 32'd0;
            ifg_count <= 4'd0;
            ifg_ready <= 1'b1;
            packet_count <= 32'd0;
            video_packet_count <= 32'd0;
            packet_count_gray <= 32'd0;
            video_packet_count_gray <= 32'd0;
            video_source_underflow <= 1'b0;
            video_frame_marker_error <= 1'b0;
        end else begin
            if (!status_packet_active) begin
                // 预装为线上的大端字节顺序，发送时复用快照寄存器移位。
                status_diagnostics_shift <= RX_DIAGNOSTICS_ENABLE ? {
                    rx_diagnostics_i[8+:8], rx_diagnostics_i[0+:8],
                    rx_diagnostics_i[24+:8], rx_diagnostics_i[16+:8],
                    rx_diagnostics_i[40+:8], rx_diagnostics_i[32+:8],
                    rx_diagnostics_i[56+:8], rx_diagnostics_i[48+:8],
                    rx_diagnostics_i[72+:8], rx_diagnostics_i[64+:8],
                    rx_diagnostics_i[88+:8], rx_diagnostics_i[80+:8],
                    rx_diagnostics_i[104+:8], rx_diagnostics_i[96+:8],
                    rx_diagnostics_i[120+:8], rx_diagnostics_i[112+:8]} : 128'd0;
                packet_rx_diagnostics_raw <= rx_diagnostics_raw_i===1'b1;
            end else if (payload_req && status_index_next[4])
                status_diagnostics_shift <= {status_diagnostics_shift[119:0],8'd0};
            phy_init_tx_sync1 <= phy_init_done;
            phy_init_tx_sync2 <= phy_init_tx_sync1;
            phy_link_tx_sync1 <= phy_link_raw;
            phy_link_tx_sync2 <= phy_link_tx_sync1;
            fifo_overflow_tx_sync1 <= fifo_overflow_cam || timestamp_overflow;
            fifo_overflow_tx_sync2 <= fifo_overflow_tx_sync1;
            // 诊断计数的 Gray 码延迟一拍，消除递增到 Gray 编码的串接。
            packet_count_gray<=packet_count^(packet_count>>1);
            video_packet_count_gray<=video_packet_count^(video_packet_count>>1);
            video_packet_ready_q <= video_packet_ready;
            // A packet finishes before the next one can start: the Ethernet
            // inter-frame gap leaves time for this precomputed threshold.
            ready_launch_pixels <= launch_pixels;ready_last_pixel_index<=launch_pixels-1'b1;
            ready_payload_length<=16'd12+{5'd0,ready_launch_pixels,1'b0};
            ready_last_fragment<=video_pixels_remaining<=MAX_PIXELS_PER_PACKET;
            tx_en_pulse <= 1'b0;
            telemetry_start <= 1'b0;

            if (!phy_init_tx_sync2 || !phy_link_tx_sync2) begin
                status_interval_count <= 32'd0;status_carry16<=0;
                status_due <= 1'b0;
            end else if (status_interval_count == STATUS_INTERVAL_CYCLES - 1) begin
                status_interval_count <= 32'd0;status_carry16<=0;
                status_due <= 1'b1;
            end else begin
                status_interval_count <= status_count_next;
                status_carry16 <= ((status_interval_count & 32'hffff)==32'hfffe);
            end

            if (discard_until_sof && fifo_rd_valid) begin
                // If capture started mid-frame, discard until the next tagged
                // first pixel instead of manufacturing a false RTP frame.
            end
            if (begin_video_frame) begin
                video_frame_active <= 1'b1;
                video_pixels_remaining <= FRAME_PIXELS;
                if(USE_EXTERNAL_TIMESTAMP)rtp_timestamp<=fifo_timestamp;
            end

            if (payload_req) begin
                payload_index <= payload_index + 1'b1;
                if(payload_index==RTP_HEADER_LENGTH-1'b1)pixel_payload_active<=1;
                if (packet_is_video && pixel_payload_active)
                    pixel_byte_phase <= ~pixel_byte_phase;
                else if (packet_is_video)
                    rtp_header_shift <= {rtp_header_shift[87:0],8'd0};
            end

            if (fifo_pixel_pop) begin
                if (!fifo_rd_valid) begin
                    video_source_underflow <= 1'b1;
                end else begin
                    if (fifo_rd_data[16] !=
                        (packet_starts_frame &&
                         (packet_pixels_consumed == 0)))
                        video_frame_marker_error <= 1'b1;
                    if (fifo_rd_data[17] !=
                        (packet_is_last_video_fragment &&
                         (packet_pixels_consumed == packet_last_pixel_index)))
                        video_frame_marker_error <= 1'b1;
                    packet_pixels_consumed <= packet_pixels_consumed + 1'b1;
                end
            end

            if (tx_done) begin
                tx_busy <= 1'b0;
                packet_count <= packet_count_next;

                ifg_count <= ETHERNET_IFG_BYTES;
                ifg_ready <= 1'b0;
                if (packet_is_video) begin
                    video_packet_count <= video_count_next;

                    rtp_sequence <= rtp_sequence + 1'b1;
                    if (packet_is_last_video_fragment) begin
                        video_frame_active <= 1'b0;
                        video_pixels_remaining <= FRAME_PIXELS;
                        if(!USE_EXTERNAL_TIMESTAMP)rtp_timestamp <= rtp_timestamp + RTP_TIMESTAMP_STEP;
                    end else begin
                        video_pixels_remaining <= video_pixels_remaining - packet_pixels;
                    end
                end
            end else if (ifg_count != 0) begin
                ifg_count <= ifg_count - 1'b1;
                ifg_ready <= (ifg_count == 4'd1);
            end else begin
                ifg_ready <= 1'b1;
            end

            if (packet_launch_pending) begin
                // 仲裁结果已寄存；第二拍只启动已选中的包并装载其已锁存参数。
                if(tx_launch_ready) begin
                    packet_launch_pending<=1'b0;
                    tx_en_pulse<=1'b1;
                    tx_busy<=1'b1;
                    case(packet_launch_kind)
                        LAUNCH_REPLY: begin
                            packet_is_video<=1'b0;
                            packet_is_reply<=1'b1;
                            packet_is_telemetry<=1'b0;
                            packet_payload_length<=16'd32;
                            payload_index<=11'd0;pixel_payload_active<=0;
                        end
                        LAUNCH_STATUS: begin
                            packet_is_video<=1'b0;
                            packet_is_reply<=1'b0;
                            packet_is_telemetry<=1'b0;
                            packet_payload_length<=STATUS_PAYLOAD_LENGTH;
                            payload_index<=11'd0;pixel_payload_active<=0;
                            status_due<=1'b0;
                        end
                        LAUNCH_TELEMETRY: begin
                            packet_is_video<=1'b0;
                            packet_is_reply<=1'b0;
                            packet_is_telemetry<=1'b1;
                            telemetry_start<=1'b1;
                            telemetry_yield_video<=1'b1;
                            packet_payload_length<=launch_telemetry_length;
                            payload_index<=11'd0;pixel_payload_active<=0;
                        end
                        LAUNCH_VIDEO: begin
                            packet_is_video<=1'b1;
                            packet_is_reply<=1'b0;
                            packet_is_telemetry<=1'b0;
                            telemetry_yield_video<=1'b0;
                            rtp_header_shift<={8'h80,launch_last_fragment,VIDEO_RTP_PAYLOAD_TYPE,
                                launch_rtp_sequence,launch_rtp_timestamp,32'h41_43_47_36};
                            packet_payload_length<=launch_payload_length;
                            packet_pixels<=pending_launch_pixels;
                            packet_last_pixel_index<=pending_launch_last_pixel_index;
                            packet_starts_frame<=launch_starts_frame;
                            packet_is_last_video_fragment<=launch_last_fragment;
                            packet_pixels_consumed<=10'd0;
                            pixel_byte_phase<=1'b0;
                            payload_index<=11'd0;pixel_payload_active<=0;
                        end
                        default: begin
                            packet_is_video<=1'b0;
                            packet_is_reply<=1'b0;
                            packet_is_telemetry<=1'b0;
                            tx_busy<=1'b0;
                            tx_en_pulse<=1'b0;
                        end
                    endcase
                end
            end else if (tx_launch_ready) begin
                // 第一拍只仲裁并锁存包参数；held_valid 不再穿过包选择到 RTP 头寄存器。
                if (reply_valid) begin
                    packet_launch_pending<=1'b1;
                    packet_launch_kind<=LAUNCH_REPLY;
                end else if (status_due) begin
                    packet_launch_pending<=1'b1;
                    packet_launch_kind<=LAUNCH_STATUS;
                end else if (TELEMETRY_ENABLE && telemetry_offer_ready &&
                             !(telemetry_yield_video && video_packet_ready_q && video_frame_active)) begin
                    packet_launch_pending<=1'b1;
                    packet_launch_kind<=LAUNCH_TELEMETRY;
                end else if (video_packet_ready_q && video_frame_active) begin
                    packet_launch_pending<=1'b1;
                    packet_launch_kind<=LAUNCH_VIDEO;
                end
            end
        end
    end

    // 仲裁前准备视频和遥测参数；选中后保持，避免仲裁优先级穿过参数寄存器使能。
    always @(posedge clk125m) begin
        if (!packet_launch_pending) begin
            launch_telemetry_length<=telemetry_length;
            launch_rtp_sequence<=rtp_sequence;
            launch_rtp_timestamp<=rtp_timestamp;
            launch_payload_length<=ready_payload_length;
            pending_launch_pixels<=ready_launch_pixels;
            pending_launch_last_pixel_index<=ready_last_pixel_index;
            launch_starts_frame<=(video_pixels_remaining==FRAME_PIXELS);
            launch_last_fragment<=ready_last_fragment;
        end
    end

    // 回复数据源在 ready 握手前保持稳定；空闲或 IFG 期间提前装入发送移位寄存器。
    // 发送期间只移位当前包，pending 事务期间不覆盖已锁存的回复。
    always @(posedge clk125m or posedge tx_reset_p) begin
        if(tx_reset_p)packet_reply<=256'd0;
        else if(payload_req && packet_is_reply)
            packet_reply<={packet_reply[247:0],8'd0};
        else if(!reply_packet_active)
            packet_reply<=reply_data;
    end

    eth_udp_tx_gmii udp_tx (
        .clk125m(clk125m),
        .reset_p(tx_reset_p),
        .tx_en_pulse(tx_en_pulse),
        .tx_done(tx_done),
        .dst_mac(DST_MAC),
        .src_mac(SRC_MAC),
        .dst_ip(DST_IP),
        .src_ip(SRC_IP),
        .dst_port(packet_is_reply ? REPLY_PORT : packet_is_telemetry ? TELEMETRY_PORT : DST_PORT),
        .src_port(SRC_PORT),
        .data_length(packet_payload_length),
        .payload_req_o(payload_req),
        .payload_req_telemetry_o(payload_req_telemetry),
        .payload_dat_i(payload_data),
        .gmii_tx_clk(gmii_tx_clk),
        .gmii_txen(gmii_txen),
        .gmii_txd(gmii_txd)
    );

    gmii_to_rgmii rgmii_tx (
        // PHY 初始化完成已同步到发送域，发包也以此为前提；避免把 50 MHz
        // 复位计数比较直接送到 125 MHz 输出寄存器的异步复位。
        .reset_n(phy_init_tx_sync2),
        .gmii_tx_clk(gmii_tx_clk),
        .gmii_txd(gmii_txd),
        .gmii_txen(gmii_txen),
        .gmii_txer(1'b0),
        .rgmii_tx_clk(rgmii_gtxc),
        .rgmii_txd(rgmii_txd),
        .rgmii_txen(rgmii_txen)
    );
endmodule
