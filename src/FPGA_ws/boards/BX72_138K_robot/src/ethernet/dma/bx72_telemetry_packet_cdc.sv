`timescale 1ns/1ps
`default_nettype none
// 50 MHz 公共包总线到 125 MHz 发送域的窄包 CDC。
// 源端先完整写入字节 FIFO，最后一个字节与长度 descriptor 同拍提交。
module bx72_telemetry_packet_cdc (
    input wire src_clk,
    input wire src_reset_i,
    input wire tx_clk,
    input wire reset_n,

    input wire packet_valid_i,
    input wire packet_length_nonzero_i,
    input wire [7:0] packet_length_i,
    input wire [7:0] packet_data_i,
    output wire packet_start_o,
    output wire packet_read_o,

    output wire tx_valid_o,
    output wire tx_length_nonzero_o,
    output wire [7:0] tx_length_o,
    output wire [7:0] tx_data_o,
    input wire tx_start_i,
    input wire tx_read_i
);
    localparam [0:0] SRC_IDLE=1'b0, SRC_COPY=1'b1;

    // 125 MHz 域唯一的复位同步链，供 SDC 精确约束；异步断言、同步释放。
    (* ASYNC_REG="TRUE" *) reg [1:0] tx_reset_sync;
    always @(posedge tx_clk or negedge reset_n) begin
        if(!reset_n)tx_reset_sync<=2'b11;
        else tx_reset_sync<={tx_reset_sync[0],1'b0};
    end
    wire tx_reset=tx_reset_sync[1];
    wire src_reset=src_reset_i;

    reg src_state;
    reg [7:0] copy_length;
    reg [7:0] copy_bytes_left;
    wire descriptor_full,descriptor_empty,descriptor_valid;
    wire [7:0] descriptor_data;
    wire byte_full,fifo_byte_valid;
    wire [7:0] fifo_byte_data;
    wire [11:0] byte_count;
    // 必须先于实例端口声明；Gowin 对后置声明可能保留隐式未驱动网线。
    wire descriptor_read;
    wire byte_prefetch;

    // 非零标志须与长度一致；只接收1..255字节的完整包。
    wire source_accept=(src_state==SRC_IDLE) && packet_valid_i &&
        packet_length_nonzero_i &&
        !descriptor_full && !src_reset;
    assign packet_start_o=source_accept;
    wire byte_write=(src_state==SRC_COPY) && !byte_full && !src_reset;
    assign packet_read_o=byte_write;
    wire descriptor_write=byte_write && (copy_bytes_left==1);

    always @(posedge src_clk or posedge src_reset) begin
        if(src_reset) begin
            src_state<=SRC_IDLE;
            copy_length<=0;
            copy_bytes_left<=0;
        end else begin
            if(source_accept) begin
                src_state<=SRC_COPY;
                copy_length<=packet_length_i;
                copy_bytes_left<=packet_length_i;
            end else if(byte_write) begin
                if(copy_bytes_left==1) begin
                    src_state<=SRC_IDLE;
                    copy_bytes_left<=0;
                end else begin
                    copy_bytes_left<=copy_bytes_left-1'b1;
                end
            end
        end
    end

    // 2048 个字节槽可容纳突发遥测；FIFO 满时 read 与 write 同时暂停。
    async_pixel_fifo #(.ADDR_WIDTH(11),.DATA_WIDTH(8)) byte_fifo (
        .arst(!reset_n),.wr_clk(src_clk),.wr_en(byte_write),.wr_data(packet_data_i),
        .full(byte_full),.overflow_sticky(),.rd_clk(tx_clk),.rd_en(byte_prefetch),
        .rd_data(fifo_byte_data),.rd_valid(fifo_byte_valid),.rd_count(byte_count)
    );
    // 单笔 COPY 保留一个 descriptor 槽；descriptor 只在整包写入后可见。
    async_frame_fifo #(.ADDR_WIDTH(4),.DATA_WIDTH(8),.REGISTER_FULL(1)) length_fifo (
        .wr_clk_i(src_clk),.wr_rst_n_i(!src_reset),.wr_valid_i(descriptor_write),
        .wr_data_i(copy_length),.wr_full_o(descriptor_full),.wr_overflow_o(),
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
`default_nettype wire
