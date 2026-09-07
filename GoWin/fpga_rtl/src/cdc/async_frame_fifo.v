`timescale 1ns/1ps

// 小深度整帧 CDC FIFO。写侧必须一次提供完整记录；存储体先写完，
// Gray 指针再发布已提交项，因而读侧永远看不到半帧。
// rd_data_o 只在 rd_fire 时更新；读侧背压或等待 rd_valid_o 时数据保持稳定。
// 两侧复位必须同时异步置位，可在各自时钟域独立同步释放；单边独立复位
// 不能保证保留队列内容。ADDR_WIDTH 至少为 2，本项目固定使用深度 4。
module async_frame_fifo #(
    parameter integer DATA_WIDTH = 8,
    parameter integer ADDR_WIDTH = 2
) (
    input  wire                  wr_clk_i,
    input  wire                  wr_rst_n_i,
    input  wire                  wr_valid_i,
    input  wire [DATA_WIDTH-1:0] wr_data_i,
    output wire                  wr_full_o,
    output reg                   wr_overflow_o,

    input  wire                  rd_clk_i,
    input  wire                  rd_rst_n_i,
    input  wire                  rd_ready_i,
    output reg  [DATA_WIDTH-1:0] rd_data_o,
    output reg                   rd_valid_o,
    output wire                  rd_empty_o
);
    localparam integer DEPTH = (1 << ADDR_WIDTH);

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    reg [ADDR_WIDTH:0] wr_bin_q;
    reg [ADDR_WIDTH:0] wr_gray_q;
    // sync1 是唯一允许接收异步 Gray 指针的第一级；满/空判定只使用 sync2。
    reg [ADDR_WIDTH:0] rd_gray_wr_sync1_q;
    reg [ADDR_WIDTH:0] rd_gray_wr_sync2_q;

    reg [ADDR_WIDTH:0] rd_bin_q;
    reg [ADDR_WIDTH:0] rd_gray_q;
    reg [ADDR_WIDTH:0] wr_gray_rd_sync1_q;
    reg [ADDR_WIDTH:0] wr_gray_rd_sync2_q;

    wire wr_fire = wr_valid_i && !wr_full_o;
    wire rd_fire = rd_ready_i && !rd_empty_o;
    wire [ADDR_WIDTH:0] wr_bin_next = wr_bin_q + wr_fire;
    wire [ADDR_WIDTH:0] rd_bin_next = rd_bin_q + rd_fire;
    wire [ADDR_WIDTH:0] wr_gray_next = (wr_bin_next >> 1) ^ wr_bin_next;
    wire [ADDR_WIDTH:0] rd_gray_next = (rd_bin_next >> 1) ^ rd_bin_next;

    assign wr_full_o = wr_gray_q == {
        ~rd_gray_wr_sync2_q[ADDR_WIDTH:ADDR_WIDTH-1],
        rd_gray_wr_sync2_q[ADDR_WIDTH-2:0]
    };
    assign rd_empty_o = rd_gray_q == wr_gray_rd_sync2_q;

    always @(posedge wr_clk_i or negedge wr_rst_n_i) begin
        if (!wr_rst_n_i) begin
            wr_bin_q <= {(ADDR_WIDTH+1){1'b0}};
            wr_gray_q <= {(ADDR_WIDTH+1){1'b0}};
            rd_gray_wr_sync1_q <= {(ADDR_WIDTH+1){1'b0}};
            rd_gray_wr_sync2_q <= {(ADDR_WIDTH+1){1'b0}};
            wr_overflow_o <= 1'b0;
        end else begin
            rd_gray_wr_sync1_q <= rd_gray_q;
            rd_gray_wr_sync2_q <= rd_gray_wr_sync1_q;
            wr_overflow_o <= wr_valid_i && wr_full_o;
            if (wr_fire) begin
                mem[wr_bin_q[ADDR_WIDTH-1:0]] <= wr_data_i;
                wr_bin_q <= wr_bin_next;
                wr_gray_q <= wr_gray_next;
            end
        end
    end

    always @(posedge rd_clk_i or negedge rd_rst_n_i) begin
        if (!rd_rst_n_i) begin
            rd_bin_q <= {(ADDR_WIDTH+1){1'b0}};
            rd_gray_q <= {(ADDR_WIDTH+1){1'b0}};
            wr_gray_rd_sync1_q <= {(ADDR_WIDTH+1){1'b0}};
            wr_gray_rd_sync2_q <= {(ADDR_WIDTH+1){1'b0}};
            rd_data_o <= {DATA_WIDTH{1'b0}};
            rd_valid_o <= 1'b0;
        end else begin
            wr_gray_rd_sync1_q <= wr_gray_q;
            wr_gray_rd_sync2_q <= wr_gray_rd_sync1_q;
            rd_valid_o <= rd_fire;
            if (rd_fire) begin
                rd_data_o <= mem[rd_bin_q[ADDR_WIDTH-1:0]];
                rd_bin_q <= rd_bin_next;
                rd_gray_q <= rd_gray_next;
            end
        end
    end

endmodule
