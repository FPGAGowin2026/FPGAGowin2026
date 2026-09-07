`timescale 1ns/1ps

// UDP 元数据 FIFO：16 项、112 位、标准读模式。
// 保持官方 ch58 顶层接口，但不依赖与顶层位宽不一致的加密 FIFO 生成件。
module eth_msg_buf (
    input  wire [111:0] Data,
    input  wire         Clk,
    input  wire         WrEn,
    input  wire         RdEn,
    input  wire         Reset,
    output reg  [111:0] Q,
    output wire [4:0]   Wnum,
    output wire         Empty,
    output wire         Full
);
    reg [111:0] mem [0:15];
    reg [3:0] wr_ptr_q;
    reg [3:0] rd_ptr_q;
    reg [4:0] count_q;

    wire write_fire = WrEn && !Full;
    wire read_fire = RdEn && !Empty;

    assign Wnum = count_q;
    assign Empty = count_q == 5'd0;
    assign Full = count_q == 5'd16;

    always @(posedge Clk or posedge Reset) begin
        if (Reset) begin
            wr_ptr_q <= 4'd0;
            rd_ptr_q <= 4'd0;
            count_q <= 5'd0;
            Q <= 112'd0;
        end else begin
            if (write_fire) begin
                mem[wr_ptr_q] <= Data;
                wr_ptr_q <= wr_ptr_q + 1'b1;
            end
            if (read_fire) begin
                Q <= mem[rd_ptr_q];
                rd_ptr_q <= rd_ptr_q + 1'b1;
            end

            case ({write_fire, read_fire})
                2'b10: count_q <= count_q + 1'b1;
                2'b01: count_q <= count_q - 1'b1;
                default: count_q <= count_q;
            endcase
        end
    end
endmodule
