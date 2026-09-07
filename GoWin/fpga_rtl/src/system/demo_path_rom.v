`timescale 1ns/1ps

// 可直接仿真的四段演示路径 ROM。真实路线只需替换本模块内容或接 BRAM，
// 不改变 path_segment_executor 握手：MOVE 1000、ROTATE 1571、MOVE 500、STOP。
module demo_path_rom (
    input  wire               segment_ready_i,
    input  wire [7:0]         segment_index_i,
    output reg                segment_valid_o,
    output reg [1:0]          segment_opcode_o,
    output reg [31:0]         segment_target_o,
    output reg signed [31:0]  segment_command_o,
    output reg                segment_last_o
);
    localparam [1:0] OP_MOVE = 2'd0;
    localparam [1:0] OP_ROTATE = 2'd1;
    localparam [1:0] OP_STOP = 2'd2;

    always @(*) begin
        segment_valid_o = segment_ready_i;
        segment_opcode_o = OP_STOP;
        segment_target_o = 32'd0;
        segment_command_o = 32'sd0;
        segment_last_o = 1'b1;
        case (segment_index_i)
            8'd0: begin
                segment_opcode_o = OP_MOVE;
                segment_target_o = 32'd1000;
                segment_command_o = 32'sd300;
                segment_last_o = 1'b0;
            end
            8'd1: begin
                segment_opcode_o = OP_ROTATE;
                segment_target_o = 32'd1571;
                segment_command_o = 32'sd400;
                segment_last_o = 1'b0;
            end
            8'd2: begin
                segment_opcode_o = OP_MOVE;
                segment_target_o = 32'd500;
                segment_command_o = 32'sd250;
                segment_last_o = 1'b0;
            end
            default: begin
                segment_opcode_o = OP_STOP;
                segment_target_o = 32'd0;
                segment_command_o = 32'sd0;
                segment_last_o = 1'b1;
            end
        endcase
    end
endmodule
