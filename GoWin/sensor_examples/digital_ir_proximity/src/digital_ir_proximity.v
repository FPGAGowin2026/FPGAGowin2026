`timescale 1ns/1ps

// 数字红外近接传感器输入整形。默认按 GP2Y0D810Z0F 类模块的低有效输出处理。
module digital_ir_proximity #(
    parameter integer ASSERT_SAMPLES = 4,
    parameter integer RELEASE_SAMPLES = 4
) (
    input  wire clk_i,
    input  wire rst_n_i,
    input  wire sensor_n_i,
    output reg  object_near_o,
    output reg  changed_o
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] sync_q;
    localparam integer ASSERT_COUNT = (ASSERT_SAMPLES < 1) ? 1 : ASSERT_SAMPLES;
    localparam integer RELEASE_COUNT = (RELEASE_SAMPLES < 1) ? 1 : RELEASE_SAMPLES;
    reg [31:0] count_q;
    wire detected = ~sync_q[1];

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            sync_q <= 2'b11;
            count_q <= 32'd0;
            object_near_o <= 1'b0;
            changed_o <= 1'b0;
        end else begin
            sync_q <= {sync_q[0], sensor_n_i};
            changed_o <= 1'b0;
            if (detected == object_near_o) begin
                count_q <= 32'd0;
            end else if ((!object_near_o && count_q >= ASSERT_COUNT-1) ||
                         ( object_near_o && count_q >= RELEASE_COUNT-1)) begin
                object_near_o <= detected;
                count_q <= 32'd0;
                changed_o <= 1'b1;
            end else begin
                count_q <= count_q + 1'b1;
            end
        end
    end
endmodule
