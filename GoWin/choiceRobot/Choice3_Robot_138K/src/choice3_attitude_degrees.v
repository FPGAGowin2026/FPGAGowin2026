`timescale 1ns/1ps

// JY61P 角度原始值换算为整数度：raw * 180 / 32768。
// 整机 OSD 使用整数度；原始 16 位值仍可在软核寄存器中扩展为更高精度。
module choice3_attitude_degrees (
    input  wire               clk_i,
    input  wire               rst_n_i,
    input  wire               sample_valid_i,
    input  wire signed [15:0] roll_raw_i,
    input  wire signed [15:0] pitch_raw_i,
    input  wire signed [15:0] yaw_raw_i,
    output reg  signed [15:0] roll_deg_o,
    output reg  signed [15:0] pitch_deg_o,
    output reg  signed [15:0] yaw_deg_o,
    output reg                valid_o
);
    wire signed [31:0] roll_scaled_w  = $signed(roll_raw_i)  * 32'sd180;
    wire signed [31:0] pitch_scaled_w = $signed(pitch_raw_i) * 32'sd180;
    wire signed [31:0] yaw_scaled_w   = $signed(yaw_raw_i)   * 32'sd180;
    wire signed [31:0] roll_degrees_w  = roll_scaled_w >>> 15;
    wire signed [31:0] pitch_degrees_w = pitch_scaled_w >>> 15;
    wire signed [31:0] yaw_degrees_w   = yaw_scaled_w >>> 15;

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            roll_deg_o <= 0;
            pitch_deg_o <= 0;
            yaw_deg_o <= 0;
            valid_o <= 0;
        end else begin
            valid_o <= 0;
            if (sample_valid_i) begin
                roll_deg_o <= roll_degrees_w[15:0];
                pitch_deg_o <= pitch_degrees_w[15:0];
                yaw_deg_o <= yaw_degrees_w[15:0];
                valid_o <= 1;
            end
        end
    end
endmodule
