`timescale 1ns/1ps

// 保存最近一次通过完整协议校验的 Jetson 命令，并在心跳超时后失效。
// 急停报文立即清零；解除急停必须由后续新的非急停报文完成。
module control_command_watchdog #(
    parameter integer TIMEOUT_CYCLES = 25000000
) (
    input  wire               clk_i,
    input  wire               rst_n_i,
    input  wire               command_update_i,
    input  wire               command_invalidate_i,
    input  wire signed [31:0] linear_mm_s_i,
    input  wire signed [31:0] angular_mrad_s_i,
    input  wire               estop_i,
    output reg                command_valid_o,
    output reg signed [31:0]  linear_mm_s_o,
    output reg signed [31:0]  angular_mrad_s_o,
    output reg                software_estop_o,
    output reg                timeout_pulse_o
);
    localparam integer TIMEOUT_LIMIT = (TIMEOUT_CYCLES > 0) ? TIMEOUT_CYCLES - 1 : 0;
    reg [31:0] age_q;

    always @(posedge clk_i or negedge rst_n_i) begin
        if (!rst_n_i) begin
            age_q <= 32'd0;
            command_valid_o <= 1'b0;
            linear_mm_s_o <= 32'sd0;
            angular_mrad_s_o <= 32'sd0;
            software_estop_o <= 1'b0;
            timeout_pulse_o <= 1'b0;
        end else begin
            timeout_pulse_o <= 1'b0;
            // 物理急停、严重故障或离开 Jetson 模式时丢弃缓存速度。
            // 软件急停状态仍只由完整的新控制报文设置或解除。
            if (command_update_i)
                software_estop_o <= estop_i;

            if (command_invalidate_i || (command_update_i && estop_i)) begin
                age_q <= 32'd0;
                command_valid_o <= 1'b0;
                linear_mm_s_o <= 32'sd0;
                angular_mrad_s_o <= 32'sd0;
            end else if (command_update_i) begin
                age_q <= 32'd0;
                command_valid_o <= 1'b1;
                linear_mm_s_o <= linear_mm_s_i;
                angular_mrad_s_o <= angular_mrad_s_i;
            end else if (command_valid_o) begin
                if (age_q >= TIMEOUT_LIMIT) begin
                    age_q <= 32'd0;
                    command_valid_o <= 1'b0;
                    linear_mm_s_o <= 32'sd0;
                    angular_mrad_s_o <= 32'sd0;
                    timeout_pulse_o <= 1'b1;
                end else begin
                    age_q <= age_q + 1'b1;
                end
            end
        end
    end
endmodule
