`timescale 1ns/1ps

// 软核服务层独立看门狗。
//
// 上电后服务动作保持禁止，只有收到首次 heartbeat_write_i 脉冲才进入 alive。
// alive 后若连续 TIMEOUT_CYCLES 个时钟没有心跳，则锁存 fault 并产生一次
// path_abort_o。故障锁存后，后续 CPU 心跳不能自行恢复；只有外部安全复位
// safety_reset_n_i 能清除故障，确保固件重启不会自动重新授权运动服务动作。
module softcore_service_watchdog #(
    parameter integer TIMEOUT_CYCLES = 2500000
) (
    input  wire clk_i,
    input  wire safety_reset_n_i,
    input  wire heartbeat_write_i,

    output reg  alive_o,
    output reg  fault_o,
    output wire allow_commit_o,
    output wire allow_start_o,
    output reg  path_abort_o
);

    // TIMEOUT_CYCLES 小于 1 时按 1 处理，避免无效的负比较边界。
    localparam integer EFFECTIVE_TIMEOUT =
        (TIMEOUT_CYCLES < 1) ? 1 : TIMEOUT_CYCLES;
    localparam integer TIMEOUT_LIMIT = EFFECTIVE_TIMEOUT - 1;

    reg [31:0] heartbeat_age_q;

    assign allow_commit_o = alive_o && !fault_o;
    assign allow_start_o  = alive_o && !fault_o;

    always @(posedge clk_i or negedge safety_reset_n_i) begin
        if (!safety_reset_n_i) begin
            heartbeat_age_q <= 32'd0;
            alive_o         <= 1'b0;
            fault_o         <= 1'b0;
            path_abort_o    <= 1'b0;
        end else begin
            // 仅在首次进入 fault 的周期置位，后续自动清零。
            path_abort_o <= 1'b0;

            if (fault_o) begin
                // 故障只能由外部安全复位清除，CPU 心跳在此状态无效。
                heartbeat_age_q <= 32'd0;
                alive_o         <= 1'b0;
            end else if (heartbeat_write_i) begin
                heartbeat_age_q <= 32'd0;
                alive_o         <= 1'b1;
            end else if (alive_o) begin
                if (heartbeat_age_q >= TIMEOUT_LIMIT) begin
                    heartbeat_age_q <= 32'd0;
                    alive_o         <= 1'b0;
                    fault_o         <= 1'b1;
                    path_abort_o    <= 1'b1;
                end else begin
                    heartbeat_age_q <= heartbeat_age_q + 1'b1;
                end
            end
        end
    end

endmodule
