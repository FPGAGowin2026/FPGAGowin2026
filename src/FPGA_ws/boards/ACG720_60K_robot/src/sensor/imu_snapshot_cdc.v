module imu_snapshot_cdc #(
    parameter integer WIDTH = 105
) (
    input wire clk, reset_n,
    input wire source_toggle,
    input wire [WIDTH-1:0] source_snapshot,
    output reg updated,
    output reg [WIDTH-1:0] snapshot
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] toggle_sync;
    reg seen;

    // 源端发布后将整份快照保持 20 ms，接收端在同步翻转后一次锁存。
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            toggle_sync <= 0;
            seen <= 0;
            updated <= 0;
            snapshot <= 0;
        end else begin
            toggle_sync <= {toggle_sync[0], source_toggle};
            updated <= toggle_sync[1] != seen;
            if (toggle_sync[1] != seen) begin
                seen <= toggle_sync[1];
                snapshot <= source_snapshot;
            end
        end
    end
endmodule
