// 125 MHz 同频 RX PLL；移相须结合专用输入通路和 PHY 延迟实测。
// init_clk 为独立 50 MHz 时钟；锁定后才释放 RX 解析器与应答 FIFO。
module bx72_rx_pll #(
    parameter integer PHASE_TICKS = 30000000,
    parameter integer VERIFY_TICKS = 30000000,
    parameter integer SETTLE_TICKS = 1000
) (
    input wire clkin, init_clk, reset_p,
    input wire good_toggle_i,
    output wire clkout0, lock,
    output wire sampling_reset_o, ready_o,
    output wire [3:0] phase_index_o,
    output wire [15:0] good_mask_o
);
    wire pll_lock,pll_reset;
    wire pll_init_reset;
    wire rx_input_clk;
    wire pspulse;
    wire [5:0] icpsel;
    wire [2:0] lpfres;
    (* ASYNC_REG = "TRUE" *) reg pll_lock_reset_meta, pll_lock_reset_sync;
    (* ASYNC_REG = "TRUE" *) reg pll_lock_meta, pll_lock_sync;
    reg pll_lock_seen;
    reg [1:0] pll_retrain_count;
    wire pll_retrain_reset = (pll_retrain_count != 0);
    assign pll_init_reset = reset_p || pll_retrain_reset;

    // PLL_INIT/O_RST 只会在外部复位时重启；若运行中失锁，显式重新走完整初始化，
    // 使动态相位计数器回到静态 0 相位，避免扫描索引和 PLL 内部相位脱节。
    always @(posedge init_clk or posedge reset_p) begin
        if (reset_p) begin
            pll_lock_reset_meta <= 1'b1;
            pll_lock_reset_sync <= 1'b1;
        end else begin
            pll_lock_reset_meta <= 1'b0;
            pll_lock_reset_sync <= pll_lock_reset_meta;
        end
    end

    always @(posedge init_clk or posedge pll_lock_reset_sync) begin
        if (pll_lock_reset_sync) begin
            pll_lock_meta <= 1'b0;
            pll_lock_sync <= 1'b0;
            pll_lock_seen <= 1'b0;
            pll_retrain_count <= 2'd0;
        end else begin
            pll_lock_meta <= lock;
            pll_lock_sync <= pll_lock_meta;
            if (pll_retrain_count != 0) begin
                pll_retrain_count <= pll_retrain_count - 1'b1;
            end else if (pll_lock_seen && !pll_lock_sync) begin
                pll_retrain_count <= 2'd2;
                pll_lock_seen <= 1'b0;
            end else if (pll_lock_sync) begin
                pll_lock_seen <= 1'b1;
            end
        end
    end
    // V19 是 SGCLKC_4；先走 HCLK 专用通路再进入 PLL，避免输入段普通布线。
    // DIV_MODE=1 只选择时钟通路，不改变 PHY 的 125 MHz 频率。
    CLKDIV #(.DIV_MODE("1")) u_rx_hclk(.HCLKIN(clkin),.RESETN(!reset_p),
        .CALIB(1'b0),.CLKOUT(rx_input_clk));
    bx72_rx_pll_mod u_pll(.lock(pll_lock),.clkout0(clkout0),.clkin(rx_input_clk),
        .reset(pll_reset),.pspulse_i(pspulse),.icpsel(icpsel),.lpfres(lpfres),.lpfcap(2'b00));
    PLL_INIT #(.CLK_PERIOD(20),.MULTI_FAC(12)) u_pll_init(
        .CLKIN(init_clk),.I_RST(pll_init_reset),.O_RST(pll_reset),.PLLLOCK(pll_lock),
        .O_LOCK(lock),.ICPSEL(icpsel),.LPFRES(lpfres));
    rx_pll_phase_scan #(
        .PHASE_TICKS(PHASE_TICKS),
        .VERIFY_TICKS(VERIFY_TICKS),
        .SETTLE_TICKS(SETTLE_TICKS)
    ) u_phase_scan (
        .init_clk(init_clk),
        .resetlink_i(pll_init_reset),
        .pll_lock_i(lock),
        .good_toggle_i(good_toggle_i),
        .pspulse_o(pspulse),
        .sampling_reset_o(sampling_reset_o),
        .ready_o(ready_o),
        .phase_index_o(phase_index_o),
        .good_mask_o(good_mask_o)
    );
endmodule
