// ==================== bx72_timebase.sv ====================
`timescale 1ns/1ps
// 与本板PLL一致的微秒相位合同：1000MHz/30为精确33⅓MHz，不能长期按33333333Hz累计。
package bx72_clock_contract;
    function automatic integer phase_modulus(input integer hz);
        if(hz==33333333)phase_modulus=100;
        else if(hz%1000000==0)phase_modulus=hz/1000000;
        else phase_modulus=hz;
    endfunction
    function automatic integer phase_increment(input integer hz);
        if(hz==33333333)phase_increment=3;
        else if(hz%1000000==0)phase_increment=1;
        else phase_increment=1000000;
    endfunction
    function automatic integer phase_width(input integer hz);
        phase_width=$clog2(phase_modulus(hz)+phase_increment(hz));
        if(phase_width<1)phase_width=1;
    endfunction
endpackage

// 两路UART共用接收状态机；输入同步由调用端完成。
// 参数保留电机和雷达各自的分频、复位及数据提交时刻。
module bx72_uart_rx #(
    parameter integer BIT_CYCLES=434,
    parameter integer HALF_CYCLES=BIT_CYCLES/2,
    parameter integer ASYNC_RESET=1,
    parameter integer DATA_ON_SAMPLE=0
)(
    input wire clk,reset_n,rx_i,
    output reg [7:0] data_o,
    output reg valid_o,framing_error_o
);
    localparam integer COUNT_W=(BIT_CYCLES<2)?1:$clog2(BIT_CYCLES+1);
    localparam [1:0] IDLE=0,START=1,DATA=2,STOP=3;

    reg [1:0] state;
    reg [COUNT_W-1:0] count;
    reg [2:0] bit_index;
    reg [7:0] shift;
    // 复位方式在展开期选择，使异步边沿与复位条件严格一致。
    task receive_step;
        begin

            valid_o<=0;framing_error_o<=0;
            case(state)
                IDLE: if(!rx_i) begin count<=HALF_CYCLES-1;state<=START;end
                START: begin
                    if(count!=0) count<=count-1'b1;
                    else if(!rx_i) begin bit_index<=0;count<=BIT_CYCLES-1;state<=DATA;end
                    else state<=IDLE;
                end
                DATA: begin
                    if(count!=0) count<=count-1'b1;
                    else begin
                        shift[bit_index]<=rx_i;
                        if(DATA_ON_SAMPLE) data_o[bit_index]<=rx_i;
                        count<=BIT_CYCLES-1;
                        if(bit_index==7) state<=STOP;
                        else bit_index<=bit_index+1'b1;
                    end
                end
                STOP: begin
                    if(count!=0) count<=count-1'b1;
                    else begin
                        if(rx_i) begin
                            if(!DATA_ON_SAMPLE) data_o<=shift;
                            valid_o<=1;
                        end else framing_error_o<=1;
                        state<=IDLE;
                    end
                end
                default: state<=IDLE;
            endcase
        end
    endtask
    generate if(ASYNC_RESET) begin : g_async_reset
        always @(posedge clk or negedge reset_n) begin
            if(!reset_n) begin
                state<=IDLE;count<=0;bit_index<=0;shift<=0;
                data_o<=0;valid_o<=0;framing_error_o<=0;
            end else receive_step();
        end
    end else begin : g_sync_reset
        always @(posedge clk) begin
            if(!reset_n) begin
                state<=IDLE;count<=0;bit_index<=0;shift<=0;
                data_o<=0;valid_o<=0;framing_error_o<=0;
            end else receive_step();
        end
    end endgenerate
endmodule

// 微秒时标先相减、再比较，避免32位借位链串入保护及大扇出控制。
// 提前1us判失效，覆盖两拍寄存延迟；valid撤销仍直接闭锁。
module bx72_deadline_age #(
    parameter integer TIMEOUT_US = 100000
)(
    input wire clk,rst,
    input wire [31:0] now_us_i,last_us_i,
    input wire valid_i,
    output reg [31:0] age_us_o,
    output wire fresh_o
);
    localparam [31:0] EARLY_LIMIT_US = TIMEOUT_US-1;
    reg fresh_q;
    always @(posedge clk) begin
        if(rst)begin age_us_o<=32'hffffffff;fresh_q<=0;end
        else begin
            age_us_o<=now_us_i-last_us_i;
            fresh_q<=valid_i&&(age_us_o<EARLY_LIMIT_US);
        end
    end
    assign fresh_o=valid_i&&fresh_q;
endmodule

// 共享下一相位与tick逻辑；状态仍由各时钟域持有，禁止直接跨域采样计数总线。
module bx72_us_phase #(
    parameter integer CLK_HZ=50000000,
    parameter integer WIDTH=bx72_clock_contract::phase_width(CLK_HZ)
)(
    input wire [WIDTH-1:0] phase_i,
    output wire [WIDTH-1:0] next_phase_o,
    output wire tick_o
);
    localparam integer MODULUS=bx72_clock_contract::phase_modulus(CLK_HZ);
    localparam integer INCREMENT=bx72_clock_contract::phase_increment(CLK_HZ);
    wire [WIDTH:0] next_sum={1'b0,phase_i}+INCREMENT;
    assign tick_o=next_sum>=MODULUS;
    wire [31:0] reduced_phase=tick_o ? next_sum-MODULUS : next_sum;
    // 正常相位小于MODULUS，WIDTH由MODULUS+INCREMENT确定，显式选取有效位。
    assign next_phase_o=reduced_phase[WIDTH-1:0];
endmodule

// ==================== bx72_nav_ram.sv ====================
`timescale 1ns/1ps
// 导航片上RAM：同一时钟的A读/写口与B只读口，无数据复位或隐式初始化。
// 地址/使能在上升沿采样，读值在该沿NBA更新，顺序消费者从下一沿消费。
// A写入时A读输出保持；禁用端口输出保持。控制器必须禁止A写与B同址读。
module bx72_nav_ram #(
    parameter integer DATA_W=1,
    parameter integer ADDR_W=14
)(
    input wire clk,
    input wire a_en,a_we,
    input wire [ADDR_W-1:0] a_addr,
    input wire [DATA_W-1:0] a_wdata,
    output reg [DATA_W-1:0] a_rdata,
    input wire b_en,
    input wire [ADDR_W-1:0] b_addr,
    output reg [DATA_W-1:0] b_rdata
);
    // SUG550-2.4第5.16节：block_ram指定BSRAM；真实映射仍由综合报告确认。
    reg [DATA_W-1:0] mem [0:(1<<ADDR_W)-1] /* synthesis syn_ramstyle = "block_ram" */;
    always @(posedge clk) begin
        if(a_en) begin
            if(a_we) mem[a_addr]<=a_wdata;
            else a_rdata<=mem[a_addr];
        end
    end
    always @(posedge clk) begin
        if(b_en) b_rdata<=mem[b_addr];
    end
    // synthesis translate_off
    always @(posedge clk) begin
        if(a_en && a_we && b_en && a_addr==b_addr)
            $fatal(1,"导航RAM控制器发生跨端口同址读写碰撞 addr=%0d",a_addr);
    end
    // synthesis translate_on
endmodule

// ==================== async_frame_fifo.v ====================
`timescale 1ns/1ps

// 小深度整帧 CDC FIFO。写侧必须一次提供完整记录；存储体先写完，
// Gray 指针再发布已提交项，因而读侧永远看不到半帧。
// rd_data_o 只在 rd_fire 时更新；读侧背压或等待 rd_valid_o 时数据保持稳定。
// 两侧复位必须同时异步置位，可在各自时钟域独立同步释放；单边独立复位
// 不能保证保留队列内容。ADDR_WIDTH 至少为 2，本项目固定使用深度 4。
module async_frame_fifo #(
    parameter integer DATA_WIDTH = 8,
    parameter integer ADDR_WIDTH = 2,
    parameter integer REGISTER_FULL = 0
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
    wire [ADDR_WIDTH:0] wr_bin_after_write = wr_bin_q + 1'b1;
    wire [ADDR_WIDTH:0] wr_gray_after_write =
        (wr_bin_after_write >> 1) ^ wr_bin_after_write;
    wire [ADDR_WIDTH:0] wr_full_target = {
        ~rd_gray_wr_sync2_q[ADDR_WIDTH:ADDR_WIDTH-1],
        rd_gray_wr_sync2_q[ADDR_WIDTH-2:0]
    };
    wire wr_full_without_write = wr_gray_q == wr_full_target;
    wire wr_full_after_write = wr_gray_after_write == wr_full_target;

    reg full_registered;
    assign wr_full_o = REGISTER_FULL ? full_registered : wr_full_without_write;
    assign rd_empty_o = rd_gray_q == wr_gray_rd_sync2_q;

    always @(posedge wr_clk_i or negedge wr_rst_n_i) begin
        if (!wr_rst_n_i) begin
            wr_bin_q <= {(ADDR_WIDTH+1){1'b0}};
            wr_gray_q <= {(ADDR_WIDTH+1){1'b0}};
            rd_gray_wr_sync1_q <= {(ADDR_WIDTH+1){1'b0}};
            rd_gray_wr_sync2_q <= {(ADDR_WIDTH+1){1'b0}};
            wr_overflow_o <= 1'b0;full_registered<=1'b0;
        end else begin
            rd_gray_wr_sync1_q <= rd_gray_q;
            rd_gray_wr_sync2_q <= rd_gray_wr_sync1_q;
            wr_overflow_o <= wr_valid_i && wr_full_o;
            // 诊断 RX 的宽记录 RAM 写使能使用寄存满标志，避免指针比较高扇出。
            // 先独立计算写/不写两种满状态，再用原 wr_fire 选择，
            // 避免提交校验信号串过指针加一与 Gray 比较的整条路径。
            full_registered <= wr_fire ? wr_full_after_write : wr_full_without_write;
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
