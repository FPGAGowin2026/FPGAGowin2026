`timescale 1ns/1ps

// Periodic HC-SR04 trigger generator and echo pulse-width measurement.
// The distance conversion uses the common room-temperature relationship:
// distance_mm = echo_time_us * 10 / 58.
module hc_sr04_controller #(
    parameter integer CLK_FREQ_HZ      = 27000000,
    parameter integer TRIGGER_US       = 10,
    parameter integer TRIGGER_PERIOD_US = 60000,
    parameter integer ECHO_TIMEOUT_US  = 30000
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        echo_in,
    output reg         trigger_out,
    output reg  [15:0] distance_mm,
    output reg  [15:0] pulse_width_us,
    output reg         sample_valid,
    output reg         timeout_error,
    output reg         busy
);

    // 向上取整可保证非整数 MHz 时触发高电平不会短于规格下限。
    localparam integer RAW_CYCLES_PER_US = (CLK_FREQ_HZ + 999999) / 1000000;
    localparam integer CYCLES_PER_US =
        (RAW_CYCLES_PER_US < 1) ? 1 : RAW_CYCLES_PER_US;
    localparam integer RAW_TRIGGER_CYCLES = TRIGGER_US * CYCLES_PER_US;
    localparam integer RAW_PERIOD_CYCLES = TRIGGER_PERIOD_US * CYCLES_PER_US;
    localparam integer RAW_TIMEOUT_CYCLES = ECHO_TIMEOUT_US * CYCLES_PER_US;
    localparam integer TRIGGER_CYCLES = (RAW_TRIGGER_CYCLES < 1) ? 1 : RAW_TRIGGER_CYCLES;
    localparam integer PERIOD_CYCLES = (RAW_PERIOD_CYCLES < 1) ? 1 : RAW_PERIOD_CYCLES;
    localparam integer TIMEOUT_CYCLES = (RAW_TIMEOUT_CYCLES < 1) ? 1 : RAW_TIMEOUT_CYCLES;
    localparam [15:0] DISTANCE_SCALE_Q16 = 16'd11299;

    localparam [2:0] ST_PERIOD    = 3'd0;
    localparam [2:0] ST_TRIGGER   = 3'd1;
    localparam [2:0] ST_WAIT_ECHO = 3'd2;
    localparam [2:0] ST_MEASURE   = 3'd3;
    localparam [2:0] ST_CALCULATE = 3'd4;

    (* ASYNC_REG = "TRUE" *) reg echo_meta;
    (* ASYNC_REG = "TRUE" *) reg echo_sync;
    reg echo_previous;
    wire echo_rise;
    wire echo_fall;

    reg [2:0] state;
    reg [31:0] timer_count;
    reg [31:0] echo_cycle_count;
    reg [31:0] echo_us_phase;
    reg [15:0] echo_us_count;
    reg [31:0] distance_product;

    assign echo_rise = echo_sync & ~echo_previous;
    assign echo_fall = ~echo_sync & echo_previous;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            echo_meta     <= 1'b0;
            echo_sync     <= 1'b0;
            echo_previous <= 1'b0;
        end else begin
            echo_meta     <= echo_in;
            echo_sync     <= echo_meta;
            echo_previous <= echo_sync;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= ST_PERIOD;
            timer_count      <= 32'd0;
            echo_cycle_count <= 32'd0;
            echo_us_phase    <= 32'd0;
            echo_us_count    <= 16'd0;
            distance_product <= 32'd0;
            trigger_out      <= 1'b0;
            distance_mm      <= 16'd0;
            pulse_width_us   <= 16'd0;
            sample_valid     <= 1'b0;
            timeout_error    <= 1'b0;
            busy             <= 1'b0;
        end else begin
            sample_valid  <= 1'b0;
            timeout_error <= 1'b0;

            case (state)
                ST_PERIOD: begin
                    trigger_out <= 1'b0;
                    busy        <= 1'b0;
                    if (timer_count >= (PERIOD_CYCLES - 1)) begin
                        timer_count <= 32'd0;
                        trigger_out <= 1'b1;
                        busy        <= 1'b1;
                        state       <= ST_TRIGGER;
                    end else begin
                        timer_count <= timer_count + 1'b1;
                    end
                end

                ST_TRIGGER: begin
                    trigger_out <= 1'b1;
                    busy        <= 1'b1;
                    if (timer_count >= (TRIGGER_CYCLES - 1)) begin
                        timer_count <= 32'd0;
                        trigger_out <= 1'b0;
                        state       <= ST_WAIT_ECHO;
                    end else begin
                        timer_count <= timer_count + 1'b1;
                    end
                end

                ST_WAIT_ECHO: begin
                    trigger_out <= 1'b0;
                    busy        <= 1'b1;
                    if (echo_rise) begin
                        timer_count      <= 32'd0;
                        echo_cycle_count <= 32'd1;
                        // 相位累加器从半微秒偏置开始，实现任意 CLK_FREQ_HZ
                        // 下的四舍五入；上升沿所在周期也计入脉宽。
                        if (((CLK_FREQ_HZ / 2) + 1000000) >= CLK_FREQ_HZ) begin
                            echo_us_phase <= (CLK_FREQ_HZ / 2) + 1000000 - CLK_FREQ_HZ;
                            echo_us_count <= 16'd1;
                        end else begin
                            echo_us_phase <= (CLK_FREQ_HZ / 2) + 1000000;
                            echo_us_count <= 16'd0;
                        end
                        state            <= ST_MEASURE;
                    end else if (timer_count >= (TIMEOUT_CYCLES - 1)) begin
                        timer_count   <= 32'd0;
                        timeout_error <= 1'b1;
                        busy          <= 1'b0;
                        state         <= ST_PERIOD;
                    end else begin
                        timer_count <= timer_count + 1'b1;
                    end
                end

                ST_MEASURE: begin
                    trigger_out <= 1'b0;
                    busy        <= 1'b1;
                    if (echo_fall) begin
                        // 先把周期计数舍入为微秒，再用 Q16 常数近似 5/29。
                        // 乘法结果寄存后下一拍发布，避免综合出长组合除法路径。
                        pulse_width_us <= echo_us_count;
                        distance_product <=
                            (echo_us_count * DISTANCE_SCALE_Q16) + 16'd32768;
                        echo_cycle_count <= 32'd0;
                        echo_us_phase    <= 32'd0;
                        echo_us_count    <= 16'd0;
                        state            <= ST_CALCULATE;
                    end else if (echo_cycle_count >= TIMEOUT_CYCLES) begin
                        timeout_error    <= 1'b1;
                        timer_count      <= 32'd0;
                        echo_cycle_count <= 32'd0;
                        echo_us_phase    <= 32'd0;
                        echo_us_count    <= 16'd0;
                        busy             <= 1'b0;
                        state            <= ST_PERIOD;
                    end else begin
                        echo_cycle_count <= echo_cycle_count + 1'b1;
                        if ((echo_us_phase + 1000000) >= CLK_FREQ_HZ) begin
                            echo_us_phase <= echo_us_phase + 1000000 - CLK_FREQ_HZ;
                            echo_us_count <= echo_us_count + 1'b1;
                        end else begin
                            echo_us_phase <= echo_us_phase + 1000000;
                        end
                    end
                end

                ST_CALCULATE: begin
                    distance_mm  <= distance_product[31:16];
                    sample_valid <= 1'b1;
                    timer_count  <= 32'd0;
                    busy         <= 1'b0;
                    state        <= ST_PERIOD;
                end

                default: begin
                    state       <= ST_PERIOD;
                    timer_count <= 32'd0;
                    trigger_out <= 1'b0;
                    busy        <= 1'b0;
                end
            endcase
        end
    end

endmodule
