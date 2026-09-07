`timescale 1ns/1ps

// TSL1401/TSL1401CL 128 像素线阵时序与外部 ADC 握手前端。
// AO 是模拟电压，绝不能直接接入普通 FPGA GPIO；adc_data/adc_valid 必须
// 来自外部并行 ADC、SPI ADC 的采样桥或板上专用 ADC 接口。
module tsl1401_adc_frontend #(
    parameter integer ADC_WIDTH = 12,
    parameter integer CLK_HALF_PERIOD_CYCLES = 8,
    parameter integer PIXEL_SETTLE_CYCLES = 8,
    parameter integer ADC_TIMEOUT_CYCLES = 256,
    parameter integer INTEGRATION_CYCLES = 270000,
    // 27 MHz 下 540 个周期等于数据手册要求的最小 tqt=20 us。
    // 改变系统时钟时必须相应调整；有效帧间隔取它与 INTEGRATION_CYCLES 的较大值。
    parameter integer CHARGE_TRANSFER_CYCLES = 540
) (
    input  wire                 clk,
    input  wire                 reset_n,
    output reg                  sensor_clk,
    output reg                  sensor_si,
    output reg                  adc_start,
    input  wire [ADC_WIDTH-1:0] adc_data,
    input  wire                 adc_valid,
    output reg  [6:0]           pixel_index,
    output reg  [ADC_WIDTH-1:0] pixel_data,
    output reg                  pixel_valid,
    output reg                  frame_start,
    output reg                  frame_done,
    output reg                  frame_active,
    output reg                  adc_timeout
);
    localparam [3:0] ST_INTEGRATE = 4'd0;
    localparam [3:0] ST_SI_SETUP  = 4'd1;
    localparam [3:0] ST_CLK_HIGH  = 4'd2;
    localparam [3:0] ST_CLK_LOW   = 4'd3;
    localparam [3:0] ST_SETTLE    = 4'd4;
    localparam [3:0] ST_ADC_START = 4'd5;
    localparam [3:0] ST_ADC_WAIT  = 4'd6;
    localparam [3:0] ST_NEXT      = 4'd7;
    localparam [3:0] ST_FLUSH_HIGH = 4'd8;
    localparam [3:0] ST_FLUSH_LOW  = 4'd9;

    localparam integer HALF_PERIOD =
        (CLK_HALF_PERIOD_CYCLES < 1) ? 1 : CLK_HALF_PERIOD_CYCLES;
    localparam integer SETTLE_TIME =
        (PIXEL_SETTLE_CYCLES < 1) ? 1 : PIXEL_SETTLE_CYCLES;
    localparam integer ADC_TIMEOUT_LIMIT =
        (ADC_TIMEOUT_CYCLES < 1) ? 1 : ADC_TIMEOUT_CYCLES;
    localparam integer CHARGE_TRANSFER_TIME =
        (CHARGE_TRANSFER_CYCLES < 1) ? 1 : CHARGE_TRANSFER_CYCLES;
    localparam integer REQUESTED_INTEGRATION_TIME =
        (INTEGRATION_CYCLES < 1) ? 1 : INTEGRATION_CYCLES;
    localparam integer INTEGRATION_TIME =
        (REQUESTED_INTEGRATION_TIME < CHARGE_TRANSFER_TIME) ?
        CHARGE_TRANSFER_TIME : REQUESTED_INTEGRATION_TIME;

    reg [3:0] state;
    reg [31:0] timer;
    // TSL1401 每行必须有 129 个 CLK 上升沿：前 128 个移出像素，
    // 第 129 个用于结束本次移位。ADC 超时时也把剩余时钟补齐，避免
    // 下一次 SI 从未知的模拟移位位置开始。
    reg [7:0] flush_clocks_remaining;
    reg       flush_publish_frame;
    reg       primed;
    reg       prime_cycle;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= ST_INTEGRATE;
            timer <= 0;
            sensor_clk <= 0;
            sensor_si <= 0;
            adc_start <= 0;
            pixel_index <= 0;
            pixel_data <= 0;
            pixel_valid <= 0;
            frame_start <= 0;
            frame_done <= 0;
            frame_active <= 0;
            adc_timeout <= 0;
            flush_clocks_remaining <= 0;
            flush_publish_frame <= 0;
            primed <= 0;
            prime_cycle <= 0;
        end else begin
            adc_start <= 1'b0;
            pixel_valid <= 1'b0;
            frame_start <= 1'b0;
            frame_done <= 1'b0;
            case (state)
                ST_INTEGRATE: begin
                    sensor_clk <= 1'b0;
                    sensor_si <= 1'b0;
                    frame_active <= 1'b0;
                    if (timer >= INTEGRATION_TIME-1) begin
                        timer <= 0;
                        pixel_index <= 0;
                        prime_cycle <= !primed;
                        state <= ST_SI_SETUP;
                    end else timer <= timer + 1'b1;
                end
                ST_SI_SETUP: begin
                    // SI 在首个 CLK 上升沿前建立，只保持一个传感器时钟周期。
                    sensor_si <= 1'b1;
                    sensor_clk <= 1'b0;
                    frame_active <= 1'b1;
                    // 上电后的第一次 SI+129 CLK 仅清出不确定数据。
                    frame_start <= primed;
                    timer <= 0;
                    state <= ST_CLK_HIGH;
                end
                ST_CLK_HIGH: begin
                    sensor_clk <= 1'b1;
                    if (timer >= HALF_PERIOD-1) begin
                        timer <= 0;
                        state <= ST_CLK_LOW;
                    end else timer <= timer + 1'b1;
                end
                ST_CLK_LOW: begin
                    sensor_clk <= 1'b0;
                    sensor_si <= 1'b0;
                    if (timer >= HALF_PERIOD-1) begin
                        timer <= 0;
                        if (prime_cycle) begin
                            // 首个 CLK 已经完成，再补 128 个，总计 129 个。
                            flush_clocks_remaining <= 8'd128;
                            flush_publish_frame <= 1'b0;
                            state <= ST_FLUSH_HIGH;
                        end else state <= ST_SETTLE;
                    end else timer <= timer + 1'b1;
                end
                ST_SETTLE: begin
                    if (timer >= SETTLE_TIME-1) begin
                        timer <= 0;
                        state <= ST_ADC_START;
                    end else timer <= timer + 1'b1;
                end
                ST_ADC_START: begin
                    adc_start <= 1'b1;
                    timer <= 0;
                    state <= ST_ADC_WAIT;
                end
                ST_ADC_WAIT: begin
                    if (adc_valid) begin
                        pixel_data <= adc_data;
                        pixel_valid <= 1'b1;
                        timer <= 0;
                        state <= ST_NEXT;
                    end else if (timer >= ADC_TIMEOUT_LIMIT-1) begin
                        // 当前像素的 CLK 已经发出；补足到一行总计 129 个
                        // CLK 后丢弃整帧，不发布 frame_done。
                        adc_timeout <= 1'b1;
                        sensor_si <= 1'b0;
                        flush_clocks_remaining <= 8'd128 - {1'b0, pixel_index};
                        flush_publish_frame <= 1'b0;
                        timer <= 0;
                        state <= ST_FLUSH_HIGH;
                    end else timer <= timer + 1'b1;
                end
                ST_NEXT: begin
                    if (pixel_index == 7'd127) begin
                        // 128 个像素采样完毕后仍需第 129 个终止时钟。
                        flush_clocks_remaining <= 8'd1;
                        flush_publish_frame <= 1'b1;
                        timer <= 0;
                        state <= ST_FLUSH_HIGH;
                    end else begin
                        pixel_index <= pixel_index + 1'b1;
                        timer <= 0;
                        state <= ST_CLK_HIGH;
                    end
                end
                ST_FLUSH_HIGH: begin
                    sensor_si <= 1'b0;
                    sensor_clk <= 1'b1;
                    if (timer >= HALF_PERIOD-1) begin
                        timer <= 0;
                        state <= ST_FLUSH_LOW;
                    end else timer <= timer + 1'b1;
                end
                ST_FLUSH_LOW: begin
                    sensor_clk <= 1'b0;
                    sensor_si <= 1'b0;
                    if (timer >= HALF_PERIOD-1) begin
                        timer <= 0;
                        if (flush_clocks_remaining == 8'd1) begin
                            if (flush_publish_frame)
                                frame_done <= 1'b1;
                            frame_active <= 1'b0;
                            flush_clocks_remaining <= 0;
                            flush_publish_frame <= 1'b0;
                            if (prime_cycle) begin
                                primed <= 1'b1;
                                prime_cycle <= 1'b0;
                            end
                            state <= ST_INTEGRATE;
                        end else begin
                            flush_clocks_remaining <= flush_clocks_remaining - 1'b1;
                            state <= ST_FLUSH_HIGH;
                        end
                    end else timer <= timer + 1'b1;
                end
                default: state <= ST_INTEGRATE;
            endcase
        end
    end
endmodule
