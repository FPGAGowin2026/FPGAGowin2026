`timescale 1ns/1ps

// LIS3MDL SPI Mode 3 例程：校验 WHO_AM_I、配置连续转换，随后突发读取 XYZ。
module lis3mdl_spi #(
    parameter integer CLK_DIV = 8,
    parameter integer STARTUP_DELAY_CYCLES = 270000,
    parameter integer SAMPLE_INTERVAL_CYCLES = 270000
) (
    input  wire               clk,
    input  wire               reset_n,
    input  wire               spi_miso,
    output wire               spi_cs_n,
    output wire               spi_sclk,
    output wire               spi_mosi,
    output reg                sensor_ok,
    output reg                initialized,
    output reg signed [15:0]  mag_x,
    output reg signed [15:0]  mag_y,
    output reg signed [15:0]  mag_z,
    output reg                sample_valid
);

    localparam [7:0] REG_WHO_AM_I = 8'h0F;
    localparam [7:0] REG_CTRL1    = 8'h20;
    localparam [7:0] REG_CTRL2    = 8'h21;
    localparam [7:0] REG_CTRL3    = 8'h22;
    localparam [7:0] REG_CTRL4    = 8'h23;
    localparam [7:0] REG_CTRL5    = 8'h24;
    localparam [7:0] REG_STATUS   = 8'h27;
    localparam [7:0] REG_OUT_X_L  = 8'h28;
    localparam [7:0] WHO_VALUE    = 8'h3D;

    localparam [3:0] ST_DELAY       = 4'd0;
    localparam [3:0] ST_WHO_START   = 4'd1;
    localparam [3:0] ST_WHO_WAIT    = 4'd2;
    localparam [3:0] ST_CFG_START   = 4'd3;
    localparam [3:0] ST_CFG_WAIT    = 4'd4;
    localparam [3:0] ST_SAMPLE_WAIT = 4'd5;
    localparam [3:0] ST_STATUS_START= 4'd6;
    localparam [3:0] ST_STATUS_WAIT = 4'd7;
    localparam [3:0] ST_READ_START  = 4'd8;
    localparam [3:0] ST_READ_WAIT   = 4'd9;
    localparam [3:0] ST_PUBLISH     = 4'd10;
    localparam [3:0] ST_FAULT       = 4'd11;

    reg [3:0] state;
    reg [2:0] cfg_index;
    reg [31:0] delay_count;
    reg [31:0] sample_count;
    reg txn_start;
    reg txn_read;
    reg [7:0] txn_reg;
    reg [7:0] txn_wdata;
    reg [7:0] txn_extra_mask;
    reg [3:0] txn_read_len;
    wire txn_busy, txn_done, txn_rx_valid;
    wire [3:0] txn_rx_index;
    wire [7:0] txn_rx_data;
    reg [7:0] rx_bytes [0:5];
    integer i;

    spi_register_transaction_lis3mdl #(
        .CLK_DIV(CLK_DIV)
    ) u_spi_transaction (
        .clk(clk), .reset_n(reset_n), .start(txn_start),
        .is_read(txn_read), .reg_addr(txn_reg),
        .command_extra_mask(txn_extra_mask),
        .write_data(txn_wdata), .read_length(txn_read_len),
        .busy(txn_busy), .done(txn_done),
        .rx_valid(txn_rx_valid), .rx_index(txn_rx_index), .rx_data(txn_rx_data),
        .spi_miso(spi_miso), .spi_cs_n(spi_cs_n),
        .spi_sclk(spi_sclk), .spi_mosi(spi_mosi)
    );

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= ST_DELAY;
            cfg_index <= 3'd0;
            delay_count <= 32'd0;
            sample_count <= 32'd0;
            txn_start <= 1'b0;
            txn_read <= 1'b0;
            txn_reg <= 8'd0;
            txn_wdata <= 8'd0;
            txn_extra_mask <= 8'd0;
            txn_read_len <= 4'd0;
            sensor_ok <= 1'b0;
            initialized <= 1'b0;
            mag_x <= 16'sd0;
            mag_y <= 16'sd0;
            mag_z <= 16'sd0;
            sample_valid <= 1'b0;
            for (i = 0; i < 6; i = i + 1)
                rx_bytes[i] <= 8'd0;
        end else begin
            txn_start <= 1'b0;
            sample_valid <= 1'b0;
            if (txn_rx_valid && txn_rx_index < 6)
                rx_bytes[txn_rx_index] <= txn_rx_data;

            case (state)
                ST_DELAY: begin
                    if (delay_count >= STARTUP_DELAY_CYCLES - 1) begin
                        delay_count <= 32'd0;
                        state <= ST_WHO_START;
                    end else delay_count <= delay_count + 1'b1;
                end

                ST_WHO_START: begin
                    if (!txn_busy) begin
                        txn_read <= 1'b1;
                        txn_reg <= REG_WHO_AM_I;
                        txn_extra_mask <= 8'h00;
                        txn_read_len <= 4'd1;
                        txn_start <= 1'b1;
                        state <= ST_WHO_WAIT;
                    end
                end

                ST_WHO_WAIT: begin
                    if (txn_done) begin
                        if (rx_bytes[0] == WHO_VALUE) begin
                            sensor_ok <= 1'b1;
                            cfg_index <= 3'd0;
                            state <= ST_CFG_START;
                        end else begin
                            sensor_ok <= 1'b0;
                            state <= ST_FAULT;
                        end
                    end
                end

                ST_CFG_START: begin
                    if (!txn_busy) begin
                        txn_read <= 1'b0;
                        txn_extra_mask <= 8'h00;
                        txn_read_len <= 4'd0;
                        case (cfg_index)
                            3'd0: begin txn_reg <= REG_CTRL1; txn_wdata <= 8'h70; end
                            3'd1: begin txn_reg <= REG_CTRL2; txn_wdata <= 8'h00; end
                            3'd2: begin txn_reg <= REG_CTRL3; txn_wdata <= 8'h00; end
                            3'd3: begin txn_reg <= REG_CTRL4; txn_wdata <= 8'h0C; end
                            default: begin txn_reg <= REG_CTRL5; txn_wdata <= 8'h40; end
                        endcase
                        txn_start <= 1'b1;
                        state <= ST_CFG_WAIT;
                    end
                end

                ST_CFG_WAIT: begin
                    if (txn_done) begin
                        if (cfg_index == 3'd4) begin
                            initialized <= 1'b1;
                            sample_count <= 32'd0;
                            state <= ST_SAMPLE_WAIT;
                        end else begin
                            cfg_index <= cfg_index + 1'b1;
                            state <= ST_CFG_START;
                        end
                    end
                end

                ST_SAMPLE_WAIT: begin
                    if (sample_count >= SAMPLE_INTERVAL_CYCLES - 1) begin
                        sample_count <= 32'd0;
                        state <= ST_STATUS_START;
                    end else sample_count <= sample_count + 1'b1;
                end

                // CTRL1=0x70 对应 10 Hz ODR；轮询周期允许更短，因此先检查
                // STATUS_REG.ZYXDA，只有三轴新数据全部就绪时才读取并发布。
                ST_STATUS_START: begin
                    if (!txn_busy) begin
                        txn_read <= 1'b1;
                        txn_reg <= REG_STATUS;
                        txn_extra_mask <= 8'h00;
                        txn_read_len <= 4'd1;
                        txn_start <= 1'b1;
                        state <= ST_STATUS_WAIT;
                    end
                end

                ST_STATUS_WAIT: begin
                    if (txn_done) begin
                        if (rx_bytes[0][3])
                            state <= ST_READ_START;
                        else
                            state <= ST_SAMPLE_WAIT;
                    end
                end

                ST_READ_START: begin
                    if (!txn_busy) begin
                        txn_read <= 1'b1;
                        txn_reg <= REG_OUT_X_L;
                        txn_extra_mask <= 8'h40;
                        txn_read_len <= 4'd6;
                        txn_start <= 1'b1;
                        state <= ST_READ_WAIT;
                    end
                end

                ST_READ_WAIT: if (txn_done) state <= ST_PUBLISH;

                ST_PUBLISH: begin
                    mag_x <= {rx_bytes[1], rx_bytes[0]};
                    mag_y <= {rx_bytes[3], rx_bytes[2]};
                    mag_z <= {rx_bytes[5], rx_bytes[4]};
                    sample_valid <= 1'b1;
                    state <= ST_SAMPLE_WAIT;
                end

                default: state <= ST_FAULT;
            endcase
        end
    end
endmodule

module spi_register_transaction_lis3mdl #(
    parameter integer CLK_DIV = 8
) (
    input wire clk, input wire reset_n, input wire start,
    input wire is_read, input wire [7:0] reg_addr,
    input wire [7:0] command_extra_mask, input wire [7:0] write_data,
    input wire [3:0] read_length,
    output reg busy, output reg done,
    output reg rx_valid, output reg [3:0] rx_index, output reg [7:0] rx_data,
    input wire spi_miso, output reg spi_cs_n,
    output wire spi_sclk, output wire spi_mosi
);
    localparam [2:0] ST_IDLE=0, ST_CMD_START=1, ST_CMD_WAIT=2,
                     ST_DATA_START=3, ST_DATA_WAIT=4, ST_FINISH=5;
    reg [2:0] state;
    reg byte_start;
    reg [7:0] byte_tx;
    wire byte_busy, byte_done;
    wire [7:0] byte_rx;
    reg job_read;
    reg [7:0] job_wdata;
    reg [3:0] job_length, byte_index;

    spi_mode3_byte_master_lis3mdl #(.CLK_DIV(CLK_DIV)) u_byte (
        .clk(clk), .reset_n(reset_n), .start(byte_start), .tx_data(byte_tx),
        .busy(byte_busy), .done(byte_done), .rx_data(byte_rx),
        .spi_miso(spi_miso), .spi_sclk(spi_sclk), .spi_mosi(spi_mosi)
    );

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state<=ST_IDLE; byte_start<=0; byte_tx<=0; busy<=0; done<=0;
            rx_valid<=0; rx_index<=0; rx_data<=0; spi_cs_n<=1;
            job_read<=0; job_wdata<=0; job_length<=0; byte_index<=0;
        end else begin
            byte_start<=0; done<=0; rx_valid<=0;
            case (state)
                ST_IDLE: begin
                    busy<=0; spi_cs_n<=1;
                    if (start) begin
                        busy<=1; spi_cs_n<=0; job_read<=is_read;
                        job_wdata<=write_data; job_length<=read_length; byte_index<=0;
                        byte_tx<=(is_read ? 8'h80 : 8'h00)|command_extra_mask|reg_addr;
                        state<=ST_CMD_START;
                    end
                end
                ST_CMD_START: if (!byte_busy) begin byte_start<=1; state<=ST_CMD_WAIT; end
                ST_CMD_WAIT: if (byte_done) begin byte_tx<=job_read?8'h00:job_wdata; state<=ST_DATA_START; end
                ST_DATA_START: if (!byte_busy) begin byte_start<=1; state<=ST_DATA_WAIT; end
                ST_DATA_WAIT: if (byte_done) begin
                    if (job_read) begin
                        rx_valid<=1; rx_index<=byte_index; rx_data<=byte_rx;
                        if (byte_index+1'b1>=job_length) state<=ST_FINISH;
                        else begin byte_index<=byte_index+1'b1; byte_tx<=0; state<=ST_DATA_START; end
                    end else state<=ST_FINISH;
                end
                ST_FINISH: begin spi_cs_n<=1; busy<=0; done<=1; state<=ST_IDLE; end
                default: state<=ST_IDLE;
            endcase
        end
    end
endmodule

// LIS3MDL 要求 SPC 在 CS 空闲时保持高电平：CPOL=1、CPHA=1（SPI Mode 3）。
// MOSI 在下降沿更新，MISO 在上升沿采样。
module spi_mode3_byte_master_lis3mdl #(
    parameter integer CLK_DIV = 8
) (
    input wire clk, input wire reset_n, input wire start, input wire [7:0] tx_data,
    output reg busy, output reg done, output reg [7:0] rx_data,
    input wire spi_miso, output reg spi_sclk, output reg spi_mosi
);
    reg [31:0] div_count;
    reg [2:0] bit_index;
    reg [7:0] tx_latched, rx_shift;
    localparam integer SAFE_CLK_DIV = (CLK_DIV < 1) ? 1 : CLK_DIV;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            busy<=0; done<=0; rx_data<=0; spi_sclk<=1; spi_mosi<=0;
            div_count<=0; bit_index<=0; tx_latched<=0; rx_shift<=0;
        end else begin
            done<=0;
            if (!busy) begin
                spi_sclk<=1; div_count<=0;
                if (start) begin
                    busy<=1; bit_index<=7; tx_latched<=tx_data;
                    rx_shift<=0; spi_mosi<=tx_data[7];
                end
            end else if (div_count>=SAFE_CLK_DIV-1) begin
                div_count<=0;
                if (spi_sclk) begin
                    spi_sclk<=0;
                    spi_mosi<=tx_latched[bit_index];
                end else begin
                    spi_sclk<=1;
                    rx_shift[bit_index]<=spi_miso;
                    if (bit_index==0) begin
                        busy<=0;
                        done<=1;
                        rx_data<={rx_shift[7:1],spi_miso};
                    end else begin
                        bit_index<=bit_index-1'b1;
                    end
                end
            end else div_count<=div_count+1'b1;
        end
    end
endmodule
