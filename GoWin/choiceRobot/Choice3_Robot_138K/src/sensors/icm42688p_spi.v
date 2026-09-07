`timescale 1ns/1ps

// ICM-42688-P SPI Mode 0 例程：校验 WHO_AM_I、初始化 Bank 0，随后突发读取 12 字节六轴样本。
module icm42688p_spi #(
    parameter integer CLK_DIV = 8,
    parameter integer STARTUP_DELAY_CYCLES = 270000,
    // PWR_MGMT0 使能六轴后保守等待 45 ms（27 MHz 下 1_215_000 拍），
    // 同时覆盖器件规定的 200 us 禁止寄存器访问窗口和典型 30 ms 陀螺启动时间。
    parameter integer SENSOR_READY_DELAY_CYCLES = 1215000,
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
    output reg signed [15:0]  accel_x,
    output reg signed [15:0]  accel_y,
    output reg signed [15:0]  accel_z,
    output reg signed [15:0]  gyro_x,
    output reg signed [15:0]  gyro_y,
    output reg signed [15:0]  gyro_z,
    output reg                sample_valid
);

    localparam [7:0] REG_WHO_AM_I     = 8'h75;
    localparam [7:0] REG_BANK_SEL     = 8'h76;
    localparam [7:0] REG_PWR_MGMT0    = 8'h4E;
    localparam [7:0] REG_GYRO_CONFIG0 = 8'h4F;
    localparam [7:0] REG_ACCEL_CONFIG0= 8'h50;
    localparam [7:0] REG_ACCEL_X1     = 8'h1F;
    localparam [7:0] REG_INT_STATUS   = 8'h2D;
    localparam [7:0] WHO_AM_I_VALUE   = 8'h47;

    localparam [3:0] ST_DELAY       = 4'd0;
    localparam [3:0] ST_WHO_START   = 4'd1;
    localparam [3:0] ST_WHO_WAIT    = 4'd2;
    localparam [3:0] ST_CFG_START   = 4'd3;
    localparam [3:0] ST_CFG_WAIT    = 4'd4;
    localparam [3:0] ST_SAMPLE_WAIT = 4'd5;
    localparam [3:0] ST_READ_START  = 4'd6;
    localparam [3:0] ST_READ_WAIT   = 4'd7;
    localparam [3:0] ST_PUBLISH     = 4'd8;
    localparam [3:0] ST_READY_WAIT  = 4'd9;
    localparam [3:0] ST_STATUS_START= 4'd10;
    localparam [3:0] ST_STATUS_WAIT = 4'd11;
    localparam [3:0] ST_FAULT       = 4'd12;

    localparam integer STARTUP_DELAY =
        (STARTUP_DELAY_CYCLES < 1) ? 1 : STARTUP_DELAY_CYCLES;
    localparam integer SENSOR_READY_DELAY =
        (SENSOR_READY_DELAY_CYCLES < 1) ? 1 : SENSOR_READY_DELAY_CYCLES;
    localparam integer SAMPLE_INTERVAL =
        (SAMPLE_INTERVAL_CYCLES < 1) ? 1 : SAMPLE_INTERVAL_CYCLES;

    reg [3:0] state;
    reg [2:0] cfg_index;
    reg [31:0] delay_count;
    reg [31:0] sample_count;
    reg txn_start;
    reg txn_read;
    reg [7:0] txn_reg;
    reg [7:0] txn_wdata;
    reg [3:0] txn_read_len;
    wire txn_busy;
    wire txn_done;
    wire txn_rx_valid;
    wire [3:0] txn_rx_index;
    wire [7:0] txn_rx_data;
    reg [7:0] rx_bytes [0:11];
    integer i;

    spi_register_transaction #(
        .CLK_DIV(CLK_DIV)
    ) u_spi_transaction (
        .clk(clk),
        .reset_n(reset_n),
        .start(txn_start),
        .is_read(txn_read),
        .reg_addr(txn_reg),
        .command_extra_mask(8'h00),
        .write_data(txn_wdata),
        .read_length(txn_read_len),
        .busy(txn_busy),
        .done(txn_done),
        .rx_valid(txn_rx_valid),
        .rx_index(txn_rx_index),
        .rx_data(txn_rx_data),
        .spi_miso(spi_miso),
        .spi_cs_n(spi_cs_n),
        .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi)
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
            txn_read_len <= 4'd0;
            sensor_ok <= 1'b0;
            initialized <= 1'b0;
            accel_x <= 16'sd0;
            accel_y <= 16'sd0;
            accel_z <= 16'sd0;
            gyro_x <= 16'sd0;
            gyro_y <= 16'sd0;
            gyro_z <= 16'sd0;
            sample_valid <= 1'b0;
            for (i = 0; i < 12; i = i + 1)
                rx_bytes[i] <= 8'd0;
        end else begin
            txn_start <= 1'b0;
            sample_valid <= 1'b0;

            if (txn_rx_valid && txn_rx_index < 12)
                rx_bytes[txn_rx_index] <= txn_rx_data;

            case (state)
                ST_DELAY: begin
                    if (delay_count >= STARTUP_DELAY - 1) begin
                        delay_count <= 32'd0;
                        state <= ST_WHO_START;
                    end else begin
                        delay_count <= delay_count + 1'b1;
                    end
                end

                ST_WHO_START: begin
                    if (!txn_busy) begin
                        txn_read <= 1'b1;
                        txn_reg <= REG_WHO_AM_I;
                        txn_read_len <= 4'd1;
                        txn_start <= 1'b1;
                        state <= ST_WHO_WAIT;
                    end
                end

                ST_WHO_WAIT: begin
                    if (txn_done) begin
                        if (rx_bytes[0] == WHO_AM_I_VALUE) begin
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
                        txn_read_len <= 4'd0;
                        case (cfg_index)
                            3'd0: begin txn_reg <= REG_BANK_SEL;      txn_wdata <= 8'h00; end
                            3'd1: begin txn_reg <= REG_GYRO_CONFIG0;  txn_wdata <= 8'h06; end
                            3'd2: begin txn_reg <= REG_ACCEL_CONFIG0; txn_wdata <= 8'h06; end
                            // PWR_MGMT0 必须最后写；写后进入 ST_READY_WAIT，期间不再访问寄存器。
                            default: begin txn_reg <= REG_PWR_MGMT0; txn_wdata <= 8'h0F; end
                        endcase
                        txn_start <= 1'b1;
                        state <= ST_CFG_WAIT;
                    end
                end

                ST_CFG_WAIT: begin
                    if (txn_done) begin
                        if (cfg_index == 3'd3) begin
                            delay_count <= 32'd0;
                            state <= ST_READY_WAIT;
                        end else begin
                            cfg_index <= cfg_index + 1'b1;
                            state <= ST_CFG_START;
                        end
                    end
                end

                ST_READY_WAIT: begin
                    if (delay_count >= SENSOR_READY_DELAY - 1) begin
                        delay_count <= 32'd0;
                        initialized <= 1'b1;
                        state <= ST_STATUS_START;
                    end else begin
                        delay_count <= delay_count + 1'b1;
                    end
                end

                ST_SAMPLE_WAIT: begin
                    if (sample_count >= SAMPLE_INTERVAL - 1) begin
                        sample_count <= 32'd0;
                        state <= ST_STATUS_START;
                    end else begin
                        sample_count <= sample_count + 1'b1;
                    end
                end

                ST_STATUS_START: begin
                    if (!txn_busy) begin
                        txn_read <= 1'b1;
                        txn_reg <= REG_INT_STATUS;
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
                        txn_reg <= REG_ACCEL_X1;
                        txn_read_len <= 4'd12;
                        txn_start <= 1'b1;
                        state <= ST_READ_WAIT;
                    end
                end

                ST_READ_WAIT: begin
                    if (txn_done)
                        state <= ST_PUBLISH;
                end

                ST_PUBLISH: begin
                    // POR 后首批数据可能使用 16'h8000 表示无效；任一轴出现
                    // 该哨兵值时整组六轴样本原子丢弃，不污染下游姿态计算。
                    if ({rx_bytes[0],rx_bytes[1]} != 16'h8000 &&
                        {rx_bytes[2],rx_bytes[3]} != 16'h8000 &&
                        {rx_bytes[4],rx_bytes[5]} != 16'h8000 &&
                        {rx_bytes[6],rx_bytes[7]} != 16'h8000 &&
                        {rx_bytes[8],rx_bytes[9]} != 16'h8000 &&
                        {rx_bytes[10],rx_bytes[11]} != 16'h8000) begin
                        accel_x <= {rx_bytes[0], rx_bytes[1]};
                        accel_y <= {rx_bytes[2], rx_bytes[3]};
                        accel_z <= {rx_bytes[4], rx_bytes[5]};
                        gyro_x  <= {rx_bytes[6], rx_bytes[7]};
                        gyro_y  <= {rx_bytes[8], rx_bytes[9]};
                        gyro_z  <= {rx_bytes[10], rx_bytes[11]};
                        sample_valid <= 1'b1;
                    end
                    state <= ST_SAMPLE_WAIT;
                end

                default: state <= ST_FAULT;
            endcase
        end
    end

endmodule

// 单次 SPI 寄存器事务。命令字 bit7 为读标志；CS 在整个突发事务中保持有效。
module spi_register_transaction #(
    parameter integer CLK_DIV = 8
) (
    input  wire       clk,
    input  wire       reset_n,
    input  wire       start,
    input  wire       is_read,
    input  wire [7:0] reg_addr,
    input  wire [7:0] command_extra_mask,
    input  wire [7:0] write_data,
    input  wire [3:0] read_length,
    output reg        busy,
    output reg        done,
    output reg        rx_valid,
    output reg  [3:0] rx_index,
    output reg  [7:0] rx_data,
    input  wire       spi_miso,
    output reg        spi_cs_n,
    output wire       spi_sclk,
    output wire       spi_mosi
);

    localparam [2:0] ST_IDLE       = 3'd0;
    localparam [2:0] ST_CMD_START  = 3'd1;
    localparam [2:0] ST_CMD_WAIT   = 3'd2;
    localparam [2:0] ST_DATA_START = 3'd3;
    localparam [2:0] ST_DATA_WAIT  = 3'd4;
    localparam [2:0] ST_FINISH     = 3'd5;

    reg [2:0] state;
    reg byte_start;
    reg [7:0] byte_tx;
    wire byte_busy;
    wire byte_done;
    wire [7:0] byte_rx;
    reg job_read;
    reg [7:0] job_wdata;
    reg [3:0] job_length;
    reg [3:0] byte_index;

    spi_mode0_byte_master #(
        .CLK_DIV(CLK_DIV)
    ) u_byte_master (
        .clk(clk),
        .reset_n(reset_n),
        .start(byte_start),
        .tx_data(byte_tx),
        .busy(byte_busy),
        .done(byte_done),
        .rx_data(byte_rx),
        .spi_miso(spi_miso),
        .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi)
    );

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= ST_IDLE;
            byte_start <= 1'b0;
            byte_tx <= 8'd0;
            busy <= 1'b0;
            done <= 1'b0;
            rx_valid <= 1'b0;
            rx_index <= 4'd0;
            rx_data <= 8'd0;
            spi_cs_n <= 1'b1;
            job_read <= 1'b0;
            job_wdata <= 8'd0;
            job_length <= 4'd0;
            byte_index <= 4'd0;
        end else begin
            byte_start <= 1'b0;
            done <= 1'b0;
            rx_valid <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                    spi_cs_n <= 1'b1;
                    if (start) begin
                        busy <= 1'b1;
                        spi_cs_n <= 1'b0;
                        job_read <= is_read;
                        job_wdata <= write_data;
                        job_length <= read_length;
                        byte_index <= 4'd0;
                        byte_tx <= (is_read ? 8'h80 : 8'h00) | command_extra_mask | reg_addr;
                        state <= ST_CMD_START;
                    end
                end

                ST_CMD_START: begin
                    if (!byte_busy) begin
                        byte_start <= 1'b1;
                        state <= ST_CMD_WAIT;
                    end
                end

                ST_CMD_WAIT: begin
                    if (byte_done) begin
                        byte_tx <= job_read ? 8'h00 : job_wdata;
                        state <= ST_DATA_START;
                    end
                end

                ST_DATA_START: begin
                    if (!byte_busy) begin
                        byte_start <= 1'b1;
                        state <= ST_DATA_WAIT;
                    end
                end

                ST_DATA_WAIT: begin
                    if (byte_done) begin
                        if (job_read) begin
                            rx_valid <= 1'b1;
                            rx_index <= byte_index;
                            rx_data <= byte_rx;
                            if (byte_index + 1'b1 >= job_length) begin
                                state <= ST_FINISH;
                            end else begin
                                byte_index <= byte_index + 1'b1;
                                byte_tx <= 8'h00;
                                state <= ST_DATA_START;
                            end
                        end else begin
                            state <= ST_FINISH;
                        end
                    end
                end

                ST_FINISH: begin
                    spi_cs_n <= 1'b1;
                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule

module spi_mode0_byte_master #(
    parameter integer CLK_DIV = 8
) (
    input  wire       clk,
    input  wire       reset_n,
    input  wire       start,
    input  wire [7:0] tx_data,
    output reg        busy,
    output reg        done,
    output reg  [7:0] rx_data,
    input  wire       spi_miso,
    output reg        spi_sclk,
    output reg        spi_mosi
);

    reg [31:0] div_count;
    reg [2:0] bit_index;
    reg [7:0] tx_latched;
    reg [7:0] rx_shift;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            busy <= 1'b0;
            done <= 1'b0;
            rx_data <= 8'd0;
            spi_sclk <= 1'b0;
            spi_mosi <= 1'b0;
            div_count <= 32'd0;
            bit_index <= 3'd0;
            tx_latched <= 8'd0;
            rx_shift <= 8'd0;
        end else begin
            done <= 1'b0;
            if (!busy) begin
                spi_sclk <= 1'b0;
                div_count <= 32'd0;
                if (start) begin
                    busy <= 1'b1;
                    bit_index <= 3'd7;
                    tx_latched <= tx_data;
                    rx_shift <= 8'd0;
                    spi_mosi <= tx_data[7];
                end
            end else if (div_count >= CLK_DIV - 1) begin
                div_count <= 32'd0;
                if (!spi_sclk) begin
                    spi_sclk <= 1'b1;
                    rx_shift[bit_index] <= spi_miso;
                end else begin
                    spi_sclk <= 1'b0;
                    if (bit_index == 3'd0) begin
                        busy <= 1'b0;
                        done <= 1'b1;
                        rx_data <= rx_shift;
                    end else begin
                        bit_index <= bit_index - 1'b1;
                        spi_mosi <= tx_latched[bit_index - 1'b1];
                    end
                end
            end else begin
                div_count <= div_count + 1'b1;
            end
        end
    end

endmodule
