`timescale 1ns/1ps

// IMU660RC 模块上的 LSM6DSV16X 采集器：检查 WHO_AM_I、设置 BDU 和地址递增，
// 配置加速度计与陀螺仪，并轮询两路数据就绪位后读取连续输出寄存器 (0x22..0x2D)。
// 输出为有符号16位原始计数：当前 CTRL6_G=0x02 为 ±500 dps、17.50 mdps/LSB；
// CTRL8_XL=0x01 为 ±4 g、0.122 mg/LSB。姿态角和角速度积分由软件处理。
module imu660rc_spi_reader #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer SPI_HZ = 1_000_000,
    parameter integer POWERUP_CYCLES = CLK_HZ / 100,
    parameter integer RETRY_CYCLES = CLK_HZ / 10,
    parameter integer POLL_CYCLES = CLK_HZ / 1000
) (
    input  wire               clk,
    input  wire               rst,
    input  wire               spi_miso_i,
    output wire               spi_cs_n_o,
    output wire               spi_sck_o,
    output wire               spi_mosi_o,
    output reg                imu_present_o,
    output reg                identity_checked_o,
    output reg [31:0]          identity_error_count_o,
    output reg                imu_configured_o,
    output reg                sample_valid_o,
    output reg signed [15:0]  accel_x_o,
    output reg signed [15:0]  accel_y_o,
    output reg signed [15:0]  accel_z_o,
    output reg signed [15:0]  gyro_x_o,
    output reg signed [15:0]  gyro_y_o,
    output reg signed [15:0]  gyro_z_o,
    output reg [15:0]         sample_sequence_o,
    output reg [7:0]          who_am_i_o
);
    localparam [3:0] S_POWERUP     = 4'd0;
    localparam [3:0] S_WHO_START   = 4'd1;
    localparam [3:0] S_WHO_WAIT    = 4'd2;
    localparam [3:0] S_RETRY       = 4'd3;
    localparam [3:0] S_CFG_START   = 4'd4;
    localparam [3:0] S_CFG_WAIT    = 4'd5;
    localparam [3:0] S_POLL_WAIT   = 4'd6;
    localparam [3:0] S_STATUS_START= 4'd7;
    localparam [3:0] S_STATUS_WAIT = 4'd8;
    localparam [3:0] S_DATA_START  = 4'd9;
    localparam [3:0] S_DATA_WAIT   = 4'd10;

    reg [3:0] state;
    reg [31:0] timer;
    reg [2:0] config_index;
    reg spi_start;
    reg spi_read;
    reg [6:0] spi_address;
    reg [3:0] spi_length;
    reg [7:0] spi_write_data;
    wire [95:0] spi_read_data;
    wire spi_busy;
    wire spi_done;

    function [6:0] config_address;
        input [2:0] index;
        begin
            case (index)
                3'd0: config_address = 7'h12; // CTRL3_C：BDU 与地址递增
                3'd1: config_address = 7'h15; // CTRL6：陀螺仪量程
                3'd2: config_address = 7'h17; // CTRL8：加速度计量程
                3'd3: config_address = 7'h10; // CTRL1_XL：120 Hz
                default: config_address = 7'h11; // CTRL2_G：120 Hz
            endcase
        end
    endfunction

    function [7:0] config_value;
        input [2:0] index;
        begin
            case (index)
                3'd0: config_value = 8'h44; // BDU 与地址递增
                3'd1: config_value = 8'h02; // CTRL6_G：±500 dps，17.50 mdps/LSB
                3'd2: config_value = 8'h01; // CTRL8_XL：±4 g，0.122 mg/LSB
                3'd3: config_value = 8'h06;
                default: config_value = 8'h06;
            endcase
        end
    endfunction

    lsm6dsv16x_spi_master #(.CLK_HZ(CLK_HZ), .SPI_HZ(SPI_HZ)) u_spi (
        .clk(clk), .rst(rst), .start_i(spi_start), .read_i(spi_read),
        .address_i(spi_address), .length_i(spi_length),
        .write_data_i(spi_write_data), .read_data_o(spi_read_data),
        .busy_o(spi_busy), .done_o(spi_done), .spi_cs_n_o(spi_cs_n_o),
        .spi_sck_o(spi_sck_o), .spi_mosi_o(spi_mosi_o),
        .spi_miso_i(spi_miso_i)
    );

    always @(posedge clk) begin
        if (rst) begin
            state <= S_POWERUP;
            timer <= 0;
            config_index <= 0;
            spi_start <= 1'b0;
            spi_read <= 1'b0;
            spi_address <= 0;
            spi_length <= 1;
            spi_write_data <= 0;
            imu_present_o <= 1'b0;
            identity_checked_o <= 1'b0;
            identity_error_count_o <= 32'd0;
            imu_configured_o <= 1'b0;
            sample_valid_o <= 1'b0;
            accel_x_o <= 0;
            accel_y_o <= 0;
            accel_z_o <= 0;
            gyro_x_o <= 0;
            gyro_y_o <= 0;
            gyro_z_o <= 0;
            sample_sequence_o <= 0;
            who_am_i_o <= 0;
        end else begin
            spi_start <= 1'b0;
            sample_valid_o <= 1'b0;
            case (state)
                S_POWERUP: begin
                    if (timer >= POWERUP_CYCLES-1) begin
                        timer <= 0;
                        state <= S_WHO_START;
                    end else timer <= timer + 1'b1;
                end
                S_WHO_START: begin
                    if (!spi_busy) begin
                        spi_read <= 1'b1;
                        spi_address <= 7'h0F;
                        spi_length <= 1;
                        spi_start <= 1'b1;
                        state <= S_WHO_WAIT;
                    end
                end
                S_WHO_WAIT: begin
                    if (spi_done) begin
                        who_am_i_o <= spi_read_data[7:0];
                        identity_checked_o <= 1'b1;
                        if (spi_read_data[7:0] == 8'h70) begin
                            imu_present_o <= 1'b1;
                            config_index <= 0;
                            state <= S_CFG_START;
                        end else begin
                            imu_present_o <= 1'b0;
                            imu_configured_o <= 1'b0;
                            identity_error_count_o <= identity_error_count_o + 1'b1;
                            timer <= 0;
                            state <= S_RETRY;
                        end
                    end
                end
                S_RETRY: begin
                    if (timer >= RETRY_CYCLES-1) begin
                        timer <= 0;
                        state <= S_WHO_START;
                    end else timer <= timer + 1'b1;
                end
                S_CFG_START: begin
                    if (!spi_busy) begin
                        spi_read <= 1'b0;
                        spi_address <= config_address(config_index);
                        spi_write_data <= config_value(config_index);
                        spi_length <= 1;
                        spi_start <= 1'b1;
                        state <= S_CFG_WAIT;
                    end
                end
                S_CFG_WAIT: begin
                    if (spi_done) begin
                        if (config_index == 3'd4) begin
                            imu_configured_o <= 1'b1;
                            timer <= 0;
                            state <= S_POLL_WAIT;
                        end else begin
                            config_index <= config_index + 1'b1;
                            state <= S_CFG_START;
                        end
                    end
                end
                S_POLL_WAIT: begin
                    if (timer >= POLL_CYCLES-1) begin
                        timer <= 0;
                        state <= S_STATUS_START;
                    end else timer <= timer + 1'b1;
                end
                S_STATUS_START: begin
                    if (!spi_busy) begin
                        spi_read <= 1'b1;
                        spi_address <= 7'h1E;
                        spi_length <= 1;
                        spi_start <= 1'b1;
                        state <= S_STATUS_WAIT;
                    end
                end
                S_STATUS_WAIT: begin
                    if (spi_done) begin
                        if (spi_read_data[1:0] == 2'b11)
                            state <= S_DATA_START;
                        else begin
                            timer <= 0;
                            state <= S_POLL_WAIT;
                        end
                    end
                end
                S_DATA_START: begin
                    if (!spi_busy) begin
                        spi_read <= 1'b1;
                        spi_address <= 7'h22;
                        spi_length <= 4'd12;
                        spi_start <= 1'b1;
                        state <= S_DATA_WAIT;
                    end
                end
                S_DATA_WAIT: begin
                    if (spi_done) begin
                        gyro_x_o <= $signed({spi_read_data[87:80], spi_read_data[95:88]});
                        gyro_y_o <= $signed({spi_read_data[71:64], spi_read_data[79:72]});
                        gyro_z_o <= $signed({spi_read_data[55:48], spi_read_data[63:56]});
                        accel_x_o <= $signed({spi_read_data[39:32], spi_read_data[47:40]});
                        accel_y_o <= $signed({spi_read_data[23:16], spi_read_data[31:24]});
                        accel_z_o <= $signed({spi_read_data[7:0], spi_read_data[15:8]});
                        sample_sequence_o <= sample_sequence_o + 1'b1;
                        sample_valid_o <= 1'b1;
                        timer <= 0;
                        state <= S_POLL_WAIT;
                    end
                end
                default: state <= S_POWERUP;
            endcase
        end
    end
endmodule

`timescale 1ns/1ps

// LSM6DSV16X 四线 SPI 控制器，使用 SPI Mode 3。
// 读取数据按位 MSB 优先移入；读取 12 字节时，首字节位于 [95:88]，末字节位于 [7:0]。
module lsm6dsv16x_spi_master #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer SPI_HZ = 1_000_000
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         start_i,
    input  wire         read_i,
    input  wire [6:0]   address_i,
    input  wire [3:0]   length_i,
    input  wire [7:0]   write_data_i,
    output reg  [95:0]  read_data_o,
    output reg          busy_o,
    output reg          done_o,
    output reg          spi_cs_n_o,
    output reg          spi_sck_o,
    output reg          spi_mosi_o,
    input  wire         spi_miso_i
);
    localparam integer HALF_DIV = (CLK_HZ + (SPI_HZ * 2 - 1)) /
                                  (SPI_HZ * 2);
    localparam integer HALF_DIV_W = (HALF_DIV <= 2) ? 1 : $clog2(HALF_DIV);
    reg [HALF_DIV_W-1:0] divider;
    reg [6:0] bit_count;
    reg [6:0] total_bits;
    reg [7:0] command_byte;
    reg [7:0] data_byte;
    reg read_active;
    reg finish_hold;

    always @(posedge clk) begin
        if (rst) begin
            divider      <= 0;
            bit_count    <= 0;
            total_bits   <= 0;
            command_byte <= 0;
            data_byte    <= 0;
            read_active  <= 0;
            finish_hold  <= 1'b0;
            read_data_o  <= 0;
            busy_o       <= 1'b0;
            done_o       <= 1'b0;
            spi_cs_n_o   <= 1'b1;
            spi_sck_o    <= 1'b1; // CPOL=1，对应 Mode 3
            spi_mosi_o   <= 1'b0;
        end else begin
            done_o <= 1'b0;
            if (!busy_o) begin
                if (start_i && (length_i >= 1) && (length_i <= 12)) begin
                    busy_o       <= 1'b1;
                    spi_cs_n_o   <= 1'b0;
                    spi_sck_o    <= 1'b1;
                    divider      <= 0;
                    bit_count    <= 0;
                    total_bits   <= ({3'd0, length_i} + 7'd1) << 3;
                    command_byte <= {read_i, address_i};
                    data_byte    <= write_data_i;
                    read_active  <= read_i;
                    finish_hold  <= 1'b0;
                    read_data_o  <= 0;
                end
            end else if (divider == HALF_DIV-1) begin
                divider <= 0;
                if (finish_hold) begin
                    // 最后一个下降沿后继续保持 CS 有效半个周期，再在 SCK 空闲时释放 CS。
                    finish_hold <= 1'b0;
                    busy_o <= 1'b0;
                    done_o <= 1'b1;
                    spi_cs_n_o <= 1'b1;
                    spi_mosi_o <= 1'b0;
                end else if (spi_sck_o) begin
                    // CPHA=1：在第一个边沿更新下一输出位。
                    spi_sck_o <= 1'b0;
                    if (bit_count < 8)
                        spi_mosi_o <= command_byte[7-bit_count[2:0]];
                    else if (!read_active && bit_count < 16)
                        spi_mosi_o <= data_byte[7-bit_count[2:0]];
                    else
                        spi_mosi_o <= 1'b0;
                end else begin
                    // 在第二个边沿采样输入位（Mode 3 中为上升沿）。
                    spi_sck_o <= 1'b1;
                    if (read_active && (bit_count >= 8))
                        read_data_o <= {read_data_o[94:0], spi_miso_i};
                    if (bit_count + 1'b1 >= total_bits) begin
                        finish_hold <= 1'b1;
                    end else begin
                        bit_count <= bit_count + 1'b1;
                    end
                end
            end else begin
                divider <= divider + 1'b1;
            end
        end
    end
endmodule
