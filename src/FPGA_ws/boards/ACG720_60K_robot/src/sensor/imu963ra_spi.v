// LSM6DSR 六轴 SPI 采集驱动（IMU963RA 模块）
// 50 MHz 系统时钟下产生 1 MHz、Mode 0 SPI；输出原始有符号二补码数据。
module imu963ra_spi (
    input  wire        clk,
    input  wire        reset_n,
    input  wire        miso,
    output wire        sclk,
    output wire        mosi,
    output reg         cs_n,
    output reg  [7:0]  who_am_i,
    output reg         ready,
    output reg         sample_valid,
    output reg  [95:0] sample
);

    localparam [31:0] BOOT_WAIT_CYCLES   = 32'd500000;   // 10 ms
    localparam [31:0] RETRY_WAIT_CYCLES  = 32'd2500000;  // 50 ms
    localparam [31:0] RESET_WAIT_CYCLES  = 32'd100000;   // 2 ms
    localparam [31:0] POLL_WAIT_CYCLES   = 32'd50000;    // 1 ms
    localparam [31:0] SAMPLE_STALE_LIMIT = 32'd2500000;  // 50 ms

    localparam [5:0] ST_BOOT_WAIT       = 6'd0;
    localparam [5:0] ST_RETRY_WAIT      = 6'd1;
    localparam [5:0] ST_PROBE_RESULT    = 6'd2;
    localparam [5:0] ST_PAGE_CLEAR      = 6'd3;
    localparam [5:0] ST_RESET_SETTLE    = 6'd4;
    localparam [5:0] ST_RESET_RESULT    = 6'd5;
    localparam [5:0] ST_RESET_POLL_WAIT = 6'd6;
    localparam [5:0] ST_POST_RESET_ID   = 6'd7;
    localparam [5:0] ST_CFG_CTRL3       = 6'd8;
    localparam [5:0] ST_CFG_CTRL9       = 6'd9;
    localparam [5:0] ST_CFG_CTRL1       = 6'd10;
    localparam [5:0] ST_CFG_CTRL2       = 6'd11;
    localparam [5:0] ST_VERIFY_CTRL3    = 6'd12;
    localparam [5:0] ST_VERIFY_CTRL9    = 6'd13;
    localparam [5:0] ST_VERIFY_CTRL1    = 6'd14;
    localparam [5:0] ST_VERIFY_CTRL2    = 6'd15;
    localparam [5:0] ST_POLL_WAIT       = 6'd16;
    localparam [5:0] ST_STATUS_RESULT   = 6'd17;
    localparam [5:0] ST_BURST_CMD_WAIT  = 6'd18;
    localparam [5:0] ST_BURST_DATA_WAIT = 6'd19;
    localparam [5:0] ST_AFTER_SAMPLE    = 6'd20;
    localparam [5:0] ST_HEALTH_RESULT   = 6'd21;
    localparam [5:0] ST_SPI_CMD_WAIT    = 6'd22;
    localparam [5:0] ST_SPI_WRITE_WAIT  = 6'd23;
    localparam [5:0] ST_SPI_READ_WAIT   = 6'd24;

    reg [5:0] state;
    reg [5:0] return_state;
    reg [31:0] wait_count;
    reg [31:0] sample_age;
    reg [6:0] reset_poll_count;
    reg [5:0] health_sample_count;
    reg [3:0] burst_index;

    reg        op_is_read;
    reg [6:0]  op_addr;
    reg [7:0]  op_value;
    reg [7:0]  op_result;

    reg [7:0]  spi_tx_byte;
    reg        spi_start;
    reg        spi_abort;
    wire       spi_done;
    wire [7:0] spi_rx_byte;
    reg [7:0]  sample_low_byte;
    reg [95:16] sample_staging;

    reg [7:0]  spi_rx_data;
    reg [7:0]  spi_tx_shift;
    reg [7:0]  spi_rx_shift;
    reg [5:0]  spi_div_count;
    reg [2:0]  spi_bit_index;
    reg        spi_busy_reg;
    reg        spi_done_reg;
    reg        spi_sclk_reg;
    reg        spi_mosi_reg;

    assign sclk        = spi_sclk_reg;
    assign mosi        = spi_mosi_reg;
    assign spi_done    = spi_done_reg;
    assign spi_rx_byte = spi_rx_data;

    // 每个字节按 Mode 0 传输：上升沿采 MISO，下降沿更新 MOSI。
    // 25 个系统时钟为半周期，因此 50 MHz / 50 = 1 MHz。
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            spi_rx_data   <= 8'h00;
            spi_tx_shift  <= 8'h00;
            spi_rx_shift  <= 8'h00;
            spi_div_count <= 6'd0;
            spi_bit_index <= 3'd0;
            spi_busy_reg  <= 1'b0;
            spi_done_reg  <= 1'b0;
            spi_sclk_reg  <= 1'b0;
            spi_mosi_reg  <= 1'b0;
        end else begin
            spi_done_reg <= 1'b0;

            if (spi_abort) begin
                spi_busy_reg  <= 1'b0;
                spi_div_count <= 6'd0;
                spi_bit_index <= 3'd0;
                spi_sclk_reg  <= 1'b0;
                spi_mosi_reg  <= 1'b0;
            end else if (spi_start && !spi_busy_reg) begin
                spi_busy_reg  <= 1'b1;
                spi_div_count <= 6'd0;
                spi_bit_index <= 3'd7;
                spi_tx_shift  <= spi_tx_byte;
                spi_rx_shift  <= 8'h00;
                spi_sclk_reg  <= 1'b0;
                spi_mosi_reg  <= spi_tx_byte[7];
            end else if (spi_busy_reg) begin
                if (spi_div_count == 6'd24) begin
                    spi_div_count <= 6'd0;
                    if (!spi_sclk_reg) begin
                        spi_sclk_reg <= 1'b1;
                        spi_rx_shift <= {spi_rx_shift[6:0], miso};
                    end else begin
                        spi_sclk_reg <= 1'b0;
                        if (spi_bit_index == 3'd0) begin
                            spi_busy_reg <= 1'b0;
                            spi_done_reg <= 1'b1;
                            spi_rx_data  <= spi_rx_shift;
                        end else begin
                            spi_bit_index <= spi_bit_index - 3'd1;
                            spi_mosi_reg  <= spi_tx_shift[spi_bit_index - 3'd1];
                        end
                    end
                end else begin
                    spi_div_count <= spi_div_count + 6'd1;
                end
            end
        end
    end

    // 启动一个单寄存器 SPI 读事务。
    task issue_read;
        input [6:0] addr;
        input [5:0] next_state;
        begin
            op_is_read   <= 1'b1;
            op_addr      <= addr;
            return_state <= next_state;
            cs_n         <= 1'b0;
            spi_tx_byte  <= {1'b1, addr};
            spi_start    <= 1'b1;
            state        <= ST_SPI_CMD_WAIT;
        end
    endtask

    // 启动一个单寄存器 SPI 写事务。
    task issue_write;
        input [6:0] addr;
        input [7:0] value;
        input [5:0] next_state;
        begin
            op_is_read   <= 1'b0;
            op_addr      <= addr;
            op_value     <= value;
            return_state <= next_state;
            cs_n         <= 1'b0;
            spi_tx_byte  <= {1'b0, addr};
            spi_start    <= 1'b1;
            state        <= ST_SPI_CMD_WAIT;
        end
    endtask

    // 初始化、可恢复探测、配置回读和六轴数据采集。
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state              <= ST_BOOT_WAIT;
            return_state       <= ST_BOOT_WAIT;
            wait_count         <= 32'd0;
            sample_age         <= 32'd0;
            reset_poll_count   <= 7'd0;
            health_sample_count<= 6'd0;
            burst_index        <= 4'd0;
            op_is_read         <= 1'b0;
            op_addr            <= 7'h00;
            op_value           <= 8'h00;
            op_result          <= 8'h00;
            spi_tx_byte        <= 8'h00;
            spi_start          <= 1'b0;
            spi_abort          <= 1'b0;
            sample_low_byte    <= 8'h00;
            sample_staging     <= 80'd0;
            cs_n               <= 1'b1;
            who_am_i           <= 8'h00;
            ready              <= 1'b0;
            sample_valid       <= 1'b0;
            sample             <= 96'd0;
        end else begin
            spi_start    <= 1'b0;
            spi_abort    <= 1'b0;
            sample_valid <= 1'b0;

            if (ready && sample_age >= (SAMPLE_STALE_LIMIT - 1'b1)) begin
                // 超过 50 ms 没有完整样本时撤销 ready 并重新探测。
                ready               <= 1'b0;
                sample_age          <= 32'd0;
                wait_count          <= 32'd0;
                health_sample_count<= 6'd0;
                sample_valid        <= 1'b0;
                cs_n                <= 1'b1;
                spi_abort           <= 1'b1;
                state               <= ST_RETRY_WAIT;
            end else begin
                if (ready)
                    sample_age <= sample_age + 32'd1;
                else
                    sample_age <= 32'd0;

                case (state)
                    ST_BOOT_WAIT: begin
                        if (wait_count >= (BOOT_WAIT_CYCLES - 1'b1)) begin
                            wait_count <= 32'd0;
                            issue_read(7'h0F, ST_PROBE_RESULT);
                        end else begin
                            wait_count <= wait_count + 32'd1;
                        end
                    end

                    ST_RETRY_WAIT: begin
                        if (wait_count >= (RETRY_WAIT_CYCLES - 1'b1)) begin
                            wait_count <= 32'd0;
                            issue_read(7'h0F, ST_PROBE_RESULT);
                        end else begin
                            wait_count <= wait_count + 32'd1;
                        end
                    end

                    ST_PROBE_RESULT: begin
                        if (op_result == 8'h6B) begin
                            issue_write(7'h01, 8'h00, ST_PAGE_CLEAR);
                        end else begin
                            ready      <= 1'b0;
                            wait_count <= 32'd0;
                            state      <= ST_RETRY_WAIT;
                        end
                    end

                    ST_PAGE_CLEAR: begin
                        wait_count       <= 32'd0;
                        reset_poll_count <= 7'd0;
                        issue_write(7'h12, 8'h01, ST_RESET_SETTLE);
                    end

                    ST_RESET_SETTLE: begin
                        if (wait_count >= (RESET_WAIT_CYCLES - 1'b1)) begin
                            wait_count <= 32'd0;
                            issue_read(7'h12, ST_RESET_RESULT);
                        end else begin
                            wait_count <= wait_count + 32'd1;
                        end
                    end

                    ST_RESET_RESULT: begin
                        if (!op_result[0]) begin
                            issue_read(7'h0F, ST_POST_RESET_ID);
                        end else if (reset_poll_count >= 7'd49) begin
                            ready      <= 1'b0;
                            wait_count <= 32'd0;
                            state      <= ST_RETRY_WAIT;
                        end else begin
                            reset_poll_count <= reset_poll_count + 7'd1;
                            wait_count       <= 32'd0;
                            state            <= ST_RESET_POLL_WAIT;
                        end
                    end

                    ST_RESET_POLL_WAIT: begin
                        if (wait_count >= (POLL_WAIT_CYCLES - 1'b1)) begin
                            wait_count <= 32'd0;
                            issue_read(7'h12, ST_RESET_RESULT);
                        end else begin
                            wait_count <= wait_count + 32'd1;
                        end
                    end

                    ST_POST_RESET_ID: begin
                        if (op_result == 8'h6B) begin
                            issue_write(7'h12, 8'h44, ST_CFG_CTRL3);
                        end else begin
                            ready      <= 1'b0;
                            wait_count <= 32'd0;
                            state      <= ST_RETRY_WAIT;
                        end
                    end

                    ST_CFG_CTRL3: begin
                        issue_write(7'h18, 8'h02, ST_CFG_CTRL9);
                    end

                    ST_CFG_CTRL9: begin
                        issue_write(7'h10, 8'h5C, ST_CFG_CTRL1);
                    end

                    ST_CFG_CTRL1: begin
                        issue_write(7'h11, 8'h58, ST_CFG_CTRL2);
                    end

                    ST_CFG_CTRL2: begin
                        issue_read(7'h12, ST_VERIFY_CTRL3);
                    end

                    ST_VERIFY_CTRL3: begin
                        if (op_result == 8'h44)
                            issue_read(7'h18, ST_VERIFY_CTRL9);
                        else begin
                            ready      <= 1'b0;
                            wait_count <= 32'd0;
                            state      <= ST_RETRY_WAIT;
                        end
                    end

                    ST_VERIFY_CTRL9: begin
                        if (op_result == 8'h02)
                            issue_read(7'h10, ST_VERIFY_CTRL1);
                        else begin
                            ready      <= 1'b0;
                            wait_count <= 32'd0;
                            state      <= ST_RETRY_WAIT;
                        end
                    end

                    ST_VERIFY_CTRL1: begin
                        if (op_result == 8'h5C)
                            issue_read(7'h11, ST_VERIFY_CTRL2);
                        else begin
                            ready      <= 1'b0;
                            wait_count <= 32'd0;
                            state      <= ST_RETRY_WAIT;
                        end
                    end

                    ST_VERIFY_CTRL2: begin
                        if (op_result == 8'h58) begin
                            ready               <= 1'b1;
                            sample_age          <= 32'd0;
                            health_sample_count<= 6'd0;
                            wait_count          <= 32'd0;
                            state               <= ST_POLL_WAIT;
                        end else begin
                            ready      <= 1'b0;
                            wait_count <= 32'd0;
                            state      <= ST_RETRY_WAIT;
                        end
                    end

                    ST_POLL_WAIT: begin
                        if (wait_count >= (POLL_WAIT_CYCLES - 1'b1)) begin
                            wait_count <= 32'd0;
                            issue_read(7'h1E, ST_STATUS_RESULT);
                        end else begin
                            wait_count <= wait_count + 32'd1;
                        end
                    end

                    ST_STATUS_RESULT: begin
                        if (op_result[1:0] == 2'b11) begin
                            cs_n        <= 1'b0;
                            spi_tx_byte <= {1'b1, 7'h22};
                            spi_start   <= 1'b1;
                            burst_index <= 4'd0;
                            state       <= ST_BURST_CMD_WAIT;
                        end else begin
                            wait_count <= 32'd0;
                            state      <= ST_POLL_WAIT;
                        end
                    end

                    ST_BURST_CMD_WAIT: begin
                        if (spi_done) begin
                            spi_tx_byte <= 8'h00;
                            spi_start   <= 1'b1;
                            burst_index <= 4'd0;
                            state       <= ST_BURST_DATA_WAIT;
                        end
                    end

                    ST_BURST_DATA_WAIT: begin
                        if (spi_done) begin
                            if (burst_index[0] == 1'b0) begin
                                sample_low_byte <= spi_rx_byte;
                            end else begin
                                case (burst_index)
                                    4'd1:  sample_staging[95:80] <= {spi_rx_byte, sample_low_byte};
                                    4'd3:  sample_staging[79:64] <= {spi_rx_byte, sample_low_byte};
                                    4'd5:  sample_staging[63:48] <= {spi_rx_byte, sample_low_byte};
                                    4'd7:  sample_staging[47:32] <= {spi_rx_byte, sample_low_byte};
                                    4'd9:  sample_staging[31:16] <= {spi_rx_byte, sample_low_byte};
                                    default: begin end
                                endcase
                            end

                            if (burst_index == 4'd11) begin
                                cs_n         <= 1'b1;
                                sample        <= {sample_staging[95:16], spi_rx_byte, sample_low_byte};
                                sample_valid  <= 1'b1;
                                sample_age    <= 32'd0;
                                state         <= ST_AFTER_SAMPLE;
                            end else begin
                                burst_index <= burst_index + 4'd1;
                                spi_tx_byte <= 8'h00;
                                spi_start   <= 1'b1;
                            end
                        end
                    end

                    ST_AFTER_SAMPLE: begin
                        if (health_sample_count == 6'd31) begin
                            health_sample_count <= 6'd0;
                            issue_read(7'h0F, ST_HEALTH_RESULT);
                        end else begin
                            health_sample_count <= health_sample_count + 6'd1;
                            wait_count          <= 32'd0;
                            state               <= ST_POLL_WAIT;
                        end
                    end

                    ST_HEALTH_RESULT: begin
                        if (op_result == 8'h6B) begin
                            wait_count <= 32'd0;
                            state      <= ST_POLL_WAIT;
                        end else begin
                            ready               <= 1'b0;
                            sample_age          <= 32'd0;
                            health_sample_count<= 6'd0;
                            wait_count          <= 32'd0;
                            state               <= ST_RETRY_WAIT;
                        end
                    end

                    ST_SPI_CMD_WAIT: begin
                        if (spi_done) begin
                            if (op_is_read) begin
                                spi_tx_byte <= 8'h00;
                                spi_start   <= 1'b1;
                                state       <= ST_SPI_READ_WAIT;
                            end else begin
                                spi_tx_byte <= op_value;
                                spi_start   <= 1'b1;
                                state       <= ST_SPI_WRITE_WAIT;
                            end
                        end
                    end

                    ST_SPI_WRITE_WAIT: begin
                        if (spi_done) begin
                            cs_n  <= 1'b1;
                            state <= return_state;
                        end
                    end

                    ST_SPI_READ_WAIT: begin
                        if (spi_done) begin
                            op_result <= spi_rx_byte;
                            if (op_addr == 7'h0F)
                                who_am_i <= spi_rx_byte;
                            cs_n  <= 1'b1;
                            state <= return_state;
                        end
                    end

                    default: begin
                        ready        <= 1'b0;
                        cs_n          <= 1'b1;
                        wait_count    <= 32'd0;
                        state         <= ST_RETRY_WAIT;
                    end
                endcase
            end
        end
    end

endmodule
