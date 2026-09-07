`timescale 1ns/1ps

module tb_icm42688p;
    reg clk = 1'b0;
    reg reset_n = 1'b0;
    wire cs_n;
    wire sclk;
    wire mosi;
    reg miso = 1'b0;
    wire sensor_ok;
    wire initialized;
    reg [2:0] axis_select = 3'd0;
    wire signed [15:0] axis_data;
    wire sample_valid;
    reg [7:0] regs [0:127];
    reg [7:0] cmd;
    reg [7:0] value;
    reg [6:0] addr;
    reg [6:0] start_addr;
    reg byte_completed;
    integer i;
    integer errors = 0;
    integer system_cycles = 0;
    integer power_enable_cycle = -1;
    reg post_power_access_checked = 1'b0;
    integer status_reads = 0;
    integer burst_reads = 0;

    localparam integer TB_READY_DELAY_CYCLES = 20;

    always #5 clk = ~clk;
    always @(posedge clk) system_cycles = system_cycles + 1;

    sensor_example_top #(
        // 100 MHz TB 时钟下得到约 16.7 MHz SCLK，低于器件 24 MHz 上限。
        .CLK_DIV(3),
        .STARTUP_DELAY_CYCLES(4),
        .SENSOR_READY_DELAY_CYCLES(TB_READY_DELAY_CYCLES),
        .SAMPLE_INTERVAL_CYCLES(8)
    ) dut (
        .clk(clk), .reset_n(reset_n), .imu_miso(miso),
        .imu_cs_n(cs_n), .imu_sclk(sclk), .imu_mosi(mosi),
        .sensor_ok(sensor_ok), .initialized(initialized),
        .axis_select(axis_select), .axis_data(axis_data),
        .sample_valid(sample_valid)
    );

    task spi_recv_byte;
        output reg [7:0] data;
        integer bit_no;
        begin
            data = 8'd0;
            for (bit_no = 7; bit_no >= 0; bit_no = bit_no - 1) begin
                @(posedge sclk);
                data[bit_no] = mosi;
            end
        end
    endtask

    task spi_send_byte;
        input [7:0] data;
        integer bit_no;
        begin
            for (bit_no = 7; bit_no >= 0; bit_no = bit_no - 1) begin
                @(negedge sclk);
                miso = data[bit_no];
            end
        end
    endtask

    initial begin
        for (i = 0; i < 128; i = i + 1)
            regs[i] = 8'h00;
        regs[7'h75] = 8'h47;
        regs[7'h2D] = 8'h00;
        regs[7'h1F] = 8'h12; regs[7'h20] = 8'h34;
        regs[7'h21] = 8'hFE; regs[7'h22] = 8'hDC;
        regs[7'h23] = 8'h00; regs[7'h24] = 8'h80;
        regs[7'h25] = 8'h7F; regs[7'h26] = 8'hFF;
        regs[7'h27] = 8'h80; regs[7'h28] = 8'h00;
        regs[7'h29] = 8'hAA; regs[7'h2A] = 8'h55;

        forever begin
            @(negedge cs_n);
            if (power_enable_cycle >= 0 && !post_power_access_checked) begin
                if (system_cycles - power_enable_cycle < TB_READY_DELAY_CYCLES) begin
                    $display("ERROR: access before sensor-ready guard elapsed");
                    errors = errors + 1;
                end
                post_power_access_checked = 1'b1;
            end
            spi_recv_byte(cmd);
            addr = cmd[6:0];
            start_addr = cmd[6:0];
            if (post_power_access_checked && power_enable_cycle >= 0 && !cmd[7]) begin
                $display("ERROR: PWR_MGMT0 was not the final configuration write");
                errors = errors + 1;
            end
            if (cmd[7]) begin
                while (!cs_n) begin
                    byte_completed = 1'b0;
                    fork : send_or_chip_select_end
                        begin
                            spi_send_byte(regs[addr]);
                            byte_completed = 1'b1;
                        end
                        begin
                            @(posedge cs_n);
                        end
                    join_any
                    disable send_or_chip_select_end;
                    if (byte_completed)
                        addr = addr + 1'b1;
                end
                if (start_addr == 7'h2D) begin
                    status_reads = status_reads + 1;
                    if (status_reads == 1) regs[7'h2D] = 8'h08;
                end
                if (start_addr == 7'h1F) begin
                    burst_reads = burst_reads + 1;
                    // 第一批 gyro_y=0x8000 是 POR 无效哨兵，下一批改为有效值。
                    if (burst_reads == 1) begin
                        regs[7'h27] = 8'h80;
                        regs[7'h28] = 8'h01;
                    end
                end
            end else begin
                spi_recv_byte(value);
                regs[addr] = value;
                @(posedge cs_n);
                if (addr == 7'h4E && value == 8'h0F)
                    power_enable_cycle = system_cycles;
            end
            miso = 1'b0;
        end
    end

    initial begin
        #30 reset_n = 1'b1;
        wait(initialized);
        if (!sensor_ok) begin $display("ERROR: WHO_AM_I mismatch"); errors = errors + 1; end
        if (regs[7'h76] !== 8'h00) begin $display("ERROR: REG_BANK_SEL"); errors = errors + 1; end
        if (regs[7'h4E] !== 8'h0F) begin $display("ERROR: PWR_MGMT0"); errors = errors + 1; end
        if (regs[7'h4F] !== 8'h06) begin $display("ERROR: GYRO_CONFIG0"); errors = errors + 1; end
        if (regs[7'h50] !== 8'h06) begin $display("ERROR: ACCEL_CONFIG0"); errors = errors + 1; end
        @(posedge sample_valid);
        #1;
        if (status_reads < 2) begin $display("ERROR: DATA_RDY was not polled"); errors = errors + 1; end
        if (burst_reads < 2) begin $display("ERROR: 0x8000 sentinel was not rejected"); errors = errors + 1; end
        axis_select = 3'd0; #1; if (axis_data !== 16'sh1234) begin $display("ERROR: accel_x=%h", axis_data); errors = errors + 1; end
        axis_select = 3'd1; #1; if (axis_data !== 16'shFEDC) begin $display("ERROR: accel_y=%h", axis_data); errors = errors + 1; end
        axis_select = 3'd2; #1; if (axis_data !== 16'sh0080) begin $display("ERROR: accel_z=%h", axis_data); errors = errors + 1; end
        axis_select = 3'd3; #1; if (axis_data !== 16'sh7FFF) begin $display("ERROR: gyro_x=%h", axis_data); errors = errors + 1; end
        axis_select = 3'd4; #1; if (axis_data !== 16'sh8001) begin $display("ERROR: gyro_y=%h", axis_data); errors = errors + 1; end
        axis_select = 3'd5; #1; if (axis_data !== 16'shAA55) begin $display("ERROR: gyro_z=%h", axis_data); errors = errors + 1; end
        if (errors == 0) $display("PASS: ICM-42688-P SPI initialization and burst sample read");
        else $fatal(1, "FAIL: %0d errors", errors);
        #20 $finish;
    end

    initial begin
        #200000 $fatal(1, "FAIL: timeout");
    end
endmodule
