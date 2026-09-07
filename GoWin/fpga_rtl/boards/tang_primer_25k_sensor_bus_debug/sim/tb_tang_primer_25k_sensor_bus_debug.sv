`timescale 1ns/1ps

module tb_tang_primer_25k_sensor_bus_debug;
    localparam integer CLK_FREQ_HZ = 10000000;
    localparam integer BAUD_RATE = 1000000;
    localparam integer BIT_NS = 1000;

    reg clk_i;
    reg reset_i;
    reg uart_rx_i;
    wire uart_tx_o;
    tri1 ina219_scl_io, ina219_sda_io;
    tri1 mpu6050_scl_io, mpu6050_sda_io;
    tri1 vl53l0x_scl_io, vl53l0x_sda_io;
    tri1 vl53l1x_scl_io, vl53l1x_sda_io;

    reg [3:0] scl_low_seen;
    reg [3:0] sda_low_seen;
    reg [7:0] received_byte;
    integer index;

    localparam [31*8-1:0] EXPECT_BANNER =
        {"GW5A25 SENSOR BUS DEBUG READY", 8'h0d, 8'h0a};
    localparam [22*8-1:0] EXPECT_PONG =
        {"PONG;BUS_DEBUG=READY", 8'h0d, 8'h0a};
    localparam [46*8-1:0] EXPECT_BIST =
        {"BUS_BIST=PASS;I2C=4/4;SPI=8/8;MODE=NO_DEVICE", 8'h0d, 8'h0a};
    localparam [42*8-1:0] EXPECT_DETAIL =
        {"I2C_ERR=0xF;SPI_TXN=0xFF;SPI_REJECT=0xFF", 8'h0d, 8'h0a};
    localparam [12*8-1:0] EXPECT_HELP =
        {"CMDS:P,B,D", 8'h0d, 8'h0a};

    tang_primer_25k_sensor_bus_debug_top #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD_RATE(BAUD_RATE),
        .POR_CYCLES(8), .I2C_CLK_DIV(2),
        .I2C_STARTUP_CYCLES(8), .I2C_RESET_DELAY_CYCLES(8),
        .SPI_CLK_DIV(1), .SPI_STARTUP_CYCLES(8),
        .SPI_RESET_DELAY_CYCLES(8), .SPI_RETRY_LIMIT(2)
    ) dut (
        .clk_i(clk_i), .reset_i(reset_i),
        .uart_rx_i(uart_rx_i), .uart_tx_o(uart_tx_o),
        .ina219_scl_io(ina219_scl_io), .ina219_sda_io(ina219_sda_io),
        .mpu6050_scl_io(mpu6050_scl_io), .mpu6050_sda_io(mpu6050_sda_io),
        .vl53l0x_scl_io(vl53l0x_scl_io), .vl53l0x_sda_io(vl53l0x_sda_io),
        .vl53l1x_scl_io(vl53l1x_scl_io), .vl53l1x_sda_io(vl53l1x_sda_io)
    );

    always #50 clk_i = ~clk_i;

    always @(posedge clk_i) begin
        if (ina219_scl_io === 1'b0) scl_low_seen[0] <= 1'b1;
        if (mpu6050_scl_io === 1'b0) scl_low_seen[1] <= 1'b1;
        if (vl53l0x_scl_io === 1'b0) scl_low_seen[2] <= 1'b1;
        if (vl53l1x_scl_io === 1'b0) scl_low_seen[3] <= 1'b1;
        if (ina219_sda_io === 1'b0) sda_low_seen[0] <= 1'b1;
        if (mpu6050_sda_io === 1'b0) sda_low_seen[1] <= 1'b1;
        if (vl53l0x_sda_io === 1'b0) sda_low_seen[2] <= 1'b1;
        if (vl53l1x_sda_io === 1'b0) sda_low_seen[3] <= 1'b1;
    end

    task send_uart_byte;
        input [7:0] value;
        integer bit_number;
        begin
            uart_rx_i = 1'b0;
            #(BIT_NS);
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
                uart_rx_i = value[bit_number];
                #(BIT_NS);
            end
            uart_rx_i = 1'b1;
            #(BIT_NS);
        end
    endtask

    task receive_uart_byte;
        output [7:0] value;
        integer bit_number;
        begin
            @(negedge uart_tx_o);
            #(BIT_NS + BIT_NS/2);
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1) begin
                value[bit_number] = uart_tx_o;
                #(BIT_NS);
            end
            if (!uart_tx_o)
                $fatal(1, "UART stop bit was low");
            #(BIT_NS/2);
        end
    endtask

    task expect_banner;
        begin
            for (index = 0; index < 31; index = index + 1) begin
                receive_uart_byte(received_byte);
                if (received_byte !== EXPECT_BANNER[(30-index)*8 +: 8])
                    $fatal(1, "banner mismatch at byte %0d", index);
            end
        end
    endtask

    task expect_pong;
        begin
            for (index = 0; index < 22; index = index + 1) begin
                receive_uart_byte(received_byte);
                if (received_byte !== EXPECT_PONG[(21-index)*8 +: 8])
                    $fatal(1, "P response mismatch at byte %0d", index);
            end
        end
    endtask

    task expect_bist;
        begin
            for (index = 0; index < 46; index = index + 1) begin
                receive_uart_byte(received_byte);
                if (received_byte !== EXPECT_BIST[(45-index)*8 +: 8])
                    $fatal(1, "B response mismatch at byte %0d", index);
            end
        end
    endtask

    task expect_detail;
        begin
            for (index = 0; index < 42; index = index + 1) begin
                receive_uart_byte(received_byte);
                if (received_byte !== EXPECT_DETAIL[(41-index)*8 +: 8])
                    $fatal(1, "D response mismatch at byte %0d", index);
            end
        end
    endtask

    task expect_help;
        begin
            for (index = 0; index < 12; index = index + 1) begin
                receive_uart_byte(received_byte);
                if (received_byte !== EXPECT_HELP[(11-index)*8 +: 8])
                    $fatal(1, "help mismatch at byte %0d", index);
            end
        end
    endtask

    initial begin
        clk_i = 1'b0;
        reset_i = 1'b0;
        uart_rx_i = 1'b1;
        scl_low_seen = 4'd0;
        sda_low_seen = 4'd0;

        expect_banner();
        wait (dut.i2c_error_bitmap == 4'hf &&
              dut.spi_transaction_bitmap == 8'hff &&
              dut.spi_reject_bitmap == 8'hff);

        if (scl_low_seen !== 4'hf || sda_low_seen !== 4'hf)
            $fatal(1, "not every I2C core generated START/SCL activity: scl=%h sda=%h",
                   scl_low_seen, sda_low_seen);
        if (dut.spi_sensor_ok !== 8'h00)
            $fatal(1, "an SPI core accepted the injected all-one response: %h",
                   dut.spi_sensor_ok);

        send_uart_byte("P");
        expect_pong();
        send_uart_byte("B");
        expect_bist();
        send_uart_byte("D");
        expect_detail();
        send_uart_byte("?");
        expect_help();

        $display("PASS: Tang Primer 25K sensor bus no-device diagnostic");
        $display("I2C_ERR=0x%h SPI_TXN=0x%h SPI_REJECT=0x%h",
                 dut.i2c_error_bitmap, dut.spi_transaction_bitmap,
                 dut.spi_reject_bitmap);
        $finish;
    end

    initial begin
        #5000000;
        $fatal(1, "simulation timeout");
    end
endmodule
