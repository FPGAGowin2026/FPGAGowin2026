`timescale 1ns/1ps

module tb_tang_primer_25k_bringup;
    localparam integer CLK_FREQ_HZ = 10000000;
    localparam integer BAUD_RATE   = 1000000;
    localparam integer BIT_NS      = 1000;

    reg clk_i;
    reg reset_i;
    reg uart_rx_i;
    wire uart_tx_o;
    integer index;
    reg [7:0] received_byte;

    localparam [22*8-1:0] EXPECT_BANNER =
        {"GW5A25 BRINGUP READY", 8'h0d, 8'h0a};
    localparam [6*8-1:0]  EXPECT_PONG =
        {"PONG", 8'h0d, 8'h0a};
    localparam [19*8-1:0] EXPECT_ID =
        {"ID=GW5A-LV25MG121", 8'h0d, 8'h0a};
    localparam [31*8-1:0] EXPECT_STATUS =
        {"STATUS=OK;CLK=50M;BAUD=115200", 8'h0d, 8'h0a};
    localparam [14*8-1:0] EXPECT_HELP =
        {"CMDS:P,I,S,?", 8'h0d, 8'h0a};

    tang_primer_25k_bringup_top #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUD_RATE(BAUD_RATE),
        .HEARTBEAT_CYCLES(1000000),
        .POR_CYCLES(16)
    ) dut (
        .clk_i(clk_i),
        .reset_i(reset_i),
        .uart_rx_i(uart_rx_i),
        .uart_tx_o(uart_tx_o)
    );

    always #50 clk_i = ~clk_i;

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

    initial begin
        clk_i     = 1'b0;
        reset_i   = 1'b0;
        uart_rx_i = 1'b1;

        for (index = 0; index < 22; index = index + 1) begin
            receive_uart_byte(received_byte);
            if (received_byte !== EXPECT_BANNER[(21-index)*8 +: 8])
                $fatal(1, "banner mismatch at byte %0d", index);
        end

        send_uart_byte("P");
        for (index = 0; index < 6; index = index + 1) begin
            receive_uart_byte(received_byte);
            if (received_byte !== EXPECT_PONG[(5-index)*8 +: 8])
                $fatal(1, "PONG mismatch at byte %0d", index);
        end

        send_uart_byte("i");
        for (index = 0; index < 19; index = index + 1) begin
            receive_uart_byte(received_byte);
            if (received_byte !== EXPECT_ID[(18-index)*8 +: 8])
                $fatal(1, "ID mismatch at byte %0d", index);
        end

        send_uart_byte("s");
        for (index = 0; index < 31; index = index + 1) begin
            receive_uart_byte(received_byte);
            if (received_byte !== EXPECT_STATUS[(30-index)*8 +: 8])
                $fatal(1, "status mismatch at byte %0d", index);
        end

        send_uart_byte("?");
        for (index = 0; index < 14; index = index + 1) begin
            receive_uart_byte(received_byte);
            if (received_byte !== EXPECT_HELP[(13-index)*8 +: 8])
                $fatal(1, "help mismatch at byte %0d", index);
        end

        $display("PASS: Tang Primer 25K UART bring-up console");
        $finish;
    end

    initial begin
        #2000000;
        $fatal(1, "simulation timeout");
    end

endmodule
