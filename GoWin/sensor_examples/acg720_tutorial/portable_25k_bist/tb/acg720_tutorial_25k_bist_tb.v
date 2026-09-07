`timescale 1ns/1ps

module acg720_tutorial_25k_bist_tb;
    reg clk_i = 1'b0;
    reg reset_i = 1'b1;
    reg uart_rx_i = 1'b1;
    wire uart_tx_o;
    integer i;
    reg [7:0] received;

    always #10 clk_i = ~clk_i;

    acg720_tutorial_25k_bist_top dut (
        .clk_i(clk_i),
        .reset_i(reset_i),
        .uart_rx_i(uart_rx_i),
        .uart_tx_o(uart_tx_o)
    );

    task receive_uart_byte;
        output [7:0] value;
        integer bit_index;
        begin
            @(negedge uart_tx_o);
            // 50 MHz / 115200 uses the tutorial divider value 433, i.e.
            // 434 system clocks per bit. Move to the center of data bit 0.
            repeat (651) @(posedge clk_i);
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
                value[bit_index] = uart_tx_o;
                repeat (434) @(posedge clk_i);
            end
            if (uart_tx_o !== 1'b1)
                $fatal(1, "UART stop bit error");
        end
    endtask

    function [7:0] expected_report_byte;
        input [5:0] index;
        begin
            case (index)
                 0:expected_report_byte="A";  1:expected_report_byte="C";  2:expected_report_byte="G";  3:expected_report_byte="7";
                 4:expected_report_byte="2";  5:expected_report_byte="0";  6:expected_report_byte="_";  7:expected_report_byte="T";
                 8:expected_report_byte="U";  9:expected_report_byte="T"; 10:expected_report_byte="O"; 11:expected_report_byte="R";
                12:expected_report_byte="I"; 13:expected_report_byte="A"; 14:expected_report_byte="L"; 15:expected_report_byte="_";
                16:expected_report_byte="2"; 17:expected_report_byte="5"; 18:expected_report_byte="K"; 19:expected_report_byte=" ";
                20:expected_report_byte="B"; 21:expected_report_byte="="; 22:expected_report_byte="1"; 23:expected_report_byte=" ";
                24:expected_report_byte="P"; 25:expected_report_byte="="; 26:expected_report_byte="1"; 27:expected_report_byte=" ";
                28:expected_report_byte="G"; 29:expected_report_byte="="; 30:expected_report_byte="1"; 31:expected_report_byte=" ";
                32:expected_report_byte="C"; 33:expected_report_byte="="; 34:expected_report_byte="1"; 35:expected_report_byte=" ";
                36:expected_report_byte="D"; 37:expected_report_byte="="; 38:expected_report_byte="1"; 39:expected_report_byte=" ";
                40:expected_report_byte="U"; 41:expected_report_byte="="; 42:expected_report_byte="1"; 43:expected_report_byte=" ";
                44:expected_report_byte="A"; 45:expected_report_byte="L"; 46:expected_report_byte="L"; 47:expected_report_byte="=";
                48:expected_report_byte="1"; 49:expected_report_byte=" "; 50:expected_report_byte="S"; 51:expected_report_byte="=";
                52:expected_report_byte="B"; 53:expected_report_byte="1"; 54:expected_report_byte="E"; 55:expected_report_byte="6";
                56:expected_report_byte=8'h0d; 57:expected_report_byte=8'h0a;
                default: expected_report_byte="?";
            endcase
        end
    endfunction

    initial begin
        repeat (20) @(posedge clk_i);
        reset_i <= 1'b0;

        receive_uart_byte(received);
        if (received !== 8'h55)
            $fatal(1, "UART challenge mismatch: %02x", received);

        for (i = 0; i <= 57; i = i + 1) begin
            receive_uart_byte(received);
            if (received !== expected_report_byte(i[5:0]))
                $fatal(1, "Report byte %0d mismatch: got=%02x expected=%02x", i, received, expected_report_byte(i[5:0]));
            $write("%c", received);
        end

        if (!dut.pass_all)
            $fatal(1, "Internal pass vector is not all ones");
        $display("ACG720_TUTORIAL_25K_SIM_PASS");
        $finish;
    end

    initial begin
        #20_000_000;
        $fatal(1, "Simulation timeout");
    end
endmodule
