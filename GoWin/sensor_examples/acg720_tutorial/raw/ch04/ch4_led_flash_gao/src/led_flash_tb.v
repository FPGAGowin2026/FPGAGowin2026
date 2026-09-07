`timescale 1ns/1ns
module led_flash_tb();

    reg Clk;
    reg Rst_n;
    wire Led;

    led_flash led_flash(
        .clk(Clk),
        .rst_n(Rst_n),
        .led(Led)
    );

    initial Clk = 1;
    always#10 Clk = ~Clk;

    initial begin
        Rst_n = 0;
        #201;
        Rst_n = 1;
        #200;
        #100000000;
    end

endmodule

