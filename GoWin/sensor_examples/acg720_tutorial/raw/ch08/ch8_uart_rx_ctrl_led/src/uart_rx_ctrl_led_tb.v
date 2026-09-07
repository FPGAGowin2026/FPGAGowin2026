`timescale 1ns / 1ps
module uart_rx_ctrl_led_tb();
 
    reg Clk;
    reg Reset_n;
    wire Led;
    reg uart_rx;
    wire [31:0] delay_time;
    
    uart_rx_ctrl_led uart_rx_ctrl_led(
        .Clk(Clk),
        .Reset_n(Reset_n),
        .Led(Led),
        .uart_rx(uart_rx)
    );
    parameter Baud_Set = 3'd4;
    
    assign delay_time = (Baud_Set == 3'd0) ? 20'd104166:
                         (Baud_Set == 3'd1) ? 20'd52083:
                         (Baud_Set == 3'd2) ? 20'd26041:
                         (Baud_Set == 3'd3) ? 20'd17361:
                                              20'd8680;
    
    
    initial Clk = 1;
    always#10 Clk = ~Clk;
    
    initial begin
       Reset_n = 0;
       uart_rx = 1;
       #201;
       Reset_n = 1;
       #200; 
       
       uart_tx_byte(8'h55);
       #(delay_time*10);
       uart_tx_byte(8'ha5);
       #(delay_time*10);
       uart_tx_byte(8'h55);
       #(delay_time*10);
       uart_tx_byte(8'ha5);
       #(delay_time*10);
       uart_tx_byte(8'h00);
       #(delay_time*10);
       uart_tx_byte(8'h00);
       #(delay_time*10);
       uart_tx_byte(8'hc3);
       #(delay_time*10);
       uart_tx_byte(8'h50);
       #(delay_time*10);  
       uart_tx_byte(8'haa);
       #(delay_time*10);       
       uart_tx_byte(8'hf0);
       #(delay_time*10);    
       
       
       uart_tx_byte(8'h55);
       #(delay_time*10);
       uart_tx_byte(8'ha5);
       #(delay_time*10);
       uart_tx_byte(8'h9a);
       #(delay_time*10);
       uart_tx_byte(8'h78);
       #(delay_time*10);
       uart_tx_byte(8'h56);
       #(delay_time*10);
       uart_tx_byte(8'h34);
       #(delay_time*10);  
       uart_tx_byte(8'h12);
       #(delay_time*10);       
       uart_tx_byte(8'hf1);
       #(delay_time*10);
       #15000000;       
       $stop;
    end
    
    task uart_tx_byte;
        input [7:0]tx_data;
        begin
            uart_rx = 1;
            #20;
            uart_rx = 0;
            #delay_time;
            uart_rx = tx_data[0];
            #delay_time;
            uart_rx = tx_data[1];
            #delay_time;
            uart_rx = tx_data[2];
            #delay_time;
            uart_rx = tx_data[3];
            #delay_time;
            uart_rx = tx_data[4];
            #delay_time;
            uart_rx = tx_data[5];
            #delay_time;
            uart_rx = tx_data[6];
            #delay_time;
            uart_rx = tx_data[7];
            #delay_time;
            uart_rx = 1;
            #delay_time;         
        end
    endtask

    
endmodule
