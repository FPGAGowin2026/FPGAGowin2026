module uart_rx_test(
    input  clk,
    input  reset_n,
    input  uart_rx,
    output uart_tx
);
    reg [7:0] data_byte_tx;
    wire [7:0] data_byte_rx;
    reg send_en;
    wire tx_done;
    wire rx_done;
    always@(posedge clk or negedge reset_n)
    if(!reset_n) begin
        data_byte_tx <= 8'd0;
        send_en <= 1'd0;
    end
    else if(rx_done) begin
        data_byte_tx <= data_byte_rx;
        send_en <= rx_done;
    end
    else begin
        data_byte_tx <= data_byte_tx;
        send_en <= 1'd0;
    end
    uart_byte_tx uart_byte_tx(
        .clk(clk),
        .reset_n(reset_n),

        .data_byte(data_byte_tx),
        .send_en(send_en),
        .baud_set(3'd0),
        
        .uart_tx(uart_tx),
        .tx_done(tx_done),
        .uart_state( )
    );
    uart_byte_rx uart_byte_rx(
        .clk(clk),
        .reset_n(reset_n),

        .baud_set(3'd0),
        .uart_rx(uart_rx),

        .data_byte(data_byte_rx),
        .rx_done(rx_done)
    );
endmodule
