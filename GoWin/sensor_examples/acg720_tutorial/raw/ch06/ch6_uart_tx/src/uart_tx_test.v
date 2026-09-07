module uart_tx_test(
    input clk,
    input reset_n,
    
    output uart_tx,
    output led
);
    parameter MCNT = 49_999_999;    //定时1S

    reg [7:0] data_byte;
    reg send_en;

    //设置定时器定时发送数据
    reg [25:0]cnt;  //定义计数器寄存器
    wire tx_done;

    //计数器计数进程   
    always@(posedge clk or negedge reset_n)
    if(!reset_n)
        cnt <= 25'd0;
    else if(cnt == MCNT)
        cnt <= 25'd0;
    else
        cnt <= cnt + 1'b1;
    
    always@(posedge clk or negedge reset_n)
    if(!reset_n)
        send_en <= 1'b0;    
    else if(cnt == MCNT)
        send_en <= 1'b1;
    else 
        send_en <= 1'b0;

    always@(posedge clk or negedge reset_n)
    if(!reset_n)
        data_byte <= 8'b0;
    else if(tx_done)
        data_byte <= data_byte + 1'b1;
    else
        data_byte <= data_byte;

	uart_byte_tx uart_byte_tx(
		.clk(clk),
		.reset_n(reset_n),

		.data_byte(data_byte),
		.send_en(send_en),
		.baud_set(3'd0),
		
		.uart_tx(uart_tx),
		.tx_done(tx_done),
		.uart_state(led)
	);

endmodule