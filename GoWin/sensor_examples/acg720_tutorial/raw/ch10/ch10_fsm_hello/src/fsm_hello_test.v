//////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: 小梅哥团队
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: fsm_hello
// Module Name: fsm_hello_test
// Project Name: fsm_hello
// Description: 实现字符串“hello”检测，每检测成功一次LED状态便翻转1次
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module fsm_hello_test(
	clk,
	reset_n,

	uart_rx,
	led
);
	wire reset=~reset_n;
	input clk;   //系统时钟输入＿50M
	input reset_n;   //复位信号输入，低有效
  
	input uart_rx;   //串口输入信号

	output led;  //LED指示灿
	reg led;
	
	wire [7:0]data;        //串口接收数据,作为字符数据输入	
	wire data_valid;  //1byte数据接收完成标志

	wire check_ok;    //字符串检测成功标诿

  //接收串口数据作为字符串检测的输入数据
	uart_byte_rx uart_byte_rx(
		.clk(clk),
		.reset_n(reset_n),

		.baud_set(3'd0),
		.uart_rx(uart_rx),
		
		.data_byte(data),
		.rx_done(data_valid)
	);
  
	fsm_hello fsm_hello(
		.clk(clk),
		.reset_n(reset_n),

		.data_valid(data_valid),
		.data_in(data),

		.check_ok(check_ok)
	);
	
	always@(posedge clk or posedge reset)
	if(reset)
		led <= 1'b0;
	else if(check_ok)
		led <= ~led;
	else
		led <= led; 

endmodule
