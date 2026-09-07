
`timescale 1ns/1ns
`define CLK_PERIOD 20

module key_filter_tb;

	reg  clk;
	reg  reset_n;
	reg  key_press;
	wire key;
	
	wire key_flag;
	wire key_state;	

	key_model key_model_inst(
		.key_press(key_press),
		.key_out(key)
	);

	key_filter key_filter_inst(
		.clk(clk),
		.reset_n(reset_n),

		.key_in(key),
		.key_flag(key_flag),
		.key_state(key_state)
	);

	initial clk= 1;
	always#(`CLK_PERIOD/2) clk = ~clk;
	
	initial begin
		reset_n = 1'b0;
		key_press = 1'b0;
		#(`CLK_PERIOD*10);
		reset_n = 1'b1;
		#(`CLK_PERIOD*10 + 1);
    
		key_press = 1'b1;
		#(`CLK_PERIOD);
		key_press = 1'b0;
		#60000000;

		key_press = 1'b1;
		#(`CLK_PERIOD);
		key_press = 1'b0;
		#60000000;
    
		$stop;
	end

endmodule