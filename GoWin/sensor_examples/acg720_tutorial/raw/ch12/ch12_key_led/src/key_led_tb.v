`timescale 1ns/1ns
`define CLK_PERIOD 20

module key_led_tb;

	reg clk;
	reg reset_n;

	reg press0;
	reg press1;	
	reg press2;
	reg press3;
	wire key_in0;
	wire key_in1;
	wire key_in2;
	wire key_in3;
	
	wire [7:0]led;

	key_model key_model_inst0(
		.key_press(press0),
		.key_out(key_in0)
	);

	key_model key_model_inst1(
		.key_press(press1),
		.key_out(key_in1)
	);
  
	key_model key_model_inst2(
		.key_press(press2),
		.key_out(key_in2)
	);

	key_model key_model_inst3(
		.key_press(press3),
		.key_out(key_in3)
	);

	key_led key_led0(
		.clk(clk),
		.reset_n(reset_n),
		.key_in0(key_in0),
		.key_in1(key_in1),
		.key_in2(key_in2),
		.key_in3(key_in3),
		.led(led)
	);

	initial clk= 1;
	always#(`CLK_PERIOD/2) clk = ~clk;

	initial begin
		reset_n = 1'b0;
		press0 = 0;
		press1 = 0;
		press2 = 0;
		press3 = 0;
		#(`CLK_PERIOD*10);
		reset_n = 1'b1;
		#(`CLK_PERIOD*10 + 1);	

		press0 = 1;
		#(`CLK_PERIOD)
		press0 = 0;

		#80_000_000;

		press0 = 1;
		#(`CLK_PERIOD)
		press0 = 0;

		#80_000_000;

		press2 = 1;
		#(`CLK_PERIOD)
		press2 = 0;

		#80_000_000;

		press2 = 1;
		#(`CLK_PERIOD)
		press2 = 0;
    
		#80_000_000;

		press3 = 1;
		#(`CLK_PERIOD)
		press3 = 0;

		#80_000_000;

		press3 = 1;
		#(`CLK_PERIOD)
		press3 = 0;

		#80_000_000;
    
		press1 = 1;
		#(`CLK_PERIOD)
		press1 = 0;

		#80_000_000;

		press1 = 1;
		#(`CLK_PERIOD)
		press1 = 0;

		#80_000_000;
		$stop;		
	end

endmodule