`timescale 1ns/1ns
`define CLK_PERIOD 20

module ir_decode_tb;
	
	reg clk;
	reg reset_n;
	reg iIR;
	
	wire dec_done;
	wire [15:0]ir_data;
	wire [15:0]ir_addr;
	
	integer i = 0;

	ir_decode ir_decode(
		.clk(clk),
		.reset_n(reset_n),
		
		.iIR(iIR),
		
		.dec_done(dec_done),
		.ir_data(ir_data),
		.ir_addr(ir_addr)
	);
	
	initial clk = 1'b1;
	always#(`CLK_PERIOD/2)clk = ~clk;

	initial begin
		reset_n = 1'b0;
		iIR = 1'b1;
		#(`CLK_PERIOD*10+1'b1);
		reset_n = 1'b1;
		#(`CLK_PERIOD*200);
    
		iIR = 1'b1;
		send_data(1,8'h12);
		#60000000;

		send_data(3,8'heb);
		#60000000;
		$stop	;
	end
	
	task send_data;
		input [15:0]addr;
		input [7:0]data;
		begin
			iIR = 0;#9000000;
			iIR = 1;#4500000;
			for(i=0;i<=15;i=i+1)begin
				bit_send(addr[i]);		
			end
			for(i=0;i<=7;i=i+1)begin
				bit_send(data[i]);		
			end
			for(i=0;i<=7;i=i+1)begin
				bit_send(~data[i]);		
			end
			iIR = 0;#560000;
			iIR = 1;		
		end
	endtask
	
	task bit_send;
		input one_bit;
		begin
			iIR = 0; #560000;
			iIR = 1;
			if(one_bit)
				#1690000;
			else
				#560000;
		end	
	endtask

endmodule
