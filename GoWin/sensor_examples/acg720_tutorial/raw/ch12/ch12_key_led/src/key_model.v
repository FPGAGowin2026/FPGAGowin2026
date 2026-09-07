`timescale 1ns/1ns

module key_model(
	key_press,
	key_out
);
	input key_press;
	output key_out;
	
	reg key_out;
	reg [15:0]myrand;
	
	initial begin
	key_out = 1'b1;
    while(1)
		begin
		  @(posedge key_press);
		  key_gen;
		end
	end
	
	task key_gen;
	begin
    key_out = 1'b1;
    repeat(50)begin
      myrand = {$random}%65536;//0~65535;
      #myrand key_out = ~key_out;			
    end
    key_out = 0;
    #25000000;
    
    repeat(50)begin
      myrand = {$random}%65536;//0~65535;
      #myrand key_out = ~key_out;			
    end
    key_out = 1;
    #25000000;		
	end	
	endtask

endmodule