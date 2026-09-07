`timescale 1ns/1ns

module Camera_ETH_Formator_tb;

	reg Rst_n;
	reg PCLK;
	reg HREF;
	reg VSYNC;
	reg [7:0]DATA;

	wire [7:0]wrdata;
	wire wrreq;

	initial PCLK = 1;
	always#10 PCLK = ~PCLK;
	
	Camera_ETH_Formator Camera_ETH_Formator(
	.Rst_n(Rst_n),
	.PCLK(PCLK),
	.HREF(HREF),
	.VSYNC(VSYNC),
	.DATA(DATA),

	.wrdata(wrdata),
	.wrreq(wrreq)
);
	
	initial begin
		Rst_n = 0;
		HREF = 0;
		VSYNC = 0;
		#201;
		Rst_n = 1;
		repeat (10)begin 
			HREF = 1;
			#201;
			HREF = 0;
			#201;
		end 
		#201;
		VSYNC = 1;
		#201;
		VSYNC = 0;
		repeat (10)begin 
			HREF = 1;
			#201;
			HREF = 0;
			#201;
		end 
		$stop;
	end
	
	always@(posedge PCLK)
	if(!Rst_n)
		DATA <= #1 0;
	else if(HREF)
		DATA <= #1 DATA + 1'b1;
	else
		DATA <= #1 DATA;

		
endmodule
