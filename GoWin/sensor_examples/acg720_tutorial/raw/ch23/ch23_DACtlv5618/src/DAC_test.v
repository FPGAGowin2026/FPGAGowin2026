

module DAC_test(
			Clk,   //模块时钟50M
			Rst_n, //模块复位
			
			DAC_CS_N,  //TLV5618的CS_N接口
			DAC_DIN,   //TLV5618的DIN接口
			DAC_SCLK   //TLV5618的SCLK接口
		);
	input Clk;
	input Rst_n;
	
	output DAC_CS_N;
	output DAC_DIN;
	output DAC_SCLK;
	reg Start;
	reg [15:0]r_DAC_DATA;
	wire [15:0]DAC_DATA;
	wire Set_Done;
	


	tlv5618 tlv5618(
		.Clk(Clk),
		.Rst_n(Rst_n),
		
		.DAC_DATA(r_DAC_DATA),
		.Start(Start),
		.Set_Done(Set_Done),
		
		.DAC_CS_N(DAC_CS_N),
		.DAC_DIN(DAC_DIN),
		.DAC_SCLK(DAC_SCLK)
	);
    
        reg [15:0] cnt;
always @ (posedge Clk or negedge Rst_n) begin
    if (~Rst_n)
        cnt <= 'd0;
    else if ((cnt=='d0)&&(r_DAC_DATA != DAC_DATA)) 
        cnt <= 'd1;
    else if (cnt>'d0)
        cnt <= cnt + 'd1;
end
	always@(posedge Clk)
		r_DAC_DATA <= DAC_DATA;
	always@(posedge Clk or negedge Rst_n)
	if(!Rst_n)
		Start <= 1'd0;
	else if(&cnt) 
		Start <= 1'b1;
	else
		Start <= 1'd0;
//	always@(posedge Clk)
//		r_DAC_DATA <= DAC_DATA;
//	always@(posedge Clk or negedge Rst_n)
//	if(!Rst_n)
//		Start <= 1'd0;
//	else if(r_DAC_DATA != DAC_DATA) 
//		Start <= 1'b1;
//	else
//		Start <= 1'd0;

endmodule
