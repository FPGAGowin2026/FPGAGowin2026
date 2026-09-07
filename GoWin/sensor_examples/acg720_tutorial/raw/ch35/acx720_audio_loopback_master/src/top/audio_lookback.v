module audio_lookback(
		input clk,                    
		input reset_n,                                   
		inout iic_0_scl,              
		inout iic_0_sda,   
	    output led,
		
		input I2S_DI,
		input I2S_RCLK,
		input I2S_BCLK,
		output I2S_DO,
		output I2S_MCLK,
		
		output [6:0] gpio
);
	
	parameter DATA_WIDTH        = 32;     
	
//    Gowin_PLL Gowin_PLL(
//        .clkout0(I2S_MCLK), //output clkout0
//        .clkin(clk) //input clkin
//    );

	assign gpio[0] = iic_0_scl;
	assign gpio[1] = iic_0_sda;
	assign gpio[2] = I2S_DI;   //103
	assign gpio[3] = I2S_RCLK;	//102
	assign gpio[4] = I2S_BCLK;  //98
	assign gpio[5] = I2S_DO;	//101
	assign gpio[6] = I2S_MCLK;	//97
	
    wire clk_100M;

/*	
    rpll100 rpll100(
        .clkout(clk_100M), //output clkout
        .clkin(clk) //input clkin
    );

    Gowin_rPLL your_instance_name(
        .clkout(I2S_MCLK), //output clkout
        .clkin(clk_100M) //input clkin
    );*/
	
//	PLL_FREQ PLL_FREQ
//	(
//		.CLKI(clk),
//		.CLKOP(I2S_MCLK)
//	);

    Gowin_PLL Gowin_PLL(
        .clkout0(I2S_MCLK), //output clkout0
        .clkin(clk) //input clkin
    );


 	wire Init_Done;
	WM8960_Init WM8960_Init(
		.Clk(clk),
		.Rst_n(reset_n),
		.I2C_Init_Done(Init_Done),
		.i2c_sclk(iic_0_scl),
		.i2c_sdat(iic_0_sda)
	);
	
	assign led = ~Init_Done;
//	assign I2S_DO = I2S_DI;
	
//	
	reg adcfifo_read;
	wire [DATA_WIDTH - 1:0] adcfifo_readdata;
	wire adcfifo_empty;

	reg dacfifo_write;
	reg [DATA_WIDTH - 1:0] dacfifo_writedata;
	wire dacfifo_full;
	

	always @ (posedge clk or negedge reset_n)
	begin
		if (~reset_n)
		begin
			adcfifo_read <= 1'b0;
		end
		else if (~adcfifo_empty)
		begin
			adcfifo_read <= 1'b1;
		end
		else
		begin
			adcfifo_read <= 1'b0;
		end
	end

	always @ (posedge clk or negedge reset_n)
	begin
		if(~reset_n)
			dacfifo_write <= 1'd0;
		else if(~dacfifo_full && (~adcfifo_empty)) begin
			dacfifo_write <= 1'd1;
			dacfifo_writedata <= adcfifo_readdata;
		end
		else begin
			dacfifo_write <= 1'd0;
		end
	end

	i2s_rx 
	#(
		.DATA_WIDTH(DATA_WIDTH) 
	)i2s_rx
	(
		.reset_n(reset_n),
		.bclk(I2S_BCLK),
		.adclrc(I2S_RCLK),
		.adcdat(I2S_DI),
		.adcfifo_rdclk(clk),
		.adcfifo_read(adcfifo_read),
		.adcfifo_empty(adcfifo_empty),
		.adcfifo_readdata(adcfifo_readdata)
	);
	
	i2s_tx
	#(
		 .DATA_WIDTH(DATA_WIDTH)
	)i2s_tx
	(
		 .reset_n(reset_n),
		 .dacfifo_wrclk(clk),
		 .dacfifo_wren(dacfifo_write),
		 .dacfifo_wrdata(dacfifo_writedata),
		 .dacfifo_full(dacfifo_full),
		 .bclk(I2S_BCLK),
		 .daclrc(I2S_RCLK),
		 .dacdat(I2S_DO)
	);

		 
endmodule
