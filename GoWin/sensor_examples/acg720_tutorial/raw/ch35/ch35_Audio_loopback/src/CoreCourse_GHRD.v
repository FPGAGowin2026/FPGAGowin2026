module CoreCourse_GHRD(
		input clk,                    
		input reset_n,                                   
		inout iic_0_scl,              
		inout iic_0_sda,   
	    output led,
		
		input I2S_DI,
		input I2S_RCLK,
		input I2S_BCLK,
		output I2S_DO,
		output I2S_MCLK
//        output led1
);
	
//	assign I2S_DO = I2S_DI;
//	assign led1 = locked;
	parameter DATA_WIDTH        = 32;        //42:32; 46:40; 4A:48 4E:64 
    wire locked;
    wire rst_n_sync;
    
    reg reset_n_d1;
    reg reset_n_d2;
    
    always@ (posedge clk)begin
        reset_n_d1 <= reset_n;
        reset_n_d2 <= reset_n_d1;
    end
    
    assign rst_n_sync = locked & reset_n_d2;
    


    Gowin_PLL Gowin_PLL(
        .lock(locked), //output lock
        .clkout0(I2S_MCLK), //output clkout0
        .clkin(clk), //input clkin
        .reset(~reset_n) //input reset
    );
	
 	wire Init_Done;
	ES8388_Init ES8388_Init(
		.Clk(clk),
		.Rst_n(reset_n),
		.I2C_Init_Done(Init_Done),
		.i2c_sclk(iic_0_scl),
		.i2c_sdat(iic_0_sda)
	);

	
	assign led = Init_Done;
	
	reg adcfifo_read;
	wire [DATA_WIDTH - 1:0] adcfifo_readdata;
	wire adcfifo_empty;
	wire fifo_clear;
	
	reg dacfifo_write;

	
	reg [DATA_WIDTH - 1:0] dacfifo_writedata;
	wire dacfifo_full;
	

	always @ (posedge clk or negedge rst_n_sync)
	begin
		if (~rst_n_sync)
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


	always @ (posedge clk or negedge rst_n_sync)
	begin
		if(~rst_n_sync)
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
		.reset_n(rst_n_sync),
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
		 .reset_n(rst_n_sync),
		 .dacfifo_wrclk(clk),
		 .dacfifo_wren(dacfifo_write),
		 .dacfifo_wrdata(dacfifo_writedata),
		 .dacfifo_full(dacfifo_full),
		 .bclk(I2S_BCLK),
		 .daclrc(I2S_RCLK),
		 .dacdat(I2S_DO)
	);

		 
endmodule
