`timescale 1ns/1ns
module uart_eeprom_tb();

    wire i2c_sclk;
    wire i2c_sdat;
    reg clk50M;
    reg rst_n;
    wire uart_tx;
    wire uart_rx;
    wire Tx_Done;    
    wire uart_tx_t;
    assign uart_rx = uart_tx_t;
    
    reg send_en;
    
    pullup PUP(i2c_sdat);
    
    uart_eeprom uart_eeprom(
        .clk50M(clk50M),
        .rst_n(rst_n),
        .i2c_sclk(i2c_sclk),
        .i2c_sdat(i2c_sdat),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx)
    );
    M24LC64 M24LC64(
		.A0(0), 
		.A1(0), 
		.A2(0), 
		.WP(0), 
		.SDA(i2c_sdat), 
		.SCL(i2c_sclk), 
		.RESET(~rst_n)
	);
	
	initial clk50M = 1;
	always #10 clk50M = ~clk50M;
	
	reg [175:0] test_data;
	
	
	initial begin
	   rst_n = 0;
	   #201;
	   rst_n = 1;
	   #200;
	   test_data = 176'h55_A5_00_03_04_12_34_56_78_9A_BC_DE_F0_23_45_67_89_AB_CD_EF_34_F0;
	   
	   send_en = 1;
	   #20;
	   send_en = 0;
	   #200;
	   @(posedge uart_eeprom.eeprom_wr_done);
	   
	   #201;
	   test_data = 176'h55_A5_00_04_83_12_34_56_78_9A_BC_DE_F0_23_45_67_89_AB_CD_EF_34_F0;
	   send_en = 1;
	   #20;
	   send_en = 0;
	   #200;
	   @(posedge uart_eeprom.eeprom_rd_done);
	   #2000;
	   @(posedge uart_eeprom.uart_data_tx.Tx_Done);
	   #100000;
	   $stop;
	end
	
    uart_data_tx
    #(
        .DATA_WIDTH(176),
        .MSB_FIRST(1)
    )
    uart_data_tx(
        .Clk(clk50M),
        .Rst_n(rst_n),
        .data(test_data),
        .send_en(send_en),   
        .Baud_Set(3'd4),  
        .uart_tx(uart_tx_t),  
        .Tx_Done(Tx_Done),   
        .uart_state()
    );

endmodule
