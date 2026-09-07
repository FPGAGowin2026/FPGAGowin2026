`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/10/23 23:32:08
// Design Name: 
// Module Name: i2c_control_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module i2c_control_tb;
    
    reg Clk;
	reg Rst_n;
	reg wrreg_req;
	reg rdreg_req;
	reg [15:0]addr;
	reg addr_mode;
	reg [7:0]wrdata;
	wire [7:0]rddata;
	reg [7:0]device_id;
	wire RW_Done;
	wire ack;
	wire i2c_sclk;
	wire i2c_sdat;
	
	pullup PUP(i2c_sdat);
    
    i2c_control DUT(
        .Clk        (Clk      ),
        .Rst_n      (Rst_n    ),
        .wrreg_req  (wrreg_req),
        .rdreg_req  (rdreg_req),
        .addr       (addr     ),
        .addr_mode  (addr_mode),
        .wrdata     (wrdata   ),
        .rddata     (rddata   ),
        .device_id  (device_id),
        .RW_Done    (RW_Done  ),
        .ack        (ack      ),
        .i2c_sclk   (i2c_sclk ),
        .i2c_sdat   (i2c_sdat )
    );
    
    M24LC04B M24LC04B(
		.A0(0), 
		.A1(0), 
		.A2(0), 
		.WP(0), 
		.SDA(i2c_sdat), 
		.SCL(i2c_sclk), 
		.RESET(~Rst_n)
	);
	
	initial Clk = 1;
	always #10 Clk = ~Clk;
	
	initial begin
		Rst_n = 0;
		rdreg_req = 0;
		wrreg_req = 0;
		#2001;
		Rst_n = 1;
		#2000;
		
		write_one_byte(8'hA0,8'h0A,8'hd1);
		write_one_byte(8'hA0,8'h0B,8'hd2);
		write_one_byte(8'hA0,8'h0C,8'hd3);
		write_one_byte(8'hA0,8'h0F,8'hd4);
		
		read_one_byte(8'hA0,8'h0A);
		read_one_byte(8'hA0,8'h0B);
		read_one_byte(8'hA0,8'h0C);
		read_one_byte(8'hA0,8'h0F);
		$stop;	
	end
	
	task write_one_byte;
		input [7:0]id;
		input [7:0]mem_address; 
		input [7:0]data;
		begin
			addr = {8'd0,mem_address};
			device_id = id;
			addr_mode = 0;
			wrdata = data;
			wrreg_req = 1;
			#20;
			wrreg_req = 0;
			@(posedge RW_Done);
			#20000;
		end
	endtask
	
	task read_one_byte;
		input [7:0]id;
		input [7:0]mem_address; 
		begin
			addr = {8'd0,mem_address};
			device_id = id;
			addr_mode = 0;
			rdreg_req = 1;
			#20;
			rdreg_req = 0;
			@(posedge RW_Done);
			#20000;
		end
	endtask
	
endmodule
