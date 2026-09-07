`timescale 1ns/1ns

module eth_send_tb;

	reg rst_n;
	reg tx_go;
	reg [15:0]data_length;
	reg [47:0]des_mac;
	reg [47:0]src_mac;
	reg [15:0]type_length;
	reg [31:0]CRC_Result;
	
	wire fifo_rdreq;
	reg [7:0]fifo_rddata;
	wire fifo_rdclk;
	
	//MII 接口信号	
	reg gmii_tx_clk;
	wire gmii_tx_en;
	wire gmii_tx_er;
	wire [7:0]gmii_tx_data;

	eth_send eth_send(
		.rst_n(rst_n),
		.tx_go(tx_go),
		.data_length(data_length),
		.des_mac(des_mac),
		.src_mac(src_mac),
		.type_length(type_length),
		.CRC_Result(CRC_Result),
		.fifo_rdreq(fifo_rdreq),
		.fifo_rddata(fifo_rddata),
		.fifo_rdclk(fifo_rdclk),
		.gmii_tx_clk(gmii_tx_clk),
		.gmii_tx_en(gmii_tx_en),
		.gmii_tx_er(gmii_tx_er),
		.gmii_tx_data(gmii_tx_data)
	);
	
	initial gmii_tx_clk = 1;
	always #20 gmii_tx_clk = ~gmii_tx_clk;
	
	initial begin
		rst_n = 0;
		tx_go = 0;
		data_length = 100;
		des_mac = 48'h84_7b_eb_48_94_13;
		src_mac = 48'h00_0a_35_01_fe_c0;
		type_length = 16'h08_00;
		CRC_Result = 32'h12_34_56_78;
		#201;
		rst_n = 1;
		#200;
		tx_go = 1;	//启动一次发送
		#40;
		tx_go = 0;
		#40000;
		
		data_length = 10;
		des_mac = 48'h84_7b_eb_48_94_13;
		src_mac = 48'h00_0a_35_01_fe_c0;
		type_length = 16'h08_00;
		CRC_Result = 32'h12_34_56_78;
		#200;
		tx_go = 1;	//启动一次发送
		#40;
		tx_go = 0;
		#40000;
		
		$stop;
	end
	
	//产生测试数据
	initial begin
		fifo_rddata = 0;
		wait(gmii_tx_en);
		forever begin
			fifo_rddata <= fifo_rddata + 1;
			#40;		
		end
	end

endmodule
