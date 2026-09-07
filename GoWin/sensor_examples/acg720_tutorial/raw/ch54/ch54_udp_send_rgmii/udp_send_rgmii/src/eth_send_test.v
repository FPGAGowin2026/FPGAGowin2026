`timescale 1ns/1ns

module eth_send_test(
	rst_n,
	
	//GMII 接口信号	
	gmii_tx_clk,
	gmii_tx_en,
	gmii_tx_er,
	gmii_tx_data,
	
	phy_rst_n
);

	input rst_n;
	
	//MII 接口信号	
	input  gmii_tx_clk;	//GMII接口发送时钟，由PHY芯片产生，25MHz
	output gmii_tx_en;	//GMII接口发送数据使能信号，高电平有效
	output gmii_tx_er;	//发送错误，用以破坏数据包发送
	output [7:0]gmii_tx_data; //GMII接口发送数据线，FPGA通过该数据线将需要发送的数据依次送给PHY芯片
	output phy_rst_n;	// PHY 复位信号

	assign phy_rst_n = 1;
	
	wire fifo_rdreq;
	wire tx_go;
	
	reg [7:0] fifo_rddata;
	wire fifo_rdclk;
	reg [11:0]data_cnt;
	wire CRC_EN;
	
	wire [31:0]CRC_result;

	CRC32_D8 CRC32_D8(
		.Clk(gmii_tx_clk),
		.Reset(rst_n),
		.Data_in(gmii_tx_data), 
		.Enable(CRC_EN),
		.Initialize(~gmii_tx_en),
		.Crc(),
		.CrcNext(),
		.Crc_eth(CRC_result)
	);
	
	eth_send eth_send(
		.rst_n(rst_n),
		.tx_go(tx_go),
		.data_length(16'd52),	//需要发送的数据长度，以字节为单位
		.des_mac(48'hff_ff_ff_ff_ff_ff),
		.src_mac(48'h00_0a_35_01_fe_c0),
		.type_length(16'h08_00),
		.CRC_Result(CRC_result),
		.CRC_EN(CRC_EN),
		.fifo_rdreq(fifo_rdreq),
		.fifo_rddata(fifo_rddata),
		.fifo_rdclk(fifo_rdclk),
		.gmii_tx_clk(gmii_tx_clk),
		.gmii_tx_en(gmii_tx_en),
		.gmii_tx_er(gmii_tx_er),
		.gmii_tx_data(gmii_tx_data)
	);
	
	//计数发送数据个数，用于产生对应的数据
	always@(posedge gmii_tx_clk or negedge rst_n)
	if(!rst_n)
		data_cnt <= #1 12'd0;
	else if(fifo_rdreq)
		data_cnt <= #1 data_cnt + 1'b1;
	else
		data_cnt <= #1 12'd0;

	//UDP包	
	always@(*)
	begin
		case(data_cnt)
			0   : fifo_rddata = 8'h45;//协议版本,首部长度
			
			//服务类型
			1   : fifo_rddata = 8'h00;
			
			//IP数据报总长度（IP报头+数据）
			2   : fifo_rddata = 8'h00;
			3   : fifo_rddata = 8'h34;
			
			//数据包标识
			4   : fifo_rddata = 8'h00;
			5   : fifo_rddata = 8'h00;
			
			//标识+分段偏移
			6  : fifo_rddata = 8'h00;
			7  : fifo_rddata = 8'h00;
			
			//生存时间 64
			8  : fifo_rddata = 8'h40;
			
			//数据报类型 17： UDP
			9  : fifo_rddata = 8'h11;
			
			//IP报头校验和
			10  : fifo_rddata = 8'hf9;
			11  : fifo_rddata = 8'h63;
			
			//源地址192.168.0.2
			12  : fifo_rddata = 8'hc0;
			13  : fifo_rddata = 8'ha8;
			14  : fifo_rddata = 8'h00;
			15  : fifo_rddata = 8'h02;
			
			//目的地址192.168.0.3
			16  : fifo_rddata = 8'hc0;
			17  : fifo_rddata = 8'ha8;
			18  : fifo_rddata = 8'h00;
			19  : fifo_rddata = 8'h03;
			
			//源端口号5000(0x1388)
			20  : fifo_rddata = 8'h13;
			21  : fifo_rddata = 8'h88;
			
			//目的端口号6102(0x17d6)
			22  : fifo_rddata = 8'h17;
			23  : fifo_rddata = 8'hd6;
			
			//UDP数据报总长度（UDP报头+数据）
			24  : fifo_rddata = 8'h00;
			25  : fifo_rddata = 8'h20;
			
			//UDP报头校验和
			26  : fifo_rddata = 8'h91;
			27  : fifo_rddata = 8'h49;
			
//			//UDP报头校验和  忽略
//			26  : fifo_rddata = 8'h00;
//			27  : fifo_rddata = 8'h00;
		
//			//UDP报头校验和  错误校验和
//			26  : fifo_rddata = 8'h12;
//			27  : fifo_rddata = 8'h34;
			
			//用户数据：Hello, welcom to FPGA!
			28  : fifo_rddata = 8'h48;//H			
			29  : fifo_rddata = 8'h65;//e			
			30  : fifo_rddata = 8'h6c;//l			
			31  : fifo_rddata = 8'h6c;//l			
			32  : fifo_rddata = 8'h6f;//o
			33  : fifo_rddata = 8'h2c;//,
            34  : fifo_rddata = 8'h20;//
            35  : fifo_rddata = 8'h77;//w
            36  : fifo_rddata = 8'h65;//e
            37  : fifo_rddata = 8'h6c;//l
            38  : fifo_rddata = 8'h63;//c
            39  : fifo_rddata = 8'h6f;//o
            40  : fifo_rddata = 8'h6d;//m
            41  : fifo_rddata = 8'h20;// 
            42  : fifo_rddata = 8'h74;//t
            43  : fifo_rddata = 8'h6f;//o
            44  : fifo_rddata = 8'h20;// 
            45  : fifo_rddata = 8'h46;//F
            46  : fifo_rddata = 8'h50;//P
            47  : fifo_rddata = 8'h47;//G
            48  : fifo_rddata = 8'h41;//A
            49  : fifo_rddata = 8'h21;//!
            50  : fifo_rddata = 8'h21;//!
            51  : fifo_rddata = 8'h21;//!
            
			default:fifo_rddata = 8'h0;
		endcase
	end	
	
	//发送间隔计数器
	reg [23:0]cnt;
	
	always@(posedge gmii_tx_clk or negedge rst_n)
	if(!rst_n)
		cnt <= #1 0;
		else //计数器自增，不考虑溢出，接受溢出自动清零
		cnt <= #1 cnt + 1'b1;
		
	//24位cnt计满一次启动一次发送，该时间大约为134ms
	assign tx_go = (cnt == 24'd1);

endmodule

