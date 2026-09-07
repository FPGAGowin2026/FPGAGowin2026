module ad7606_driver(
	Clk,
	Reset_n,
	Go,
	Speed_Set,
	Conv_Done,
	ad7606_cs_n_o,   
	ad7606_rd_n_o,   
	ad7606_busy_i,   
	ad7606_db_i,     
	ad7606_os_o,     
	ad7606_reset_o,  
	ad7606_convst_o, 
	
	data_flag,
	data_mult_ch,
	data1,
	data2, 
	data3, 
	data4, 
	data5, 
	data6, 
	data7, 
	data8
);


	input  wire 		 Clk;
	input  wire 		 Reset_n;
	input  wire 		 Go;
	input [24:0]		 Speed_Set;
	
	output reg  		 Conv_Done;
	output wire        ad7606_cs_n_o;         
	output reg         ad7606_rd_n_o;         
	input  wire        ad7606_busy_i;         
	input  wire [15:0] ad7606_db_i;           
	output wire [2:0]  ad7606_os_o;           
	output reg         ad7606_reset_o;        
	
	output reg         ad7606_convst_o;       
	output reg [7:0]data_flag;
	output reg [15:0]data_mult_ch;
	output reg [15:0]data1,data2, data3, data4, data5, data6, data7, data8;

	assign ad7606_os_o = 0;	//不使用过采样	
	assign ad7606_cs_n_o = ad7606_rd_n_o;
	reg [6:0]state;
	
	reg [1:0]ad7606_busy_r;
	always@(posedge Clk)
		ad7606_busy_r <= {ad7606_busy_r[0],ad7606_busy_i};
	
	always@(posedge Clk or negedge Reset_n)
	if(!Reset_n)
		data_flag <= 0;
	else begin
		data_flag[0] <= state == 15;
		data_flag[1] <= state == 20;
		data_flag[2] <= state == 25;
		data_flag[3] <= state == 30;
		data_flag[4] <= state == 35;
		data_flag[5] <= state == 40;
		data_flag[6] <= state == 45;
		data_flag[7] <= state == 50;	
	end
	
	always@(posedge Clk or negedge Reset_n)
	if(!Reset_n)
		data_mult_ch <= 0;
	else begin
		data_mult_ch <= 
			 (  (state == 15)
			 || (state == 20)
			 || (state == 25)
			 || (state == 30)
			 || (state == 35)
			 || (state == 40)
			 || (state == 45)
			 || (state == 50)
			)? ad7606_db_i:data_mult_ch;	
	end

	reg [24:0]cnt;
	always@(posedge Clk or negedge Reset_n)
	if(!Reset_n)
		cnt <= 0;
	else if(cnt >= Speed_Set)
		cnt <= 0;
	else
		cnt <= cnt + 1'b1;
		
	wire trig = cnt == Speed_Set;
	
	always@(posedge Clk or negedge Reset_n)
	if(!Reset_n)begin
		state <= 0;
		ad7606_convst_o <= 1;
		Conv_Done <= 0;
		data1 <= 0;
		data2 <= 0;
		data3 <= 0;
		data4 <= 0;
		data5 <= 0;
		data6 <= 0;
		data7 <= 0;
		data8 <= 0;
		ad7606_rd_n_o <= 1;
		ad7606_reset_o <= 0;
	end
	else begin
		case(state)
			0:
				if(Go && trig)begin
					state <= 5;
					ad7606_convst_o <= 0;
					ad7606_rd_n_o <= 1;
					Conv_Done <= 0;
					ad7606_reset_o <= 0;
				end
				else begin
					state <= 0;
					ad7606_convst_o <= 1;
					ad7606_rd_n_o <= 1;
					ad7606_reset_o <= 0;
				end
					
			1: state <= state + 1'b1;
			2: state <= state + 1'b1;
			3: state <= state + 1'b1;
			4: state <= state + 1'b1;
			5: state <= state + 1'b1;
			6: begin state <= state + 1'b1;ad7606_convst_o <= 1;end
			7: state <= state + 1'b1;
			8: state <= state + 1'b1;
			9: state <= state + 1'b1;
			10: if(ad7606_busy_r[1])state <= state;else begin state <= state + 3'd4;ad7606_rd_n_o <= 0;end
			11: state <= state + 1'b1;
			12: state <= state + 1'b1;
			13: begin ad7606_rd_n_o <= 0; state <= state + 1'b1;end
			14: begin state <= state + 1'b1;end
			15: begin ad7606_rd_n_o <= 1; data1 <=  ad7606_db_i;state <= state + 1'b1;end
			16: state <= state + 1'b1;
			17: state <= state + 1'b1;
			18: begin ad7606_rd_n_o <= 0; state <= state + 1'b1;end
			19: begin state <= state + 1'b1;end
			20: begin ad7606_rd_n_o <= 1; data2 <= ad7606_db_i; state <= state + 1'b1;end
			21: state <= state + 1'b1;
			22: state <= state + 1'b1;
			23: begin ad7606_rd_n_o <= 0; state <= state + 1'b1;end
			24: begin state <= state + 1'b1;end
			25: begin ad7606_rd_n_o <= 1; data3 <= ad7606_db_i; state <= state + 1'b1;end		
			26: state <= state + 1'b1;
			27: state <= state + 1'b1;
			28: begin ad7606_rd_n_o <= 0; state <= state + 1'b1;end
			29: begin state <= state + 1'b1;end
			30: begin ad7606_rd_n_o <= 1; data4 <=  ad7606_db_i; state <= state + 1'b1;end			
			31: state <= state + 1'b1;
			32: state <= state + 1'b1;
			33: begin ad7606_rd_n_o <= 0; state <= state + 1'b1;end
			34: begin state <= state + 1'b1;end
			35: begin ad7606_rd_n_o <= 1; data5 <=  ad7606_db_i; state <= state + 1'b1;end
			36: state <= state + 1'b1;
			37: state <= state + 1'b1;
			38: begin ad7606_rd_n_o <= 0; state <= state + 1'b1;end
			39: begin state <= state + 1'b1;end
			40: begin ad7606_rd_n_o <= 1; data6 <=  ad7606_db_i; state <= state + 1'b1;end
			41: state <= state + 1'b1;
			42: state <= state + 1'b1;
			43: begin ad7606_rd_n_o <= 0; state <= state + 1'b1;end
			44: begin state <= state + 1'b1;end
			45: begin ad7606_rd_n_o <= 1; data7 <=  ad7606_db_i; state <= state + 1'b1;end
			46: state <= state + 1'b1;
			47: state <= state + 1'b1;
			48: begin ad7606_rd_n_o <= 0; state <= state + 1'b1;end
			49: begin state <= state + 1'b1;end
			50: begin ad7606_rd_n_o <= 1; data8 <=  ad7606_db_i; state <= state + 1'b1;end
			51: begin state <= state + 1'b1; ad7606_reset_o <= 1;end
			52: begin state <= state + 1'b1; end
			53: begin state <= state + 1'b1; Conv_Done <= 1;end
			54: begin state <= 0;ad7606_reset_o <= 0; Conv_Done <= 0; end
			default:
				begin
					state <= 0;
					ad7606_convst_o <= 1;
					Conv_Done <= 0;
					data1 <= 0;
					data2 <= 0;
					data3 <= 0;
					data4 <= 0;
					data5 <= 0;
					data6 <= 0;
					data7 <= 0;
					data8 <= 0;
					ad7606_rd_n_o <= 1;
					ad7606_reset_o <= 0;
				end
		endcase
	end

endmodule
