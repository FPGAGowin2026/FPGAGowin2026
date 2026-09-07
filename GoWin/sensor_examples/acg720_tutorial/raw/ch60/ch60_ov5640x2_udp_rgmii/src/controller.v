module controller(
	clk,
	rst_n,
	rdclk,
	
	rdreq1,
	rddata1,
	rdusedw1,
	
	rdreq2,
	rddata2,
	rdusedw2,
	
	wrclk,
	fifo_aclr,
	fifo_wrreq,
	fifo_wrdata
);
	input clk;
	input rst_n;
	output rdclk;
	
	output reg rdreq1;
	input [7:0] rddata1;
	input [13:0] rdusedw1;
	
	output reg rdreq2;
	input [7:0] rddata2;
	input [13:0] rdusedw2;
	
	output wrclk;
	output fifo_aclr;
	output reg fifo_wrreq;
	output reg [7:0] fifo_wrdata;
	
	localparam IDLE  = 3'b001;
	localparam SIDEA = 3'b010;
	localparam SIDEB = 3'b100;
	
	parameter IMG_WIDTH = 640;
	
	assign rdclk = clk;
	assign wrclk = clk;
	assign fifo_aclr = ~rst_n;
	
	reg [2:0] state;
	reg [11:0] cnt;//count send data for state jump
	always @ (posedge clk or negedge rst_n)
	if (!rst_n) begin
		cnt <= 'd0;
		state <= IDLE;
	end else begin
		case (state)
			IDLE:
				begin
					if (rdusedw1 >= IMG_WIDTH * 2 + 2) begin
						cnt <= 'd0;
						state <= SIDEA;
					end else if (rdusedw2 >= IMG_WIDTH * 2 + 2) begin
						cnt <= 'd0;
						state <= SIDEB;
					end else
						state <= IDLE;
				end
			SIDEA:
				begin
					if (cnt < IMG_WIDTH * 2 + 2 - 1) begin
						cnt <= cnt + 1'b1;
						state <= SIDEA;
					end else if (rdusedw2 >= IMG_WIDTH * 2 + 2) begin
						cnt <= 'd0;
						state <= SIDEB;
					end else begin
						cnt <= 'd0;
						state <= IDLE;
					end
				end
			SIDEB:
				begin
					if (cnt < IMG_WIDTH * 2 + 2 - 1) begin
						cnt <= cnt + 1'b1;
						state <= SIDEB;
					end else if (rdusedw1 >= IMG_WIDTH * 2 + 2) begin
						cnt <= 'd0;
						state <= SIDEA;
					end else begin
						cnt <= 'd0;
						state <= IDLE;
					end
				end
			default	:	state <= IDLE;
		endcase
	end
	
//	always @ (posedge clk or negedge rst_n)
//	if (!rst_n) begin
//		rdreq1 <= 1'b0;
//		rdreq2 <= 1'b0;
//		fifo_wrreq <= 1'b0;
//		fifo_wrdata <= 8'd0;
//	end else begin
//		case (state)
//			IDLE:
//				begin
//					rdreq1 <= 1'b0;
//					rdreq2 <= 1'b0;
//					fifo_wrreq <= 1'b0;
//					fifo_wrdata <= 8'd0;
//				end
//			SIDEA:
//				begin
//					rdreq1 <= 1'b1;
//					rdreq2 <= 1'b0;
//					fifo_wrreq <= 1'b1;
//					fifo_wrdata <= rddata1;
//				end
//			SIDEB:
//				begin
//					rdreq1 <= 1'b0;
//					rdreq2 <= 1'b1;
//					
//					if (cnt > 1) begin	// 扔掉第二幅图的行编号2字节
//						fifo_wrreq <= 1'b1;
//						fifo_wrdata <= rddata2;
//					end else begin
//						fifo_wrreq <= 1'b0;
//						fifo_wrdata <= 8'd0;
//					end	
//				end
//			default:
//				begin
//					rdreq1 <= 1'b0;
//					rdreq2 <= 1'b0;
//					fifo_wrreq <= 1'b0;
//					fifo_wrdata <= 8'd0;
//				end
//		endcase
//	end
	
	always @ (*) begin
		case (state)
			IDLE:
				begin
					rdreq1 = 1'b0;
					rdreq2 = 1'b0;
					fifo_wrreq = 1'b0;
					fifo_wrdata = 8'd0;
				end
			SIDEA:
				begin
					rdreq1 = 1'b1;
					rdreq2 = 1'b0;
					fifo_wrreq = 1'b1;
					fifo_wrdata = rddata1;
				end
			SIDEB:
				begin
					rdreq1 = 1'b0;
					rdreq2 = 1'b1;
					
					if (cnt > 1) begin	// 扔掉第二幅图的行编号2字节
						fifo_wrreq = 1'b1;
						fifo_wrdata = rddata2;
					end else begin
						fifo_wrreq = 1'b0;
						fifo_wrdata = 8'd0;
					end	
				end
			default:
				begin
					rdreq1 = 1'b0;
					rdreq2 = 1'b0;
					fifo_wrreq = 1'b0;
					fifo_wrdata = 8'd0;
				end
		endcase
	end

endmodule
