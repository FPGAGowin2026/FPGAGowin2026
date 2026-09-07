/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : state_ctrl
// Description   : ADC采集数据DDR3缓存网口发送状态控制模块
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module state_ctrl(
	input clk,
	input reset,

	input ddr3_init_done,  //DDR初始化完成标志信号
	
	input start_sample, //ADC启动采集标志信号
	input [31:0]set_sample_num,//需要采集的数量，32位，4G
	
	input rdfifo_empty, //DDR双端口模块读FIFO为空标志信号
	input [15:0]rdfifo_dout, //从DDR中读出的16位数据
	input wrfifo_full,  //DDR写FIFO为满标志信号
	
    input adc_data_en,  //ADC输出数据使能信号
    input ad_out_valid,	 //ADC输出数据有效信号
    
	output reg wr_load,
	output reg rd_load, 
	output reg rdfifo_rden, //DDR双端口模块的读使能信号
	
    output reg ad_sample_en,   //ADC采样使能标志信号
	output reg eth_fifo_wrreq, //以太网发送fifo_tx的写请求信号
	output reg [15:0]eth_fifo_wrdata    //需要想以太网发送fifo_tx中写入的数据
);
	//*//状态机的状态总位宽为[3:0]也可行
	reg [4:0]state;
	//统计ADC向DDR传送的数据，和set_sample_num位深相同
	reg [31:0]adc_sample_cnt;
	//统计向网口发送ADC数据的个数，和set_sample_num位深一致
	reg [31:0]send_data_cnt;
	//将start_sample_rm进行采样，锁定其只在IDLE状态进行工作有效，其余状态均无效
	reg start_sample_rm;
    reg [4:0]wr_load_cnt;
    reg [4:0]rd_load_cnt;
    reg [31:0] data_num;

   localparam IDLE                   = 4'd0;   //空闲状态
   localparam DDR_WR_LOAD            = 4'd1;   //DDR写入数据更新状态
   localparam ADC_SAMPLE             = 4'd2;   //ADC采样数据状态
   localparam DDR_RD_LOAD            = 4'd3;   //DDR读出数据更新状态
   localparam DATA_SEND_START        = 4'd4;   //数据发送状态
   localparam DATA_SEND_WORKING      = 4'd5;   //数据发送完成状态
   
	always@(posedge clk or posedge reset)
	if(reset)begin
		state<=IDLE;
		data_num <= 32'd0;
		rdfifo_rden <= 1'd0;
	end
	else 
    case(state)
		IDLE: //0
        begin
            if(start_sample_rm) begin   //DDR初始化完成并且产生启动采样信号
               state <= DDR_WR_LOAD; 
            end
            else begin
               state <= state;
            end
        end
        
        DDR_WR_LOAD://1  
        begin
            if(!wrfifo_full && (wr_load_cnt==9))
                state<=ADC_SAMPLE;
            else
                state<=state;
        end
        
        ADC_SAMPLE://2
        begin
            if((adc_sample_cnt>=set_sample_num-1'b1)&& adc_data_en)   
                state<=DDR_RD_LOAD;
            else
                state<=state;
        end
        
        DDR_RD_LOAD://3
        begin
            if(!rdfifo_empty && (rd_load_cnt==9))begin
                state<=DATA_SEND_START;
            end
            else
                state<=state;
            end
         
        DATA_SEND_START://4
        begin
            state <= DATA_SEND_WORKING;
        end
        
        DATA_SEND_WORKING://5
        begin
            if(send_data_cnt >= set_sample_num-1'b1) begin //DDR中存储的数据全部读完
                rdfifo_rden <= 1'b0;
                state <= IDLE;
            end
            else begin
                rdfifo_rden <= 1'b1;
                state <= DATA_SEND_WORKING;
            end
        end  
     
        default: state <= IDLE;  
    endcase

    always@(posedge clk or posedge reset)
    if(reset) begin
        eth_fifo_wrreq <= 1'b0;
        eth_fifo_wrdata <= 'd0;
    end
    else if(rdfifo_rden) begin
        eth_fifo_wrreq <= 1'b1;
        eth_fifo_wrdata <= rdfifo_dout;
    end
    else begin
        eth_fifo_wrreq <= 1'b0;
        eth_fifo_wrdata <= 'd0;
    end
  
	always@(posedge clk or posedge reset)begin  //对start_sample采样起始位进行寄存，同时限定其只工作在状态IDLE
	if(reset)
		start_sample_rm <= 1'b0;
	else if(state==IDLE && ddr3_init_done==1'b1)
		start_sample_rm <= start_sample;
	else 
		start_sample_rm <= 1'b0;
	end

	always@(posedge clk or posedge reset)begin
	if(reset)
		wr_load_cnt<=0;
	else if(state==DDR_WR_LOAD)
	begin 
		if(wr_load_cnt==9)
			wr_load_cnt<=4'd9;
		else
			wr_load_cnt<=wr_load_cnt+1'b1;
	end
	else
		wr_load_cnt<=1'b0;
	end
	
	always@(posedge clk or posedge reset)begin
	if (reset)
		wr_load<=0;
	else if(ddr3_init_done==1'b0)
		wr_load<=1'b1;
	else if(state==DDR_WR_LOAD)
        begin
            if(wr_load_cnt==0||wr_load_cnt==1||wr_load_cnt==2)
                wr_load<=1'b1;
            else
                wr_load<=1'b0;
        end
	else 
		wr_load<=1'b0;
	end
	
    always@(posedge clk or posedge reset)begin
    if(reset)
      ad_sample_en<=0;
    else if(state==ADC_SAMPLE)
      ad_sample_en<=1;
    else
      ad_sample_en<=0;
    end


	always@(posedge clk or posedge reset)begin
    if(reset)
		ad_sample_en<=0;
	else if(state==ADC_SAMPLE)
		ad_sample_en<=1;
	else
		ad_sample_en<=0;
	end
	
//以下//如果adc_sample_cnt在ADC_SAMPLE状态，则每个时钟周期自加1
	always@(posedge clk or posedge reset)begin  
    if(reset)                                  
		adc_sample_cnt<=1'b0;
    else if(state==ADC_SAMPLE)begin
        if(adc_data_en)
		adc_sample_cnt<=adc_sample_cnt+1'b1;
		else
		adc_sample_cnt<=adc_sample_cnt;
	end
    else
		adc_sample_cnt<=1'b0;
	end

//以下//清除DDR读FIFO的计数器，10拍后指挥状态的跳转
	always@(posedge clk or posedge reset)begin
    if(reset)
		rd_load_cnt<=0;
    else if(state==DDR_RD_LOAD)//如果进入了清fifo状态
    begin 
		if(rd_load_cnt==9)
			rd_load_cnt<=4'd9;
		else
			rd_load_cnt<=rd_load_cnt+1'b1;
    end
    else
		rd_load_cnt<=1'b0;
	end

	always@(posedge clk or posedge reset)begin
    if (reset)
		rd_load<=0;
    else if(ddr3_init_done==1'b0)
		rd_load<=1'b1;
    else if(state==DDR_RD_LOAD)
    begin
		if(rd_load_cnt==0||rd_load_cnt==1||rd_load_cnt==2)
			rd_load<=1'b1;
     else
			rd_load<=1'b0;
    end
    else 
		rd_load<=1'b0;
	end
	
/*每个send_data_cnt在rdfifo_rden为1的状态下加1，
   由于rdfifo_rden为高连续持续一拍，
   保证了每次读16bit数时send_data_cnt只持续加1*/
  always@(posedge clk or posedge reset)begin
  if(reset)
    send_data_cnt<=32'd0;
  else if(state==IDLE)
    send_data_cnt<=32'd0;
  else if(rdfifo_rden)
    send_data_cnt<=send_data_cnt+1;
  else 
    send_data_cnt<=send_data_cnt;
  end
endmodule