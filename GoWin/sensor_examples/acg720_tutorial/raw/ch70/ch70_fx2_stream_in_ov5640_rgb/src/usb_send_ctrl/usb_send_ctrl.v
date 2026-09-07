`timescale 1ns / 1ps

module usb_send_ctrl
(
  input                            reset_p          ,

  input                            clk              ,  //pclk_bufg_o
  input  [15:0]                    data_i           ,
  input                            data_valid_i     ,
  input                            start_sample     , //启动数据传输标志
  input                            camera_vs_start  , //摄像一副图像数据的起始开始
  
  input                            usb_fifo_full,
  output reg                      inout_switch      ,
  output reg                      usb_fifo_wrreq    , //USB FIFO的写请求信号，产生该信号,USB开始发送数据
  output reg                      fx2_clear         ,//USB清除
  output reg    [15:0]            usb_fifo_wrdata    //USB需要发送的数据
  );
  reg [1:0]state;
   
  localparam IDLE                   = 2'd0;   //空闲状态
  localparam RESET_USB              = 2'd1;   //复位USB状态
  localparam DATA_SEND_START        = 2'd2;   //数据发送开启状态
  localparam DATA_SEND_WORKING      = 2'd3;   //数据发送状态
  
   //清零USB FIFO延时计数器 
  reg [19:0] rst_usb_cnt;
  reg [31:0] read_cnt;

  always@(posedge clk or posedge reset_p)
  if(reset_p) begin
    state <= IDLE;
    fx2_clear <= 1'b1;
    rst_usb_cnt <= 0;
    inout_switch<=1'b0;
  end
  else begin
    case(state)
        IDLE: //0
        begin
            if(start_sample) begin
                inout_switch <= 1'b1;
                state <= RESET_USB;
                fx2_clear <= 1'b1;
            end
            else begin
                inout_switch <= 1'b0;
                fx2_clear <= 1'b1;
                state <= state;
            end
        end
        
        RESET_USB: //1
        begin
            if(rst_usb_cnt >= 20'hffff0)begin
                rst_usb_cnt <= 0;
                state <= DATA_SEND_START;
            end
            else if(rst_usb_cnt >= 20'h7fff0)begin
                rst_usb_cnt <= rst_usb_cnt + 1'd1;
                fx2_clear <= 1'b0;
            end
            else
                rst_usb_cnt <= rst_usb_cnt + 1'd1; 
        end
        
        DATA_SEND_START : 
        begin
            if(camera_vs_start) begin 
                state <= DATA_SEND_WORKING;
            end
            else
                state <= state;
        end
        
        DATA_SEND_WORKING:
        begin
              if(read_cnt >= 32'hffffff)  
              begin
                state <= IDLE;
              end
              else
                state <= DATA_SEND_WORKING;
        end
        
        default:;
    endcase
  end
  
  always@(posedge clk or posedge reset_p)
  if(reset_p)
    read_cnt <= 32'd0;
  else if(read_cnt >= 32'hffffff)
    read_cnt <= 32'd0;
  else if((state == DATA_SEND_WORKING) && usb_fifo_full) 
    read_cnt <= read_cnt +1'd1;
  else
     read_cnt <= 32'd0;
  
  
  always@(posedge clk or posedge reset_p)
  if(reset_p)
      usb_fifo_wrreq <= 1'd0;
  else if((state == DATA_SEND_WORKING) && data_valid_i)
  begin
      usb_fifo_wrreq <= 1'd1;
      usb_fifo_wrdata <= data_i;
  end
  else
  begin
      usb_fifo_wrreq <= 1'd0;
      usb_fifo_wrdata <= 16'd0;
  end
      
endmodule
