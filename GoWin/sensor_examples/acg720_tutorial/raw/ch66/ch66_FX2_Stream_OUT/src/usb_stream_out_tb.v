`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/09/29 13:26:58
// Design Name: 
// Module Name: usb_stream_out_tb
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


module usb_stream_out_tb;
reg [15:0] fx2_fdata;
    reg fx2_flagb;
    reg fx2_flagc;
    reg fx2_ifclk;
    reg reset_n;

    wire [15:0]  fifo_q;
    wire [1:0]  fx2_faddr;
    wire fx2_sloe;
    wire fx2_slrd;
    wire fx2_slwr;

    reg source_ready;
    
    wire [15:0] usb_data;
    wire usb_data_valid;
    
    wire fifo_empty;
    wire [11:0]fifo_usedw;
    reg fifo_rdreq;
    
    //例化USB Stream OUT数据读取模块
    usb_stream_out usb_ctl(
        .fx2_fdata(fx2_fdata),                               
        .fx2_faddr(fx2_faddr),                               
        .fx2_slrd(fx2_slrd),                                
        .fx2_slwr(fx2_slwr),                                
        .fx2_sloe(fx2_sloe), 
        .fx2_flagc(fx2_flagc),                               
        .fx2_flagb(fx2_flagb),                               
        .fx2_ifclk(fx2_ifclk),                                 
        .reset_n(reset_n),
        .data_out(usb_data),
        .data_valid(usb_data_valid),
        .source_ready(source_ready)
    );

    //例化一个FIFO，用来存储从USB中读取到的数据
    fifo_top fifo_top(
        .Data(usb_data), //input [15:0] Data
        .Reset(~reset_n), //input Reset
        .WrClk(fx2_ifclk), //input WrClk
        .RdClk(fx2_ifclk), //input RdClk
        .WrEn(usb_data_valid), //input WrEn
        .RdEn(fifo_rdreq), //input RdEn
        .Rnum(fifo_usedw), //output [12:0] Rnum
        .Q(fifo_q), //output [15:0] Q
        .Empty(fifo_empty), //output Empty
        .Full() //output Full
    );

    //产生USB时钟信号
    initial fx2_ifclk = 1;
    always #10 fx2_ifclk = !fx2_ifclk;

    initial begin                                                  
        reset_n = 0;
        fx2_flagb = 0;
        #200;
        reset_n = 1;
        #2000;
        
        USB_TRANSFER_DATA(512);
        
        #2000;
        USB_TRANSFER_DATA(512);        
        
        
        #2000;
        USB_TRANSFER_DATA(512);        
        
        
        #2000;
        USB_TRANSFER_DATA(512);	
        
        #2000;
        USB_TRANSFER_DATA(512);	
        
        #2000;
        USB_TRANSFER_DATA(512);			
        
        #2000;
        USB_TRANSFER_DATA(512);		
        #20000;
        $stop;
    end 

    //使用一个计数器，用来模拟控制外部使用数据的逻辑从FIFO中读走数据的速率
    reg [12:0]cnt;
    always@(posedge fx2_ifclk or negedge reset_n)
    if(!reset_n)
        cnt <= 0;
    else
        cnt <= cnt + 1'b1;
        
    //cnt大于7000的情况下，根据fifo_usedw中的值，从FIFO中将数据读走
    always@(posedge fx2_ifclk or negedge reset_n)
    if(!reset_n)
        fifo_rdreq <= 0;
    else if(cnt > 7000)begin
        if(fifo_usedw >= 2)
            fifo_rdreq <= 1;
        else
            fifo_rdreq <= 0;
    end
    
    //当fifo中的剩余空间小于一半，即2048个字节时，拉低source_ready以暂停下一次从USB中读取数据过程
    always@(posedge fx2_ifclk or negedge reset_n)
    if(!reset_n)
        source_ready <= 0;
    else if(fifo_usedw >= 2047)
        source_ready <= 0;
    else
        source_ready <= 1;
        
    //模拟产生测试USB测试数据	
    task USB_TRANSFER_DATA;
        input [9:0] datanum;
        reg [9:0] cnt;
        reg clk_edge;
        reg [15:0] timeout_cnt; // 新增超时计数器
        reg break_loop; // 新增标志位用于控制循环终止
        begin
            cnt = 0;
            timeout_cnt = 0; // 初始化超时计数器
            break_loop = 0; // 初始化标志位
            #200;
            fx2_flagb <= 1;
            fx2_fdata <= 0;
            cnt = 0;
            clk_edge = 0;
            while(cnt < datanum - 1 && !break_loop)begin
                if(clk_edge)begin
                    if((!fx2_sloe) && (!fx2_slrd))begin
                        fx2_fdata <= fx2_fdata + 1;
                        cnt <= cnt + 1;
                        timeout_cnt = 0; // 条件满足，重置超时计数器
                    end else begin
                        #100; // 等待一段时间
                        timeout_cnt = timeout_cnt + 1; // 超时计数器加1
                        if (timeout_cnt > 100) begin // 假设超时次数为100
                            $display("Timeout in USB_TRANSFER_DATA!");
                            break_loop = 1; // 设置标志位以终止循环
                        end
                    end
                end
                clk_edge = ~clk_edge;
                #10; // 模拟时钟周期
            end
            cnt = 0;
            fx2_flagb <= 0;
        end
    endtask

endmodule    