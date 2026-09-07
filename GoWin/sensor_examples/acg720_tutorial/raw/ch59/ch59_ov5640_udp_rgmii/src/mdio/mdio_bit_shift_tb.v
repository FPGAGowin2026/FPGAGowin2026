`timescale 1ns/1ps
module mdio_bit_shift_tb;
    reg             Clk;
    wire            mdc;         //时钟接口
    wire            mdio;        //数据接口
    reg             rst_n;       //模块复位，低电平有效
    reg             if_read;     //读写方向控制 1:读，0:写
    reg      [4:0]  phy_addr;    //phy地址
    reg      [4:0]  reg_addr;    //reg地址
    reg      [15:0] wrdata;      //写入phy寄存器的数据
    reg             start;       //开始传输标志
    wire            done;        //操作完成标志
    wire     [15:0] rddata;      //phy寄存器读出的数据
    reg            Go;
    
    pullup PUP(mdio);  //模拟外部上拉电阻
    mdio_bit_shift DUT(
        .Clk      (Clk      ),
        .Rst_n    (rst_n    ),
        
        .Mode     (if_read  ),
        .Go       (Go),

        .Phy_addr (phy_addr ),
        .Reg_addr (reg_addr ),
        .Tx_Data   (wrdata   ),
        .Rx_Data   (rddata   ),
        .Trans_Done(done     ),
        
        .mdc      (mdc      ),
        .mdio     (mdio     )

    );
    
    initial Clk = 1;
    always #10 Clk = ~Clk;
    initial begin

        rst_n = 0;
        if_read = 0;
        Go = 0;
        phy_addr = 5'b00000;
        reg_addr = 5'b00000;
        wrdata = 16'd0;
        #201;

        rst_n = 1;
        #20;
        Go = 1;
        #20;
        Go = 0;
        if_read = 0;
        phy_addr = 5'b00001;
        reg_addr = 5'b00000;
        wrdata = 16'h2100;
        @(posedge mdc);
        @(posedge done);
        #200;
        $stop; 
    end
endmodule
