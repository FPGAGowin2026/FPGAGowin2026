`timescale 1ns / 1ps

module UDP_Send_tb;
    reg Clk;
    reg Rst_n;
    wire Tx_Done;
    reg GMII_GTXC;
    reg wrreq;
    reg [7:0]wrdata;
    reg wrclk;
    reg aclr;
    
    wire [7:0]GMII_TXD;
    wire GMII_TXEN;
    wire [12:0]wrusedw;
    
    parameter data_length = 16'd16;
    parameter des_mac = 48'hFF_FF_FF_FF_FF_FF;
    parameter src_mac = 48'h00_0a_35_01_fe_c0;
    parameter des_port = 16'd6102;
    parameter src_port = 16'd5000;
    parameter des_ip = 32'hc0_a8_00_03;
    parameter src_ip = 32'hc0_a8_00_02;
    
    UDP_Send UDP_Send(
    .Clk(Clk),
    .Rst_n(Rst_n),
    
    //	Go,
    .Tx_Done(Tx_Done),
    
    .des_mac(des_mac),
    .src_mac(src_mac),
    
    .des_port(des_port),
    .src_port(src_port),
    
    .des_ip(des_ip),
    .src_ip(src_ip),
    
    .data_length(data_length),
    
    .GMII_GTXC(GMII_GTXC),
    .GMII_TXD(GMII_TXD),
    .GMII_TXEN(GMII_TXEN),
    
    .wrreq(wrreq),
    .wrdata(wrdata),
    .wrclk(wrclk),
    .aclr(aclr),
    .wrusedw(wrusedw)
    );

    initial Clk = 1;
    always #10 Clk = ~Clk;
    
    initial wrclk = 1;
    always #10 wrclk = ~wrclk;
    
    initial GMII_GTXC = 1;
    always #4 GMII_GTXC = ~GMII_GTXC;    
    
    initial begin
        Rst_n = 0;
        #201;
        Rst_n = 1;
        #201;
        wr_data(8'h00,data_length);
        #2000;
        wr_data(8'h10,data_length);
        #2000;
        $stop; 
    end

    task wr_data;
    input [7:0]data_begin;
    input [7:0]data_length;
    begin
        wrdata = data_begin;
        wrreq = 0;
        aclr = 1;
        #601;
        #1 aclr = 0;
        #401;
        repeat(data_length)
        begin
            @(posedge wrclk);
            wrdata = wrdata +1'b1;
            wrreq = 1;
        end
        @(posedge wrclk);
        wrreq = 0;
    end
  endtask
endmodule
