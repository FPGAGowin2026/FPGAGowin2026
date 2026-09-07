`timescale 1ns/1ps
module tb_gnss_nmea_uart;
    localparam integer CPB=10;reg clk=0,rst_n=0,rx=1;wire valid;wire[1:0]typ;wire[7:0]len,cs;wire ce,fe,oe,ue;integer checks=0,valids=0,ce_count=0;
    always#500 clk=~clk;always@(posedge clk)begin if(valid)valids<=valids+1;if(ce)ce_count<=ce_count+1;end
    sensor_example_top #(.CLK_FREQ_HZ(1000000),.BAUD_RATE(100000),.MAX_BODY_CHARS(20))dut(.clk(clk),.rst_n(rst_n),.gnss_rx(rx),.sentence_valid(valid),.sentence_type(typ),.body_length(len),.checksum_value(cs),.checksum_error(ce),.format_error(fe),.overflow_error(oe),.framing_error(ue));
    task check;input c;input[255:0]m;begin checks=checks+1;if(!c)begin $display("FAIL: %0s",m);$fatal(1);end end endtask
    task send_byte;input[7:0]v;integer i;begin rx=0;repeat(CPB)@(posedge clk);for(i=0;i<8;i=i+1)begin rx=v[i];repeat(CPB)@(posedge clk);end rx=1;repeat(CPB)@(posedge clk);end endtask
    function[7:0]hexchar;input[3:0]n;begin hexchar=(n<10)?("0"+n):("A"+n-10);end endfunction
    task send_gga;input bad;reg[7:0]body[0:10];reg[7:0]x;integer i;begin body[0]="G";body[1]="P";body[2]="G";body[3]="G";body[4]="A";body[5]=",";body[6]="1";body[7]=",";body[8]="2";body[9]=",";body[10]="3";x=0;send_byte("$");for(i=0;i<11;i=i+1)begin x=x^body[i];send_byte(body[i]);end send_byte("*");send_byte(hexchar(x[7:4]));send_byte(hexchar(x[3:0]^(bad?4'h1:4'h0)));send_byte(8'h0D);send_byte(8'h0A);end endtask
    initial begin repeat(5)@(posedge clk);rst_n=1;repeat(3)@(posedge clk);send_gga(0);repeat(3)@(posedge clk);check(valids==1&&typ==1,"valid GGA classified");check(len==11,"body length retained");send_gga(1);repeat(3)@(posedge clk);check(ce_count==1&&valids==1,"bad NMEA XOR flagged and rejected");check(!ue&&!fe&&!oe,"no unrelated error flags");$display("PASS: gnss_nmea_uart (%0d checks)",checks);$finish;end
endmodule
