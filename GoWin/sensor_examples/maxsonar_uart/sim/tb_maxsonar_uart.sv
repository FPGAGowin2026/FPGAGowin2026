`timescale 1ns/1ps
module tb_maxsonar_uart;
    localparam integer CPB=10;reg clk=0,rst_n=0,rx=1;wire[15:0]range_raw;wire valid,format_error,framing_error;integer checks=0,valids=0,format_errors=0;
    always#500 clk=~clk;always@(posedge clk)begin if(valid)valids<=valids+1;if(format_error)format_errors<=format_errors+1;end
    sensor_example_top #(.CLK_FREQ_HZ(1000000),.BAUD_RATE(100000))dut(.clk(clk),.rst_n(rst_n),.maxsonar_rx(rx),.range_raw(range_raw),.range_valid(valid),.format_error(format_error),.framing_error(framing_error));
    task check;input c;input[255:0]m;begin checks=checks+1;if(!c)begin $display("FAIL: %0s",m);$fatal(1);end end endtask
    task send_byte;input[7:0]v;integer i;begin rx=0;repeat(CPB)@(posedge clk);for(i=0;i<8;i=i+1)begin rx=v[i];repeat(CPB)@(posedge clk);end rx=1;repeat(CPB)@(posedge clk);end endtask
    initial begin repeat(5)@(posedge clk);rst_n=1;repeat(3)@(posedge clk);
        send_byte("R");send_byte("1");send_byte("2");send_byte("3");send_byte("4");send_byte(8'h0D);repeat(3)@(posedge clk);check(valids==1&&range_raw==1234,"four-digit serial range decoded");
        send_byte("R");send_byte("1");send_byte("X");repeat(3)@(posedge clk);check(format_errors==1&&valids==1,"non-decimal frame flagged and rejected");check(!framing_error,"valid UART stop bits");
        $display("PASS: maxsonar_uart (%0d checks)",checks);$finish;end
endmodule
