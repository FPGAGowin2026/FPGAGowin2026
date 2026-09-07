`timescale 1ns/1ps
module tb_jy61p_uart;
    localparam integer CPB=10;
    reg clk=0, rst_n=0, rx=1;
    wire signed [15:0] roll,pitch,yaw; wire [15:0] ver;
    wire valid, checksum_error, framing_error;
    integer checks=0, valid_count=0, checksum_error_count=0;
    always #500 clk=~clk;
    jy61p_uart #(.CLK_FREQ_HZ(1000000),.BAUD_RATE(100000)) dut(
        .clk(clk),.rst_n(rst_n),.uart_rx(rx),.roll_raw(roll),.pitch_raw(pitch),
        .yaw_raw(yaw),.version_raw(ver),.frame_valid(valid),
        .checksum_error(checksum_error),.framing_error(framing_error));
    always @(posedge clk) begin if(valid) valid_count<=valid_count+1; if(checksum_error) checksum_error_count<=checksum_error_count+1; end
    task check; input condition; input [255:0] msg; begin checks=checks+1; if(!condition) begin $display("FAIL: %0s",msg); $fatal(1); end end endtask
    task send_byte; input [7:0] b; integer i; begin
        rx=0; repeat(CPB) @(posedge clk); for(i=0;i<8;i=i+1) begin rx=b[i]; repeat(CPB) @(posedge clk); end rx=1; repeat(CPB) @(posedge clk);
    end endtask
    task send_frame; input bad; reg [7:0] bytes[0:9]; reg [7:0] sum; integer i; begin
        bytes[0]=8'h55; bytes[1]=8'h53; bytes[2]=8'h34; bytes[3]=8'h12;
        bytes[4]=8'h00; bytes[5]=8'h80; bytes[6]=8'hFE; bytes[7]=8'hFF;
        bytes[8]=8'h78; bytes[9]=8'h56; sum=0;
        for(i=0;i<10;i=i+1) begin sum=sum+bytes[i]; send_byte(bytes[i]); end
        send_byte(bad ? (sum^8'h01) : sum);
    end endtask
    initial begin
        repeat(5) @(posedge clk); rst_n=1; repeat(3) @(posedge clk);
        send_frame(0); repeat(3) @(posedge clk);
        check(valid_count==1,"valid packet accepted once");
        check(roll===16'sh1234 && pitch===16'sh8000 && yaw===16'shFFFE,"signed angle fields decoded little endian");
        check(ver==16'h5678,"version field decoded");
        send_frame(1); repeat(3) @(posedge clk);
        check(checksum_error_count==1 && valid_count==1,"bad checksum is flagged and does not publish data");
        check(!framing_error,"well-formed UART stop bits");
        $display("PASS: jy61p_uart (%0d checks)",checks); $finish;
    end
endmodule
