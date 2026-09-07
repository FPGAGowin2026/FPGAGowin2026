`timescale 1ns/1ps
module tb_menc15a;
    logic clk=0,reset_n=0,miso=0; wire cs_n,sclk,mosi,initialized,sensor_ok;
    wire [14:0] position_raw; wire signed [15:0] speed_raw;
    wire position_status,speed_status,sample_valid,protocol_error;
    integer bit_no=0,round_no=0; logic [15:0] command=0,response=0; logic error_seen=0;
    sensor_example_top #(.CLK_DIV(2),.STARTUP_DELAY_CYCLES(3),.SAMPLE_INTERVAL_CYCLES(5)) dut(
        .clk(clk),.reset_n(reset_n),.encoder_miso(miso),.encoder_cs_n(cs_n),
        .encoder_sclk(sclk),.encoder_mosi(mosi),.*);
    always #5 clk=~clk;
    always @(posedge clk) if(protocol_error) error_seen<=1;
    always @(negedge cs_n) begin
        if(sclk!==1'b0)$fatal(1,"MENC15A SCLK must idle low");
        bit_no=0;command=0;miso=0;
    end
    always @(negedge sclk) if(!cs_n) begin
        if(bit_no<16)command={command[14:0],mosi};
        bit_no=bit_no+1;
    end
    always @(posedge sclk) if(!cs_n&&bit_no>=16&&bit_no<32) begin
        if(command==16'h8021)response=(round_no==0)?16'hffff:16'h1234;
        else if(command==16'h8032)response=16'h7ffe;
        else response=16'hffff;
        miso=response[31-bit_no];
    end
    always @(posedge cs_n) if(command==16'h8032 || (command==16'h8021&&round_no==0)) round_no=round_no+1;
    initial begin
        repeat(3)@(posedge clk);reset_n=1;
        repeat(6000)begin @(posedge clk);if(sample_valid)begin
            if(!initialized||!sensor_ok||!error_seen)$fatal(1,"disconnect signature path missing");
            if(position_raw!==15'h1234)$fatal(1,"position decode error");
            if(speed_raw!==-16'sd2)$fatal(1,"signed speed decode error: %0d",speed_raw);
            $display("PASS: MENC15A Mode-1 command/response capture rejects disconnected bus and recovers");$finish;
        end end
        $fatal(1,"timeout waiting for MENC15A sample");
    end
endmodule
