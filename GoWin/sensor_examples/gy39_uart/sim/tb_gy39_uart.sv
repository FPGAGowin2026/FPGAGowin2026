`timescale 1ns/1ps
module tb_gy39_uart;
    localparam integer CPB=10; reg clk=0,rst_n=0,rx=1;
    wire [31:0] lux,press; wire signed [15:0] temp,alt; wire [15:0] hum;
    wire lv,ev,ce,fe,ue; integer checks=0,lv_count=0,ev_count=0,ce_count=0;
    always #500 clk=~clk;
    gy39_uart #(.CLK_FREQ_HZ(1000000),.BAUD_RATE(100000)) dut(
        .clk(clk),.rst_n(rst_n),.uart_rx(rx),.lux_centi_lux(lux),.temperature_centi_c(temp),
        .pressure_centi_pa(press),.humidity_centi_rh(hum),.altitude_m(alt),.lux_valid(lv),
        .environment_valid(ev),.checksum_error(ce),.format_error(fe),.framing_error(ue));
    always @(posedge clk) begin if(lv)lv_count<=lv_count+1;if(ev)ev_count<=ev_count+1;if(ce)ce_count<=ce_count+1;end
    task check;input c;input[255:0]m;begin checks=checks+1;if(!c)begin $display("FAIL: %0s",m);$fatal(1);end end endtask
    task send_byte;input[7:0]v;integer i;begin rx=0;repeat(CPB)@(posedge clk);for(i=0;i<8;i=i+1)begin rx=v[i];repeat(CPB)@(posedge clk);end rx=1;repeat(CPB)@(posedge clk);end endtask
    task send_lux;input bad;reg[7:0]s;begin s=0;send_byte(8'h5A);s=s+8'h5A;send_byte(8'h5A);s=s+8'h5A;send_byte(8'h15);s=s+8'h15;send_byte(4);s=s+4;send_byte(0);send_byte(0);send_byte(8'h30);s=s+8'h30;send_byte(8'h39);s=s+8'h39;send_byte(bad?(s+1):s);end endtask
    task sb;input[7:0]v;inout[7:0]s;begin send_byte(v);s=s+v;end endtask
    task send_env;reg[7:0]s;begin s=0;sb(8'h5A,s);sb(8'h5A,s);sb(8'h45,s);sb(10,s);sb(8'hF6,s);sb(8'h3C,s);sb(8'h00,s);sb(8'h0F,s);sb(8'h42,s);sb(8'h40,s);sb(8'h13,s);sb(8'h88,s);sb(8'hFF,s);sb(8'hF4,s);send_byte(s);end endtask
    initial begin repeat(5)@(posedge clk);rst_n=1;repeat(3)@(posedge clk);
        send_lux(0);repeat(3)@(posedge clk);check(lv_count==1&&lux==32'h00003039,"lux frame decoded");
        send_env();repeat(3)@(posedge clk);check(ev_count==1,"environment frame accepted");
        check(temp===-16'sd2500&&press==32'd1000000&&hum==16'd5000&&alt===-16'sd12,"BME fields decoded big endian");
        send_lux(1);repeat(3)@(posedge clk);check(ce_count==1&&lv_count==1,"bad checksum flagged and rejected");check(!ue,"UART framing remains valid");
        $display("PASS: gy39_uart (%0d checks)",checks);$finish;end
endmodule
