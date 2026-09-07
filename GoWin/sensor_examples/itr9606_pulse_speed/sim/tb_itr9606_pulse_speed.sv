`timescale 1ns/1ps
module tb_itr9606_pulse_speed;
    reg clk=0,rst_n=0,pulse=0;wire[31:0]count,period;wire sample_valid,period_valid,overflow;integer checks=0,period_events=0;
    always #5 clk=~clk;always@(posedge clk)if(period_valid)period_events<=period_events+1;
    itr9606_pulse_speed #(.WINDOW_CYCLES(50),.MIN_EDGE_GAP_CYCLES(5)) dut(
        .clk(clk),.rst_n(rst_n),.pulse_i(pulse),.pulse_count_o(count),.period_cycles_o(period),.sample_valid_o(sample_valid),.period_valid_o(period_valid),.count_overflow_o(overflow));
    task check;input c;input[255:0]m;begin checks=checks+1;if(!c)begin $display("FAIL: %0s",m);$fatal(1);end end endtask
    task edge_pulse;begin pulse=1;repeat(2)@(posedge clk);pulse=0;end endtask
    initial begin repeat(3)@(posedge clk);rst_n=1;repeat(3)@(posedge clk);
        edge_pulse();repeat(7)@(posedge clk);edge_pulse();
        // Chatter edge less than five synchronized clock intervals later.
        @(posedge clk);pulse=1;@(posedge clk);pulse=0;
        wait(sample_valid);#1;check(count==2,"two real slot edges counted and close chatter rejected");
        check(period_events==1,"period produced only between accepted edges");check(period>=8&&period<=12,"period is measured in system clocks");check(!overflow,"no counter overflow");
        $display("PASS: itr9606_pulse_speed (%0d checks)",checks);$finish;end
endmodule
