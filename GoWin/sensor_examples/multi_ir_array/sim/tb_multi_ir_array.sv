`timescale 1ns/1ps
module tb_multi_ir_array;
    reg clk=0,rst_n=0;reg[7:0]ir_n=8'hFF;wire[7:0]mask;wire changed;integer checks=0,changes=0;
    reg[1:0]ir_high=2'b00;wire[1:0]mask_high;wire changed_high;
    always #5 clk=~clk;always@(posedge clk)if(changed)changes<=changes+1;
    sensor_example_top #(.DEBOUNCE_CYCLES(4)) dut(.clk(clk),.rst_n(rst_n),.ir_sensor_n(ir_n),.obstacle_mask(mask),.change_valid(changed));
    multi_ir_array #(.CHANNELS(2),.DEBOUNCE_CYCLES(1),.ACTIVE_LOW(1'b0)) dut_active_high(
      .clk(clk),.rst_n(rst_n),.sensor_i(ir_high),.active_mask_o(mask_high),.change_valid_o(changed_high));
    task check;input c;input[255:0]m;begin checks=checks+1;if(!c)begin $display("FAIL: %0s",m);$fatal(1);end end endtask
    initial begin repeat(4)@(posedge clk);rst_n=1;repeat(4)@(posedge clk);#1;
        check(mask_high==0&&!changed_high,"active-high startup remains inactive");
        ir_high[0]=1;repeat(4)@(posedge clk);#1;check(mask_high==2'b01,"active-high channel works with DB=1");
        ir_n[2]=0;repeat(7)@(posedge clk);#1;check(mask==8'h04,"stable active-low channel is debounced");check(changes==1,"one change pulse emitted");
        ir_n[5]=0;@(posedge clk);ir_n[5]=1;repeat(7)@(posedge clk);#1;check(mask==8'h04,"single-cycle glitch is rejected");check(changes==1,"glitch emits no event");
        ir_n[2]=1;repeat(7)@(posedge clk);#1;check(mask==0&&changes==2,"stable release is published");
        $display("PASS: multi_ir_array (%0d checks)",checks);$finish;end
endmodule
