`timescale 1ns/1ps
module tb_mt9v03x_dvp;
    localparam integer CPB=10;reg clk=0,pclk=0,rst_n=0,vsync=0,href=1;reg[7:0]data=8'h99;reg cfg_valid=0;reg[7:0]cmd=0;reg[15:0]val=0;wire cfg_ready,cfg_done,tx,frame_active,pixel_valid;wire[7:0]pixel;wire[15:0]x,y;wire fs,fe;reg[7:0]seen[0:3];integer checks=0,i;
    always#500 clk=~clk;always#7 pclk=~pclk;
    mt9v03x_dvp_capture u_cap(.pclk(pclk),.rst_n(rst_n),.vsync(vsync),.href(href),.data_i(data),.frame_active(frame_active),.pixel_valid(pixel_valid),.pixel_gray(pixel),.pixel_x(x),.line_y(y),.frame_start(fs),.frame_end(fe));
    camera_uart_cmd_tx_mt9v03x #(.CLK_FREQ_HZ(1000000),.BAUD_RATE(100000))u_cfg(.clk(clk),.rst_n(rst_n),.cfg_valid(cfg_valid),.cfg_command(cmd),.cfg_value(val),.cfg_ready(cfg_ready),.cfg_done(cfg_done),.tx_o(tx));
    task check;input c;input[255:0]m;begin checks=checks+1;if(!c)begin $display("FAIL: %0s",m);$fatal(1);end end endtask
    task recv_byte;output[7:0]v;integer j;begin v=0;@(negedge tx);repeat(CPB+CPB/2)@(posedge clk);for(j=0;j<8;j=j+1)begin v[j]=tx;repeat(CPB)@(posedge clk);end check(tx===1'b1,"UART stop bit high");end endtask
    initial begin repeat(4)@(posedge clk);rst_n=1;repeat(3)@(posedge pclk);#1;check(!pixel_valid&&!frame_active,"mid-frame reset release waits for VSYNC");@(negedge pclk);vsync=1;href=0;@(posedge pclk);@(negedge pclk);vsync=0;href=1;data=8'h3C;@(posedge pclk);#1;check(pixel_valid&&pixel==8'h3C&&x==1,"grayscale byte captured after sync");
        fork
            begin for(i=0;i<4;i=i+1)recv_byte(seen[i]);end
            begin @(negedge clk);cmd=8'h03;val=16'h00BC;cfg_valid=1;@(negedge clk);cfg_valid=0;check(!cfg_ready,"busy command interface applies backpressure");end
        join
        wait(cfg_done);#1;check(seen[0]==8'hA5&&seen[1]==8'h03&&seen[2]==0&&seen[3]==8'hBC,"four-byte camera command frame emitted");
        $display("PASS: mt9v03x_dvp (%0d checks)",checks);$finish;end
endmodule
