`timescale 1ns/1ps
module tb_scc8660_dvp;
    localparam integer CPB=10;reg clk=0,pclk=0,rst_n=0,vsync=1,href=0;reg[7:0]data=0;reg cfg_valid=0;reg[7:0]cmd=0;reg[15:0]val=0;wire cfg_ready,cfg_done,tx,frame_active,pixel_valid;wire[15:0]pixel,x,y;wire fs,fe,partial;reg[7:0]seen[0:3];integer checks=0,i;
    always#500 clk=~clk;always#7 pclk=~pclk;
    scc8660_dvp_capture u_cap(.pclk(pclk),.rst_n(rst_n),.vsync(vsync),.href(href),.data_i(data),.frame_active(frame_active),.pixel_valid(pixel_valid),.pixel_rgb565(pixel),.pixel_x(x),.line_y(y),.frame_start(fs),.frame_end(fe),.partial_pixel_error(partial));
    camera_uart_cmd_tx_scc8660 #(.CLK_FREQ_HZ(1000000),.BAUD_RATE(100000))u_cfg(.clk(clk),.rst_n(rst_n),.cfg_valid(cfg_valid),.cfg_command(cmd),.cfg_value(val),.cfg_ready(cfg_ready),.cfg_done(cfg_done),.tx_o(tx));
    task check;input c;input[255:0]m;begin checks=checks+1;if(!c)begin $display("FAIL: %0s",m);$fatal(1);end end endtask
    task pb;input[7:0]v;begin data=v;@(posedge pclk);#1;end endtask
    task recv_byte;output[7:0]v;integer j;begin v=0;@(negedge tx);repeat(CPB+CPB/2)@(posedge clk);for(j=0;j<8;j=j+1)begin v[j]=tx;repeat(CPB)@(posedge clk);end check(tx===1'b1,"UART stop bit high");end endtask
    initial begin repeat(4)@(posedge clk);rst_n=1;repeat(2)@(posedge pclk);@(negedge pclk);vsync=0;href=1;pb(8'h07);pb(8'hE0);check(pixel_valid&&pixel==16'h07E0,"RGB565 pixel assembled");pb(8'hAA);href=0;@(posedge pclk);#1;check(partial,"odd line byte is rejected and reported");
        fork
            begin for(i=0;i<4;i=i+1)recv_byte(seen[i]);end
            begin @(negedge clk);cmd=8'h09;val=16'h0002;cfg_valid=1;@(negedge clk);cfg_valid=0;end
        join
        wait(cfg_done);#1;check(seen[0]==8'hA5&&seen[1]==8'h09&&seen[2]==0&&seen[3]==2,"SCC8660 config command frame emitted");check(cfg_ready,"interface returns ready after frame");
        $display("PASS: scc8660_dvp (%0d checks)",checks);$finish;end
endmodule
