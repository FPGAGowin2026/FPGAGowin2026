`timescale 1ns/1ps
module tb_ov7725_dvp;
    reg clk=0,pclk=0,rst_n=0,vsync=1,href=0;reg[7:0]data=0;reg cfg_valid=0;reg[7:0]cfg_addr=0,cfg_data=0;wire cfg_ready,cfg_done,cfg_error,sioc;tri siod;reg slave_low=0;assign siod=slave_low?1'b0:1'bz;pullup(siod);
    wire frame_active,pixel_valid;wire[15:0]pixel,x,y;wire fs,fe,partial;integer checks=0;reg[7:0]seen[0:2];reg[7:0]shift;integer bi,bj;
    always#5 clk=~clk;always#7 pclk=~pclk;
    ov7725_dvp_capture u_cap(.pclk(pclk),.rst_n(rst_n),.vsync(vsync),.href(href),.data_i(data),.frame_active(frame_active),.pixel_valid(pixel_valid),.pixel_rgb565(pixel),.pixel_x(x),.line_y(y),.frame_start(fs),.frame_end(fe),.partial_pixel_error(partial));
    ov7725_sccb_write #(.CLK_DIV(2)) u_cfg(.clk(clk),.rst_n(rst_n),.cfg_valid(cfg_valid),.cfg_addr(cfg_addr),.cfg_data(cfg_data),.cfg_ready(cfg_ready),.cfg_done(cfg_done),.cfg_error(cfg_error),.sioc(sioc),.siod(siod));
    task check;input c;input[255:0]m;begin checks=checks+1;if(!c)begin $display("FAIL: %0s",m);$fatal(1);end end endtask
    task send_pixel_byte;input[7:0]v;begin data=v;@(posedge pclk);#1;end endtask
    task acknowledge_and_capture;begin
        wait(sioc===1'b1&&siod===1'b0);
        for(bi=0;bi<3;bi=bi+1)begin shift=0;for(bj=0;bj<8;bj=bj+1)begin @(posedge sioc);#1;shift={shift[6:0],siod};end seen[bi]=shift;@(negedge sioc);slave_low=1;@(posedge sioc);#1;slave_low=0;end
    end endtask
    initial begin repeat(4)@(posedge clk);rst_n=1;repeat(2)@(posedge pclk);@(negedge pclk);vsync=0;href=1;send_pixel_byte(8'hF8);send_pixel_byte(8'h00);check(pixel_valid&&pixel==16'hF800,"RGB565 pixel captured high-byte first");href=0;@(posedge pclk);@(negedge pclk);href=1;send_pixel_byte(8'hAA);href=0;@(posedge pclk);#1;check(partial,"odd byte at line end is reported");
        fork acknowledge_and_capture();begin @(negedge clk);cfg_addr=8'h12;cfg_data=8'h80;cfg_valid=1;@(negedge clk);cfg_valid=0;wait(cfg_done);end join
        #1;check(seen[0]==8'h42&&seen[1]==8'h12&&seen[2]==8'h80,"SCCB emits device/register/data bytes");check(!cfg_error,"ACKed SCCB write succeeds");
        @(negedge clk);cfg_addr=1;cfg_data=2;cfg_valid=1;@(negedge clk);cfg_valid=0;wait(!cfg_done);wait(cfg_done);#1;check(cfg_error,"NACK is surfaced to caller");
        $display("PASS: ov7725_dvp (%0d checks)",checks);$finish;end
endmodule
