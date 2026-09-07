`timescale 1ns/1ps
module tb_choice3_attitude_osd;
    reg clk=0,rst_n=0,sample_valid=0;
    reg signed [15:0] roll_raw=0,pitch_raw=0,yaw_raw=0;
    wire signed [15:0] roll_deg,pitch_deg,yaw_deg;wire angle_valid;
    reg [9:0] x=0,y=0;wire hit;wire [7:0] r,g,b;
    integer errors=0;
    always #5 clk=~clk;
    choice3_attitude_degrees u_deg(.clk_i(clk),.rst_n_i(rst_n),
      .sample_valid_i(sample_valid),.roll_raw_i(roll_raw),.pitch_raw_i(pitch_raw),
      .yaw_raw_i(yaw_raw),.roll_deg_o(roll_deg),.pitch_deg_o(pitch_deg),
      .yaw_deg_o(yaw_deg),.valid_o(angle_valid));
    choice3_attitude_osd u_osd(.clk_i(clk),.rst_n_i(rst_n),.x_i(x),.y_i(y),.roll_deg_i(roll_deg),
      .pitch_deg_i(pitch_deg),.yaw_deg_i(yaw_deg),.valid_i(1'b1),
      .hit_o(hit),.red_o(r),.green_o(g),.blue_o(b));
    task check(input bit cond,input string msg);if(!cond)begin $display("FAIL: %s",msg);errors=errors+1;end endtask
    initial begin
        repeat(3)@(posedge clk);rst_n=1;
        roll_raw=16'sh4000;pitch_raw=-16'sh2000;yaw_raw=16'sh7fff;
        @(negedge clk);sample_valid=1;@(negedge clk);sample_valid=0;@(posedge clk);#1;
        check(angle_valid==0,"valid 应为单拍脉冲");
        check(roll_deg==90,"roll 16384 应换算为 90 度");
        check(pitch_deg==-45,"pitch -8192 应换算为 -45 度");
        check(yaw_deg==179,"yaw 32767 的整数度应为 179");
        @(negedge clk);x=10;y=24;@(posedge clk);#1;
        check(hit&&r==8'hff&&g==0&&b==0,"R 标签像素/颜色错误");
        @(negedge clk);x=8;y=48;@(posedge clk);#1;
        check(hit&&r==0&&g==8'hff&&b==0,"P 标签像素/颜色错误");
        @(negedge clk);x=8;y=72;@(posedge clk);#1;
        check(hit&&r==0&&g==8'hff&&b==8'hff,"Y 标签像素/颜色错误");
        @(negedge clk);x=20;y=54;@(posedge clk);#1;check(hit,"负号没有显示");
        if(errors==0)$display("PASS: tb_choice3_attitude_osd");
        else begin $display("FAIL: tb_choice3_attitude_osd errors=%0d",errors);$fatal(1);end
        $finish;
    end
endmodule
