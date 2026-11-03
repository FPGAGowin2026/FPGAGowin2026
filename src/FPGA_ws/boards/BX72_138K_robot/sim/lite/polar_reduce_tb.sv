`timescale 1ns/1ps
module polar_reduce_tb;
    reg clk=0,rst=1,sample=0,hit=0,error=0;
    reg [15:0] angle=0,distance=0;
    reg [7:0] strength=0;
    reg [31:0] stamp=0;
    wire valid;
    wire [72:0] record_data;
    integer count=0;
    always #12.5 clk=~clk;
    bx72_polar_reduce dut(.clk(clk),.rst(rst),.sample_i(sample),.hit_i(hit),
        .error_i(error),.angle_i(angle),.distance_i(distance),
        .strength_i(strength),.stamp_i(stamp),.valid_o(valid),.record_o(record_data));
    task automatic point(input [15:0] a,d,input h,input [31:0] t);
        begin
            @(negedge clk); sample=1;angle=a;distance=d;hit=h;stamp=t;strength=8'd40;
            @(negedge clk);sample=0;#1;
        end
    endtask
    always @(posedge clk) if(valid) count=count+1;
    initial begin
        repeat(4)@(negedge clk);rst=0;
        point(100,1800,1,10);point(200,300,1,20);point(300,800,1,30);
        if(count!=0)$fatal(1,"unfinished interval was published");
        point(501,2200,1,40);
        if(!valid || record_data!={32'd20,1'b1,8'd40,16'd200,16'd300})
            $fatal(1,"nearest return lost or capture timestamp changed");
        point(1000,0,0,50);point(1400,0,0,60);point(1500,900,1,70);
        if(!valid || record_data[40] || record_data[15:0]!=0)
            $fatal(1,"no return became free-space evidence");
        @(negedge clk);error=1;
        @(negedge clk);error=0;
        point(2000,400,1,80);
        if(valid)$fatal(1,"error-contaminated interval published");
        point(35900,600,1,90);point(100,1000,1,100);
        if(!valid || record_data[31:16]!=35900)$fatal(1,"angle wrap lost interval");
        repeat(2)@(negedge clk);
        $display("PASS polar_reduce: nearest/timestamp/unknown/error/wrap");$finish;
    end
    initial begin #100000;$fatal(1,"timeout");end
endmodule
