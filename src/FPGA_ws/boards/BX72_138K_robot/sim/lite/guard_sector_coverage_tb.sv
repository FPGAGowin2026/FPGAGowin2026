`timescale 1ns/1ps
module guard_sector_coverage_tb;
    reg clk=0, rst=1, sample=0, hit=0, error=0;
    reg [15:0] angle=0, distance=0;
    wire stop, fresh;
    always #500 clk=~clk;
    bx72_lite_guard #(.CLK_HZ(1000000)) dut(
        .clk(clk),.rst(rst),.sample_valid_i(sample),.point_valid_i(hit),
        .angle_cdeg_i(angle),.distance_mm_i(distance),.packet_error_i(error),
        .command_valid_i(1'b1),.command_velocity_i({16'd100,16'd0,16'd0}),.kill_i(1'b0),
        .command_valid_o(),.command_velocity_o(),.stop_o(stop),.slow_o(),.lidar_fresh_o(fresh));

    task automatic point(input integer a,input integer h,input integer d);
        begin
            @(negedge clk);sample=1;hit=h;angle=a;distance=d;
            @(negedge clk);sample=0;hit=0;
        end
    endtask
    task automatic packet_error;
        begin @(negedge clk);error=1;@(negedge clk);error=0;end
    endtask
    // kind: 0 full; 1 missing rear; 2 missing inside sector 0;
    // 3 missing inside sector 7; 4 missing across zero;
    // 5 sector 0 no echo; 6 one no echo; 7 reported packet error;
    // 8 only the complete front sectors; 9 an error in the rear;
    // 10 packet error in a 3.1-degree gap just across the 0/1 boundary.
    task automatic scan(input integer kind);
        integer a;
        begin
            for(a=0;a<36000;a=a+100) begin
                if (!((kind==1 && a>=18000 && a<22500) ||
                      (kind==2 && a>=900 && a<2000) ||
                      (kind==3 && a>=33500 && a<34700) ||
                      (kind==4 && (a<600 || a>=35500)) ||
                      (kind==8 && a>=4500 && a<31500) ||
                      (kind==10 && a>=4500 && a<4800))) begin
                    if((kind==7 && a==1000) || (kind==9 && a==19000))packet_error();
                    point(a, !((kind==5 && a<4500) || (kind==6 && a==1000)), 2000);
                    if(kind==10 && a==4400)begin point(4490,1,2000);packet_error();end
                end
            end
        end
    endtask
    task automatic scans(input integer kind,input integer count);
        integer n;
        begin for(n=0;n<count;n=n+1)scan(kind);point(0,1,2000);repeat(8)@(negedge clk);end
    endtask
    task automatic check_stopped(input integer expected,input [255:0] label);
        begin
            repeat(8)@(negedge clk);
            if(stop!==expected || !fresh)
                $fatal(1,"%0s stop=%b fresh=%b distances=%0d/%0d unknown=%h",label,stop,fresh,
                    dut.sector_distance[0],dut.sector_distance[7],dut.sweep_unknown);
        end
    endtask
    initial begin
        repeat(3)@(negedge clk);rst=0;
        scans(0,2);check_stopped(0,"initial far returns");
        point(1000,1,100);check_stopped(1,"front obstacle");
        scans(1,3);check_stopped(0,"missing rear sector");
        if(dut.sector_distance[0]!=2000 || dut.sector_distance[7]!=2000)
            $fatal(1,"complete front sectors did not release independently");
        point(1000,1,100);check_stopped(1,"front-only fixture");
        scans(8,3);check_stopped(0,"only complete front sectors");
        point(1000,1,100);check_stopped(1,"rear-error fixture");
        scans(9,3);check_stopped(0,"error confined to rear");

        point(1000,1,49);check_stopped(1,"sub-50mm return");
        point(34000,1,80);
        scans(2,3);check_stopped(1,"missing inside sector 0");
        if(dut.sector_distance[0]!=49 || dut.sector_distance[7]!=2000)
            $fatal(1,"front gap cleared its near return or blocked the other complete sector");
        scans(0,2);check_stopped(0,"sector 0 recovery");

        point(34000,1,100);check_stopped(1,"sector 7 obstacle");
        scans(3,3);check_stopped(1,"missing inside sector 7");
        if(dut.sector_distance[7]!=100)$fatal(1,"sector 7 gap cleared a near return");
        scans(0,2);check_stopped(0,"sector 7 recovery");

        point(35000,1,100);check_stopped(1,"wrap obstacle");
        scans(4,3);
        // The helper supplies angle zero after the last scan; the last gap into it
        // still exceeds the threshold and must not clear the preceding sector.
        check_stopped(1,"gap across zero");
        scans(0,2);check_stopped(0,"wrap recovery");

        point(1000,1,100);check_stopped(1,"no-echo fixture");
        scans(5,3);check_stopped(1,"whole front sector no echo");
        if(dut.sector_distance[0]!=100)$fatal(1,"no-echo sector replaced old distance");
        scans(6,2);check_stopped(0,"mixed echoes retain existing semantics");

        point(1000,1,100);check_stopped(1,"error fixture");
        scans(7,3);check_stopped(1,"packet error within front sector");
        scans(0,2);check_stopped(0,"packet error recovery");
        point(4495,1,100);check_stopped(1,"boundary error fixture");
        scans(10,3);check_stopped(1,"reported error across sector boundary");
        if(dut.sector_distance[0]!=100)$fatal(1,"boundary packet error cleared a near return");
        scans(0,2);check_stopped(0,"boundary error recovery");
        $display("PASS guard_sector_coverage: rear gap, front gaps, wrap, error, no-echo, near hit");
        $finish;
    end
endmodule
