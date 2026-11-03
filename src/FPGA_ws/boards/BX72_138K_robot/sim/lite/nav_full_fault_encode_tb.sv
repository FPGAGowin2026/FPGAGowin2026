`timescale 1ns/1ps
// 单独验证生产错误编码器；此测试不代替真实CPU或运动保护验证。
module nav_full_fault_encode_tb;
    reg [12:0] faults;
    reg guard_stop;
    wire [7:0] error;
    integer i,j;
    bx72_navigation_fault_encode dut(.faults_i(faults),
        .guard_stop_i(guard_stop),.error_o(error));
    initial begin
        #1;faults=0;guard_stop=0;
        #1;if(error!==0)$fatal(1,"healthy state reported fault");
        for(i=0;i<13;i=i+1)begin
            faults=13'b1<<i;
            #1;if(error!==(8'h41+i))$fatal(1,"unarmed real fault %0d hidden",i);
            for(j=i+1;j<13;j=j+1)begin
                faults=(13'b1<<i)|(13'b1<<j);
                #1;if(error!==(8'h41+i))$fatal(1,"fault priority %0d/%0d",i,j);
            end
        end
        faults=0;guard_stop=1;
        #1;if(error!==8'h4e)$fatal(1,"guard stop hidden");
        faults=13'b1;
        #1;if(error!==8'h41)$fatal(1,"CPU fault must retain priority over guard error");
        faults=0;
        guard_stop=0;
        #1;if(error!==0)$fatal(1,"healthy state reported fault");
        $display("PASS: nav_full_fault_encode_tb");
        $finish;
    end
endmodule
