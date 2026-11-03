`timescale 1ns/1ps
// 40MHz采样微秒时标输入；只驱动接口，不改内部计数或寄存状态。
module nav_full_freshness_tb;
    reg clk=0,rst=1,valid=0;
    reg [31:0] now_us=0,last_us=0;
    wire [31:0] age_us;
    wire fresh;
    always #12.5 clk=~clk;
    bx72_deadline_age #(.TIMEOUT_US(100000)) dut(
        .clk(clk),.rst(rst),.now_us_i(now_us),.last_us_i(last_us),
        .valid_i(valid),.age_us_o(age_us),.fresh_o(fresh));
    task automatic settle;
        begin repeat(4)@(negedge clk);#1;end
    endtask
    initial begin
        settle();
        if(fresh)$fatal(1,"复位未闭锁");
        rst=0;last_us=1000000;now_us=1000010;valid=1;
        settle();
        if(!fresh||age_us!=10)$fatal(1,"新鲜时标未放行");
        now_us=last_us+99998;settle();
        if(!fresh)$fatal(1,"提前边界前错误停车");
        now_us=last_us+99999;settle();
        if(fresh)$fatal(1,"超时边界未保守停车");
        last_us=now_us;settle();
        if(!fresh)$fatal(1,"新采集时标未恢复");
        valid=0;#1;
        if(fresh)$fatal(1,"valid撤销未立即闭锁");
        settle();
        last_us=32'hfffffff0;now_us=32'h00000020;valid=1;settle();
        if(!fresh||age_us!=48)$fatal(1,"微秒32位回绕处理错误");
        now_us=last_us+32'd99999;settle();
        if(fresh)$fatal(1,"跨回绕后超时失效错误");
        rst=1;settle();
        if(fresh)$fatal(1,"运行中复位未闭锁");
        $display("PASS: nav_full_freshness 40MHz timeout=100ms early_margin=1us wrap=32bit valid_drop=immediate");
        $finish;
    end
    initial begin #10000;$fatal(1,"新鲜度测试超时");end
endmodule
