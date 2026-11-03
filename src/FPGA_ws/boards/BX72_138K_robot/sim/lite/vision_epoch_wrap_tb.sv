`timescale 1ns/1ps
// 测试生产核心内的候选 epoch 模块；通过层级赋值构造32位回绕初始条件。
module vision_epoch_wrap_tb;
    reg clk=0,rst=1,valid=0,reuse=1;
    reg [47:0] velocity=0;
    reg [31:0] output_sequence=0;
    wire [31:0] sequence_value;
    wire [47:0] velocity_value;
    wire current;
    integer i;
    always #5 clk=~clk;
    bx72_manual_command_epoch dut(
        .clk(clk),.rst(rst),.command_valid_i(valid),.velocity_i(velocity),
        .reuse_allowed_i(reuse),.task_activity_i(1'b0),
        .output_sequence_i(output_sequence),.sequence_o(sequence_value),
        .velocity_o(velocity_value),.output_current_o(current));
    task automatic send(input signed [15:0] w);
        begin @(negedge clk);valid=1;velocity={32'd0,w};@(negedge clk);valid=0;end
    endtask
    task automatic expect_current(input bit expected,input [511:0] label);
        begin #1;if(current!==expected)$fatal(1,"%0s current=%b seq=%h effective=%h output=%h",
            label,current,sequence_value,dut.effective_sequence,output_sequence);end
    endtask
    initial begin
        repeat(3)@(negedge clk);rst=0;
        @(negedge clk);
        dut.sequence_o=32'hfffffffd;
        dut.effective_sequence=32'hfffffffd;
        dut.velocity_o=0;
        output_sequence=32'd0;
        send(0);expect_current(0,"first zero not confirmed");
        output_sequence=sequence_value;expect_current(1,"zero confirmed");
        send(-16'sd236);expect_current(0,"first nonzero requires CPU");
        if(sequence_value!==32'hffffffff)$fatal(1,"fixture not at wrap boundary");
        output_sequence=sequence_value;expect_current(1,"nonzero confirmed");
        send(-16'sd238);expect_current(1,"changed nonzero wraps to zero");
        if(sequence_value!==0)$fatal(1,"raw counter did not wrap");
        for(i=0;i<100;i=i+1)begin
            send(i[0] ? -16'sd236 : -16'sd238);
            expect_current(1,"fast changes preserve confirmed epoch");
        end
        output_sequence=sequence_value+1;expect_current(0,"future sequence is rejected");
        output_sequence=32'hffffffff;expect_current(1,"old commit in current epoch remains accepted");
        send(0);expect_current(0,"stop fences pre-wrap motion output");
        output_sequence=32'hffffffff;expect_current(0,"late old commit cannot revive stop");
        send(-16'sd236);expect_current(0,"new activation needs fresh output");
        output_sequence=sequence_value;expect_current(1,"new activation confirmed");
        send(16'sd236);expect_current(1,"nonzero direction reversal stays in active epoch");
        output_sequence=dut.effective_sequence;
        reuse=0;expect_current(0,"source invalidation blocks old epoch");
        reuse=1;
        @(negedge clk);dut.sequence_o=dut.effective_sequence+32'h80000000;
        expect_current(0,"ambiguous half-range sequence distance is rejected");
        $display("PASS: wrap, fast variation, zero fence, reverse and half-range bounds");
        $finish;
    end
    initial begin #100000;$fatal(1,"epoch wrap timeout");end
endmodule
