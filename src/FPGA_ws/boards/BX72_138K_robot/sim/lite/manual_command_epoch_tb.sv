`timescale 1ns/1ps
// 验证活动命令确认逻辑：重复零速可复用已确认输出，新停车仍须CPU确认。
module manual_command_epoch_tb;
    reg clk=0, rst=1, command_valid=0, reuse_allowed=0;
    reg [47:0] velocity=0;
    reg [31:0] output_sequence=0;
    wire [31:0] seq_value;
    wire [47:0] accepted_velocity;
    wire output_current;
    always #5 clk=~clk;

    bx72_manual_command_epoch dut(
        .clk(clk),.rst(rst),.command_valid_i(command_valid),.velocity_i(velocity),
        .reuse_allowed_i(reuse_allowed),.task_activity_i(1'b0),
        .output_sequence_i(output_sequence),.sequence_o(seq_value),
        .velocity_o(accepted_velocity),.output_current_o(output_current));

    task automatic send(input [47:0] value);
        begin
            @(negedge clk); velocity=value;command_valid=1;
            @(negedge clk); command_valid=0;
        end
    endtask
    task automatic expect_current(input expected, input [255:0] label);
        begin
            #1;
            if(output_current!==expected) $fatal(1,"%0s: current=%b expected=%b",label,output_current,expected);
        end
    endtask

    initial begin
        repeat(2) @(negedge clk);rst=0;
        send(48'd0);
        expect_current(0,"initial zero needs CPU");
        output_sequence=seq_value;expect_current(1,"zero acknowledged");
        send({16'd200,16'd0,16'd0});
        expect_current(0,"new motion needs CPU");
        output_sequence=seq_value;reuse_allowed=1;expect_current(1,"motion acknowledged");
        send({16'd200,16'd0,16'd0});
        if(seq_value!==32'd3) $fatal(1,"repeat must still advance the raw command sequence");
        expect_current(1,"same live command reuses output");
        send({16'd200,16'd0,16'd0});
        if(seq_value!==32'd4) $fatal(1,"each refresh must reach firmware arbitration");
        expect_current(1,"second refresh reuses output");
        output_sequence=seq_value+1;expect_current(0,"future output cannot be accepted");
        output_sequence=32'd2;
        send({16'd100,16'd0,16'd0});
        expect_current(1,"changed nonzero motion retains current epoch");
        output_sequence=seq_value;expect_current(1,"changed motion acknowledged");
        send(48'd0);
        expect_current(0,"stop needs CPU");
        output_sequence=seq_value;expect_current(1,"stop acknowledged");
        send(48'd0);expect_current(1,"idle zero reuses acknowledged stop");
        output_sequence=seq_value;
        send({16'd200,16'd0,16'd0});output_sequence=seq_value;
        send({16'd200,16'd0,16'd0});expect_current(1,"live refresh");
        reuse_allowed=0;expect_current(0,"expiry or mode change blocks reuse");
        send({16'd200,16'd0,16'd0});expect_current(0,"new epoch after expiry needs CPU");
        output_sequence=seq_value;expect_current(1,"new epoch acknowledged");

        // 重复零速不能借用之前的非零输出，也不能接受未来序号。
        reuse_allowed=1;
        send(48'd0);expect_current(0,"new stop invalidates motion immediately");
        send(48'd0);expect_current(0,"repeated unacknowledged stop stays blocked");
        output_sequence=seq_value+1;expect_current(0,"future zero output is rejected");
        output_sequence=seq_value-1;expect_current(1,"first acknowledged zero covers refresh");
        send(48'd0);expect_current(1,"steady idle zero does not invalidate output");

        // 原始序号仍逐包递增；失效租约禁止复用空闲零速。
        output_sequence=seq_value;
        send(48'd0);
        if(seq_value!==output_sequence+1) $fatal(1,"idle refresh must advance raw sequence");
        expect_current(1,"idle refresh remains current");
        reuse_allowed=0;expect_current(0,"expired idle lease blocks reuse");
        send(48'd0);expect_current(0,"idle refresh after expiry needs CPU");
        reuse_allowed=1;expect_current(0,"restored lease cannot revive old zero output");
        output_sequence=seq_value;expect_current(1,"fresh idle output acknowledged");
        send(48'd0);expect_current(1,"fresh idle refresh reuses zero");
        $display("PASS: manual command epoch reuses acknowledged idle zeros and preserves new stops and expiry");
        $finish;
    end
endmodule
