`timescale 1ns/1ps
// 使用既有可控CPU MMIO模型，验证真实核心的栅格寄存器窗口和握手。
module local_grid_mmio_tb;
    reg clk=0,loc_clk=0,reset_n=0,ready=0;
    always #12.5 clk=~clk;
    always #15 loc_clk=~loc_clk;
    wire valid;
    wire [1663:0] payload;
    reg [1663:0] captured;
    integer commits=0,i;
    bx72_lite_robot_core dut(
        .control_clk(clk),.loc_clk33m(loc_clk),.reset_n(reset_n),
        .n10_uart_rx(1'b1),.imu_miso(1'b0),.imu_cs_n(),.imu_sclk(),.imu_mosi(),
        .encoder_a_i(4'd0),.encoder_b_i(4'd0),.motor_in1_o(),.motor_in2_o(),
        .command_valid_i(1'b0),.command_link_down_i(1'b0),.command_heartbeat_i(1'b0),
        .command_velocity_i(48'd0),.auto_mode_valid_i(1'b0),.auto_enable_i(1'b0),
        .task_fragment_valid_i(1'b0),.task_fragment_i(256'd0),.command_token_i(64'd0),
        .local_grid_valid_o(valid),.local_grid_payload_o(payload),.local_grid_ready_i(ready));
    always @(posedge clk)if(valid)begin commits=commits+1;captured=payload;end

    task automatic store(input [11:0] addr,input [31:0] data,input [3:0] strobe);
        begin
            @(negedge clk);dut.cpu.address=addr;dut.cpu.data=data;
            dut.cpu.strobe=strobe;dut.cpu.valid=1;dut.cpu.writing=1;
            @(negedge clk);dut.cpu.valid=0;dut.cpu.writing=0;dut.cpu.strobe=15;
        end
    endtask
    task automatic expect_ready(input expected);
        begin
            @(negedge clk);dut.cpu.address=12'h6d4;#1;
            if(dut.cpu.io_rdata_i!=={31'd0,expected})$fatal(1,"grid MMIO ready mismatch");
        end
    endtask
    initial begin
        repeat(8)@(negedge clk);reset_n=1;
        repeat(5)@(negedge clk);expect_ready(0);
        store(12'h600,32'hdeadbeef,15);
        if(payload!==0)$fatal(1,"busy grid window was overwritten");
        ready=1;expect_ready(1);
        for(i=0;i<52;i=i+1)store(12'h600+i*4,32'h12340000+i,15);
        store(12'h600,32'hffffffff,1);
        store(12'h6d0,0,15);
        if(commits!=0)$fatal(1,"invalid grid commit accepted");
        store(12'h6d0,32'h42583732,15);
        if(commits!=1)$fatal(1,"valid grid commit missing");
        for(i=0;i<52;i=i+1)
            if(captured[1663-i*32-:32]!==32'h12340000+i)$fatal(1,"grid word %0d swapped or corrupted",i);
        ready=0;expect_ready(0);
        store(12'h600,32'hffffffff,15);store(12'h6d0,32'h42583732,15);
        if(commits!=1||payload!==captured)$fatal(1,"busy grid commit or payload changed");
        reset_n=0;repeat(5)@(negedge clk);
        if(payload!==0||valid!==0)$fatal(1,"grid reset did not clear window");
        $display("PASS local_grid_mmio_tb: 52 words, exact payload, strobe, magic, backpressure and reset");
        $finish;
    end
    initial begin #1000000;$fatal(1,"grid MMIO timeout");end
endmodule
