`timescale 1ns/1ps
// 使用真实核心接线和既有shared_lidar_queue_tb的可控MMIO平台替身。
// 覆盖取消等终态仍展示时，相同手动命令只刷新租约，不反复失去CPU确认。
module manual_handoff_terminal_tb;
    reg clk=0,loc_clk=0,reset_n=0;
    always #12.5 clk=~clk;
    always #15 loc_clk=~loc_clk;
    reg command_valid=0,mode_valid=0,task_valid=0;
    reg [47:0] velocity=0;
    reg [31:0] acknowledged;
    integer i;
    bx72_lite_robot_core dut(
        .control_clk(clk),.loc_clk33m(loc_clk),.reset_n(reset_n),
        .n10_uart_rx(1'b1),.imu_miso(1'b0),.imu_cs_n(),.imu_sclk(),.imu_mosi(),
        .encoder_a_i(4'd0),.encoder_b_i(4'd0),.motor_in1_o(),.motor_in2_o(),
        .command_valid_i(command_valid),.command_link_down_i(1'b0),.command_heartbeat_i(1'b0),
        .command_velocity_i(velocity),.auto_mode_valid_i(mode_valid),.auto_enable_i(1'b0),
        .task_fragment_valid_i(task_valid),.task_fragment_i(256'd0),.command_token_i(64'd0),
        .navigation_valid_o(),.navigation_payload_o(),.local_plan_valid_o(),.local_plan_payload_o(),
        .auto_mode_o(),.fusion_valid_o(),.imu_valid_o(),.motor_valid_o(),.lidar_valid_o(),
        .imu_payload_o(),.motor_payload_o(),.lidar_payload_o(),.fusion_payload_o(),.sensor_health_o(),
        .cpu_ready_o(),.cpu_fault_o(),.cpu_busy_o(),.control_reset_o(),.cpu_fault_code_o());

    task automatic store(input [11:0] address,input [31:0] value);
        begin
            @(negedge clk);dut.cpu.address=address;dut.cpu.data=value;
            dut.cpu.valid=1;dut.cpu.writing=1;
            @(negedge clk);dut.cpu.valid=0;dut.cpu.writing=0;
        end
    endtask
    task automatic send(input [47:0] value);
        begin
            @(negedge clk);velocity=value;command_valid=1;
            @(negedge clk);command_valid=0;
        end
    endtask
    task automatic confirm;
        begin
            store(12'h200,$signed(dut.remote_velocity[47:32]));
            store(12'h204,$signed(dut.remote_velocity[31:16]));
            store(12'h208,$signed(dut.remote_velocity[15:0]));
            store(12'h2a4,dut.command_sequence);
            store(12'h2a8,dut.mode_sequence);
            store(12'h2a0,32'd0);
            store(12'h20c,32'h42583732);
            acknowledged=dut.command_sequence;
        end
    endtask
    task automatic expect_current(input expected,input [383:0] reason);
        begin
            #1;
            if(dut.command_output_current!==expected)
                $fatal(1,"%0s: status=%0d sequence=%0d acknowledged=%0d current=%b",reason,
                       dut.nav_status[7:0],dut.command_sequence,dut.output_command_seq,dut.command_output_current);
        end
    endtask
    task automatic terminal_refresh(input [7:0] status);
        begin
            store(12'h230,{24'd0,status});
            send(48'd0);confirm();expect_current(1,"CPU confirmed stop");
            send({16'd0,16'd60,16'd0});expect_current(0,"new manual motion needs CPU");
            confirm();expect_current(1,"CPU confirmed manual takeover");
            for(i=0;i<4;i=i+1)begin
                send({16'd0,16'd60,16'd0});
                if(dut.command_sequence!==acknowledged+i+1)$fatal(1,"raw sequence not advanced");
                expect_current(1,"terminal display must not reset confirmed manual epoch");
            end
        end
    endtask
    initial begin
        repeat(5)@(negedge clk);reset_n=1;repeat(6)@(negedge clk);
        // 只隔离传感器/时间输入；命令序号、终态和CPU写回走生产核心。
        force dut.manual_fresh=1'b1;
        force dut.link_down=1'b0;
        terminal_refresh(8'd7);
        terminal_refresh(8'd0);
        terminal_refresh(8'd4);
        terminal_refresh(8'd6);
        terminal_refresh(8'd8);

        send({16'd0,16'd70,16'd0});expect_current(1,"changed nonzero speed retains confirmed epoch");
        confirm();expect_current(1,"changed speed acknowledged");
        send(48'd0);expect_current(0,"fresh stop invalidates nonzero output");
        confirm();expect_current(1,"stop acknowledged");
        send(48'd0);expect_current(1,"repeated acknowledged stop");

        // 模式、断链、人工租约失效仍阻止复用；恢复输入不能复活旧确认。
        @(negedge clk);mode_valid=1;
        send(48'd0);expect_current(0,"mode transition needs CPU");
        @(negedge clk);mode_valid=0;
        send(48'd0);
        confirm();expect_current(1,"mode transition acknowledged");
        send({16'd0,16'd60,16'd0});confirm();
        force dut.link_down=1'b1;repeat(3)@(negedge clk);
        expect_current(0,"link loss invalidates even exact acknowledged output");
        force dut.link_down=1'b0;repeat(3)@(negedge clk);
        expect_current(0,"heartbeat-only link return cannot revive old output");
        confirm();expect_current(0,"CPU repeat without new command cannot revive old output");
        send(48'd0);expect_current(0,"fresh post-link command still requires CPU");
        confirm();expect_current(1,"link recovery acknowledged");
        send({16'd0,16'd60,16'd0});confirm();
        force dut.manual_fresh=1'b0;repeat(3)@(negedge clk);
        expect_current(0,"lease loss invalidates exact acknowledged output");
        force dut.manual_fresh=1'b1;repeat(3)@(negedge clk);
        expect_current(0,"lease return cannot revive old output");
        confirm();expect_current(0,"CPU repeat without new intent cannot revive expired output");
        send(48'd0);expect_current(0,"fresh post-expiry command still requires CPU");
        confirm();expect_current(1,"lease recovery acknowledged");
        $display("PASS: confirmed motion survives nonzero updates; boundaries require fresh intent and CPU");
        $finish;
    end
    initial begin #1000000;$fatal(1,"manual handoff test timeout");end
endmodule
