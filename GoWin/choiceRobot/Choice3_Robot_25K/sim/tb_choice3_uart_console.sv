`timescale 1ns/1ps

module tb_choice3_uart_console;
    localparam integer CLKS_PER_BIT=10;
    reg clk=0,rst_n=0,uart_rx=1;
    reg armed=0,drive=0,arm_rejected=0;
    reg [7:0] health=8'h1f,fault=8'h04;
    reg [7:0] vlraw=8'h09;
    reg [3:0] nav=4'h2;
    wire uart_tx,arm_pulse,disarm_pulse,clear_pulse,route_pulse;
    wire manual_valid;wire [2:0] manual_motion;
    wire [7:0] monitor_data;
    wire monitor_valid,monitor_error;
    byte received[0:127];
    integer received_count=0,errors=0;
    bit saw_arm=0,saw_disarm=0,saw_clear=0,saw_route=0;
    bit saw_manual_forward=0;
    always #5 clk=~clk;

    choice3_uart_console #(.CLK_FREQ_HZ(1000),.BAUD_RATE(100)) dut(
        .clk_i(clk),.rst_n_i(rst_n),.uart_rx_i(uart_rx),.uart_tx_o(uart_tx),
        .armed_i(armed),.drive_enabled_i(drive),.health_i(health),
        .fault_i(fault),.nav_state_i(nav),
        .vl53_range_status_raw_i(vlraw),
        .arm_rejected_pulse_i(arm_rejected),.arm_pulse_o(arm_pulse),
        .disarm_pulse_o(disarm_pulse),.clear_fault_pulse_o(clear_pulse),
        .route_start_pulse_o(route_pulse),.manual_command_valid_o(manual_valid),
        .manual_motion_o(manual_motion));
    bringup_uart_rx #(.CLK_FREQ_HZ(1000),.BAUD_RATE(100)) u_monitor(
        .clk(clk),.rst_n(rst_n),.rx_i(uart_tx),.data_o(monitor_data),
        .data_valid_o(monitor_valid),.frame_error_o(monitor_error));

    always @(posedge clk) begin
        if(monitor_valid) begin received[received_count]=monitor_data;received_count=received_count+1;end
        if(arm_pulse)saw_arm=1;if(disarm_pulse)saw_disarm=1;
        if(clear_pulse)saw_clear=1;if(route_pulse)saw_route=1;
        if(manual_valid&&manual_motion==1)saw_manual_forward=1;
        if(monitor_error) begin $display("FAIL: UART TX 停止位错误");errors=errors+1;end
    end
    task automatic check(input bit condition,input string message);
        if(!condition)begin $display("FAIL: %s",message);errors=errors+1;end
    endtask
    task automatic send_byte(input byte value);
        integer i;
        begin
            @(negedge clk);uart_rx=0;repeat(CLKS_PER_BIT)@(negedge clk);
            for(i=0;i<8;i=i+1)begin uart_rx=value[i];repeat(CLKS_PER_BIT)@(negedge clk);end
            uart_rx=1;repeat(CLKS_PER_BIT)@(negedge clk);repeat(2)@(negedge clk);
        end
    endtask
    task automatic wait_bytes(input integer amount);
        integer guard;
        begin guard=0;while(received_count<amount && guard<20000)begin @(posedge clk);guard=guard+1;end
            check(received_count>=amount,"等待 UART 响应超时");end
    endtask
    task automatic check_ascii(input integer index,input byte expected,input string label);
        begin check(received[index]===expected,$sformatf("%s[%0d] 期望 %02x 实际 %02x",label,index,expected,received[index]));end
    endtask

    initial begin
        repeat(5)@(posedge clk);rst_n=1;
        // 上电先输出动态状态行。
        wait_bytes(53);
        check_ascii(0,"S","上电状态");check_ascii(6,";","上电状态");
        check_ascii(11,"0","ARM字段");check_ascii(20,"1","HEALTH高位");
        check_ascii(21,"F","HEALTH低位");check_ascii(29,"0","FAULT高位");
        check_ascii(30,"4","FAULT低位");check_ascii(36,"2","NAV字段");
        check_ascii(43,"0","VLRS高位");check_ascii(44,"9","VLRS低位");
        check_ascii(50,"0","DRV字段");
        check_ascii(51,8'h0d,"CR");check_ascii(52,8'h0a,"LF");

        received_count=0;send_byte("P");wait_bytes(20);
        check_ascii(0,"P","PONG");check_ascii(4,";","PONG");
        check_ascii(5,"C","PONG");check_ascii(17,"Y","PONG");
        check_ascii(18,8'h0d,"PONG CR");check_ascii(19,8'h0a,"PONG LF");

        received_count=0;send_byte("A");wait_bytes(9);
        check(saw_arm,"A 命令未产生单拍 arm");check_ascii(4,"A","ACK ARM");
        received_count=0;send_byte("D");wait_bytes(12);
        check(saw_disarm,"D 命令未产生单拍 disarm");
        received_count=0;send_byte("G");wait_bytes(14);
        check(saw_route,"G 命令未产生单拍 route_start");
        received_count=0;send_byte("X");wait_bytes(17);
        check(saw_clear,"X 命令未产生单拍 clear_fault");
        received_count=0;send_byte("F");wait_bytes(12);
        check(saw_manual_forward,"F 命令未产生手动前进动作");

        // 状态发送前快照，验证动态十六进制字段。
        armed=1;drive=1;health=8'ha5;fault=8'hc3;nav=4'he;vlraw=8'hb7;
        received_count=0;send_byte("S");wait_bytes(53);
        check_ascii(11,"1","动态 ARM");check_ascii(20,"A","动态 HEALTH");
        check_ascii(21,"5","动态 HEALTH");check_ascii(29,"C","动态 FAULT");
        check_ascii(30,"3","动态 FAULT");check_ascii(36,"E","动态 NAV");
        check_ascii(43,"B","动态 VLRS");check_ascii(44,"7","动态 VLRS");
        check_ascii(50,"1","动态 DRV");

        check((^{arm_pulse,disarm_pulse,clear_pulse,route_pulse,uart_tx})!==1'bx,
              "UART 控制输出出现 X/Z");
        if(errors==0)$display("PASS: tb_choice3_uart_console");
        else begin $display("FAIL: tb_choice3_uart_console errors=%0d",errors);$fatal(1);end
        $finish;
    end
    initial begin #10_000_000;$display("FAIL: UART TB timeout");$fatal(1);end
endmodule
