`timescale 1ns/1ps
// 50 MHz 公共包总线经异步字节/descriptor FIFO 到 125 MHz 的定向测试。
module nav_full_telemetry_packet_cdc_tb;
    reg src_clk=0,tx_clk=0;
    always #10 src_clk=~src_clk;
    always #4 tx_clk=~tx_clk;

    reg reset_n=0,src_reset=1;
    reg packet_valid=0,packet_length_nonzero=0;
    reg [7:0] packet_length=0,packet_data=0;
    wire packet_start,packet_read;
    wire tx_valid,tx_length_nonzero;
    wire [7:0] tx_length,tx_data;
    reg tx_start=0,tx_read=0;

    bx72_telemetry_packet_cdc dut (
        .src_clk(src_clk),.src_reset_i(src_reset),.tx_clk(tx_clk),.reset_n(reset_n),
        .packet_valid_i(packet_valid),.packet_length_nonzero_i(packet_length_nonzero),
        .packet_length_i(packet_length),.packet_data_i(packet_data),
        .packet_start_o(packet_start),.packet_read_o(packet_read),
        .tx_valid_o(tx_valid),.tx_length_nonzero_o(tx_length_nonzero),
        .tx_length_o(tx_length),.tx_data_o(tx_data),
        .tx_start_i(tx_start),.tx_read_i(tx_read)
    );

    function [7:0] pattern_byte;
        input integer packet_id;
        input integer byte_index;
        begin
            pattern_byte=(packet_id*17 + byte_index*13) ^ 8'h5A;
        end
    endfunction

    task automatic send_packet;
        input integer packet_id;
        input integer packet_bytes;
        input integer check_no_early_offer;
        integer index,guard;
        begin
            @(negedge src_clk);
            packet_length=packet_bytes[7:0];
            packet_length_nonzero=(packet_bytes!=0);
            packet_data=pattern_byte(packet_id,0);
            packet_valid=1'b1;
            guard=0;
            begin : wait_for_start
                forever begin
                    @(posedge src_clk);
                    if(packet_start)begin
                        if(packet_read)$fatal(1,"source read coincided with packet start");
                        #1;
                        packet_valid=1'b0;
                        disable wait_for_start;
                    end
                    guard=guard+1;
                    if(guard>20000)$fatal(1,"packet start timeout id=%0d",packet_id);
                end
            end
            index=0;
            while(index<packet_bytes)begin
                @(negedge src_clk);
                packet_data=pattern_byte(packet_id,index);
                if(packet_read)begin
                    @(posedge src_clk); #1;
                    index=index+1;
                    if(check_no_early_offer && index<packet_bytes && tx_valid)
                        $fatal(1,"TX offered before complete source packet id=%0d",packet_id);
                end
            end
        end
    endtask

    task automatic receive_packet;
        input integer packet_id;
        input integer packet_bytes;
        input integer start_read_same_cycle;
        integer index,guard;
        reg [7:0] expected,held_byte;
        begin
            guard=0;
            while(tx_valid!==1'b1 && guard<50000)begin
                @(negedge tx_clk); guard=guard+1;
            end
            if(guard>=50000)$fatal(1,"TX offer timeout id=%0d",packet_id);
            if(tx_length!==packet_bytes[7:0] || tx_length_nonzero!==1'b1)
                $fatal(1,"length descriptor mismatch id=%0d length=%0d",packet_id,tx_length);

            if(start_read_same_cycle!=0)begin
                @(negedge tx_clk);
                tx_start=1'b1;
                tx_read=1'b1;
                for(index=0;index<packet_bytes;index=index+1)begin
                    expected=pattern_byte(packet_id,index);
                    if(tx_data!==expected)
                        $fatal(1,"TX byte mismatch id=%0d byte=%0d got=%02x expected=%02x",
                            packet_id,index,tx_data,expected);
                    @(posedge tx_clk); #1;
                    @(negedge tx_clk);
                    tx_start=1'b0;
                end
                tx_read=1'b0;
            end else begin
                @(negedge tx_clk);
                tx_start=1'b1;
                tx_read=1'b0;
                @(posedge tx_clk); #1;
                @(negedge tx_clk);
                tx_start=1'b0;
                held_byte=tx_data;
                // start 后暂停读 8 个 125 MHz 周期，首字节必须保持稳定。
                repeat(8)begin
                    @(posedge tx_clk); #1;
                    if(tx_data!==held_byte || tx_valid!==1'b0)
                        $fatal(1,"TX head changed during downstream stall");
                    @(negedge tx_clk);
                end
                tx_read=1'b1;
                for(index=0;index<packet_bytes;index=index+1)begin
                    expected=pattern_byte(packet_id,index);
                    if(tx_data!==expected)
                        $fatal(1,"TX byte mismatch id=%0d byte=%0d got=%02x expected=%02x",
                            packet_id,index,tx_data,expected);
                    @(posedge tx_clk); #1;
                    @(negedge tx_clk);
                end
                tx_read=1'b0;
            end
            tx_start=1'b0;
        end
    endtask

    task automatic receive_packet_with_gaps;
        input integer packet_id;
        input integer packet_bytes;
        integer index,guard;
        reg [7:0] expected,held_byte;
        begin
            guard=0;
            while(tx_valid!==1'b1 && guard<50000)begin
                @(negedge tx_clk); guard=guard+1;
            end
            if(guard>=50000 || tx_length!==packet_bytes[7:0] || !tx_length_nonzero)
                $fatal(1,"gapped packet offer/length failure id=%0d",packet_id);
            @(negedge tx_clk); tx_start=1;
            @(posedge tx_clk); #1;
            @(negedge tx_clk); tx_start=0;
            for(index=0;index<packet_bytes;index=index+1)begin
                if(index%3==0)begin
                    held_byte=tx_data;
                    repeat(3)begin
                        @(posedge tx_clk); #1;
                        if(tx_data!==held_byte)$fatal(1,"head changed during active stall");
                        @(negedge tx_clk);
                    end
                end
                expected=pattern_byte(packet_id,index);
                if(tx_data!==expected)
                    $fatal(1,"gapped packet mismatch id=%0d byte=%0d got=%02x expected=%02x",
                           packet_id,index,tx_data,expected);
                tx_read=1;
                @(posedge tx_clk); #1;
                @(negedge tx_clk); tx_read=0;
            end
        end
    endtask

    integer packet_id;
    reg [7:0] burst_first_byte;
    initial begin
        repeat(8)@(negedge tx_clk);
        reset_n=1'b1;
        repeat(4)@(negedge src_clk);
        src_reset=1'b0;

        // 长度标志是输入合同：零长且标志为0时不得接收或读源数据。
        @(negedge src_clk); packet_valid=1; packet_length=0; packet_length_nonzero=0;
        repeat(8)begin
            @(posedge src_clk); #1;
            if(packet_start || packet_read || tx_valid)$fatal(1,"zero-length packet was accepted");
            @(negedge src_clk);
        end
        packet_valid=0;

        // 空闲读和尚未start的读都不消费字节。
        @(negedge tx_clk); tx_start=1; tx_read=1;
        repeat(6)@(negedge tx_clk);
        if(tx_valid)$fatal(1,"idle read created an offer");
        tx_start=0; tx_read=0;

        // 连续单字节包覆盖同拍、分拍及前一包后持续read的边界。
        send_packet(1,1,1);
        receive_packet(1,1,1);
        @(negedge tx_clk); tx_read=1;
        send_packet(4,1,1);
        wait(tx_valid===1'b1);
        burst_first_byte=tx_data;
        repeat(6)begin
            @(posedge tx_clk); #1;
            if(!tx_valid || tx_data!==burst_first_byte)
                $fatal(1,"read without start consumed the next single-byte packet");
            @(negedge tx_clk);
        end
        receive_packet(4,1,1);
        send_packet(5,1,1);
        receive_packet(5,1,0);
        send_packet(6,2,1);
        receive_packet(6,2,1);
        send_packet(7,255,1);
        receive_packet_with_gaps(7,255);
        send_packet(2,48,1);
        receive_packet(2,48,0);
        send_packet(3,160,1);
        receive_packet(3,160,1);

        // 不读取 TX OFFER，连续排入 13 个 160 字节包并验证源端满 FIFO 暂停。
        fork
            begin : burst_producer
                integer producer_id;
                for(producer_id=10;producer_id<23;producer_id=producer_id+1)
                    send_packet(producer_id,160,0);
            end
            begin : burst_consumer
                integer guard,consumer_id;
                guard=0;
                while(!(dut.src_state==1'b1 && dut.byte_full) && guard<50000)begin
                    @(negedge src_clk); guard=guard+1;
                end
                if(guard>=50000)$fatal(1,"source FIFO did not backpressure packet copy");
                if(packet_read!==1'b0 || tx_valid!==1'b1 || tx_length!==8'd160)
                    $fatal(1,"expected copy stall and first complete TX offer");
                burst_first_byte=tx_data;
                repeat(24)begin
                    @(posedge tx_clk); #1;
                    if(!tx_valid || tx_data!==burst_first_byte)
                        $fatal(1,"TX offer changed while downstream was stalled");
                    @(negedge tx_clk);
                end
                for(consumer_id=10;consumer_id<23;consumer_id=consumer_id+1)
                    receive_packet(consumer_id,160,0);
            end
        join
        repeat(20)@(posedge tx_clk);
        if(tx_valid || dut.src_state!==dut.SRC_IDLE)
            $fatal(1,"burst did not fully drain");

        // 全局复位打断一个已完整入队、但 TX 尚未 start 的在途包。
        send_packet(30,160,1);
        begin : wait_reset_target_offer
            integer guard;
            guard=0;
            while(tx_valid!==1'b1 && guard<50000)begin
                @(negedge tx_clk); guard=guard+1;
            end
            if(guard>=50000)$fatal(1,"reset target packet did not reach TX offer");
        end
        @(negedge tx_clk);
        reset_n=1'b0;
        src_reset=1'b1;
        #1;
        if(tx_valid!==1'b0)$fatal(1,"global reset did not abort pending TX offer");
        repeat(8)@(negedge tx_clk);
        reset_n=1'b1;
        repeat(4)@(negedge src_clk);
        src_reset=1'b0;
        send_packet(31,48,1);
        receive_packet(31,48,0);

        // 活动包已读出部分字节时复位，后续不得发布旧包尾部。
        send_packet(32,160,1);
        wait(tx_valid===1'b1);
        @(negedge tx_clk); tx_start=1; tx_read=1;
        for(packet_id=0;packet_id<5;packet_id=packet_id+1)begin
            if(tx_data!==pattern_byte(32,packet_id))$fatal(1,"active-reset precondition byte mismatch");
            @(posedge tx_clk); #1;
            @(negedge tx_clk); tx_start=0;
        end
        reset_n=0; src_reset=1; tx_read=0;
        #1;
        if(tx_valid!==1'b0)$fatal(1,"global reset did not abort active packet");
        repeat(8)@(negedge tx_clk);
        reset_n=1;
        repeat(4)@(negedge src_clk); src_reset=0;
        send_packet(33,48,1);
        receive_packet(33,48,0);
        repeat(100)@(posedge tx_clk);
        if(tx_valid)$fatal(1,"stale descriptor survived global reset");
        $display("PASS: 50-to-125 MHz packet CDC, 1/2/255-byte handshakes, active stalls, FIFO backpressure, offer/active reset abort");
        $finish;
    end

    initial begin
        #5000000;
        $fatal(1,"timeout in telemetry packet CDC contract test");
    end
endmodule
