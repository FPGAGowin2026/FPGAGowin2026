`timescale 1ns/1ps
// 两时钟分组、固定快照串行化、完整长载荷及满字节 FIFO 的定向合同测试。
module nav_full_telemetry_serializer_tb;
    reg raw_clk=0, control_clk=0, tx_clk=0, reset_n=0;
    always #15 raw_clk=~raw_clk;       // 33.333 MHz
    always #12.5 control_clk=~control_clk; // 40 MHz
    always #4 tx_clk=~tx_clk;         // 125 MHz

    reg imu_valid=0, motor_valid=0, lidar_valid=0, fusion_valid=0;
    reg health_valid=0, navigation_valid=0;
    reg [255:0] imu_payload=0;
    reg [703:0] motor_payload=0;
    reg [1151:0] lidar_payload=0;
    reg [831:0] fusion_payload=0;
    reg [895:0] health_payload=0;
    reg [543:0] navigation_payload=0;
    reg local_plan_valid=0;
    reg [319:0] local_plan_payload=0;
    reg local_grid_valid=0;
    reg [1663:0] local_grid_payload=0;
    wire local_grid_ready;

    wire raw_valid, raw_length_nonzero, raw_tx_reset;
    wire [7:0] raw_length, raw_data;
    wire [159:0] raw_dropped;
    reg raw_start=0, raw_read=0;
    wire control_valid, control_length_nonzero, control_tx_reset;
    wire [7:0] control_length, control_data;
    wire [159:0] control_dropped;
    reg control_start=0, control_read=0;

    bx72_telemetry #(.SRC_CLK_HZ(33333333),.CHANNEL_MASK(6'b010101)) raw_telemetry (
        .src_clk(raw_clk),.tx_clk(tx_clk),.reset_n(reset_n),
        .imu_valid_i(imu_valid),.imu_payload_i(imu_payload),
        .motor_valid_i(motor_valid),.motor_payload_i(motor_payload),
        .lidar_valid_i(lidar_valid),.lidar_payload_i(lidar_payload),
        .fusion_valid_i(fusion_valid),.fusion_payload_i(fusion_payload),
        .health_valid_i(health_valid),.health_payload_i(health_payload),
        .navigation_valid_i(navigation_valid),.navigation_payload_i(navigation_payload),
        .tx_valid_o(raw_valid),.tx_length_o(raw_length),.tx_length_nonzero_o(raw_length_nonzero),
        .tx_data_o(raw_data),.tx_start_i(raw_start),.tx_read_i(raw_read),
        .dropped_o(raw_dropped),.src_reset_i(1'b0),.tx_reset_i(1'b0),.tx_reset_o(raw_tx_reset)
    );

    bx72_telemetry #(.SRC_CLK_HZ(40000000),.CHANNEL_MASK(8'b11101010)) control_telemetry (
        .src_clk(control_clk),.tx_clk(tx_clk),.reset_n(reset_n),
        .imu_valid_i(imu_valid),.imu_payload_i(imu_payload),
        .motor_valid_i(motor_valid),.motor_payload_i(motor_payload),
        .lidar_valid_i(lidar_valid),.lidar_payload_i(lidar_payload),
        .fusion_valid_i(fusion_valid),.fusion_payload_i(fusion_payload),
        .health_valid_i(health_valid),.health_payload_i(health_payload),
        .navigation_valid_i(navigation_valid),.navigation_payload_i(navigation_payload),
        .local_plan_valid_i(local_plan_valid),.local_plan_payload_i(local_plan_payload),
        .local_grid_valid_i(local_grid_valid),.local_grid_payload_i(local_grid_payload),
        .local_grid_ready_o(local_grid_ready),
        .tx_valid_o(control_valid),.tx_length_o(control_length),.tx_length_nonzero_o(control_length_nonzero),
        .tx_data_o(control_data),.tx_start_i(control_start),.tx_read_i(control_read),
        .dropped_o(control_dropped),.src_reset_i(1'b0),.tx_reset_i(1'b0),.tx_reset_o(control_tx_reset)
    );

    reg [31:0] lidar_time_by_generation [0:127];

    function [31:0] input_word;
        input integer channel_id;
        input integer generation;
        input integer word_index;
        begin
            input_word=32'hA5000000 | (channel_id << 20) | (generation << 8) | word_index;
        end
    endfunction

    function [7:0] expected_byte;
        input integer packet_kind;
        input integer packet_length;
        input integer packet_sequence;
        input [31:0] packet_timestamp;
        input integer generation;
        input [31:0] drop_count;
        input integer byte_index;
        reg [127:0] header;
        reg [31:0] payload_word;
        integer channel_id, word_index, byte_in_word;
        begin
            header={32'h42583732,8'd1,packet_kind[7:0],packet_length[15:0],
                    packet_sequence[31:0],packet_timestamp};
            if(byte_index<16) begin
                expected_byte=header[127-byte_index*8-:8];
            end else begin
                channel_id=packet_kind-3;
                word_index=(byte_index-16)/4;
                byte_in_word=(byte_index-16)%4;
                if((packet_kind==3 && word_index==7) ||
                   (packet_kind==4 && word_index==21) ||
                   (packet_kind==5 && word_index==3))
                    payload_word=drop_count;
                else if(packet_kind==7 && word_index==27)
                    payload_word=32'h475200A9;
                else
                    payload_word=input_word(channel_id,generation,word_index);
                expected_byte=payload_word[31-byte_in_word*8-:8];
            end
        end
    endfunction

    task receive_packet;
        input integer lane;
        input integer packet_kind;
        input integer packet_length;
        input integer packet_sequence;
        input integer generation;
        input [31:0] packet_timestamp;
        input [31:0] drop_count;
        integer timeout_count, index, first_index;
        reg [7:0] observed, expected;
        begin
            timeout_count=0;
            while (((lane==0) ? (raw_valid!==1'b1) : (control_valid!==1'b1)) && timeout_count<50000) begin
                @(negedge tx_clk);
                timeout_count=timeout_count+1;
            end
            if(timeout_count>=50000)$fatal(1,"telemetry offer timeout kind=%0d",packet_kind);
            if(lane==0) begin
                if(raw_length!==packet_length[7:0] || raw_length_nonzero!==1'b1)
                    $fatal(1,"raw descriptor kind=%0d length=%0d",packet_kind,raw_length);
                raw_start=1'b1;
            end else begin
                if(control_length!==packet_length[7:0] || control_length_nonzero!==1'b1)
                    $fatal(1,"control descriptor kind=%0d length=%0d",packet_kind,control_length);
                control_start=1'b1;
            end
            // 在两个真实 bx72_telemetry 消费者上分别覆盖 start/read 同拍。
            first_index=0;
            if(packet_kind==3 || packet_kind==4)begin
                if(lane==0)raw_read=1'b1;
                else control_read=1'b1;
                observed=(lane==0) ? raw_data : control_data;
                expected=expected_byte(packet_kind,packet_length,packet_sequence,
                    packet_timestamp,generation,drop_count,0);
                if(observed!==expected)$fatal(1,"same-cycle first byte mismatch kind=%0d",packet_kind);
                first_index=1;
            end
            @(posedge tx_clk);
            @(negedge tx_clk);
            if(lane==0) begin raw_start=1'b0; raw_read=1'b1; end
            else begin control_start=1'b0; control_read=1'b1; end
            for(index=first_index;index<packet_length;index=index+1) begin
                observed=(lane==0) ? raw_data : control_data;
                expected=expected_byte(packet_kind,packet_length,packet_sequence,
                    packet_timestamp,generation,drop_count,index);
                if(observed!==expected)
                    $fatal(1,"packet byte mismatch kind=%0d seq=%0d byte=%0d got=%02x expected=%02x",
                        packet_kind,packet_sequence,index,observed,expected);
                @(posedge tx_clk);
                @(negedge tx_clk);
            end
            if(lane==0)raw_read=1'b0;
            else control_read=1'b0;
        end
    endtask

    task pulse_lidar;
        input integer generation;
        input integer wait_for_slot;
        input integer expect_accept;
        integer guard, word_index;
        reg [31:0] old_sequence, old_drop;
        begin
            if(wait_for_slot!=0) begin
                guard=0;
                while(raw_telemetry.pending[2]!==1'b0 && guard<20000) begin
                    @(negedge raw_clk); guard=guard+1;
                end
                if(guard>=20000)$fatal(1,"lidar pending did not drain before event %0d",generation);
            end
            old_sequence=raw_telemetry.sequence_next[2];
            old_drop=raw_telemetry.dropped[2];
            @(negedge raw_clk);
            for(word_index=0;word_index<36;word_index=word_index+1)
                lidar_payload[1151-word_index*32-:32]=input_word(2,generation,word_index);
            lidar_valid=1'b1;
            @(posedge raw_clk); #1;
            if(raw_telemetry.sequence_next[2]!==old_sequence+1'b1)
                $fatal(1,"lidar sequence did not advance for event %0d",generation);
            if(expect_accept!=0) begin
                if(raw_telemetry.snapshot_sequence[2]!==old_sequence)
                    $fatal(1,"lidar snapshot sequence mismatch event %0d",generation);
                lidar_time_by_generation[generation]=raw_telemetry.snapshot_time[2];
            end else if(raw_telemetry.dropped[2]!==old_drop+1'b1) begin
                $fatal(1,"busy lidar event was not counted as dropped");
            end
            @(negedge raw_clk); lidar_valid=1'b0;
        end
    endtask

    integer word_index, generation;
    reg [31:0] fusion_collision_timestamp,plan_timestamp,grid_timestamp;
    initial begin
        for(word_index=0;word_index<8;word_index=word_index+1)
            imu_payload[255-word_index*32-:32]=input_word(0,1,word_index);
        for(word_index=0;word_index<22;word_index=word_index+1)
            motor_payload[703-word_index*32-:32]=input_word(1,1,word_index);
        for(word_index=0;word_index<36;word_index=word_index+1)
            lidar_payload[1151-word_index*32-:32]=input_word(2,1,word_index);
        for(word_index=0;word_index<26;word_index=word_index+1)
            fusion_payload[831-word_index*32-:32]=input_word(3,1,word_index);
        for(word_index=0;word_index<28;word_index=word_index+1)
            health_payload[895-word_index*32-:32]=input_word(4,1,word_index);
        health_payload[31:0]=32'h475200A9;
        for(word_index=0;word_index<17;word_index=word_index+1)
            navigation_payload[543-word_index*32-:32]=input_word(5,1,word_index);

        repeat(8)@(negedge raw_clk);
        reset_n=1'b1;
        repeat(4)@(negedge raw_clk);
        if(raw_tx_reset!==1'b0 || control_tx_reset!==1'b0)
            $fatal(1,"TX synchronized reset did not release");
        // 每组事件只维持自己的一个源周期，避免30ns脉冲在40MHz被采样两次。
        fork
            begin
                imu_valid=1'b1; lidar_valid=1'b1; health_valid=1'b1;
                @(posedge raw_clk); #1;
                lidar_time_by_generation[1]=raw_telemetry.snapshot_time[2];
                if(raw_telemetry.pending[5:0]!==6'b010101)
                    $fatal(1,"raw channel mask accepted the wrong channels: %b",raw_telemetry.pending);
                @(negedge raw_clk);
                imu_valid=1'b0; lidar_valid=1'b0; health_valid=1'b0;
            end
            begin
                @(negedge control_clk);
                motor_valid=1'b1; fusion_valid=1'b1; navigation_valid=1'b1;
                @(posedge control_clk); #1;
                if(control_telemetry.pending[5:0]!==6'b101010)
                    $fatal(1,"control channel mask accepted the wrong channels: %b",control_telemetry.pending);
                @(negedge control_clk);
                motor_valid=1'b0; fusion_valid=1'b0; navigation_valid=1'b0;
            end
        join

        receive_packet(0,3,48,0,1,32'd0,32'd0);
        receive_packet(1,4,104,0,1,32'd0,32'd0);
        receive_packet(0,5,160,0,1,lidar_time_by_generation[1],32'd0);
        receive_packet(1,6,120,0,1,32'd0,32'd0);
        receive_packet(0,7,128,0,1,32'd0,32'd0);
        receive_packet(1,8,84,0,1,32'd0,32'd0);

        // 第二个 fusion_valid 与候选被选中同拍，必须丢弃新事件且冻结首个快照。
        @(negedge control_clk);
        for(word_index=0;word_index<26;word_index=word_index+1)
            fusion_payload[831-word_index*32-:32]=input_word(3,2,word_index);
        fusion_valid=1'b1;
        @(posedge control_clk); #1;
        if(control_telemetry.snapshot_sequence[3]!==32'd1)
            $fatal(1,"first fusion snapshot sequence mismatch");
        fusion_collision_timestamp=control_telemetry.snapshot_time[3];
        if(control_telemetry.select_fire!==1'b1 || control_telemetry.selected!==3'd3)
            $fatal(1,"fusion snapshot was not selected on the expected collision edge");
        @(posedge control_clk); #1;
        if(control_telemetry.dropped[3]!==32'd1 ||
           control_telemetry.fusion_snapshot[831-25*32-:32]!==input_word(3,2,25))
            $fatal(1,"fusion selection collision overwrote its active snapshot");
        @(negedge control_clk); fusion_valid=1'b0;
        receive_packet(1,6,120,1,2,fusion_collision_timestamp,32'd0);
        if(control_dropped[127:96]!==32'd1)$fatal(1,"fusion selection drop not visible");

        repeat(500)@(posedge tx_clk);
        if(raw_valid || control_valid)$fatal(1,"disabled channels emitted extra packets");

        // 13 个雷达包填满 2048 字节 FIFO；第 13 包停在中途时同类新事件必须丢弃。
        for(generation=2;generation<=14;generation=generation+1)
            pulse_lidar(generation,1,1);
        begin : wait_fifo_full
            integer guard;
            guard=0;
            while(!(raw_telemetry.serializing && raw_telemetry.byte_full) && guard<20000) begin
                @(negedge raw_clk); guard=guard+1;
            end
            if(guard>=20000)$fatal(1,"byte FIFO never applied source-side backpressure");
        end
        if(raw_telemetry.pending[2]!==1'b1)
            $fatal(1,"full FIFO did not retain the active lidar snapshot");
        pulse_lidar(99,0,0);
        if(raw_dropped[95:64]!==32'd1)$fatal(1,"lidar drop counter not visible");

        for(generation=2;generation<=13;generation=generation+1)
            receive_packet(0,5,160,generation-1,generation,
                lidar_time_by_generation[generation],32'd0);
        begin : wait_thirteenth_complete
            integer guard;
            guard=0;
            while(raw_telemetry.pending[2]!==1'b0 && guard<20000) begin
                @(negedge raw_clk); guard=guard+1;
            end
            if(guard>=20000)$fatal(1,"stalled lidar packet did not resume after reads");
        end
        pulse_lidar(15,1,1);
        receive_packet(0,5,160,13,14,lidar_time_by_generation[14],32'd0);
        receive_packet(0,5,160,15,15,lidar_time_by_generation[15],32'd1);
        if(raw_telemetry.dropped[2]!==32'd1)
            $fatal(1,"drop counter changed unexpectedly");
        // 本地规划和栅格同拍进入各自快照，逐字节检查大端顺序与长包边界。
        @(negedge control_clk);
        for(word_index=0;word_index<10;word_index=word_index+1)
            local_plan_payload[319-word_index*32-:32]=input_word(6,1,word_index);
        for(word_index=0;word_index<52;word_index=word_index+1)
            local_grid_payload[1663-word_index*32-:32]=input_word(7,1,word_index);
        if(local_grid_ready!==1'b1)$fatal(1,"grid snapshot was not ready");
        local_plan_valid=1;local_grid_valid=1;
        @(posedge control_clk); #1;
        plan_timestamp=control_telemetry.snapshot_time[6];
        grid_timestamp=control_telemetry.snapshot_time[7];
        if(control_telemetry.pending[7:6]!==2'b11 || local_grid_ready!==1'b0)
            $fatal(1,"plan/grid snapshots were not accepted or grid ready stayed high");
        @(negedge control_clk);local_plan_valid=0;local_grid_valid=0;
        receive_packet(1,9,56,0,1,plan_timestamp,32'd0);
        receive_packet(1,10,224,0,1,grid_timestamp,32'd0);
        repeat(500)@(posedge tx_clk);
        if(control_valid || local_grid_ready!==1'b1 || control_telemetry.pending[7:6]!==2'b00)
            $fatal(1,"plan/grid channels did not complete cleanly");
        $display("PASS: kind3..10 snapshots, FIFO stall/drop/sequence gap");
        $finish;
    end

    initial begin
        #5000000;
        $fatal(1,"timeout in telemetry serializer contract test");
    end
endmodule
