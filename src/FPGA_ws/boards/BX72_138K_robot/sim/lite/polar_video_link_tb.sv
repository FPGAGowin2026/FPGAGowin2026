`timescale 1ns/1ps
// Source-only contract test for polar-v2 fusion/navigation words through the
// production dual-clock telemetry serializer. Video RTP/PT97 framing remains
// covered by tb_eth_udp_video_sender; this test does not alter that path.
module polar_video_link_tb;
    reg src_clk=0, tx_clk=0, reset_n=0;
    always #15 src_clk=~src_clk;
    always #4 tx_clk=~tx_clk;

    reg fusion_valid=0, navigation_valid=0;
    reg [831:0] fusion_payload=0;
    reg [543:0] navigation_payload=0;
    wire tx_valid, tx_length_nonzero, tx_reset;
    wire [7:0] tx_length, tx_data;
    reg tx_start=0, tx_read=0;
    wire [159:0] dropped;

    bx72_telemetry #(.SRC_CLK_HZ(33333333),.CHANNEL_MASK(6'b101000)) dut (
        .src_clk(src_clk),.tx_clk(tx_clk),.reset_n(reset_n),
        .imu_valid_i(1'b0),.imu_payload_i(256'd0),
        .motor_valid_i(1'b0),.motor_payload_i(704'd0),
        .lidar_valid_i(1'b0),.lidar_payload_i(1152'd0),
        .fusion_valid_i(fusion_valid),.fusion_payload_i(fusion_payload),
        .health_valid_i(1'b0),.health_payload_i(896'd0),
        .navigation_valid_i(navigation_valid),.navigation_payload_i(navigation_payload),
        .tx_valid_o(tx_valid),.tx_length_o(tx_length),
        .tx_length_nonzero_o(tx_length_nonzero),.tx_data_o(tx_data),
        .tx_start_i(tx_start),.tx_read_i(tx_read),.dropped_o(dropped),
        .src_reset_i(1'b0),.tx_reset_i(1'b0),.tx_reset_o(tx_reset)
    );

    reg [31:0] fusion_words [0:25];
    reg [31:0] navigation_words [0:16];
    integer packet_count=0;
    integer i;

    task receive_and_check;
        integer guard, index, word_index, byte_index, expected_length;
        reg [7:0] packet_kind;
        reg [31:0] expected_word;
        begin
            guard=0;
            while(tx_valid!==1'b1 && guard<50000) begin
                @(negedge tx_clk);
                guard=guard+1;
            end
            if(guard>=50000)$fatal(1,"polar telemetry packet offer timeout");
            if(!tx_length_nonzero)$fatal(1,"telemetry length flag missing");
            tx_start=1'b1;
            @(posedge tx_clk);
            @(negedge tx_clk);
            tx_start=1'b0;
            tx_read=1'b1;
            packet_kind=8'hff;
            expected_length=0;
            for(index=0;index<tx_length;index=index+1) begin
                if(index==5) begin
                    packet_kind=tx_data;
                    if(packet_kind==6) expected_length=120;
                    else if(packet_kind==8) expected_length=84;
                    else $fatal(1,"unexpected telemetry kind %0d",packet_kind);
                end
                if(index==0 && tx_data!==8'h42)$fatal(1,"bad telemetry magic byte 0");
                if(index==1 && tx_data!==8'h58)$fatal(1,"bad telemetry magic byte 1");
                if(index==2 && tx_data!==8'h37)$fatal(1,"bad telemetry magic byte 2");
                if(index==3 && tx_data!==8'h32)$fatal(1,"bad telemetry magic byte 3");
                if(index==4 && tx_data!==8'h01)$fatal(1,"bad telemetry version");
                if(index==6 && tx_data!==8'd0)$fatal(1,"bad telemetry length high byte");
                if(index==7 && tx_data!==expected_length[7:0])
                    $fatal(1,"telemetry length mismatch kind=%0d got=%0d expected=%0d",
                        packet_kind,tx_data,expected_length);
                if(index>=16) begin
                    word_index=(index-16)/4;
                    byte_index=(index-16)%4;
                    if(packet_kind==6)expected_word=fusion_words[word_index];
                    else expected_word=navigation_words[word_index];
                    if(tx_data!==expected_word[31-byte_index*8-:8])
                        $fatal(1,"polar payload changed kind=%0d word=%0d byte=%0d got=%02x expected=%02x",
                            packet_kind,word_index,byte_index,tx_data,
                            expected_word[31-byte_index*8-:8]);
                end
                @(posedge tx_clk);
                @(negedge tx_clk);
            end
            tx_read=1'b0;
            if(packet_kind==6) begin
                if(fusion_words[14]!==32'hc0004001 || fusion_words[21]!==32'hf0000001 ||
                   fusion_words[24]!==32'h00007f03 || fusion_words[25]!==32'he0000002)
                    $fatal(1,"fusion polar-v2 sentinel setup invalid");
            end else begin
                if(navigation_words[8][7:0]!==8'hc7 ||
                   navigation_words[14]!==32'hfffffff1 ||
                   navigation_words[16]!==32'h87654321)
                    $fatal(1,"navigation polar-v2 sentinel setup invalid");
            end
            packet_count=packet_count+1;
        end
    endtask

    initial begin
        for(i=0;i<26;i=i+1)fusion_words[i]=32'h51000000+i;
        for(i=0;i<17;i=i+1)navigation_words[i]=32'h62000000+i;

        // 26-word fusion packet: error estimates at 9..11, unsigned raw and
        // reduced sample counts at 12..13, polar flags at 14, diagnostics at 24.
        fusion_words[9]=32'h41280000; // 10.5 mm
        fusion_words[10]=32'h41a20000; // 20.25 mm
        fusion_words[11]=32'h40400000; // 3 mrad
        fusion_words[12]=32'hfffffff1;
        fusion_words[13]=32'hf0000002;
        fusion_words[14]=32'hc0004001;
        fusion_words[21]=32'hf0000001;
        fusion_words[24]=32'h00007f03;
        fusion_words[25]=32'he0000002;

        // nav_polar_v2 keeps the old Q16 goal contract and gives the three
        // diagnostic fields polar-specific names at their established offsets.
        navigation_words[8]={8'h03,8'h00,8'h03,8'hc7};
        navigation_words[9]=32'h1234fedc;
        navigation_words[10]={16'h03e8,16'd72};
        navigation_words[11]=32'h12345678;
        navigation_words[12]=32'h89abcdef;
        navigation_words[13]=32'h00000018;
        navigation_words[14]=32'hfffffff1;
        navigation_words[15]=32'h00000020;
        navigation_words[16]=32'h87654321;
        for(i=0;i<26;i=i+1)
            fusion_payload[831-i*32-:32]=fusion_words[i];
        for(i=0;i<17;i=i+1)
            navigation_payload[543-i*32-:32]=navigation_words[i];

        repeat(5)@(negedge src_clk);
        reset_n=1'b1;
        repeat(5)@(negedge src_clk);
        fusion_valid=1'b1;
        navigation_valid=1'b1;
        @(negedge src_clk);
        fusion_valid=1'b0;
        navigation_valid=1'b0;

        receive_and_check();
        receive_and_check();
        if(packet_count!=2)$fatal(1,"expected fusion and navigation packets");
        if(dropped!==160'd0)$fatal(1,"telemetry serializer dropped a polar packet");
        $display("PASS polar_video_link: polar-v2 fusion/navigation payloads preserved");
        $finish;
    end

    initial begin
        #2000000;
        $fatal(1,"polar telemetry link timeout");
    end
endmodule
