`timescale 1ns/1ps

// 官方 ch58 工程的 PLL 为器件生成核；RTL 仿真只替换时钟外壳。
module Gowin_PLL(
  input  wire clkin,
  input  wire init_clk,
  output wire clkout0
);
  assign clkout0 = clkin;
endmodule

module tb_eth_udp_loopback_rgmii;
  localparam integer FRAME_BYTES = 72;

  reg        reset_n = 1'b0;
  reg        rgmii_rx_clk_i = 1'b0;
  reg  [3:0] rgmii_rxd = 4'h0;
  reg        rgmii_rxdv = 1'b0;
  wire       eth_rst_n;
  wire       rgmii_tx_clk;
  wire [3:0] rgmii_txd;
  wire       rgmii_txen;

  reg [7:0] request_frame [0:FRAME_BYTES-1];
  reg [7:0] expected_reply [0:FRAME_BYTES-1];
  reg [7:0] tx_capture [0:127];
  integer tx_count = 0;
  integer captured_len = 0;
  integer completed_frames = 0;
  integer errors = 0;
  reg capturing = 1'b0;

  // Gowin 原语模型通过固定层次名读取全局复位。
  GSR GSR(.GSRI(1'b1));

  eth_udp_loopback_rgmii dut(
    .reset_n(reset_n),
    .rgmii_rx_clk_i(rgmii_rx_clk_i),
    .rgmii_rxd(rgmii_rxd),
    .rgmii_rxdv(rgmii_rxdv),
    .eth_rst_n(eth_rst_n),
    .rgmii_tx_clk(rgmii_tx_clk),
    .rgmii_txd(rgmii_txd),
    .rgmii_txen(rgmii_txen)
  );

  always #4 rgmii_rx_clk_i = ~rgmii_rx_clk_i;

  function automatic [31:0] crc32_byte;
    input [31:0] crc_in;
    input [7:0] data;
    reg [31:0] crc;
    integer bit_index;
    begin
      crc = crc_in;
      for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
        if (crc[0] ^ data[bit_index])
          crc = (crc >> 1) ^ 32'hedb88320;
        else
          crc = crc >> 1;
      end
      crc32_byte = crc;
    end
  endfunction

  task automatic build_request;
    input bad_fcs;
    input payload_variant;
    integer i;
    reg [31:0] crc;
    begin
      for (i = 0; i < 7; i = i + 1)
        request_frame[i] = 8'h55;
      request_frame[7] = 8'hd5;

      // 目的 MAC 00:0a:35:01:fe:c0，源 MAC 02:00:00:00:00:01。
      request_frame[8]  = 8'h00; request_frame[9]  = 8'h0a;
      request_frame[10] = 8'h35; request_frame[11] = 8'h01;
      request_frame[12] = 8'hfe; request_frame[13] = 8'hc0;
      request_frame[14] = 8'h02; request_frame[15] = 8'h00;
      request_frame[16] = 8'h00; request_frame[17] = 8'h00;
      request_frame[18] = 8'h00; request_frame[19] = 8'h01;
      request_frame[20] = 8'h08; request_frame[21] = 8'h00;

      // IPv4/UDP：192.168.0.1:6000 -> 192.168.0.2:5000，负载 8 字节。
      request_frame[22] = 8'h45; request_frame[23] = 8'h00;
      request_frame[24] = 8'h00; request_frame[25] = 8'h24;
      request_frame[26] = 8'h00; request_frame[27] = 8'h00;
      request_frame[28] = 8'h00; request_frame[29] = 8'h00;
      request_frame[30] = 8'h40; request_frame[31] = 8'h11;
      request_frame[32] = 8'hf9; request_frame[33] = 8'h75;
      request_frame[34] = 8'hc0; request_frame[35] = 8'ha8;
      request_frame[36] = 8'h00; request_frame[37] = 8'h01;
      request_frame[38] = 8'hc0; request_frame[39] = 8'ha8;
      request_frame[40] = 8'h00; request_frame[41] = 8'h02;
      request_frame[42] = 8'h17; request_frame[43] = 8'h70;
      request_frame[44] = 8'h13; request_frame[45] = 8'h88;
      request_frame[46] = 8'h00; request_frame[47] = 8'h10;
      request_frame[48] = 8'h00; request_frame[49] = 8'h00;
      if(payload_variant) begin
        request_frame[50] = 8'h10; request_frame[51] = 8'h32;
        request_frame[52] = 8'h54; request_frame[53] = 8'h76;
        request_frame[54] = 8'h98; request_frame[55] = 8'hba;
        request_frame[56] = 8'hdc; request_frame[57] = 8'hfe;
      end else begin
        request_frame[50] = 8'hde; request_frame[51] = 8'had;
        request_frame[52] = 8'hbe; request_frame[53] = 8'hef;
        request_frame[54] = 8'h01; request_frame[55] = 8'h23;
        request_frame[56] = 8'h45; request_frame[57] = 8'h67;
      end
      for (i = 58; i < 68; i = i + 1)
        request_frame[i] = 8'h00;

      crc = 32'hffffffff;
      for (i = 8; i < 68; i = i + 1)
        crc = crc32_byte(crc, request_frame[i]);
      crc = ~crc;
      request_frame[68] = crc[7:0] ^ (bad_fcs ? 8'h01 : 8'h00);
      request_frame[69] = crc[15:8];
      request_frame[70] = crc[23:16];
      request_frame[71] = crc[31:24];
    end
  endtask

  task automatic build_expected_reply;
    input payload_variant;
    integer i;
    reg [31:0] crc;
    begin
      build_request(1'b0, payload_variant);
      for (i = 0; i < 68; i = i + 1)
        expected_reply[i] = request_frame[i];

      expected_reply[8]  = 8'h02; expected_reply[9]  = 8'h00;
      expected_reply[10] = 8'h00; expected_reply[11] = 8'h00;
      expected_reply[12] = 8'h00; expected_reply[13] = 8'h01;
      expected_reply[14] = 8'h00; expected_reply[15] = 8'h0a;
      expected_reply[16] = 8'h35; expected_reply[17] = 8'h01;
      expected_reply[18] = 8'hfe; expected_reply[19] = 8'hc0;
      expected_reply[34] = 8'hc0; expected_reply[35] = 8'ha8;
      expected_reply[36] = 8'h00; expected_reply[37] = 8'h02;
      expected_reply[38] = 8'hc0; expected_reply[39] = 8'ha8;
      expected_reply[40] = 8'h00; expected_reply[41] = 8'h01;
      expected_reply[42] = 8'h13; expected_reply[43] = 8'h88;
      expected_reply[44] = 8'h17; expected_reply[45] = 8'h70;

      crc = 32'hffffffff;
      for (i = 8; i < 68; i = i + 1)
        crc = crc32_byte(crc, expected_reply[i]);
      crc = ~crc;
      expected_reply[68] = crc[7:0];
      expected_reply[69] = crc[15:8];
      expected_reply[70] = crc[23:16];
      expected_reply[71] = crc[31:24];
    end
  endtask

  task automatic drive_rgmii_byte;
    input [7:0] data;
    input       valid;
    begin
      @(negedge rgmii_rx_clk_i);
      rgmii_rxd  <= data[3:0];
      rgmii_rxdv <= valid;
      @(posedge rgmii_rx_clk_i);
      rgmii_rxd  <= data[7:4];
      rgmii_rxdv <= valid;
    end
  endtask

  task automatic send_request;
    input bad_fcs;
    input payload_variant;
    integer i;
    begin
      build_request(bad_fcs, payload_variant);
      for (i = 0; i < FRAME_BYTES; i = i + 1)
        drive_rgmii_byte(request_frame[i], 1'b1);
      for (i = 0; i < 16; i = i + 1)
        drive_rgmii_byte(8'h00, 1'b0);
    end
  endtask

  task automatic check_latest_reply;
    integer i;
    begin
      if (captured_len != FRAME_BYTES) begin
        $display("[FAIL] 回环报文长度=%0d，期望=%0d", captured_len, FRAME_BYTES);
        errors = errors + 1;
      end
      for (i = 0; i < FRAME_BYTES; i = i + 1) begin
        if ((i < captured_len) && (tx_capture[i] !== expected_reply[i])) begin
          $display("[FAIL] 回环字节[%0d]=%02x，期望=%02x", i, tx_capture[i], expected_reply[i]);
          errors = errors + 1;
        end
      end
    end
  endtask

  task automatic wait_for_frame;
    input integer initial_count;
    output reg got_frame;
    integer guard;
    begin
      guard = 0;
      got_frame = 1'b0;
      while ((completed_frames == initial_count) && (guard < 2000)) begin
        @(posedge rgmii_rx_clk_i);
        guard = guard + 1;
      end
      if (completed_frames != initial_count)
        got_frame = 1'b1;
    end
  endtask

  always @(posedge rgmii_rx_clk_i) begin
    #1;
    if (dut.gmii_txen) begin
      if (!capturing) begin
        capturing = 1'b1;
        tx_count = 0;
      end
      tx_capture[tx_count] = dut.gmii_txd;
      tx_count = tx_count + 1;
    end else if (capturing) begin
      capturing = 1'b0;
      captured_len = tx_count;
      completed_frames = completed_frames + 1;
    end
  end

  initial begin : run_test
    integer frames_before;
    reg got_frame;

    if ($test$plusargs("VCD")) begin
      $dumpfile("udp_loopback.vcd");
      $dumpvars(0, tb_eth_udp_loopback_rgmii);
    end

    repeat (8) @(posedge rgmii_rx_clk_i);
    reset_n = 1'b1;
    repeat (12) drive_rgmii_byte(8'h00, 1'b0);

    build_expected_reply(1'b0);
    frames_before = completed_frames;
    send_request(1'b0, 1'b0);
    wait_for_frame(frames_before, got_frame);
    if (!got_frame) begin
      $display("[FAIL] 合法 UDP 帧未产生回环报文");
      errors = errors + 1;
    end else begin
      check_latest_reply();
      if ((dut.pkt_right_cnt != 4'd1) || (dut.pkt_err_cnt != 4'd0)) begin
        $display("[FAIL] 合法帧分类异常，pkt_right_cnt=%0d pkt_err_cnt=%0d",
                 dut.pkt_right_cnt, dut.pkt_err_cnt);
        errors = errors + 1;
      end
      if (errors == 0)
        $display("[PASS] 合法 UDP 帧完成 72 字节逐字节回环校验");
    end

    if ($test$plusargs("VALID_ONLY")) begin
      if (errors != 0)
        $fatal(1, "ACG720 官方 UDP 合法帧回环自检失败，共 %0d 项", errors);
      $display("[PASS] ACG720 官方 UDP 合法帧回环基线通过");
      $finish;
    end

    // 错 FCS 帧必须被接收端计为错误，且不应进入发送队列。
    frames_before = completed_frames;
    send_request(1'b1, 1'b0);
    repeat (600) @(posedge rgmii_rx_clk_i);
    if (dut.pkt_err_cnt != 4'd1) begin
      $display("[FAIL] 错 FCS 帧未被错误计数器识别，pkt_err_cnt=%0d", dut.pkt_err_cnt);
      errors = errors + 1;
    end else begin
      $display("[PASS] 错 FCS 帧被接收核错误计数器识别");
    end
    if (completed_frames != frames_before) begin
      $display("[FAIL] 错 FCS 帧仍触发了 UDP 回环发送");
      errors = errors + 1;
    end else begin
      $display("[PASS] 错 FCS 帧被拒绝，未产生回环报文");
    end

    // 注入描述符 FIFO Full，验证该边界信号会拒绝整帧，而非只丢元数据。
    frames_before = completed_frames;
    force dut.eth_msg_full = 1'b1;
    send_request(1'b0, 1'b0);
    repeat (600) @(posedge rgmii_rx_clk_i);
    release dut.eth_msg_full;
    if (dut.pkt_err_cnt != 4'd2) begin
      $display("[FAIL] 描述符 FIFO Full 注入未原子拒绝报文，pkt_err_cnt=%0d", dut.pkt_err_cnt);
      errors = errors + 1;
    end else begin
      $display("[PASS] 描述符 FIFO Full 信号触发整帧拒绝");
    end
    if (completed_frames != frames_before) begin
      $display("[FAIL] 描述符 FIFO 满仍触发了 UDP 回环发送");
      errors = errors + 1;
    end
    if (dut.eth_packet_data_buf.speculative_count != 0 ||
        dut.eth_packet_data_buf.wr_ptr != dut.eth_packet_data_buf.frame_start_ptr) begin
      $display("[FAIL] 描述符 FIFO Full 拒绝后负载暂存指针未回滚");
      errors = errors + 1;
    end

    // 两类坏帧之后换一组负载发送合法帧，验证回滚不会污染下一帧。
    build_expected_reply(1'b1);
    frames_before = completed_frames;
    send_request(1'b0, 1'b1);
    wait_for_frame(frames_before, got_frame);
    if (!got_frame) begin
      $display("[FAIL] 坏 FCS 帧后的合法 UDP 帧未产生回环报文");
      errors = errors + 1;
    end else begin
      check_latest_reply();
      if ((dut.pkt_right_cnt != 4'd2) || (dut.pkt_err_cnt != 4'd2)) begin
        $display("[FAIL] 坏帧后合法帧分类异常，pkt_right_cnt=%0d pkt_err_cnt=%0d",
                 dut.pkt_right_cnt, dut.pkt_err_cnt);
        errors = errors + 1;
      end
      if (errors == 0)
        $display("[PASS] 坏 FCS/Full 拒绝帧未污染下一合法帧的 8 字节负载");
    end

    if (errors != 0)
      $fatal(1, "ACG720 官方 UDP 回环自检失败，共 %0d 项", errors);

    $display("[PASS] ACG720 官方 UDP 回环自检全部通过");
    $finish;
  end
endmodule
