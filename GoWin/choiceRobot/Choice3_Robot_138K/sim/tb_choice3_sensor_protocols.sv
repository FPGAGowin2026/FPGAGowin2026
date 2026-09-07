`timescale 1ns/1ps

module tb_choice3_sensor_protocols;
  localparam integer JY_CLKS_PER_BIT=50_000_000/9_600;
  localparam integer TF_CLKS_PER_BIT=50_000_000/115_200;

  reg clk=0,rst_n=0,jy_uart_rx=1,tf_uart_rx=1;
  wire signed [15:0] jy_roll,jy_pitch,jy_yaw;
  wire [15:0] jy_version;
  wire jy_frame_valid,jy_checksum_error,jy_framing_error;
  wire [15:0] tf_distance,tf_strength,tf_temperature;
  wire tf_frame_valid,tf_checksum_error;
  integer errors=0;
  integer jy_valid_count=0,jy_checksum_count=0,jy_framing_count=0;
  integer tf_valid_count=0,tf_checksum_count=0;

  always #10 clk=~clk;

  jy61p_uart #(.CLK_FREQ_HZ(50_000_000),.BAUD_RATE(9_600)) u_jy61p (
    .clk(clk),.rst_n(rst_n),.uart_rx(jy_uart_rx),
    .roll_raw(jy_roll),.pitch_raw(jy_pitch),.yaw_raw(jy_yaw),
    .version_raw(jy_version),.frame_valid(jy_frame_valid),
    .checksum_error(jy_checksum_error),.framing_error(jy_framing_error)
  );

  tf_luna_uart #(.CLK_FREQ_HZ(50_000_000),.BAUD_RATE(115_200)) u_tf_luna (
    .clk(clk),.rst_n(rst_n),.uart_rx(tf_uart_rx),
    .distance_cm(tf_distance),.strength(tf_strength),
    .temperature_raw(tf_temperature),.frame_valid(tf_frame_valid),
    .checksum_error(tf_checksum_error)
  );

  always @(posedge clk)begin
    if(jy_frame_valid)jy_valid_count=jy_valid_count+1;
    if(jy_checksum_error)jy_checksum_count=jy_checksum_count+1;
    if(jy_framing_error)jy_framing_count=jy_framing_count+1;
    if(tf_frame_valid)tf_valid_count=tf_valid_count+1;
    if(tf_checksum_error)tf_checksum_count=tf_checksum_count+1;
  end

  task check(input bit condition,input string message);
    if(!condition)begin
      $display("FAIL: %s",message);
      errors=errors+1;
    end
  endtask

  task automatic send_jy_byte(input [7:0] value);
    integer bit_index;
    begin
      jy_uart_rx=0;repeat(JY_CLKS_PER_BIT)@(posedge clk);
      for(bit_index=0;bit_index<8;bit_index=bit_index+1)begin
        jy_uart_rx=value[bit_index];repeat(JY_CLKS_PER_BIT)@(posedge clk);
      end
      jy_uart_rx=1;repeat(JY_CLKS_PER_BIT)@(posedge clk);
    end
  endtask

  task automatic send_tf_byte(input [7:0] value);
    integer bit_index;
    begin
      tf_uart_rx=0;repeat(TF_CLKS_PER_BIT)@(posedge clk);
      for(bit_index=0;bit_index<8;bit_index=bit_index+1)begin
        tf_uart_rx=value[bit_index];repeat(TF_CLKS_PER_BIT)@(posedge clk);
      end
      tf_uart_rx=1;repeat(TF_CLKS_PER_BIT)@(posedge clk);
    end
  endtask

  task automatic send_jy_frame(input bit corrupt_checksum);
    reg [7:0] frame[0:9];
    reg [7:0] checksum;
    integer index;
    begin
      frame[0]=8'h55;frame[1]=8'h53;
      frame[2]=8'h34;frame[3]=8'h12;
      frame[4]=8'h00;frame[5]=8'h80;
      frame[6]=8'hfe;frame[7]=8'hff;
      frame[8]=8'h78;frame[9]=8'h56;
      checksum=0;
      for(index=0;index<10;index=index+1)begin
        checksum=checksum+frame[index];
        send_jy_byte(frame[index]);
      end
      send_jy_byte(corrupt_checksum ? checksum+1'b1 : checksum);
    end
  endtask

  task automatic send_tf_frame(
    input [15:0] distance,input [15:0] signal_strength,
    input [15:0] temperature,input bit corrupt_checksum
  );
    reg [7:0] frame[0:7];
    reg [7:0] checksum;
    integer index;
    begin
      frame[0]=8'h59;frame[1]=8'h59;
      frame[2]=distance[7:0];frame[3]=distance[15:8];
      frame[4]=signal_strength[7:0];frame[5]=signal_strength[15:8];
      frame[6]=temperature[7:0];frame[7]=temperature[15:8];
      checksum=0;
      for(index=0;index<8;index=index+1)begin
        checksum=checksum+frame[index];
        send_tf_byte(frame[index]);
      end
      send_tf_byte(corrupt_checksum ? checksum+1'b1 : checksum);
    end
  endtask

  initial begin
    repeat(5)@(posedge clk);
    @(negedge clk);rst_n=1;
    repeat(5)@(posedge clk);

    send_jy_frame(0);repeat(6)@(posedge clk);
    check(jy_valid_count==1 && jy_checksum_count==0 && jy_framing_count==0,
          "valid JY61P frame did not produce exactly one clean sample");
    check(jy_roll===16'h1234 && jy_pitch===16'h8000 &&
          jy_yaw===16'hfffe && jy_version===16'h5678,
          "JY61P angle/version byte order mismatch");

    send_jy_frame(1);repeat(6)@(posedge clk);
    check(jy_valid_count==1 && jy_checksum_count==1 && jy_framing_count==0,
          "bad JY61P checksum was accepted or not reported");
    check(jy_roll===16'h1234 && jy_pitch===16'h8000 &&
          jy_yaw===16'hfffe && jy_version===16'h5678,
          "bad JY61P frame overwrote the last valid sample");

    send_jy_frame(0);repeat(6)@(posedge clk);
    check(jy_valid_count==2 && jy_checksum_count==1 && jy_framing_count==0,
          "JY61P parser did not recover after checksum rejection");

    send_tf_frame(16'h1234,16'h5678,16'h9abc,0);repeat(6)@(posedge clk);
    check(tf_valid_count==1 && tf_checksum_count==0,
          "valid TF-Luna frame did not produce exactly one sample");
    check(tf_distance===16'h1234 && tf_strength===16'h5678 &&
          tf_temperature===16'h9abc,"TF-Luna payload byte order mismatch");

    send_tf_frame(16'h0102,16'h0304,16'h0506,1);repeat(6)@(posedge clk);
    check(tf_valid_count==1 && tf_checksum_count==1,
          "bad TF-Luna checksum was accepted or not reported");
    check(tf_distance===16'h1234 && tf_strength===16'h5678 &&
          tf_temperature===16'h9abc,
          "bad TF-Luna frame overwrote the last valid sample");

    send_tf_frame(16'h0201,16'h0403,16'h0605,0);repeat(6)@(posedge clk);
    check(tf_valid_count==2 && tf_checksum_count==1,
          "TF-Luna parser did not recover after checksum rejection");
    check(tf_distance===16'h0201 && tf_strength===16'h0403 &&
          tf_temperature===16'h0605,"TF-Luna recovery sample mismatch");

    if(errors==0)$display("PASS: tb_choice3_sensor_protocols");
    else begin
      $display("FAIL: tb_choice3_sensor_protocols errors=%0d",errors);
      $fatal(1);
    end
    $finish;
  end

  initial begin #50_000_000;$display("FAIL: timeout");$fatal(1);end
endmodule
