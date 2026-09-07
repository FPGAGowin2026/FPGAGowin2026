`timescale 1ns/1ps
module tb_icm20602;
 logic clk=0,reset_n=0,miso=0;logic [2:0] axis_select=0;wire cs_n,sclk,mosi,initialized,sensor_ok,sample_valid,protocol_error;wire signed[15:0]axis_data;
 integer bit_no=0,who_reads=0,burst_reads=0,status_reads=0;logic[7:0]command=0;logic[111:0]burst_data;logic[7:0]byte_data;logic error_seen=0;
 sensor_example_top #(.CLK_DIV(2),.STARTUP_DELAY_CYCLES(3),.RESET_DELAY_CYCLES(3),.SAMPLE_INTERVAL_CYCLES(4),.ID_RETRY_LIMIT(4))dut(
  .clk(clk),.reset_n(reset_n),.imu_miso(miso),.imu_cs_n(cs_n),.imu_sclk(sclk),.imu_mosi(mosi),.*);
 always #5 clk=~clk;always@(posedge clk)if(protocol_error)error_seen<=1;
 always@(negedge cs_n)begin if(sclk!==0)$fatal(1,"ICM20602 SCLK must idle low");bit_no=0;command=0;miso=0;end
 always@(posedge sclk)if(!cs_n)begin if(bit_no<8)command={command[6:0],mosi};bit_no=bit_no+1;end
 always@(negedge sclk)if(!cs_n&&bit_no>=8)begin
  if(command==8'hF5)begin byte_data=(who_reads==0)?8'h00:8'h12;if(bit_no<16)miso=byte_data[15-bit_no];end
  else if(command==8'hBA)begin byte_data=(status_reads==0)?8'h00:8'h01;if(bit_no<16)miso=byte_data[15-bit_no];end
  else if(command==8'hBB)begin burst_data=(burst_reads==0)?{112{1'b1}}:112'h0001_FFFE_1234_0000_8000_7FFF_A5A5;if(bit_no<120)miso=burst_data[119-bit_no];end
  else miso=0;
 end
 always@(posedge cs_n)begin if(command==8'hF5)who_reads=who_reads+1;if(command==8'hBA)status_reads=status_reads+1;if(command==8'hBB)burst_reads=burst_reads+1;end
 initial begin repeat(3)@(posedge clk);reset_n=1;repeat(30000)begin @(posedge clk);if(sample_valid)begin
  if(!initialized||!sensor_ok||!error_seen||who_reads<2||burst_reads<2)$fatal(1,"identity/protocol recovery not exercised");
  axis_select=0;#1;if(axis_data!==16'sh0001)$fatal(1,"accel X byte order");axis_select=1;#1;if(axis_data!==-16'sd2)$fatal(1,"accel Y byte order");
  axis_select=3;#1;if(axis_data!==-16'sh8000)$fatal(1,"gyro X byte order");axis_select=5;#1;if(axis_data!==-16'sh5A5B)$fatal(1,"gyro Z byte order");
  $display("PASS: ICM20602 Mode-0 identity, data-ready and burst error recovery verified");$finish;end end $fatal(1,"ICM20602 timeout");end
endmodule
