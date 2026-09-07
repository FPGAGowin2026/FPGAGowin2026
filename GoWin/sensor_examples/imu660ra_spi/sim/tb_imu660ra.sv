`timescale 1ns/1ps
module tb_imu660ra;
 logic clk=0,reset_n=0,miso=0;logic[2:0]axis_select=0;wire cs_n,sclk,mosi,initialized,sensor_ok,configuration_required,sample_valid,protocol_error;wire signed[15:0]axis_data;
 integer bit_no=0,id_reads=0,acc_reads=0;logic[7:0]command=0;logic[15:0]id_response;logic[55:0]data_response;logic error_seen=0;
 sensor_example_top#(.CLK_DIV(2),.STARTUP_DELAY_CYCLES(3),.SAMPLE_INTERVAL_CYCLES(4),.ID_RETRY_LIMIT(4))dut(.clk(clk),.reset_n(reset_n),.imu_miso(miso),.imu_cs_n(cs_n),.imu_sclk(sclk),.imu_mosi(mosi),.*);
 always#5 clk=~clk;always@(posedge clk)if(protocol_error)error_seen<=1;
 always@(negedge cs_n)begin if(sclk!==0)$fatal(1,"IMU660RA SPI mode error");bit_no=0;command=0;miso=0;end
 always@(posedge sclk)if(!cs_n)begin if(bit_no<8)command={command[6:0],mosi};bit_no=bit_no+1;end
 always@(negedge sclk)if(!cs_n&&bit_no>=8)begin
  if(command==8'h80)begin id_response={8'h00,(id_reads==0)?8'h00:8'h24};if(bit_no<24)miso=id_response[23-bit_no];end
  else begin if(command==8'h8C)data_response=(acc_reads==0)?56'h00_FFFFFFFFFFFF:56'h00_0100_FEFF_3412;else data_response=56'h00_0080_FF7F_A5A5;if(bit_no<64)miso=data_response[63-bit_no];end
 end
 always@(posedge cs_n)begin if(command==8'h80)id_reads=id_reads+1;if(command==8'h8C)acc_reads=acc_reads+1;end
 initial begin repeat(3)@(posedge clk);reset_n=1;repeat(30000)begin @(posedge clk);if(sample_valid)begin
  if(!initialized||!sensor_ok||!configuration_required||!error_seen||id_reads<2||acc_reads<2)$fatal(1,"identity/dummy/error boundary not covered");
  axis_select=0;#1;if(axis_data!==16'sh0001)$fatal(1,"RA little endian X");axis_select=1;#1;if(axis_data!==-16'sd2)$fatal(1,"RA little endian Y");axis_select=3;#1;if(axis_data!==-16'sh8000)$fatal(1,"RA gyro X");
  $display("PASS: IMU660RA BMI270 transport validates identity/dummy byte and raw-frame recovery");$finish;end end $fatal(1,"IMU660RA timeout");end
endmodule
