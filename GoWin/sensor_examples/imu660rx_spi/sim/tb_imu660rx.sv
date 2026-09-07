`timescale 1ns/1ps
module tb_imu660rx;
 logic clk=0,reset_n=0,miso=0;logic[2:0]axis_select=0;wire cs_n,sclk,mosi,initialized,sensor_ok,configuration_required,sample_valid,protocol_error;wire[1:0]detected_variant;wire signed[15:0]axis_data;
 integer bit_no=0,idb_reads=0,acc_reads=0;logic[7:0]command=0,id_byte;logic[15:0]id_resp;logic[47:0]data_resp;logic error_seen=0;
 sensor_example_top#(.CLK_DIV(2),.STARTUP_DELAY_CYCLES(3),.SAMPLE_INTERVAL_CYCLES(4),.ID_RETRY_LIMIT(4))dut(.clk(clk),.reset_n(reset_n),.imu_miso(miso),.imu_cs_n(cs_n),.imu_sclk(sclk),.imu_mosi(mosi),.*);
 always#5 clk=~clk;always@(posedge clk)if(protocol_error)error_seen<=1;
 always@(negedge cs_n)begin if(sclk!==0)$fatal(1,"IMU660RX mode error");bit_no=0;command=0;miso=0;end
 always@(posedge sclk)if(!cs_n)begin if(bit_no<8)command={command[6:0],mosi};bit_no=bit_no+1;end
 always@(negedge sclk)if(!cs_n&&bit_no>=8)begin
  if(command==8'h80)begin id_resp=16'h0000;if(bit_no<24)miso=id_resp[23-bit_no];end
  else if(command==8'h8F)begin id_byte=(idb_reads==0)?8'h00:8'h6B;if(bit_no<16)miso=id_byte[15-bit_no];end
  else if(command==8'hA8||command==8'hA2)begin if(command==8'hA8)data_resp=(acc_reads==0)?48'hFFFFFFFFFFFF:48'h0100_FEFF_3412;else data_resp=48'h0080_FF7F_A5A5;if(bit_no<56)miso=data_resp[55-bit_no];end
  else miso=0;
 end
 always@(posedge cs_n)begin if(command==8'h8F)idb_reads=idb_reads+1;if(command==8'hA8)acc_reads=acc_reads+1;end
 initial begin repeat(3)@(posedge clk);reset_n=1;repeat(40000)begin @(posedge clk);if(sample_valid)begin if(!initialized||!sensor_ok||detected_variant!=2||configuration_required||!error_seen||idb_reads<2||acc_reads<2)$fatal(1,"RX variant/error recovery missing");axis_select=0;#1;if(axis_data!==16'sh0001)$fatal(1,"RX accel X");axis_select=3;#1;if(axis_data!==-16'sh8000)$fatal(1,"RX gyro X");$display("PASS: IMU660RX probes RA/RB identities and recovers RB raw-frame protocol errors");$finish;end end $fatal(1,"IMU660RX timeout");end
endmodule
