`timescale 1ns/1ps

// PicoRV32 service plane.  Software can issue only high-level commands; PWM,
// direction and STBY remain private to choice3_control_core and the final
// physical safety gate.  The read-only extension window exposes AI/host lease
// and anti-replay evidence to firmware without granting a bypass path.
module choice3_robot_service_regs (
    input  wire        wb_clk_i,input wire wb_rst_i,
    input  wire        wb_cyc_i,input wire wb_stb_i,input wire wb_we_i,
    input  wire [31:0] wb_adr_i,input wire [31:0] wb_dat_i,
    input  wire [3:0]  wb_sel_i,output reg [31:0] wb_dat_o,
    output reg         wb_ack_o,
    input  wire [7:0]  health_i,input wire [7:0] fault_i,
    input  wire [3:0]  nav_state_i,input wire route_done_i,
    input  wire        drive_enabled_i,input wire armed_i,
    input  wire        cpu_watchdog_fault_i,
    input  wire [15:0] fused_range_mm_i,input wire fused_range_valid_i,
    input  wire        front_blocked_i,
    input  wire signed [15:0] roll_deg_i,input wire signed [15:0] pitch_deg_i,
    input  wire signed [15:0] yaw_deg_i,
    input  wire        ext_lease_active_i,input wire ext_sequence_seen_i,
    input  wire        ext_accepted_pulse_i,input wire ext_rejected_pulse_i,
    input  wire        ext_timeout_pulse_i,input wire ext_conflict_pulse_i,
    input  wire [2:0]  ext_reject_reason_i,
    input  wire [31:0] ext_last_sequence_i,
    input  wire [31:0] ext_accepted_count_i,
    input  wire [31:0] ext_rejected_count_i,
    input  wire [31:0] ext_timeout_count_i,
    output reg  [31:0] heartbeat_o,output reg heartbeat_write_o,
    output reg         arm_pulse_o,output reg disarm_pulse_o,
    output reg         route_start_pulse_o,output reg clear_fault_pulse_o,
    output reg         manual_command_valid_o,output reg [2:0] manual_motion_o,
    output reg  [31:0] scratch_o,output reg [31:0] transaction_count_o,
    output reg  [31:0] error_count_o
);
    localparam [31:0] ADDR_SYS_ID       = 32'h2000_0000;
    localparam [31:0] ADDR_VERSION      = 32'h2000_0004;
    localparam [31:0] ADDR_CAPABILITIES = 32'h2000_0008;
    localparam [31:0] ADDR_HEARTBEAT    = 32'h2000_000c;
    localparam [31:0] ADDR_STATUS0      = 32'h2000_1000;
    localparam [31:0] ADDR_RANGE        = 32'h2000_1004;
    localparam [31:0] ADDR_ANGLES0      = 32'h2000_1008;
    localparam [31:0] ADDR_ANGLES1      = 32'h2000_100c;
    localparam [31:0] ADDR_EXT_STATUS   = 32'h2000_1014;
    localparam [31:0] ADDR_EXT_LAST_SEQ = 32'h2000_1018;
    localparam [31:0] ADDR_EXT_ACCEPTED = 32'h2000_101c;
    localparam [31:0] ADDR_EXT_REJECTED = 32'h2000_1020;
    localparam [31:0] ADDR_EXT_TIMEOUTS = 32'h2000_1024;
    localparam [31:0] ADDR_COMMAND      = 32'h2000_2000;
    localparam [31:0] ADDR_SCRATCH      = 32'h2000_f000;
    localparam [31:0] ADDR_TXN_COUNT    = 32'h2000_f004;
    localparam [31:0] ADDR_ERROR_COUNT  = 32'h2000_f008;
    localparam [31:0] INVALID_DATA      = 32'hdead_beef;

    wire [31:0] status0_w = {health_i,fault_i,nav_state_i,route_done_i,
      drive_enabled_i,armed_i,1'b0,cpu_watchdog_fault_i,7'b0};
    wire [31:0] range_w = {fused_range_valid_i,front_blocked_i,14'b0,
      fused_range_mm_i};
    wire [31:0] angles0_w = {pitch_deg_i,roll_deg_i};
    wire [31:0] angles1_w = {16'd0,yaw_deg_i};
    wire [31:0] ext_status_w = {23'd0,ext_conflict_pulse_i,
      ext_timeout_pulse_i,ext_rejected_pulse_i,ext_accepted_pulse_i,
      ext_reject_reason_i,ext_sequence_seen_i,ext_lease_active_i};
    wire request_w = wb_cyc_i && wb_stb_i;
    reg request_seen_q;

    function [31:0] merge_selected_bytes;
      input [31:0] old_value;input [31:0] new_value;input [3:0] byte_select;
      begin
        merge_selected_bytes=old_value;
        if(byte_select[0])merge_selected_bytes[7:0]=new_value[7:0];
        if(byte_select[1])merge_selected_bytes[15:8]=new_value[15:8];
        if(byte_select[2])merge_selected_bytes[23:16]=new_value[23:16];
        if(byte_select[3])merge_selected_bytes[31:24]=new_value[31:24];
      end
    endfunction

    always @(posedge wb_clk_i) begin
      if(wb_rst_i)begin
        wb_dat_o<=0;wb_ack_o<=0;request_seen_q<=0;heartbeat_o<=0;
        heartbeat_write_o<=0;arm_pulse_o<=0;disarm_pulse_o<=0;
        route_start_pulse_o<=0;clear_fault_pulse_o<=0;
        manual_command_valid_o<=0;manual_motion_o<=0;scratch_o<=0;
        transaction_count_o<=0;error_count_o<=0;
      end else begin
        wb_ack_o<=0;heartbeat_write_o<=0;arm_pulse_o<=0;disarm_pulse_o<=0;
        route_start_pulse_o<=0;clear_fault_pulse_o<=0;
        manual_command_valid_o<=0;
        if(!request_w)request_seen_q<=0;
        else if(!request_seen_q)begin
          request_seen_q<=1;wb_ack_o<=1;wb_dat_o<=0;
          transaction_count_o<=transaction_count_o+1'b1;
          if(wb_adr_i[1:0]!=0)begin
            wb_dat_o<=INVALID_DATA;error_count_o<=error_count_o+1'b1;
          end else if(wb_we_i)begin
            case(wb_adr_i)
              ADDR_HEARTBEAT:begin
                heartbeat_o<=merge_selected_bytes(heartbeat_o,wb_dat_i,wb_sel_i);
                heartbeat_write_o<=1;
              end
              ADDR_COMMAND:begin
                arm_pulse_o<=wb_dat_i[0];disarm_pulse_o<=wb_dat_i[1];
                route_start_pulse_o<=wb_dat_i[2];clear_fault_pulse_o<=wb_dat_i[3];
                manual_command_valid_o<=wb_dat_i[8];manual_motion_o<=wb_dat_i[11:9];
              end
              ADDR_SCRATCH:scratch_o<=merge_selected_bytes(scratch_o,wb_dat_i,wb_sel_i);
              default:begin wb_dat_o<=INVALID_DATA;error_count_o<=error_count_o+1'b1;end
            endcase
          end else begin
            case(wb_adr_i)
              ADDR_SYS_ID:       wb_dat_o<=32'h5242_4f54;
              ADDR_VERSION:      wb_dat_o<=32'h0003_0000;
              // bit 6 = external guarded command lease; lower bits retain
              // camera/IMU/range/encoder/navigation/DVI capabilities.
              ADDR_CAPABILITIES: wb_dat_o<=32'h0000_007f;
              ADDR_HEARTBEAT:    wb_dat_o<=heartbeat_o;
              ADDR_STATUS0:      wb_dat_o<=status0_w;
              ADDR_RANGE:        wb_dat_o<=range_w;
              ADDR_ANGLES0:      wb_dat_o<=angles0_w;
              ADDR_ANGLES1:      wb_dat_o<=angles1_w;
              ADDR_EXT_STATUS:   wb_dat_o<=ext_status_w;
              ADDR_EXT_LAST_SEQ: wb_dat_o<=ext_last_sequence_i;
              ADDR_EXT_ACCEPTED: wb_dat_o<=ext_accepted_count_i;
              ADDR_EXT_REJECTED: wb_dat_o<=ext_rejected_count_i;
              ADDR_EXT_TIMEOUTS: wb_dat_o<=ext_timeout_count_i;
              ADDR_SCRATCH:      wb_dat_o<=scratch_o;
              ADDR_TXN_COUNT:    wb_dat_o<=transaction_count_o;
              ADDR_ERROR_COUNT:  wb_dat_o<=error_count_o;
              default:begin wb_dat_o<=INVALID_DATA;error_count_o<=error_count_o+1'b1;end
            endcase
          end
        end
      end
    end
endmodule
