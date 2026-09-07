`timescale 1ns/1ps

// 115200 8-N-1 单字符控制台。命令：P 探活，S/? 状态，A 武装，D 撤武装，
// G 启动航点，X 清故障，F/B/L/R/Q/E/0 为手动动作。所有命令最终仍受安全门控。
module choice3_uart_console #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BAUD_RATE = 115200
) (
    input  wire clk_i, input wire rst_n_i,
    input  wire uart_rx_i, output wire uart_tx_o,
    input  wire armed_i,input wire drive_enabled_i,input wire [7:0] health_i,
    input  wire [7:0] fault_i, input wire [3:0] nav_state_i,
    input  wire [7:0] vl53_range_status_raw_i,
    input  wire arm_rejected_pulse_i,
    output reg arm_pulse_o, output reg disarm_pulse_o,
    output reg clear_fault_pulse_o, output reg route_start_pulse_o,
    output reg manual_command_valid_o,output reg [2:0] manual_motion_o
);
    localparam [3:0] MSG_STATUS=0, MSG_PONG=1, MSG_ACK_ARM=2,
                     MSG_ACK_DISARM=3, MSG_ACK_GO=4, MSG_ACK_CLEAR=5,
                     MSG_REJECT=6, MSG_UNKNOWN=7,MSG_ACK_MANUAL=8;
    wire [7:0] rx_data;
    wire rx_valid, rx_frame_error;
    reg [7:0] tx_data_q;
    reg tx_valid_q;
    wire tx_ready, tx_busy;
    reg [3:0] pending_kind_q, active_kind_q;
    reg pending_valid_q, message_active_q;
    reg [6:0] message_index_q;
    reg [7:0] status_health_q, status_fault_q,status_vl53_raw_q,previous_fault_q;
    reg fault_monitor_started_q;
    reg status_armed_q,status_drive_q;
    reg [3:0] status_nav_q;

    function [7:0] hex_char;
        input [3:0] value;
        begin hex_char=(value<10)?(8'h30+value):(8'h41+value-10); end
    endfunction
    function [6:0] message_length;
        input [3:0] kind;
        begin
            case(kind)
                MSG_STATUS: message_length=53;
                MSG_PONG: message_length=20;
                MSG_ACK_ARM: message_length=9;
                MSG_ACK_DISARM: message_length=12;
                MSG_ACK_GO: message_length=14;
                MSG_ACK_CLEAR: message_length=17;
                MSG_REJECT: message_length=18;
                MSG_ACK_MANUAL: message_length=12;
                default: message_length=13;
            endcase
        end
    endfunction
    function [7:0] fixed_byte;
        input [3:0] kind; input [6:0] index;
        reg [159:0] pong;
        reg [143:0] reject_text;
        reg [135:0] clear_text;
        reg [111:0] go_text;
        reg [103:0] unknown_text;
        reg [95:0] disarm_text;
        reg [71:0] arm_text;
        reg [95:0] manual_text;
        begin
            pong={"PONG;CHOICE3=READY",8'h0d,8'h0a};
            reject_text={"ERR;ARM_REJECTED",8'h0d,8'h0a};
            clear_text={"ACK;FAULT_CLEAR",8'h0d,8'h0a};
            go_text={"ACK;GO=ROUTE",8'h0d,8'h0a};
            unknown_text={"ERR;UNKNOWN",8'h0d,8'h0a};
            disarm_text={"ACK;DISARM",8'h0d,8'h0a};
            arm_text={"ACK;ARM",8'h0d,8'h0a};
            manual_text={"ACK;MANUAL",8'h0d,8'h0a};
            case(kind)
                MSG_PONG: fixed_byte=pong[(20-1-index)*8 +: 8];
                MSG_ACK_ARM: fixed_byte=arm_text[(9-1-index)*8 +: 8];
                MSG_ACK_DISARM: fixed_byte=disarm_text[(12-1-index)*8 +: 8];
                MSG_ACK_GO: fixed_byte=go_text[(14-1-index)*8 +: 8];
                MSG_ACK_CLEAR: fixed_byte=clear_text[(17-1-index)*8 +: 8];
                MSG_REJECT: fixed_byte=reject_text[(18-1-index)*8 +: 8];
                MSG_ACK_MANUAL: fixed_byte=manual_text[(12-1-index)*8 +: 8];
                default: fixed_byte=unknown_text[(13-1-index)*8 +: 8];
            endcase
        end
    endfunction
    function [7:0] status_byte;
        input [6:0] index;
        begin
            case(index)
                0:status_byte="S";1:status_byte="T";2:status_byte="A";
                3:status_byte="T";4:status_byte="U";5:status_byte="S";
                6:status_byte=";";7:status_byte="A";8:status_byte="R";
                9:status_byte="M";10:status_byte="=";
                11:status_byte=status_armed_q?"1":"0";12:status_byte=";";
                13:status_byte="H";14:status_byte="E";15:status_byte="A";
                16:status_byte="L";17:status_byte="T";18:status_byte="H";
                19:status_byte="=";20:status_byte=hex_char(status_health_q[7:4]);
                21:status_byte=hex_char(status_health_q[3:0]);22:status_byte=";";
                23:status_byte="F";24:status_byte="A";25:status_byte="U";
                26:status_byte="L";27:status_byte="T";28:status_byte="=";
                29:status_byte=hex_char(status_fault_q[7:4]);
                30:status_byte=hex_char(status_fault_q[3:0]);31:status_byte=";";
                32:status_byte="N";33:status_byte="A";34:status_byte="V";
                35:status_byte="=";36:status_byte=hex_char(status_nav_q);
                37:status_byte=";";38:status_byte="V";39:status_byte="L";
                40:status_byte="R";41:status_byte="S";42:status_byte="=";
                43:status_byte=hex_char(status_vl53_raw_q[7:4]);
                44:status_byte=hex_char(status_vl53_raw_q[3:0]);
                45:status_byte=";";46:status_byte="D";47:status_byte="R";
                48:status_byte="V";49:status_byte="=";
                50:status_byte=status_drive_q?"1":"0";
                51:status_byte=8'h0d;52:status_byte=8'h0a;
                default:status_byte=8'h3f;
            endcase
        end
    endfunction

    bringup_uart_rx #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.BAUD_RATE(BAUD_RATE)) u_rx(
        .clk(clk_i),.rst_n(rst_n_i),.rx_i(uart_rx_i),.data_o(rx_data),
        .data_valid_o(rx_valid),.frame_error_o(rx_frame_error));
    bringup_uart_tx #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.BAUD_RATE(BAUD_RATE)) u_tx(
        .clk(clk_i),.rst_n(rst_n_i),.data_i(tx_data_q),
        .data_valid_i(tx_valid_q),.data_ready_o(tx_ready),
        .tx_o(uart_tx_o),.busy_o(tx_busy));

    always @(posedge clk_i or negedge rst_n_i) begin
        if(!rst_n_i) begin
            arm_pulse_o<=0;disarm_pulse_o<=0;clear_fault_pulse_o<=0;
            route_start_pulse_o<=0;manual_command_valid_o<=0;manual_motion_o<=0;
            tx_data_q<=0;tx_valid_q<=0;
            pending_kind_q<=MSG_STATUS;pending_valid_q<=1;
            active_kind_q<=MSG_STATUS;message_active_q<=0;message_index_q<=0;
            status_health_q<=0;status_fault_q<=0;status_vl53_raw_q<=0;
            status_armed_q<=0;status_drive_q<=0;status_nav_q<=0;previous_fault_q<=0;
            fault_monitor_started_q<=0;
        end else begin
            arm_pulse_o<=0;disarm_pulse_o<=0;clear_fault_pulse_o<=0;
            route_start_pulse_o<=0;manual_command_valid_o<=0;tx_valid_q<=0;

            if(!message_active_q && pending_valid_q) begin
                active_kind_q<=pending_kind_q;message_index_q<=0;
                message_active_q<=1;pending_valid_q<=0;
                status_health_q<=health_i;status_fault_q<=fault_i;
                status_vl53_raw_q<=vl53_range_status_raw_i;
                status_armed_q<=armed_i;status_drive_q<=drive_enabled_i;
                status_nav_q<=nav_state_i;
            end else if(message_active_q && !tx_busy && !tx_valid_q) begin
                tx_data_q <= (active_kind_q==MSG_STATUS) ?
                             status_byte(message_index_q) :
                             fixed_byte(active_kind_q,message_index_q);
                tx_valid_q<=1;
                if(message_index_q>=message_length(active_kind_q)-1) begin
                    message_active_q<=0;message_index_q<=0;
                end else message_index_q<=message_index_q+1'b1;
            end

            if(arm_rejected_pulse_i) begin
                pending_kind_q<=MSG_REJECT;pending_valid_q<=1;
            end
            // 故障变化主动排队状态行；若正在发送则在当前消息结束后发送。
            if(!fault_monitor_started_q) begin
                previous_fault_q<=fault_i;fault_monitor_started_q<=1;
            end else if(fault_i!=previous_fault_q) begin
                previous_fault_q<=fault_i;pending_kind_q<=MSG_STATUS;pending_valid_q<=1;
            end
            if(rx_valid) begin
                pending_valid_q<=1;
                case(rx_data)
                    "P","p":pending_kind_q<=MSG_PONG;
                    "S","s","?":pending_kind_q<=MSG_STATUS;
                    "A","a":begin arm_pulse_o<=1;pending_kind_q<=MSG_ACK_ARM;end
                    "D","d":begin disarm_pulse_o<=1;pending_kind_q<=MSG_ACK_DISARM;end
                    "G","g":begin route_start_pulse_o<=1;pending_kind_q<=MSG_ACK_GO;end
                    "X","x":begin clear_fault_pulse_o<=1;pending_kind_q<=MSG_ACK_CLEAR;end
                    "0":begin manual_command_valid_o<=1;manual_motion_o<=0;pending_kind_q<=MSG_ACK_MANUAL;end
                    "F","f":begin manual_command_valid_o<=1;manual_motion_o<=1;pending_kind_q<=MSG_ACK_MANUAL;end
                    "B","b":begin manual_command_valid_o<=1;manual_motion_o<=2;pending_kind_q<=MSG_ACK_MANUAL;end
                    "L","l":begin manual_command_valid_o<=1;manual_motion_o<=3;pending_kind_q<=MSG_ACK_MANUAL;end
                    "R","r":begin manual_command_valid_o<=1;manual_motion_o<=4;pending_kind_q<=MSG_ACK_MANUAL;end
                    "Q","q":begin manual_command_valid_o<=1;manual_motion_o<=5;pending_kind_q<=MSG_ACK_MANUAL;end
                    "E","e":begin manual_command_valid_o<=1;manual_motion_o<=6;pending_kind_q<=MSG_ACK_MANUAL;end
                    default:pending_kind_q<=MSG_UNKNOWN;
                endcase
            end
        end
    end
endmodule
