`timescale 1ns/1ps
module sensor_example_top #(parameter integer CLK_FREQ_HZ=27000000,parameter integer BAUD_RATE=9600)(
    input wire clk,input wire rst_n,input wire gy39_rx,input wire[2:0] debug_select,
    output reg[31:0] debug_data,output wire debug_valid,output wire protocol_error);
    wire [31:0] lux_centi_lux;
    wire signed [15:0] temperature_centi_c;
    wire [31:0] pressure_centi_pa;
    wire [15:0] humidity_centi_rh;
    wire signed [15:0] altitude_m;
    wire lux_valid,environment_valid,checksum_error,format_error,framing_error;
    assign debug_valid=(debug_select==3'd0)?lux_valid:environment_valid;
    assign protocol_error=checksum_error|format_error|framing_error;
    always @* begin
        case(debug_select)
            3'd0:debug_data=lux_centi_lux;
            3'd1:debug_data={{16{temperature_centi_c[15]}},temperature_centi_c};
            3'd2:debug_data=pressure_centi_pa;
            3'd3:debug_data={16'd0,humidity_centi_rh};
            3'd4:debug_data={{16{altitude_m[15]}},altitude_m};
            default:debug_data=32'd0;
        endcase
    end
    gy39_uart #(.CLK_FREQ_HZ(CLK_FREQ_HZ),.BAUD_RATE(BAUD_RATE)) u_core(
        .clk(clk),.rst_n(rst_n),.uart_rx(gy39_rx),.lux_centi_lux(lux_centi_lux),
        .temperature_centi_c(temperature_centi_c),.pressure_centi_pa(pressure_centi_pa),
        .humidity_centi_rh(humidity_centi_rh),.altitude_m(altitude_m),.lux_valid(lux_valid),
        .environment_valid(environment_valid),.checksum_error(checksum_error),
        .format_error(format_error),.framing_error(framing_error));
endmodule
