////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 1999-2008 Easics NV.
// This source file may be used and distributed without restriction
// provided that this copyright statement is not removed from the file
// and that any derivative work contains the original copyright notice
// and the associated disclaimer.
//
// THIS SOURCE FILE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
// WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Purpose : synthesizable CRC function
//   * polynomial: x^32 + x^26 + x^23 + x^22 + x^16 + x^12 + x^11 + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x^1 + 1
//   * data width: 8
//
// Info : tools@easics.be
//        http://www.easics.com
////////////////////////////////////////////////////////////////////////////////

module crc32_d8 (
    input wire clk, reset_p,
    input wire [7:0] data,
    input wire crc_init, crc_en,
    output wire [31:0] crc_result
);
    reg [31:0] state;
    assign crc_result = ~state;

    function [31:0] next_crc;
        input [31:0] current;
        input [7:0] octet;
        reg [31:0] work;
        integer bit_index;
        begin
            work = current ^ {24'd0, octet};
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1)
                work = work[0] ? (work >> 1) ^ 32'hEDB8_8320 : work >> 1;
            next_crc = work;
        end
    endfunction

    // 反射形式按线缆 LSB 优先处理，输出含最终取反，保持原收发接口。
    always @(posedge clk or posedge reset_p) begin
        if (reset_p)
            state <= 32'hFFFF_FFFF;
        else if (crc_init)
            state <= 32'hFFFF_FFFF;
        else if (crc_en)
            state <= next_crc(state, data);
    end
endmodule