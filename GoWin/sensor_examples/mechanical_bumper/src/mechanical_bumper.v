`timescale 1ns/1ps

// Dual active-low mechanical bumper input conditioner.
// Each switch uses a saturating integrator for debounce. collision_latched
// preserves a hit until clear_latched is asserted while both bumpers are free.
module mechanical_bumper #(
    parameter integer ACTIVE_LOW      = 1,
    parameter integer DEBOUNCE_CYCLES = 270000
) (
    input  wire clk,
    input  wire rst_n,
    input  wire bumper_left_in,
    input  wire bumper_right_in,
    input  wire clear_latched,
    output reg  left_pressed,
    output reg  right_pressed,
    output wire collision,
    output reg  collision_event,
    output reg  collision_latched
);

    localparam integer DEBOUNCE_LIMIT =
        (DEBOUNCE_CYCLES < 1) ? 1 : DEBOUNCE_CYCLES;

    (* ASYNC_REG = "TRUE" *) reg left_meta;
    (* ASYNC_REG = "TRUE" *) reg left_sync;
    (* ASYNC_REG = "TRUE" *) reg right_meta;
    (* ASYNC_REG = "TRUE" *) reg right_sync;
    wire left_active;
    wire right_active;
    reg [31:0] left_count;
    reg [31:0] right_count;
    reg collision_previous;

    assign left_active  = (ACTIVE_LOW != 0) ? ~left_sync : left_sync;
    assign right_active = (ACTIVE_LOW != 0) ? ~right_sync : right_sync;
    assign collision = left_pressed | right_pressed;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            left_meta  <= (ACTIVE_LOW != 0) ? 1'b1 : 1'b0;
            left_sync  <= (ACTIVE_LOW != 0) ? 1'b1 : 1'b0;
            right_meta <= (ACTIVE_LOW != 0) ? 1'b1 : 1'b0;
            right_sync <= (ACTIVE_LOW != 0) ? 1'b1 : 1'b0;
        end else begin
            left_meta  <= bumper_left_in;
            left_sync  <= left_meta;
            right_meta <= bumper_right_in;
            right_sync <= right_meta;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            left_count    <= 32'd0;
            right_count   <= 32'd0;
            left_pressed  <= 1'b0;
            right_pressed <= 1'b0;
        end else begin
            if (left_active) begin
                if (left_count < DEBOUNCE_LIMIT)
                    left_count <= left_count + 1'b1;
                if (!left_pressed &&
                    (left_count >= (DEBOUNCE_LIMIT - 1)))
                    left_pressed <= 1'b1;
            end else begin
                if (left_count != 0)
                    left_count <= left_count - 1'b1;
                if (left_pressed && (left_count <= 1))
                    left_pressed <= 1'b0;
            end

            if (right_active) begin
                if (right_count < DEBOUNCE_LIMIT)
                    right_count <= right_count + 1'b1;
                if (!right_pressed &&
                    (right_count >= (DEBOUNCE_LIMIT - 1)))
                    right_pressed <= 1'b1;
            end else begin
                if (right_count != 0)
                    right_count <= right_count - 1'b1;
                if (right_pressed && (right_count <= 1))
                    right_pressed <= 1'b0;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            collision_previous <= 1'b0;
            collision_event    <= 1'b0;
            collision_latched  <= 1'b0;
        end else begin
            collision_event    <= collision & ~collision_previous;
            collision_previous <= collision;

            // An active collision has priority over software clearing.
            if (collision)
                collision_latched <= 1'b1;
            else if (clear_latched)
                collision_latched <= 1'b0;
        end
    end

endmodule
