// Compatibility alias for a naming error in the tutorial's key_led_tb.v.
// The board design is named key_led_top, while the supplied testbench
// instantiates key_led with the exact same ports.
module key_led(
    input  wire       clk,
    input  wire       reset_n,
    input  wire       key_in0,
    input  wire       key_in1,
    input  wire       key_in2,
    input  wire       key_in3,
    output wire [7:0] led
);
    key_led_top impl (
        .clk(clk),
        .reset_n(reset_n),
        .key_in0(key_in0),
        .key_in1(key_in1),
        .key_in2(key_in2),
        .key_in3(key_in3),
        .led(led)
    );
endmodule
