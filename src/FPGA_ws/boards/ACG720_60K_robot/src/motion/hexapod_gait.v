module hexapod_gait #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer WATCHDOG_FRAMES = 15
)(
    input wire clk, reset_n, link_up,
    input wire cmd_valid,
    input wire [3:0] cmd,
    output reg [17:0] servo_pwm,
    output wire [23:0] status
);
    localparam integer CLOCKS_PER_US = CLK_HZ / 1_000_000;
    localparam integer DIV_WIDTH = CLOCKS_PER_US > 1 ? $clog2(CLOCKS_PER_US) : 1;
    localparam [10:0] LOWER_HOME_US = 11'd1134, MIDDLE_HOME_US = 11'd1134,
                      UPPER_HOME_US = 11'd1500;
    // 左后腿下舵机 C14：增大 122 µs 对应上翘约 10°。
    localparam [10:0] L3_LOWER_HOME_US = 11'd1256;
    // 右中腿下舵机 B13 减少 122 µs，约下转 10°。
    localparam [10:0] R2_LOWER_HOME_US = 11'd1012;
    localparam [10:0] MIN_US = 11'd900, MAX_US = 11'd1750;
    localparam [10:0] TRANSITION_STEP_US = 11'd20, WALK_STEP_US = 11'd40;
    localparam [3:0] RELEASE = 4'd7, HOME = 4'd8, TRIPOD = 4'd9, RIPPLE = 4'd10;

    reg [DIV_WIDTH-1:0] us_div;
    reg [14:0] frame_us;
    wire us_tick = us_div == CLOCKS_PER_US - 1;
    wire frame_tick = us_tick && frame_us == 15'd19999;

    reg [2:0] requested, direction;
    reg [7:0] watchdog;
    reg selected_ripple, active_ripple;
    reg [6:0] frame_index;
    reg enabled, homing, acquiring, release_pending, was_moving;

    wire command = link_up && cmd_valid;
    wire release_now = release_pending || (command && cmd == RELEASE);
    wire home_now = command && cmd == HOME;
    wire freeze_now = command && cmd == 0;
    wire moving = link_up && requested != 0 && watchdog != 0 && !homing;
    wire start_trajectory = direction != requested || (!was_moving && active_ripple != selected_ripple);
    wire apply_motion = moving && !start_trajectory && !release_now && !home_now && !freeze_now;
    wire last_frame = frame_index == (active_ripple ? 7'd89 : 7'd59);
    wire [17:0] at_home, at_target;

    assign status = {selected_ripple, active_ripple, 3'd0, moving ? requested : 3'd0,
                     homing, 4'd0, (apply_motion && enabled) ? direction : 3'd0,
                     enabled, frame_index};

    wire [2:0] direction_index = direction == 0 ? 3'd0 : direction - 1'b1;
    wire [10:0] rom_address = (active_ripple ? 11'd540 : 11'd0)
                            + direction_index * 11'd90 + {4'd0, frame_index};
    (* syn_romstyle = "block_rom" *) reg [161:0] gait_rom [0:1079];
    reg [161:0] rom_word;
    initial $readmemh("src/motion/gait_offsets.hex", gait_rom);
    always @(posedge clk)
        rom_word <= gait_rom[rom_address];

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            us_div <= 0;
            frame_us <= 0;
            requested <= 0;
            direction <= 0;
            watchdog <= 0;
            selected_ripple <= 0;
            active_ripple <= 0;
            frame_index <= 0;
            enabled <= 0;
            homing <= 0;
            acquiring <= 0;
            release_pending <= 0;
            was_moving <= 0;
        end else begin
            if (us_tick) begin
                us_div <= 0;
                frame_us <= frame_tick ? 15'd0 : frame_us + 1'b1;
            end else
                us_div <= us_div + 1'b1;

            if (command && (cmd == TRIPOD || cmd == RIPPLE))
                selected_ripple <= (cmd == RIPPLE);

            if (!link_up || release_now || home_now || freeze_now) begin
                requested <= 0;
                watchdog <= 0;
            end else if (command && cmd >= 1 && cmd <= 6) begin
                requested <= cmd[2:0];
                watchdog <= WATCHDOG_FRAMES;
            end else if (frame_tick && watchdog != 0)
                watchdog <= watchdog - 1'b1;

            // 松键和失联冻结最后一帧。空格才以限速回到低姿态。
            release_pending <= release_now && !frame_tick;
            if (!link_up || release_now || freeze_now)
                homing <= 0;
            else if (home_now)
                homing <= 1;
            else if (frame_tick && homing && (&at_home) && enabled)
                homing <= 0;

            if (frame_tick) begin
                was_moving <= moving && !release_now && !home_now && !freeze_now;
                if (release_now) begin
                    enabled <= 0;
                    direction <= 0;
                    frame_index <= 0;
                end else if (link_up && homing && !freeze_now) begin
                    enabled <= 1;
                    direction <= 0;
                    frame_index <= 0;
                end else if (moving && !freeze_now && !home_now) begin
                    enabled <= 1;
                    if (start_trajectory) begin
                        direction <= requested;
                        active_ripple <= selected_ripple;
                        frame_index <= 0;
                        acquiring <= 1;
                    end else if (!acquiring || (&at_target)) begin
                        acquiring <= 0;
                        if (last_frame) begin
                            frame_index <= 0;
                            active_ripple <= selected_ripple;
                            acquiring <= (selected_ripple != active_ripple);
                        end else
                            frame_index <= frame_index + 1'b1;
                    end
                end
            end
        end
    end

    reg [10:0] pulse_us [0:17];
    genvar axis;
    generate for (axis = 0; axis < 18; axis = axis + 1) begin : channel
        localparam [14:0] START_US = (axis / 3) * 3000;
        localparam [10:0] HOME_US = axis == 12 ? L3_LOWER_HOME_US :
                                   axis == 9 ? R2_LOWER_HOME_US :
                                   axis % 3 == 0 ? LOWER_HOME_US :
                                   axis % 3 == 1 ? MIDDLE_HOME_US : UPPER_HOME_US;
        wire signed [8:0] offset = rom_word[axis*9 +: 9];
        wire signed [11:0] candidate_us = $signed({1'b0, HOME_US}) + offset;
        wire [10:0] target_us = candidate_us < $signed({1'b0, MIN_US}) ? MIN_US :
                                candidate_us > $signed({1'b0, MAX_US}) ? MAX_US : candidate_us[10:0];
        wire [10:0] goal_us = homing ? HOME_US : target_us;
        wire [10:0] step_us = (homing || acquiring) ? TRANSITION_STEP_US : WALK_STEP_US;
        assign at_home[axis] = pulse_us[axis] == HOME_US;
        assign at_target[axis] = pulse_us[axis] == target_us;

        always @(posedge clk or negedge reset_n) begin
            if (!reset_n) begin
                pulse_us[axis] <= HOME_US;
                servo_pwm[axis] <= 1'b0;
            end else if (us_tick) begin
                if (frame_tick) begin
                    if (release_now)
                        pulse_us[axis] <= HOME_US;
                    else if ((homing && link_up && !freeze_now) || apply_motion) begin
                        if (pulse_us[axis] + step_us < goal_us)
                            pulse_us[axis] <= pulse_us[axis] + step_us;
                        else if (pulse_us[axis] > goal_us + step_us)
                            pulse_us[axis] <= pulse_us[axis] - step_us;
                        else
                            pulse_us[axis] <= goal_us;
                    end
                end
                if (frame_us == START_US)
                    servo_pwm[axis] <= enabled;
                else if (frame_us == START_US + {4'b0, pulse_us[axis]})
                    servo_pwm[axis] <= 1'b0;
            end
        end
    end endgenerate
endmodule
