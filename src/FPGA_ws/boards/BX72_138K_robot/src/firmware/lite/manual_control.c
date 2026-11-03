#include "manual_control.h"

/* 沿用原人工控制的速度和20ms斜坡；轮速闭环及独立近障保护仍在RTL。 */
static int32_t clamp_i32(int32_t value, int32_t low, int32_t high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static uint32_t abs_i32(int32_t value)
{
    return value < 0 ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
}

static uint32_t distance_xy(int32_t x, int32_t y)
{
    uint32_t ax = abs_i32(x), ay = abs_i32(y);
    uint32_t hi = ax > ay ? ax : ay, lo = ax > ay ? ay : ax;
    return hi + (uint32_t)(((uint64_t)lo * 414u) / 1000u);
}

static void zero_slew(bx72_manual_state_t *state)
{
    state->vx_mm_s = state->vy_mm_s = state->wz_mrad_s = 0;
}

void bx72_manual_init(bx72_manual_state_t *state)
{
    zero_slew(state);
    state->command_seq = state->mode_seq = 0u;
    state->initialized = state->await_command = 0u;
}

int bx72_manual_control(bx72_manual_state_t *state, uint32_t status,
                         int pose_ok, uint32_t command_seq,
                         uint32_t mode_seq, int32_t vx, int32_t vy, int32_t wz,
                         bx72_velocity_t *out)
{
    uint32_t magnitude;
    int32_t dx, dy;
    out->vx_mm_s = out->vy_mm_s = out->wz_mrad_s = 0;
    if (!state->initialized) {
        state->initialized = 1u;
        state->command_seq = command_seq;
        state->mode_seq = mode_seq;
    } else if (mode_seq != state->mode_seq) {
        /* 模式停车之后须收到新的速度序号，旧命令不能被CPU重新提交。 */
        state->mode_seq = mode_seq;
        state->command_seq = command_seq;
        state->await_command = 1u;
        zero_slew(state);
    } else if (command_seq != state->command_seq) {
        state->command_seq = command_seq;
        state->await_command = 0u;
    }
    if (state->await_command || !pose_ok ||
        (status & (BX72_STATUS_LINK_DOWN | BX72_STATUS_GUARD_STOP)) != 0u ||
        (status & (BX72_STATUS_IMU_OK | BX72_STATUS_LIDAR_OK | BX72_STATUS_WHEEL_OK)) !=
            (BX72_STATUS_IMU_OK | BX72_STATUS_LIDAR_OK | BX72_STATUS_WHEEL_OK)) {
        zero_slew(state);
        return 0;
    }
    out->vx_mm_s = clamp_i32(vx, -400, 400);
    out->vy_mm_s = clamp_i32(vy, -400, 400);
    out->wz_mrad_s = clamp_i32(wz, -800, 800);
    magnitude = distance_xy(out->vx_mm_s, out->vy_mm_s);
    if (magnitude > 400u) {
        out->vx_mm_s = (int32_t)((int64_t)out->vx_mm_s * 400 / magnitude);
        out->vy_mm_s = (int32_t)((int64_t)out->vy_mm_s * 400 / magnitude);
    }
    if (!out->vx_mm_s && !out->vy_mm_s && !out->wz_mrad_s) {
        zero_slew(state);
        return 1;
    }
    dx = out->vx_mm_s - state->vx_mm_s;
    dy = out->vy_mm_s - state->vy_mm_s;
    magnitude = distance_xy(dx, dy);
    if (magnitude > 10u) {
        out->vx_mm_s = state->vx_mm_s + (int32_t)((int64_t)dx * 10 / magnitude);
        out->vy_mm_s = state->vy_mm_s + (int32_t)((int64_t)dy * 10 / magnitude);
    }
    out->wz_mrad_s = clamp_i32(out->wz_mrad_s,
        state->wz_mrad_s - 30, state->wz_mrad_s + 30);
    state->vx_mm_s = out->vx_mm_s;
    state->vy_mm_s = out->vy_mm_s;
    state->wz_mrad_s = out->wz_mrad_s;
    return 1;
}
