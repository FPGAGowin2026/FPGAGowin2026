#ifndef BX72_MANUAL_CONTROL_H
#define BX72_MANUAL_CONTROL_H

#include "bx72_lite.h"

typedef struct {
    int32_t vx_mm_s, vy_mm_s, wz_mrad_s;
    uint32_t command_seq, mode_seq;
    uint8_t initialized, await_command;
} bx72_manual_state_t;

typedef struct { int32_t vx_mm_s, vy_mm_s, wz_mrad_s; } bx72_velocity_t;

void bx72_manual_init(bx72_manual_state_t *state);
/* 返回1表示输入链路允许控制（零速也是合法命令），0表示强制停车。 */
int bx72_manual_control(bx72_manual_state_t *state, uint32_t status,
                         int pose_ok, uint32_t command_seq,
                         uint32_t mode_seq, int32_t vx, int32_t vy, int32_t wz,
                         bx72_velocity_t *out);

#endif
