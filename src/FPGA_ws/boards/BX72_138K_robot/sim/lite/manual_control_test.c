#include "manual_control.h"
#include <assert.h>
#include <stdio.h>

#define READY (BX72_STATUS_IMU_OK | BX72_STATUS_LIDAR_OK | BX72_STATUS_WHEEL_OK)

int main(void)
{
    bx72_manual_state_t state;
    bx72_velocity_t out;
    unsigned int i;
    bx72_manual_init(&state);
    /* 硬件闭锁等待CPU时，过滤后的guard状态允许提交零速并解除启动闭环。 */
    assert(bx72_manual_control(&state, READY, 1, 0, 1, 0, 0, 0, &out));
    assert(!out.vx_mm_s && !out.vy_mm_s && !out.wz_mrad_s);
    bx72_manual_control(&state, READY, 1, 1, 1, 400, 0, 800, &out);
    assert(out.vx_mm_s == 10 && out.vy_mm_s == 0 && out.wz_mrad_s == 30);
    for (i = 0; i < 100u; ++i)
        bx72_manual_control(&state, READY, 1, 1, 1, 400, 0, 800, &out);
    assert(out.vx_mm_s == 400 && out.wz_mrad_s == 800);
    /* 新模式是停车栅栏；只有之后的新速度序号才可恢复。 */
    bx72_manual_control(&state, READY, 1, 1, 2, 400, 0, 800, &out);
    assert(!out.vx_mm_s && !out.vy_mm_s && !out.wz_mrad_s);
    bx72_manual_control(&state, READY, 1, 1, 2, 400, 0, 800, &out);
    assert(!out.vx_mm_s && !out.wz_mrad_s);
    bx72_manual_control(&state, READY, 1, 2, 2, 400, 0, 800, &out);
    assert(out.vx_mm_s == 10 && out.wz_mrad_s == 30);
    bx72_manual_control(&state, READY, 1, 3, 2, 0, 0, 0, &out);
    assert(!out.vx_mm_s && !out.vy_mm_s && !out.wz_mrad_s);
    bx72_manual_control(&state, READY | BX72_STATUS_LINK_DOWN, 1, 4, 2, 400, 0, 800, &out);
    assert(!out.vx_mm_s && !out.wz_mrad_s);
    bx72_manual_control(&state, READY | BX72_STATUS_GUARD_STOP, 1, 5, 2, 400, 0, 800, &out);
    assert(!out.vx_mm_s && !out.wz_mrad_s);
    bx72_manual_control(&state, READY, 0, 6, 2, 400, 0, 800, &out);
    assert(!out.vx_mm_s && !out.wz_mrad_s);
    /* CSM输入质量不参与速度授权；原始雷达不新鲜仍必须停车。 */
    bx72_manual_control(&state, READY & ~BX72_STATUS_LIDAR_OK, 1, 7, 2, 400, 0, 800, &out);
    assert(!out.vx_mm_s && !out.wz_mrad_s);
    puts("PASS manual speed, mode fence and stale/guard stop");
    return 0;
}
