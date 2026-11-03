#include "local_grid.h"

#include <assert.h>
#include <stdio.h>

static void mark_free_from(int32_t x, int32_t y, int32_t yaw, uint32_t stamp)
{
    bx72_pose_t pose = {0};
    pose.yaw_mrad = yaw;
    pose.x_mm = x - bx72_lidar_offset_component(bx72_cos_q15(yaw));
    pose.y_mm = y - bx72_lidar_offset_component(bx72_sin_q15(yaw));
    pose.flags = BX72_POSE_VALID;
    pose.stamp_us = stamp;
    assert(bx72_grid_lidar_validated(0u, 200u, 1u, &pose, stamp, stamp));
}

int main(void)
{
    bx72_grid_unknown_exemption_t start = {0, 0, BX72_GRID_START_CONFIRM_RADIUS_MM};
    bx72_grid_reset(0, 0, 0u);
    bx72_grid_scan_begin();
    mark_free_from(-25, 25, 3142, 1u);
    mark_free_from(75, 25, 0, 2u);
    mark_free_from(25, 75, 1571, 3u);
    assert(bx72_grid_cell_at(-1, 0) == BX72_GRID_FREE);
    assert(bx72_grid_cell_at(1, 0) == BX72_GRID_FREE);
    assert(bx72_grid_cell_at(0, 1) == BX72_GRID_FREE);
    assert(bx72_grid_cell_at(0, 0) == BX72_GRID_UNKNOWN);
    assert(bx72_grid_unknown_blocking(0, 0, 0));
    assert(!bx72_grid_unknown_blocking(0, 0, &start));
    assert(bx72_grid_unknown_blocking(10, 10, &start));
    puts("unknown remains blocking despite three free neighbours");
    return 0;
}
