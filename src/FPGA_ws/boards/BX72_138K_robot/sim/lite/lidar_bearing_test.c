/* 原始角度顺时针：89度在车体前侧可见区内，对应负Y。 */
#include <assert.h>
#include <stdio.h>
#include "csm.h"

int main(void)
{
    bx72_pose_t pose = {0};
    static bx72_csm_state_t csm;
    pose.flags = BX72_POSE_VALID;
    pose.stamp_us = 1000u;
    bx72_csm_init(&csm);
    bx72_csm_point(&csm, 8900u, 1000u, 1u, &pose);
    assert(csm.collect_valid[17] && csm.collect_scan_x[17] > 147.0f &&
           csm.collect_scan_x[17] < 148.0f && csm.collect_scan_y[17] < -990.0f);
    puts("PASS clockwise lidar bearing in CSM");
    return 0;
}
