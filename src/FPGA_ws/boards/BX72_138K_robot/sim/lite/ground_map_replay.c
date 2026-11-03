/* 回放已记录的局部图和估计位姿，仅检查实时扫掠；不是逐拍实车复现。 */
#include "navigation.c"
#include <stdio.h>
#include <string.h>

uint32_t bx72_nav_test_mmio[0x2b0u / 4u];
int load_recorded_grid(int origin_x, int origin_y);

int main(void)
{
    int count, frame;
    if (scanf("%d", &count) != 1 || count <= 0) return 1;
    for (frame = 0; frame < count; ++frame) {
        static const int speed[] = {4, 20, 40, 60, 80, 100};
        bx72_pose_t pose = {0};
        int ox, oy, x, y, yaw, vx, vy, wz, ax, ay, oax, oay, offset, sx, sy;
        unsigned int i;
        if (scanf("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
                  &ox, &oy, &x, &y, &yaw, &vx, &vy, &wz,
                  &ax, &ay, &oax, &oay, &offset, &sx, &sy) != 15) return 2;
        memset(bx72_nav_test_mmio, 0, sizeof(bx72_nav_test_mmio));
        bx72_nav_test_mmio[BX72_IO_BOOT_ID / 4u] = 7u;
        bx72_nav_test_mmio[BX72_IO_ODOM_DOMAIN / 4u] = 1u;
        pose.flags = BX72_POSE_VALID;
        pose.stamp_us = 1000000u;
        bx72_nav_init();
        bx72_nav_step(&pose, pose.stamp_us);
        if (!load_recorded_grid(ox, oy)) return 3;
        map_initialized = 1u;
        map_anchor_x = ax; map_anchor_y = ay;
        map_odom_anchor_x = oax; map_odom_anchor_y = oay; map_yaw_offset = offset;
        start_exemption_valid = 1u;
        start_exemption.x_mm = sx; start_exemption.y_mm = sy;
        start_exemption.radius_mm = BX72_GRID_START_CONFIRM_RADIUS_MM;
        pose.x_mm = x; pose.y_mm = y; pose.yaw_mrad = yaw;
        /* 沿用原记录分析的零速度条件，区别于真实运动反馈。 */
        pose.vx_mm_s = pose.vy_mm_s = pose.wz_mrad_s = 0;
        previous_vx_mm_s = previous_vy_mm_s = previous_wz_mrad_s = 0;
        printf("frame %d", frame);
        for (i = 0u; i < sizeof(speed) / sizeof(speed[0]); ++i) {
            int32_t end_x, end_y;
            uint8_t unknown;
            int valid = evaluate_motion(&pose, speed[i], 0, 1, &end_x, &end_y,
                                        &unknown, current_time_us());
            printf(" %d:%d/%u", speed[i], valid, (unsigned int)unknown);
        }
        putchar('\n');
    }
    return 0;
}
