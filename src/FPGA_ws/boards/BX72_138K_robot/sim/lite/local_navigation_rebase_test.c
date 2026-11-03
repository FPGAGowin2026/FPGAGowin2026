/* Exercise the real odom/grid transform and path invalidation together. */
#include "navigation.c"

#include <assert.h>
#include <stdio.h>
#include <string.h>

uint32_t bx72_nav_test_mmio[0x2b0u / 4u];

int main(void)
{
    bx72_pose_t original = {0}, corrected, corrected_again, grid_goal;
    int64_t original_goal;
    int32_t original_target_x, original_target_y;
    int final;
    unsigned i;

    memset(bx72_nav_test_mmio, 0, sizeof(bx72_nav_test_mmio));
    bx72_nav_test_mmio[BX72_IO_BOOT_ID / 4u] = 7u;
    bx72_nav_test_mmio[BX72_IO_ODOM_DOMAIN / 4u] = 1u;
    bx72_nav_test_mmio[BX72_IO_SESSION_LO / 4u] = 2u;
    bx72_nav_init();
    original.flags = BX72_POSE_VALID;
    original.stamp_us = 1000000u;
    bx72_nav_step(&original, original.stamp_us);

    task_head = 0u;
    task_count = 1u;
    task_queue[0].goal_x_mm_q16 = 425ll * NAV_Q16_ONE;
    task_queue[0].goal_y_mm_q16 = 0;
    original_goal = task_queue[0].goal_x_mm_q16;
    task_goal_to_grid(&task_queue[0], &grid_goal);
    bx72_local_path_begin(0, 0, grid_goal.x_mm, grid_goal.y_mm);
    for (i = 0u; i < 1000u && bx72_local_path_state() == BX72_PATH_WORKING; ++i)
        bx72_local_path_step(current_time_us, 1000u);
    assert(bx72_local_path_target(&detour_x_mm, &detour_y_mm, &final));
    detour_active = 1u;
    path_target_final = final != 0;
    guide_goal_x_mm = grid_goal.x_mm;
    guide_goal_y_mm = grid_goal.y_mm;
    guide_goal_valid = selected_valid = 1u;
    original_target_x = detour_x_mm;
    original_target_y = detour_y_mm;

    corrected = original;
    corrected.x_mm += 2;
    bx72_nav_rebase(&original, &corrected);
    assert(task_queue[0].goal_x_mm_q16 == original_goal);
    assert(detour_active && selected_valid && bx72_local_path_state() == BX72_PATH_FOUND);
    task_goal_to_grid(&task_queue[0], &grid_goal);
    if (path_target_final)
        assert(detour_x_mm == grid_goal.x_mm && detour_y_mm == grid_goal.y_mm);
    else
        assert(detour_x_mm == original_target_x && detour_y_mm == original_target_y);

    corrected_again = corrected;
    corrected_again.x_mm += 60;
    bx72_nav_rebase(&corrected, &corrected_again);
    assert(task_queue[0].goal_x_mm_q16 == original_goal);
    if (final) {
        /* 最终点跨格后必须按固定 odom 目标重新投影。 */
        assert(!detour_active && bx72_local_path_state() == BX72_PATH_IDLE);
    } else {
        assert(detour_active && selected_valid && path_refresh_pending &&
               bx72_local_path_state() == BX72_PATH_IDLE);
        corrected = corrected_again;
        corrected_again.x_mm += 60;
        bx72_nav_rebase(&corrected, &corrected_again);
        assert(!detour_active && !selected_valid && bx72_local_path_state() == BX72_PATH_IDLE);
    }

    /* 临近目标时横向小修正也可能显著改变最终目标方向。 */
    {
        bx72_pose_t nearby = {0}, shifted = {0};
        detour_active = guide_goal_valid = 1u;
        guide_goal_x_mm = 200; guide_goal_y_mm = 0;
        shifted.x_mm = 200; shifted.y_mm = 60;
        assert(!rebase_keeps_guide(&nearby, &shifted));
    }

    /* 控制事件在先时先同步序号再收片；事件在后时取消任务及残片。 */
    bx72_nav_init();
    bx72_nav_sync_inputs(1000000u, 3u, 5u);
    fragment_collecting = 1u;
    bx72_nav_sync_inputs(1000001u, 3u, 5u);
    assert(fragment_collecting);
    start_exemption_valid = 1u;
    task_count = 1u;
    active_task_id = 10u;
    bx72_nav_sync_inputs(1000002u, 3u, 6u);
    assert(!fragment_collecting && task_count == 0u && start_exemption_valid);
    task_count = 1u;
    active_task_id = 11u;
    fragment_collecting = 1u;
    bx72_nav_sync_inputs(1000003u, 4u, 6u);
    assert(!fragment_collecting && task_count == 0u && start_exemption_valid);
    task_count = 1u;
    active_task_id = 12u;
    fragment_collecting = 1u;
    /* 非零及随后的零速若落在同一软件采样间隔，非零代次仍可区分。 */
    bx72_nav_test_mmio[BX72_IO_MANUAL_MOTION_SEQ / 4u] = 1u;
    bx72_nav_sync_inputs(1000004u, 6u, 6u);
    assert(!fragment_collecting && task_count == 0u && !start_exemption_valid);

    /* 中间点保持运动并提前续段；最终点依旧减速。 */
    {
        bx72_pose_t moving = {0}, grid_position = {0};
        bx72_nav_output_t output = {0};
        bx72_grid_info_t grid_info;
        uint32_t now = 2000000u;
        unsigned ray;
        bx72_nav_init();
        moving.flags = BX72_POSE_VALID;
        moving.stamp_us = now;
        bx72_nav_step(&moving, now);
        for (ray = 0u; ray < 4321u; ++ray) {
            uint32_t angle = (ray % 1440u) * 25u;
            now += 50u;
            moving.stamp_us = now;
            bx72_nav_lidar_validated(angle, 6000u, 1u, &moving, now, now);
        }
        bx72_grid_info(&grid_info);
        assert(grid_info.fresh);
        task_head = 0u; task_count = 1u;
        task_queue[0].task_id = 20u;
        task_queue[0].goal_x_mm_q16 = 1000ll * NAV_Q16_ONE;
        task_queue[0].goal_y_mm_q16 = 0;
        task_queue[0].tolerance_mm = 50u;
        task_queue[0].max_speed_mm_s = 200u;
        task_queue[0].max_accel_mm_s2 = 200u;
        task_queue[0].mode = 3u;
        task_queue[0].flags = NAV_TASK_POSITION_ONLY;
        task_queue[0].expiry_us = now_extended_us + 10000000u;
        task_start_initialized = 1u;
        active_task_id = 20u;
        best_goal_distance_mm = 600u;
        last_progress_us = now_extended_us;
        start_exemption_valid = 1u;
        start_exemption.x_mm = 0; start_exemption.y_mm = 0;
        start_exemption.radius_mm = BX72_GRID_START_CONFIRM_RADIUS_MM;
        planner_inhibited = 0u;
        detour_active = 1u; path_target_final = 0u;
        detour_x_mm = 500; detour_y_mm = 25;
        path_start_x_mm = path_start_y_mm = 0;
        selected_valid = planner_complete = 1u;
        selected_vx = 100; selected_wz = 0;
        assert(guide_speed(400u, 200u) == 200u);
        assert(guide_speed(80u, 200u) == 80u);
        path_target_final = 1u;
        assert(guide_speed(80u, 200u) == 40u);
        path_target_final = 0u;
        grid_position.x_mm = 400; grid_position.y_mm = 25;
        assert(path_guide_aligned(&grid_position, 700, 25));
        assert(!path_guide_aligned(&grid_position, 450, 25));
        assert(!path_guide_aligned(&grid_position, 600, 145));
        moving.x_mm = 400;
        bx72_nav_step(&moving, now);
        assert(path_refresh_pending && detour_x_mm == 500 && selected_valid);
        /* 续段未完成且即将越过旧点时，停住并继续等待同一搜索。 */
        now += 20000u;
        moving.x_mm = 480; moving.y_mm = 25; moving.stamp_us = now;
        bx72_nav_control(&moving, now, BX72_STATUS_AUTO | BX72_STATUS_IMU_OK |
                         BX72_STATUS_LIDAR_OK | BX72_STATUS_WHEEL_OK,
                         0u, 0, 0, 0, 0u, &output);
        assert(path_refresh_pending && detour_active && output.vx_mm_s == 0 &&
               output.status == BX72_NAV_STATUS_PLANNING);
        for (i = 0u; i < 1000u && path_refresh_pending; ++i)
            bx72_nav_step(&moving, now);
        assert(!path_refresh_pending && detour_active && detour_x_mm > 500 && selected_valid);
        /* 预取成功不放行后续新障碍，当前输出仍复用原实时扫掠。 */
        {
            int32_t end_x, end_y;
            uint8_t unknown;
            now += 100u;
            moving.stamp_us = now;
            bx72_nav_lidar_validated(0u, 50u, 1u, &moving, now, now);
            moving.vx_mm_s = previous_vx_mm_s = 200;
            assert(evaluate_motion(&moving, 200, 0, 1, &end_x, &end_y,
                                   &unknown, current_time_us()) == 0);
        }
    }
    puts("odom goal survives rebase; grid path follows new projection");
    return 0;
}
