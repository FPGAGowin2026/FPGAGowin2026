#ifndef ROBOT_NAVIGATION_H
#define ROBOT_NAVIGATION_H

#include "odometry.h"
#include "route_follow.h"

enum robot_nav_state {
    ROBOT_NAV_IDLE = 0,
    ROBOT_NAV_PLANNING = 2,
    ROBOT_NAV_FOLLOWING = 3,
    ROBOT_NAV_ARRIVED = 4,
    ROBOT_NAV_BLOCKED = 5,
    ROBOT_NAV_REJECTED = 6,
    ROBOT_NAV_CANCELLED = 7
};

typedef struct {
    local_map_t map;
    local_plan_t plan;
    route_follow_t route;
    local_guard_config_t guard;
    route_follow_config_t follow;
    int32_t goal_x_mm, goal_y_mm, goal_yaw_mrad, hold_yaw_mrad;
    int32_t observe_last_yaw_mrad, avoid_heading_mrad, avoid_offset_mrad;
    uint32_t task_id, deadline_us, last_plan_us, last_ray_us, observe_swept_mrad, avoid_last_near_us;
    uint16_t last_angle_cdeg, yaw_tolerance_mrad, max_accel_mm_s2;
    uint16_t avoid_left_scan, avoid_right_scan, avoid_left_previous, avoid_right_previous;
    uint8_t have_angle, goal_active, have_route, planning, calibration_ready;
    uint8_t position_only, position_arrived, hold_done, intermediate, observing;
    uint8_t last_guard_result;
    int8_t observe_direction, avoid_direction;
    enum robot_nav_state status;
} robot_navigation_t;

int robot_navigation_init(robot_navigation_t *nav, uint32_t now_us);
void robot_navigation_reset_map(robot_navigation_t *nav, uint32_t now_us);
void robot_navigation_lidar_gap(robot_navigation_t *nav, int32_t center_x_mm,
                                int32_t center_y_mm, uint32_t now_us);
int robot_navigation_set_goal(robot_navigation_t *nav, uint32_t task_id,
    int32_t x_mm, int32_t y_mm, int32_t hold_yaw_mrad, int32_t yaw_mrad,
    uint16_t yaw_tolerance_mrad, int position_only, int intermediate,
    uint16_t speed_mm_s, uint16_t acceleration_mm_s2, uint16_t tolerance_mm,
    uint32_t deadline_us, uint32_t now_us);
void robot_navigation_cancel(robot_navigation_t *nav);
void robot_navigation_observe(robot_navigation_t *nav, const odometry_t *odom,
    uint16_t angle_cdeg, uint16_t distance_mm, int hit,
    uint32_t sample_us, uint32_t now_us);
void robot_navigation_work(robot_navigation_t *nav, const odometry_t *odom, uint32_t now_us);
void robot_navigation_step(robot_navigation_t *nav, const odometry_t *odom,
    int front_scan_fresh, int front_guard_clear, uint32_t now_us,
    local_velocity_t *allowed);

#endif
