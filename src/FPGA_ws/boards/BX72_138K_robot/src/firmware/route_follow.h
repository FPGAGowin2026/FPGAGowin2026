#ifndef ROBOT_ROUTE_FOLLOW_H
#define ROBOT_ROUTE_FOLLOW_H

#include "local_guard.h"

typedef struct {
    int32_t max_forward_mm_s;
    int32_t max_lateral_mm_s;
    int32_t point_radius_mm;
    int32_t goal_radius_mm;
    int32_t lateral_deadband_mm;
    int32_t lookahead_mm;
    int32_t stopped_speed_mm_s;
    int32_t stopped_yaw_mrad_s;
} route_follow_config_t;

typedef struct {
    local_plan_point_t points[LOCAL_PLAN_MAX_POINTS];
    uint16_t count, cursor;
    uint8_t ends_at_goal, braking;
} route_follow_t;

enum route_follow_status {
    ROUTE_FOLLOW_IDLE,
    ROUTE_FOLLOW_MOVING,
    ROUTE_FOLLOW_BRAKING,
    ROUTE_FOLLOW_ARRIVED,
    ROUTE_FOLLOW_NEED_PATH,
    ROUTE_FOLLOW_BLOCKED,
    ROUTE_FOLLOW_INVALID
};

void route_follow_init(route_follow_t *route);

/* Replacing a reference path does not deliberately stop at intermediate points. */
int route_follow_set(route_follow_t *route, const local_plan_point_t *points,
                     uint16_t count, int ends_at_goal);

enum route_follow_status route_follow_brake(
    route_follow_t *route, const local_plan_point_t *goal, const local_plan_pose_t *pose,
    const route_follow_config_t *config, const local_guard_config_t *guard_config,
    const local_velocity_t *measured);

/*
 * Computes a fixed-heading holonomic translation and checks it against the
 * current map. Rotation and re-observation require a separate mode. The caller
 * must refresh this command at the control rate and retain the hardware guard.
 */
enum route_follow_status route_follow_step(
    route_follow_t *route, local_map_t *map, const local_plan_pose_t *pose,
    const route_follow_config_t *config, const local_guard_config_t *guard_config,
    const local_velocity_t *measured, int front_scan_fresh, int front_guard_clear,
    uint32_t now_us, local_velocity_t *allowed, enum local_guard_result *guard_result);

#endif
