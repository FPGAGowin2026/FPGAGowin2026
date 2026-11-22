#include "robot_navigation.h"
#include "robot_config.h"

#define PI_F 3.14159265358979323846f
#define PLAN_PERIOD_US 250000u
#define OBSERVATION_MAX_AGE_US 120000u
#define OBSERVATION_YAW_MRAD_S 600
#define UNKNOWN_TURN_MRAD_S 300
#define AVOID_LOOKAHEAD_MM 600
#define AVOID_HOLD_US 500000u
#define AVOID_CLEAR_HOLD_US 4000000u
#define AVOID_HEADING_MRAD 900
#define AVOID_CLOSE_HEADING_MRAD 1300

static int32_t millimetres(float metres)
{
    float mm = metres * 1000.0f;
    return (int32_t)(mm >= 0.0f ? mm + 0.5f : mm - 0.5f);
}

static int32_t unit_q15(float value)
{
    float scaled = value * 32768.0f;
    return (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static local_plan_pose_t current_pose(const odometry_t *odom)
{
    local_plan_pose_t pose;
    float sine, cosine;
    odometry_sincos(odom->state[2], &sine, &cosine);
    pose.x_mm = millimetres(odom->state[0]);
    pose.y_mm = millimetres(odom->state[1]);
    pose.cos_q15 = unit_q15(cosine);
    pose.sin_q15 = unit_q15(sine);
    pose.body_length_mm = ROBOT_BODY_LENGTH_MM;
    pose.body_width_mm = ROBOT_BODY_WIDTH_MM;
    pose.margin_mm = ROBOT_BODY_MARGIN_MM;
    pose.required_fresh_us = 0;
    return pose;
}

static void recenter_map(robot_navigation_t *nav, const local_plan_pose_t *pose, uint32_t now_us)
{
    int32_t lead = nav->follow.max_forward_mm_s > ROBOT_CRUISE_MM_S ? ROBOT_FAST_MAP_LEAD_MM : 0;
    local_map_recenter(&nav->map, pose->x_mm + lead * pose->cos_q15 / 32768,
                       pose->y_mm + lead * pose->sin_q15 / 32768, now_us);
}

static void begin_observation(robot_navigation_t *nav, const odometry_t *odom,
                              const local_plan_pose_t *pose)
{
    int64_t dx, dy, lateral;
    if (nav->observing || nav->observe_swept_mrad >= 6283u) return;
    dx = (int64_t)nav->goal_x_mm - pose->x_mm;
    dy = (int64_t)nav->goal_y_mm - pose->y_mm;
    lateral = -dx * pose->sin_q15 + dy * pose->cos_q15;
    nav->observe_direction = lateral >= 0 ? 1 : -1;
    nav->observe_last_yaw_mrad = millimetres(odom->state[2]);
    nav->observing = 1;
}

static int goal_outside_front_view(const robot_navigation_t *nav,
                                   const local_plan_pose_t *pose)
{
    int64_t dx = (int64_t)nav->goal_x_mm - pose->x_mm;
    int64_t dy = (int64_t)nav->goal_y_mm - pose->y_mm;
    int64_t forward = dx * pose->cos_q15 + dy * pose->sin_q15;
    int64_t lateral = -dx * pose->sin_q15 + dy * pose->cos_q15;
    if (lateral < 0) lateral = -lateral;
    return forward <= 0 || lateral > forward;
}

static int goal_position_reached(const robot_navigation_t *nav, const local_plan_pose_t *pose)
{
    int64_t dx = (int64_t)nav->goal_x_mm - pose->x_mm;
    int64_t dy = (int64_t)nav->goal_y_mm - pose->y_mm;
    int64_t radius = nav->follow.goal_radius_mm;
    return dx >= -radius && dx <= radius && dy >= -radius && dy <= radius &&
           dx * dx + dy * dy <= radius * radius;
}

static enum local_guard_result slow_observation_turn(
    robot_navigation_t *nav, const local_plan_pose_t *pose, const local_velocity_t *measured,
    local_velocity_t rotation, int front_scan_fresh, int front_guard_clear,
    uint32_t now_us, local_velocity_t *allowed)
{
    if (rotation.wz_mrad_s > UNKNOWN_TURN_MRAD_S) rotation.wz_mrad_s = UNKNOWN_TURN_MRAD_S;
    if (rotation.wz_mrad_s < -UNKNOWN_TURN_MRAD_S) rotation.wz_mrad_s = -UNKNOWN_TURN_MRAD_S;
    return local_guard_evaluate_exploration(&nav->map, pose, &nav->guard, measured, &rotation,
                                            front_scan_fresh, front_guard_clear, now_us, allowed);
}

static void command_observation_turn(
    robot_navigation_t *nav, const local_plan_pose_t *pose, const local_velocity_t *measured,
    local_velocity_t rotation, int front_scan_fresh, int front_guard_clear,
    uint32_t now_us, local_velocity_t *allowed)
{
    enum local_guard_result result = local_guard_evaluate(&nav->map, pose, &nav->guard, measured, &rotation,
                                                          front_scan_fresh, front_guard_clear, now_us, allowed);
    if (result == LOCAL_GUARD_UNOBSERVED || result == LOCAL_GUARD_EXPIRED)
        result = slow_observation_turn(nav, pose, measured, rotation,
                                       front_scan_fresh, front_guard_clear, now_us, allowed);
    nav->last_guard_result = (uint8_t)result;
    nav->status = result == LOCAL_GUARD_ALLOWED ? ROBOT_NAV_FOLLOWING : ROBOT_NAV_BLOCKED;
}

static uint16_t front_obstacle_distance(const robot_navigation_t *nav)
{
    uint16_t left = nav->avoid_left_scan < nav->avoid_left_previous ?
                    nav->avoid_left_scan : nav->avoid_left_previous;
    uint16_t right = nav->avoid_right_scan < nav->avoid_right_previous ?
                     nav->avoid_right_scan : nav->avoid_right_previous;
    return left < right ? left : right;
}

static int front_obstacle_near(const robot_navigation_t *nav, const local_plan_pose_t *pose,
                               uint32_t now_us)
{
    uint16_t nearest = front_obstacle_distance(nav);
    int64_t dx = (int64_t)nav->goal_x_mm - pose->x_mm;
    int64_t dy = (int64_t)nav->goal_y_mm - pose->y_mm;
    int64_t forward = (dx * pose->cos_q15 + dy * pose->sin_q15) / 32768;
    return (uint32_t)(now_us - nav->last_ray_us) <= AVOID_HOLD_US &&
           nearest <= AVOID_LOOKAHEAD_MM && (nav->avoid_direction || forward > nearest);
}

static int avoid_continuing(const robot_navigation_t *nav, uint32_t now_us)
{
    return nav->avoid_direction && (uint32_t)(now_us - nav->avoid_last_near_us) <= AVOID_CLEAR_HOLD_US;
}

static void finish_avoidance(robot_navigation_t *nav, const local_plan_pose_t *pose,
                             int32_t heading_mrad, uint32_t now_us)
{
    int64_t dx, dy, lateral;
    if (!nav->avoid_direction || front_obstacle_near(nav, pose, now_us) ||
        avoid_continuing(nav, now_us)) return;
    dx = (int64_t)nav->goal_x_mm - pose->x_mm;
    dy = (int64_t)nav->goal_y_mm - pose->y_mm;
    lateral = -dx * pose->sin_q15 + dy * pose->cos_q15;
    nav->avoid_direction = 0;
    nav->observe_direction = lateral >= 0 ? 1 : -1;
    nav->observe_last_yaw_mrad = heading_mrad;
    nav->observe_swept_mrad = 0;
}

static int slow_unknown_forward(robot_navigation_t *nav, const local_plan_pose_t *pose,
    const local_velocity_t *measured, int32_t heading_mrad, int front_scan_fresh, int front_guard_clear,
    uint32_t now_us, local_velocity_t *allowed)
{
    int64_t dx = (int64_t)nav->goal_x_mm - pose->x_mm;
    int64_t dy = (int64_t)nav->goal_y_mm - pose->y_mm;
    int64_t forward, lateral, yaw_rate;
    local_velocity_t request = {ROBOT_EXPLORATION_FORWARD_MM_S, 0, 0};
    int front_near;
    enum local_guard_result result;
    if (dx < -10000 || dx > 10000 || dy < -10000 || dy > 10000 ||
        goal_position_reached(nav, pose)) return 0;
    forward = (dx * pose->cos_q15 + dy * pose->sin_q15) / 32768;
    lateral = (-dx * pose->sin_q15 + dy * pose->cos_q15) / 32768;
    front_near = front_obstacle_near(nav, pose, now_us);
    if ((forward <= 0 || lateral < -forward || lateral > forward) &&
        !front_near && !avoid_continuing(nav, now_us)) return 0;
    yaw_rate = forward > 0 ? lateral * 400 / forward : 0;
    if (yaw_rate > 200) yaw_rate = 200;
    if (yaw_rate < -200) yaw_rate = -200;
    if (front_near) {
        int8_t previous_direction = nav->avoid_direction;
        uint16_t left = nav->avoid_left_scan < nav->avoid_left_previous ?
                        nav->avoid_left_scan : nav->avoid_left_previous;
        uint16_t right = nav->avoid_right_scan < nav->avoid_right_previous ?
                         nav->avoid_right_scan : nav->avoid_right_previous;
        if ((uint32_t)left + 75u < right) nav->avoid_direction = -1;
        else if ((uint32_t)right + 75u < left) nav->avoid_direction = 1;
        else if (!nav->avoid_direction) nav->avoid_direction = lateral >= 0 ? 1 : -1;
        if (nav->avoid_direction != previous_direction) {
            nav->avoid_heading_mrad = heading_mrad;
            nav->avoid_offset_mrad = AVOID_HEADING_MRAD;
        }
        if (front_obstacle_distance(nav) <= 300) nav->avoid_offset_mrad = AVOID_CLOSE_HEADING_MRAD;
        nav->avoid_last_near_us = now_us;
    } else if (!avoid_continuing(nav, now_us)) nav->avoid_direction = 0;
    if (nav->avoid_direction) {
        int32_t error = nav->avoid_heading_mrad + nav->avoid_direction * nav->avoid_offset_mrad - heading_mrad;
        while (error > 3142) error -= 6283;
        while (error < -3142) error += 6283;
        yaw_rate = error * 2;
        if (error > -20 && error < 20) yaw_rate = 0;
        if (front_near && front_obstacle_distance(nav) <= 300 &&
            (error < -150 || error > 150)) request.vx_mm_s = 0;
        /* Use the normal observation turn rate only inside a freshly observed
           braking envelope. Unknown space retains the exploration limits. */
        if (yaw_rate > UNKNOWN_TURN_MRAD_S || yaw_rate < -UNKNOWN_TURN_MRAD_S) {
            if (yaw_rate > OBSERVATION_YAW_MRAD_S) yaw_rate = OBSERVATION_YAW_MRAD_S;
            if (yaw_rate < -OBSERVATION_YAW_MRAD_S) yaw_rate = -OBSERVATION_YAW_MRAD_S;
            request.wz_mrad_s = (int32_t)yaw_rate;
            result = local_guard_evaluate(&nav->map, pose, &nav->guard, measured, &request,
                                          front_scan_fresh, front_guard_clear, now_us, allowed);
            if (result == LOCAL_GUARD_ALLOWED) {
                nav->last_guard_result = (uint8_t)result;
                return 1;
            }
            yaw_rate = yaw_rate > 0 ? UNKNOWN_TURN_MRAD_S : -UNKNOWN_TURN_MRAD_S;
        }
    }
    request.wz_mrad_s = (int32_t)yaw_rate;
    for (;;) {
        result = local_guard_evaluate_exploration(&nav->map, pose, &nav->guard, measured, &request,
                                                 front_scan_fresh, front_guard_clear, now_us, allowed);
        if (result < LOCAL_GUARD_UNOBSERVED) break;
        if (request.vx_mm_s > ROBOT_UNOBSERVED_FORWARD_MM_S) {
            request.vx_mm_s = ROBOT_UNOBSERVED_FORWARD_MM_S;
        } else if (result == LOCAL_GUARD_OCCUPIED && request.vx_mm_s > 0 && request.wz_mrad_s) {
            /* Finish passing a side obstacle before turning back toward the goal. */
            request.wz_mrad_s = 0;
        } else break;
    }
    nav->last_guard_result = (uint8_t)result;
    return result == LOCAL_GUARD_ALLOWED;
}

int robot_navigation_init(robot_navigation_t *nav, uint32_t now_us)
{
    if (!nav || !local_map_init(&nav->map, 0, 0, ROBOT_MAP_TTL_TICKS, now_us)) return 0;
    nav->guard.geometry_mm = ROBOT_GEOMETRY_MM;
    nav->guard.max_wheel_mm_s = ROBOT_MAX_WHEEL_MM_S;
    nav->guard.min_lateral_mm_s = ROBOT_MIN_LATERAL_MM_S;
    nav->guard.brake_x_mm_s2 = ROBOT_BRAKE_X_MM_S2;
    nav->guard.brake_y_mm_s2 = ROBOT_BRAKE_Y_MM_S2;
    nav->guard.brake_yaw_mrad_s2 = ROBOT_BRAKE_YAW_MRAD_S2;
    nav->guard.reaction_us = ROBOT_REACTION_US;
    nav->guard.observation_uncertainty_us = ROBOT_OBSERVATION_UNCERTAINTY_US;
    nav->calibration_ready = ROBOT_BRAKE_X_MM_S2 > 0 && ROBOT_BRAKE_Y_MM_S2 > 0 &&
        ROBOT_BRAKE_YAW_MRAD_S2 > 0 && ROBOT_REACTION_US > 0 &&
        ROBOT_OBSERVATION_UNCERTAINTY_US > 0;
    nav->follow.max_forward_mm_s = ROBOT_CRUISE_MM_S;
    nav->follow.max_lateral_mm_s = ROBOT_CRUISE_MM_S;
    nav->follow.point_radius_mm = 60;
    nav->follow.goal_radius_mm = 80;
    nav->follow.lateral_deadband_mm = 40;
    nav->follow.lookahead_mm = 200;
    nav->follow.stopped_speed_mm_s = 20;
    nav->follow.stopped_yaw_mrad_s = 20;
    route_follow_init(&nav->route);
    nav->plan.status = LOCAL_PLAN_IDLE;
    nav->last_plan_us = nav->last_ray_us = nav->avoid_last_near_us = now_us;
    nav->have_angle = nav->goal_active = nav->have_route = nav->planning = 0;
    nav->position_only = nav->position_arrived = nav->hold_done = nav->intermediate = nav->observing = 0;
    nav->observe_swept_mrad = 0;
    nav->avoid_left_scan = nav->avoid_right_scan = 65535u;
    nav->avoid_left_previous = nav->avoid_right_previous = 65535u;
    nav->avoid_direction = 0;
    nav->task_id = nav->deadline_us = 0;
    nav->status = ROBOT_NAV_IDLE;
    nav->last_guard_result = LOCAL_GUARD_STOP;
    return 1;
}

void robot_navigation_reset_map(robot_navigation_t *nav, uint32_t now_us)
{
    if (!nav) return;
    local_map_init(&nav->map, 0, 0, ROBOT_MAP_TTL_TICKS, now_us);
    route_follow_init(&nav->route);
    nav->have_angle = nav->have_route = nav->planning = nav->goal_active = 0;
    nav->avoid_left_scan = nav->avoid_right_scan = 65535u;
    nav->avoid_left_previous = nav->avoid_right_previous = 65535u;
    nav->avoid_direction = 0;
    nav->status = ROBOT_NAV_IDLE;
    nav->last_guard_result = LOCAL_GUARD_STOP;
}

void robot_navigation_lidar_gap(robot_navigation_t *nav, int32_t center_x_mm,
                                int32_t center_y_mm, uint32_t now_us)
{
    if (!nav) return;
    local_map_init(&nav->map, center_x_mm, center_y_mm, nav->map.ttl_ticks, now_us);
    route_follow_init(&nav->route);
    nav->have_angle = nav->have_route = nav->planning = nav->observing = 0;
    nav->observe_swept_mrad = 0;
    nav->avoid_left_scan = nav->avoid_right_scan = 65535u;
    nav->avoid_left_previous = nav->avoid_right_previous = 65535u;
    nav->avoid_direction = 0;
    nav->last_plan_us = now_us - PLAN_PERIOD_US;
    if (nav->goal_active) nav->status = ROBOT_NAV_PLANNING;
}

int robot_navigation_set_goal(robot_navigation_t *nav, uint32_t task_id,
    int32_t x_mm, int32_t y_mm, int32_t hold_yaw_mrad, int32_t yaw_mrad,
    uint16_t yaw_tolerance_mrad, int position_only, int intermediate,
    uint16_t speed_mm_s, uint16_t acceleration_mm_s2, uint16_t tolerance_mm,
    uint32_t deadline_us, uint32_t now_us)
{
    int replace_route;
    if (!nav || !task_id || speed_mm_s < ROBOT_MIN_LATERAL_MM_S ||
        speed_mm_s > ROBOT_MAX_WHEEL_MM_S || acceleration_mm_s2 < 1 || acceleration_mm_s2 > 1000 ||
        tolerance_mm < 20 || tolerance_mm > 500 ||
        yaw_mrad < -3142 || yaw_mrad > 3142 || hold_yaw_mrad < -3142 || hold_yaw_mrad > 3142 ||
        yaw_tolerance_mrad < 10 || yaw_tolerance_mrad > 1000 ||
        (int32_t)(deadline_us - now_us) <= 0) return 0;
    if (!nav->goal_active) nav->avoid_direction = 0;
    replace_route = !nav->goal_active || nav->goal_x_mm != x_mm || nav->goal_y_mm != y_mm ||
        nav->hold_yaw_mrad != hold_yaw_mrad || nav->intermediate != (intermediate != 0);
    nav->goal_x_mm = x_mm;
    nav->goal_y_mm = y_mm;
    nav->goal_yaw_mrad = yaw_mrad;
    nav->hold_yaw_mrad = hold_yaw_mrad;
    nav->yaw_tolerance_mrad = yaw_tolerance_mrad;
    nav->max_accel_mm_s2 = acceleration_mm_s2;
    nav->position_only = position_only != 0;
    nav->position_arrived = 0;
    nav->hold_done = 0;
    nav->intermediate = intermediate != 0;
    nav->observing = 0;
    nav->observe_swept_mrad = 0;
    nav->task_id = task_id;
    nav->deadline_us = deadline_us;
    local_map_set_ttl(&nav->map, speed_mm_s > ROBOT_CRUISE_MM_S ?
                      ROBOT_FAST_MAP_TTL_TICKS : ROBOT_MAP_TTL_TICKS, now_us);
    nav->follow.max_forward_mm_s = speed_mm_s;
    nav->follow.max_lateral_mm_s = speed_mm_s;
    nav->follow.goal_radius_mm = tolerance_mm;
    if (nav->follow.lateral_deadband_mm > tolerance_mm)
        nav->follow.lateral_deadband_mm = tolerance_mm;
    if (replace_route) {
        nav->have_route = 0;
        route_follow_init(&nav->route);
    }
    nav->goal_active = 1;
    nav->planning = 0;
    nav->last_plan_us = now_us - PLAN_PERIOD_US;
    nav->status = ROBOT_NAV_PLANNING;
    return 1;
}

void robot_navigation_cancel(robot_navigation_t *nav)
{
    if (!nav) return;
    nav->goal_active = nav->planning = nav->have_route = 0;
    route_follow_init(&nav->route);
    nav->status = ROBOT_NAV_CANCELLED;
    nav->last_guard_result = LOCAL_GUARD_STOP;
}

void robot_navigation_observe(robot_navigation_t *nav, const odometry_t *odom,
    uint16_t angle_cdeg, uint16_t distance_mm, int hit,
    uint32_t sample_us, uint32_t now_us)
{
    float x, y, yaw, sine, cosine, ray_sine, ray_cosine, ray_angle;
    int32_t sensor_x, sensor_y, end_x, end_y;
    if (!nav || !odom || angle_cdeg >= 36000 ||
        (angle_cdeg > 9000 && angle_cdeg < 27000) ||
        (uint32_t)(now_us - sample_us) > OBSERVATION_MAX_AGE_US ||
        !odometry_pose_at(odom, sample_us, &x, &y, &yaw)) return;
    if (!nav->have_angle || (nav->last_angle_cdeg <= 9000 && angle_cdeg >= 27000)) {
        if (nav->have_angle) {
            nav->avoid_left_previous = nav->avoid_left_scan;
            nav->avoid_right_previous = nav->avoid_right_scan;
        }
        nav->avoid_left_scan = nav->avoid_right_scan = 65535u;
        local_map_begin_scan(&nav->map, now_us);
    }
    nav->have_angle = 1;
    nav->last_angle_cdeg = angle_cdeg;
    nav->last_ray_us = sample_us;
    if (!hit || distance_mm < 50 || distance_mm == 65535) return;
    odometry_sincos(yaw, &sine, &cosine);
    sensor_x = millimetres(x) + (int32_t)(ROBOT_LIDAR_X_MM * cosine);
    sensor_y = millimetres(y) + (int32_t)(ROBOT_LIDAR_X_MM * sine);
    ray_angle = yaw - (float)angle_cdeg * (PI_F / 18000.0f);
    odometry_sincos(ray_angle, &ray_sine, &ray_cosine);
    if (distance_mm <= AVOID_LOOKAHEAD_MM &&
        (angle_cdeg <= 4500 || angle_cdeg >= 31500)) {
        float lateral = (ray_sine * cosine - ray_cosine * sine) * (float)distance_mm;
        float corridor = (float)(ROBOT_BODY_WIDTH_MM / 2 + ROBOT_BODY_MARGIN_MM + 50);
        if (lateral >= -corridor && lateral <= corridor) {
            uint16_t *nearest = angle_cdeg <= 4500 ? &nav->avoid_right_scan : &nav->avoid_left_scan;
            if (distance_mm < *nearest) *nearest = distance_mm;
        }
    }
    end_x = sensor_x + (int32_t)((float)distance_mm * ray_cosine);
    end_y = sensor_y + (int32_t)((float)distance_mm * ray_sine);
    local_map_observe_ray(&nav->map, sensor_x, sensor_y, end_x, end_y, 1, sample_us, now_us);
}

void robot_navigation_work(robot_navigation_t *nav, const odometry_t *odom, uint32_t now_us)
{
    local_plan_pose_t pose;
    enum local_plan_status result;
    local_plan_point_t points[LOCAL_PLAN_MAX_POINTS];
    uint16_t count;
    int ends_at_goal;
    if (!nav || !odom || !nav->goal_active || !nav->hold_done || nav->position_arrived ||
        !nav->calibration_ready || !odometry_valid(odom, now_us) ||
        (nav->follow.max_forward_mm_s > ROBOT_CRUISE_MM_S && nav->route.braking)) return;
    pose = current_pose(odom);
    recenter_map(nav, &pose, now_us);
    if (!nav->planning && (uint32_t)(now_us - nav->last_plan_us) >= PLAN_PERIOD_US) {
        nav->last_plan_us = now_us;
        nav->planning = local_plan_begin(&nav->plan, &nav->map, &pose,
                                        nav->goal_x_mm, nav->goal_y_mm, now_us) != 0;
    }
    if (!nav->planning) return;
    /* Yield to control after each node on the RV32 soft core. */
    result = local_plan_step(&nav->plan, 1);
    if (result == LOCAL_PLAN_RUNNING) return;
    nav->planning = 0;
    count = local_plan_result(&nav->plan, points, LOCAL_PLAN_MAX_POINTS, &ends_at_goal);
    if (count && route_follow_set(&nav->route, points, count, ends_at_goal && !nav->intermediate)) {
        nav->have_route = 1;
        if (nav->observing) {
            /* Keep the observed heading so freshly seen space stays in view. */
            nav->hold_yaw_mrad = millimetres(odom->state[2]);
            nav->hold_done = 1;
        }
        nav->observing = 0;
    } else if (!nav->have_route) {
        nav->status = ROBOT_NAV_BLOCKED;
        begin_observation(nav, odom, &pose);
    }
}

void robot_navigation_step(robot_navigation_t *nav, const odometry_t *odom,
    int front_scan_fresh, int front_guard_clear, uint32_t now_us,
    local_velocity_t *allowed)
{
    local_plan_pose_t pose;
    local_velocity_t measured;
    enum route_follow_status follow_result;
    enum local_guard_result guard_result = LOCAL_GUARD_STOP;
    int32_t yaw_error;
    if (!allowed) return;
    allowed->vx_mm_s = allowed->vy_mm_s = allowed->wz_mrad_s = 0;
    if (!nav || !odom) return;
    nav->last_guard_result = LOCAL_GUARD_STOP;
    if (!nav->goal_active) return;
    if ((int32_t)(now_us - nav->deadline_us) >= 0) {
        nav->goal_active = nav->have_route = nav->planning = 0;
        nav->status = ROBOT_NAV_BLOCKED;
        return;
    }
    if (!odometry_valid(odom, now_us) || !nav->calibration_ready || !front_scan_fresh) {
        nav->status = ROBOT_NAV_BLOCKED;
        return;
    }
    pose = current_pose(odom);
    recenter_map(nav, &pose, now_us);
    finish_avoidance(nav, &pose, millimetres(odom->state[2]), now_us);
    measured.vx_mm_s = millimetres(odom->state[3]);
    measured.vy_mm_s = millimetres(odom->state[4]);
    measured.wz_mrad_s = millimetres(odom->state[5]);
    if (!nav->hold_done || nav->position_arrived) {
        local_velocity_t rotation = {0, 0, 0};
        yaw_error = (nav->position_arrived ? nav->goal_yaw_mrad : nav->hold_yaw_mrad) - millimetres(odom->state[2]);
        while (yaw_error > 3142) yaw_error -= 6283;
        while (yaw_error < -3142) yaw_error += 6283;
        if (yaw_error <= nav->yaw_tolerance_mrad && yaw_error >= -(int32_t)nav->yaw_tolerance_mrad) {
            if (measured.wz_mrad_s <= nav->follow.stopped_yaw_mrad_s &&
                measured.wz_mrad_s >= -nav->follow.stopped_yaw_mrad_s) {
                if (nav->position_arrived) {
                    nav->goal_active = 0;
                    nav->status = ROBOT_NAV_ARRIVED;
                    return;
                }
                nav->hold_done = 1;
            } else return;
        } else {
            int32_t speed = yaw_error > 0 ? yaw_error : -yaw_error;
            speed *= 2;
            if (speed < 220) speed = 220;
            if (speed > OBSERVATION_YAW_MRAD_S) speed = OBSERVATION_YAW_MRAD_S;
            rotation.wz_mrad_s = yaw_error > 0 ? speed : -speed;
            command_observation_turn(nav, &pose, &measured, rotation,
                                     front_scan_fresh, front_guard_clear, now_us, allowed);
            return;
        }
    }
    /* A partial local route can end before the real goal enters the braking horizon. */
    if (!nav->intermediate && nav->follow.max_forward_mm_s > ROBOT_CRUISE_MM_S &&
        (!nav->have_route || !nav->route.ends_at_goal)) {
        local_plan_point_t goal = {nav->goal_x_mm, nav->goal_y_mm};
        follow_result = route_follow_brake(&nav->route, &goal, &pose, &nav->follow, &nav->guard, &measured);
        if (follow_result != ROUTE_FOLLOW_MOVING) goto apply_follow_result;
    }
    if (!nav->have_route) {
        if (goal_position_reached(nav, &pose)) {
            if (measured.vx_mm_s > nav->follow.stopped_speed_mm_s ||
                measured.vx_mm_s < -nav->follow.stopped_speed_mm_s ||
                measured.vy_mm_s > nav->follow.stopped_speed_mm_s ||
                measured.vy_mm_s < -nav->follow.stopped_speed_mm_s ||
                measured.wz_mrad_s > nav->follow.stopped_yaw_mrad_s ||
                measured.wz_mrad_s < -nav->follow.stopped_yaw_mrad_s) {
                nav->status = ROBOT_NAV_PLANNING;
            } else if (nav->position_only || nav->intermediate) {
                nav->goal_active = 0;
                nav->status = ROBOT_NAV_ARRIVED;
            } else {
                nav->position_arrived = 1;
                nav->status = ROBOT_NAV_PLANNING;
            }
            return;
        }
        if (!nav->observing && goal_outside_front_view(nav, &pose))
            begin_observation(nav, odom, &pose);
        if (slow_unknown_forward(nav, &pose, &measured, millimetres(odom->state[2]), front_scan_fresh, front_guard_clear,
                                 now_us, allowed)) {
            if (!nav->observing) begin_observation(nav, odom, &pose);
            nav->status = ROBOT_NAV_FOLLOWING;
            return;
        }
        if (nav->planning && !nav->observing) nav->status = ROBOT_NAV_PLANNING;
        else if ((nav->observing || nav->plan.status == LOCAL_PLAN_BLOCKED ||
                  nav->plan.status == LOCAL_PLAN_PARTIAL) && nav->observe_swept_mrad < 6283u) {
            local_velocity_t rotation = {0, 0, 0};
            int32_t yaw_now = millimetres(odom->state[2]);
            if (!nav->observing) {
                begin_observation(nav, odom, &pose);
            } else {
                int32_t delta = yaw_now - nav->observe_last_yaw_mrad;
                while (delta > 3142) delta -= 6283;
                while (delta < -3142) delta += 6283;
                nav->observe_swept_mrad += (uint32_t)(delta < 0 ? -delta : delta);
                nav->observe_last_yaw_mrad = yaw_now;
            }
            if (nav->observe_swept_mrad >= 6283u) {
                nav->status = ROBOT_NAV_BLOCKED;
                return;
            }
            rotation.wz_mrad_s = nav->observe_direction * OBSERVATION_YAW_MRAD_S;
            command_observation_turn(nav, &pose, &measured, rotation,
                                     front_scan_fresh, front_guard_clear, now_us, allowed);
        }
        return;
    }
    follow_result = route_follow_step(&nav->route, &nav->map, &pose, &nav->follow,
        &nav->guard, &measured, front_scan_fresh, front_guard_clear, now_us, allowed, &guard_result);
apply_follow_result:
    nav->last_guard_result = (uint8_t)guard_result;
    if (follow_result == ROUTE_FOLLOW_MOVING) {
        local_velocity_t avoid;
        /* Keep a planned lateral detour; otherwise react to a close front
           obstacle without waiting for the next local-planner result. */
        if (allowed->vx_mm_s > 0 && !allowed->vy_mm_s &&
            (front_obstacle_near(nav, &pose, now_us) || avoid_continuing(nav, now_us))) {
            if (slow_unknown_forward(nav, &pose, &measured, millimetres(odom->state[2]), front_scan_fresh, front_guard_clear,
                                     now_us, &avoid)) {
                if (avoid.vx_mm_s > allowed->vx_mm_s) avoid.vx_mm_s = allowed->vx_mm_s;
                *allowed = avoid;
            } else {
                local_velocity_t slow = {allowed->vx_mm_s < ROBOT_UNOBSERVED_FORWARD_MM_S ?
                                         allowed->vx_mm_s : ROBOT_UNOBSERVED_FORWARD_MM_S, 0, 0};
                guard_result = local_guard_evaluate_exploration(&nav->map, &pose, &nav->guard,
                    &measured, &slow, front_scan_fresh, front_guard_clear, now_us, allowed);
                nav->last_guard_result = (uint8_t)guard_result;
                if (guard_result != LOCAL_GUARD_ALLOWED) {
                    nav->status = ROBOT_NAV_BLOCKED;
                    return;
                }
            }
        }
        nav->status = ROBOT_NAV_FOLLOWING;
    }
    else if (follow_result == ROUTE_FOLLOW_ARRIVED) {
        nav->have_route = 0;
        if (nav->position_only) {
            nav->goal_active = 0;
            nav->status = ROBOT_NAV_ARRIVED;
        } else {
            nav->position_arrived = 1;
            nav->status = ROBOT_NAV_PLANNING;
        }
    } else if (follow_result == ROUTE_FOLLOW_NEED_PATH || follow_result == ROUTE_FOLLOW_BLOCKED ||
               follow_result == ROUTE_FOLLOW_INVALID) {
        nav->have_route = 0;
        nav->status = ROBOT_NAV_BLOCKED;
        if (follow_result == ROUTE_FOLLOW_BLOCKED &&
            (guard_result == LOCAL_GUARD_UNOBSERVED || guard_result == LOCAL_GUARD_EXPIRED ||
             (guard_result == LOCAL_GUARD_OCCUPIED && front_obstacle_near(nav, &pose, now_us)))) {
            if (guard_result != LOCAL_GUARD_OCCUPIED) begin_observation(nav, odom, &pose);
            if (slow_unknown_forward(nav, &pose, &measured, millimetres(odom->state[2]), front_scan_fresh, front_guard_clear,
                                     now_us, allowed)) nav->status = ROBOT_NAV_FOLLOWING;
        }
    } else nav->status = ROBOT_NAV_PLANNING;
}
