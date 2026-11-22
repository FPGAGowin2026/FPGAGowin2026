#include "route_follow.h"
#include "footprint.h"
#include "robot_config.h"

#define Q15_ONE 32768

static int32_t abs32(int32_t value)
{
    return value < 0 ? -value : value;
}

static uint64_t ceil_div(uint64_t numerator, uint64_t denominator)
{
    return numerator / denominator + (numerator % denominator != 0);
}

static uint64_t distance_squared(const local_plan_point_t *point, const local_plan_pose_t *pose)
{
    int64_t dx = (int64_t)point->x_mm - pose->x_mm;
    int64_t dy = (int64_t)point->y_mm - pose->y_mm;
    return (uint64_t)(dx * dx + dy * dy);
}

static int path_corner(const route_follow_t *route, uint16_t i,
                       const local_plan_pose_t *pose, uint32_t corner_radius_squared)
{
    const local_plan_point_t *point = &route->points[i];
    const local_plan_point_t *next = &route->points[i + 1u];
    int32_t dx = point->x_mm - (i ? route->points[i - 1u].x_mm : pose->x_mm);
    int32_t dy = point->y_mm - (i ? route->points[i - 1u].y_mm : pose->y_mm);
    int32_t next_x = next->x_mm - point->x_mm;
    int32_t next_y = next->y_mm - point->y_mm;
    int32_t cross = dx * next_y - dy * next_x;

    /* The planner omits its start cell. Allow the live pose to be slightly off the first line. */
    if (!i)
        return (int64_t)cross * cross >
               (int64_t)corner_radius_squared * (next_x * next_x + next_y * next_y);
    return cross != 0 || dx * next_x + dy * next_y < 0;
}

static int32_t floor_cell(int32_t mm)
{
    int32_t cell = mm / LOCAL_MAP_CELL_MM;
    if (mm % LOCAL_MAP_CELL_MM < 0) --cell;
    return cell;
}

/* The guarded stopping envelope alone does not establish a route to the goal. */
static int straight_path_clear(local_map_t *map, const local_plan_pose_t *pose,
                               int64_t forward_mm, uint32_t now_us)
{
    int64_t center_x, center_y, half_length, half_width, extent_x, extent_y;
    int32_t abs_cos, abs_sin;
    int32_t x, y;
    if (forward_mm <= 0 || forward_mm > LOCAL_MAP_SIDE * LOCAL_MAP_CELL_MM) return 0;
    center_x = (int64_t)pose->x_mm + (forward_mm * pose->cos_q15) / (2 * Q15_ONE);
    center_y = (int64_t)pose->y_mm + (forward_mm * pose->sin_q15) / (2 * Q15_ONE);
    half_length = pose->body_length_mm / 2 + pose->margin_mm + (forward_mm + 1) / 2 + 2;
    half_width = pose->body_width_mm / 2 + pose->margin_mm + 2;
    abs_cos = abs32(pose->cos_q15);
    abs_sin = abs32(pose->sin_q15);
    extent_x = (half_length * abs_cos + half_width * abs_sin + Q15_ONE - 1) /
               Q15_ONE + LOCAL_MAP_CELL_MM / 2;
    extent_y = (half_length * abs_sin + half_width * abs_cos + Q15_ONE - 1) /
               Q15_ONE + LOCAL_MAP_CELL_MM / 2;
    if (center_x < INT32_MIN + extent_x || center_x > INT32_MAX - extent_x ||
        center_y < INT32_MIN + extent_y || center_y > INT32_MAX - extent_y) return 0;
    for (y = floor_cell((int32_t)(center_y - extent_y));
         y <= floor_cell((int32_t)(center_y + extent_y)); ++y) {
        for (x = floor_cell((int32_t)(center_x - extent_x));
             x <= floor_cell((int32_t)(center_x + extent_x)); ++x) {
            enum local_map_state state;
            if (!footprint_cell_intersects(x, y, (int32_t)center_x, (int32_t)center_y,
                    pose->cos_q15, pose->sin_q15, (int32_t)half_length, (int32_t)half_width))
                continue;
            state = local_map_get(map, x, y, now_us, 0);
            if (state == LOCAL_MAP_OCCUPIED) return 0;
            if (state == LOCAL_MAP_UNKNOWN &&
                !footprint_cell_inside(x, y, pose->x_mm, pose->y_mm, pose->cos_q15,
                    pose->sin_q15, pose->body_length_mm / 2 - pose->margin_mm - 1,
                    pose->body_width_mm / 2 - pose->margin_mm - 1) &&
                !local_map_confirmed_free(map, x, y, now_us, 0)) return 0;
        }
    }
    return 1;
}

static int route_straight_clear(const route_follow_t *route, local_map_t *map,
                                const local_plan_pose_t *pose, int32_t deadband_mm, uint32_t now_us)
{
    int64_t final_x = (int64_t)route->points[route->count - 1].x_mm - pose->x_mm;
    int64_t final_y = (int64_t)route->points[route->count - 1].y_mm - pose->y_mm;
    int64_t forward = (final_x * pose->cos_q15 + final_y * pose->sin_q15) / Q15_ONE;
    int64_t lateral = (-final_x * pose->sin_q15 + final_y * pose->cos_q15) / Q15_ONE;
    return forward > 0 && lateral >= -deadband_mm && lateral <= deadband_mm &&
           straight_path_clear(map, pose, forward, now_us);
}

void route_follow_init(route_follow_t *route)
{
    if (!route) return;
    route->count = route->cursor = 0;
    route->ends_at_goal = route->braking = 0;
}

int route_follow_set(route_follow_t *route, const local_plan_point_t *points,
                     uint16_t count, int ends_at_goal)
{
    uint16_t i;
    int same_final;
    if (!route || !points || !count || count > LOCAL_PLAN_MAX_POINTS) return 0;
    same_final = route->count && route->ends_at_goal && ends_at_goal &&
                 route->points[route->count - 1].x_mm == points[count - 1].x_mm &&
                 route->points[route->count - 1].y_mm == points[count - 1].y_mm;
    for (i = 0; i < count; ++i) route->points[i] = points[i];
    route->count = count;
    route->cursor = 0;
    route->ends_at_goal = ends_at_goal != 0;
    if (!same_final) route->braking = 0;
    return 1;
}

enum route_follow_status route_follow_brake(
    route_follow_t *route, const local_plan_point_t *goal, const local_plan_pose_t *pose,
    const route_follow_config_t *config, const local_guard_config_t *guard_config,
    const local_velocity_t *measured)
{
    int64_t dx = (int64_t)goal->x_mm - pose->x_mm;
    int64_t dy = (int64_t)goal->y_mm - pose->y_mm;
    uint64_t goal_distance = UINT64_MAX;
    int32_t speed = abs32(measured->vx_mm_s) + abs32(measured->vy_mm_s);
    int32_t brake = guard_config->brake_x_mm_s2 < guard_config->brake_y_mm_s2 ?
                    guard_config->brake_x_mm_s2 : guard_config->brake_y_mm_s2;
    uint64_t brake_distance = ceil_div((uint64_t)speed * guard_config->reaction_us, 1000000u) +
                              ceil_div((uint64_t)speed * (uint32_t)speed, 2u * (uint32_t)brake) +
                              (uint32_t)config->goal_radius_mm;
    if (dx >= -20000 && dx <= 20000 && dy >= -20000 && dy <= 20000)
        goal_distance = (uint64_t)(dx * dx + dy * dy);
    if (brake_distance > 20000 || goal_distance <= brake_distance * brake_distance)
        route->braking = 1;
    if (route->braking) {
        if (speed > config->stopped_speed_mm_s ||
            abs32(measured->wz_mrad_s) > config->stopped_yaw_mrad_s)
            return ROUTE_FOLLOW_BRAKING;
        if (goal_distance <= (uint64_t)config->goal_radius_mm * (uint32_t)config->goal_radius_mm)
            return ROUTE_FOLLOW_ARRIVED;
        route->braking = 0;
    }
    return ROUTE_FOLLOW_MOVING;
}

enum route_follow_status route_follow_step(
    route_follow_t *route, local_map_t *map, const local_plan_pose_t *pose,
    const route_follow_config_t *config, const local_guard_config_t *guard_config,
    const local_velocity_t *measured, int front_scan_fresh, int front_guard_clear,
    uint32_t now_us, local_velocity_t *allowed, enum local_guard_result *guard_result)
{
    uint16_t target, i;
    int64_t dx, dy, body_x, body_y;
    int32_t distance_x, distance_y, corner_radius;
    uint32_t point_radius_squared, corner_radius_squared, lookahead_squared;
    int target_is_corner = 0;
    local_velocity_t request = {0, 0, 0};
    enum local_guard_result guard;
    if (!allowed) return ROUTE_FOLLOW_INVALID;
    allowed->vx_mm_s = allowed->vy_mm_s = allowed->wz_mrad_s = 0;
    if (guard_result) *guard_result = LOCAL_GUARD_STOP;
    if (!route || !pose || !config || !guard_config || !measured) return ROUTE_FOLLOW_INVALID;
    if (!route->count) return ROUTE_FOLLOW_IDLE;
    if (config->point_radius_mm <= 0 || config->point_radius_mm > 2000 ||
        config->goal_radius_mm <= 0 || config->goal_radius_mm > 2000 ||
        config->lookahead_mm < config->point_radius_mm || config->lookahead_mm > 2000 ||
        config->lateral_deadband_mm < 0 || config->lateral_deadband_mm > config->goal_radius_mm ||
        config->stopped_speed_mm_s < 0 || config->stopped_yaw_mrad_s < 0 ||
        config->max_forward_mm_s <= 0 || config->max_forward_mm_s > guard_config->max_wheel_mm_s ||
        config->max_lateral_mm_s < guard_config->min_lateral_mm_s ||
        config->max_lateral_mm_s > guard_config->max_wheel_mm_s ||
        guard_config->min_lateral_mm_s < 300 || guard_config->max_wheel_mm_s > 5000 ||
        guard_config->brake_x_mm_s2 <= 0 || guard_config->brake_y_mm_s2 <= 0 ||
        guard_config->reaction_us > 5000000u ||
        !footprint_orientation_valid(pose->cos_q15, pose->sin_q15) ||
        measured->vx_mm_s < -5000 || measured->vx_mm_s > 5000 ||
        measured->vy_mm_s < -5000 || measured->vy_mm_s > 5000 ||
        measured->wz_mrad_s < -10000 || measured->wz_mrad_s > 10000)
        return ROUTE_FOLLOW_INVALID;
    for (i = 0; i < route->count; ++i) {
        dx = (int64_t)route->points[i].x_mm - pose->x_mm;
        dy = (int64_t)route->points[i].y_mm - pose->y_mm;
        if (dx < -10000 || dx > 10000 || dy < -10000 || dy > 10000)
            return ROUTE_FOLLOW_INVALID;
    }
    if (route->ends_at_goal) {
        enum route_follow_status braking = route_follow_brake(route, &route->points[route->count - 1],
                                                              pose, config, guard_config, measured);
        if (braking != ROUTE_FOLLOW_MOVING) return braking;
    }

    /* The ordinary point radius can include an entire cell before a turn. */
    corner_radius = config->point_radius_mm < LOCAL_MAP_CELL_MM / 2 ?
                    config->point_radius_mm : LOCAL_MAP_CELL_MM / 2;
    point_radius_squared = (uint32_t)config->point_radius_mm * (uint32_t)config->point_radius_mm;
    corner_radius_squared = (uint32_t)corner_radius * (uint32_t)corner_radius;
    lookahead_squared = (uint32_t)config->lookahead_mm * (uint32_t)config->lookahead_mm;
    while (route->cursor + 1u < route->count) {
        uint64_t current_distance = distance_squared(&route->points[route->cursor], pose);
        if (path_corner(route, route->cursor, pose, corner_radius_squared)) {
            if (current_distance > corner_radius_squared) break;
        } else if (current_distance > point_radius_squared &&
                   distance_squared(&route->points[route->cursor + 1u], pose) >= current_distance) {
            break;
        }
        ++route->cursor;
    }
    for (target = route->cursor; target + 1u < route->count; ++target) {
        target_is_corner = path_corner(route, target, pose, corner_radius_squared);
        if (target_is_corner || distance_squared(&route->points[target + 1u], pose) > lookahead_squared)
            break;
    }
    if (!route->ends_at_goal && target == route->count - 1u &&
        distance_squared(&route->points[target], pose) <= point_radius_squared)
        return ROUTE_FOLLOW_NEED_PATH;
    dx = (int64_t)route->points[target].x_mm - pose->x_mm;
    dy = (int64_t)route->points[target].y_mm - pose->y_mm;
    body_x = (dx * pose->cos_q15 + dy * pose->sin_q15) / Q15_ONE;
    body_y = (-dx * pose->sin_q15 + dy * pose->cos_q15) / Q15_ONE;
    distance_x = abs32((int32_t)body_x);
    distance_y = abs32((int32_t)body_y);
    /* Near a corner, a lateral deadband wider than its radius can prevent progress. */
    if (distance_y > config->lateral_deadband_mm ||
        (target_is_corner && distance_x <= corner_radius && distance_y > 0 &&
         distance_y + 1 >= corner_radius)) {
        int32_t total = distance_x + distance_y;
        int32_t lateral = config->max_lateral_mm_s * distance_y / total;
        int32_t forward;
        if (lateral < guard_config->min_lateral_mm_s) lateral = guard_config->min_lateral_mm_s;
        forward = config->max_forward_mm_s * distance_x / total;
        if (forward > guard_config->max_wheel_mm_s - lateral)
            forward = guard_config->max_wheel_mm_s - lateral;
        request.vx_mm_s = body_x < 0 ? -forward : forward;
        request.vy_mm_s = body_y < 0 ? -lateral : lateral;
    } else if (distance_x > 0) {
        request.vx_mm_s = body_x < 0 ? -config->max_forward_mm_s : config->max_forward_mm_s;
    } else {
        return route->ends_at_goal ? ROUTE_FOLLOW_BRAKING : ROUTE_FOLLOW_NEED_PATH;
    }

    guard = local_guard_evaluate(map, pose, guard_config, measured, &request,
                                 front_scan_fresh, front_guard_clear, now_us, allowed);
    if ((guard == LOCAL_GUARD_UNOBSERVED || guard == LOCAL_GUARD_EXPIRED ||
         guard == LOCAL_GUARD_OCCUPIED) && abs32(request.vy_mm_s) > guard_config->min_lateral_mm_s) {
        local_velocity_t slower = request;
        int32_t lateral = abs32(request.vy_mm_s);
        int32_t forward = (abs32(request.vx_mm_s) * guard_config->min_lateral_mm_s + lateral - 1) / lateral;
        slower.vx_mm_s = request.vx_mm_s < 0 ? -forward : forward;
        slower.vy_mm_s = request.vy_mm_s < 0 ? -guard_config->min_lateral_mm_s : guard_config->min_lateral_mm_s;
        if (local_guard_evaluate(map, pose, guard_config, measured, &slower,
                                 front_scan_fresh, front_guard_clear, now_us, allowed) == LOCAL_GUARD_ALLOWED)
            guard = LOCAL_GUARD_ALLOWED;
    }
    if ((guard == LOCAL_GUARD_UNOBSERVED || guard == LOCAL_GUARD_EXPIRED) &&
        request.vx_mm_s > 0 && request.vy_mm_s != 0) {
        if (route_straight_clear(route, map, pose, config->lateral_deadband_mm, now_us)) {
            request.vx_mm_s = config->max_forward_mm_s;
            request.vy_mm_s = 0;
            guard = local_guard_evaluate(map, pose, guard_config, measured, &request,
                                         front_scan_fresh, front_guard_clear, now_us, allowed);
        }
    }
    if (request.vx_mm_s > ROBOT_EXPLORATION_FORWARD_MM_S && request.vy_mm_s == 0 &&
        (guard == LOCAL_GUARD_UNOBSERVED || guard == LOCAL_GUARD_EXPIRED ||
         (guard == LOCAL_GUARD_OCCUPIED &&
          ((target_is_corner && straight_path_clear(map, pose, body_x, now_us)) ||
           route_straight_clear(route, map, pose, config->lateral_deadband_mm, now_us))))) {
        int32_t speed;
        for (speed = ((request.vx_mm_s - 1) / 100) * 100;
             speed >= ROBOT_EXPLORATION_FORWARD_MM_S; speed -= 100) {
            request.vx_mm_s = speed;
            guard = local_guard_evaluate(map, pose, guard_config, measured, &request,
                                         front_scan_fresh, front_guard_clear, now_us, allowed);
            if (guard != LOCAL_GUARD_UNOBSERVED && guard != LOCAL_GUARD_EXPIRED &&
                guard != LOCAL_GUARD_OCCUPIED) break;
        }
    }
    if (guard_result) *guard_result = guard;
    return guard == LOCAL_GUARD_ALLOWED ? ROUTE_FOLLOW_MOVING : ROUTE_FOLLOW_BLOCKED;
}
