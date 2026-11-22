#include "local_guard.h"
#include "footprint.h"
#include "robot_config.h"

#define Q15_ONE 32768
#define HALF_CELL_MM (LOCAL_MAP_CELL_MM / 2)

static int32_t abs32(int32_t value)
{
    return value < 0 ? -value : value;
}

static int32_t floor_cell(int32_t mm)
{
    int32_t value = mm / LOCAL_MAP_CELL_MM;
    if (mm % LOCAL_MAP_CELL_MM < 0) --value;
    return value;
}

static uint64_t ceil_div(uint64_t numerator, uint64_t denominator)
{
    return numerator / denominator + (numerator % denominator != 0);
}

static uint64_t stop_time_us(int32_t speed, int32_t deceleration)
{
    return ceil_div((uint64_t)speed * 1000000u, (uint32_t)deceleration);
}

static uint32_t stop_distance(int32_t speed, int32_t deceleration, uint32_t reaction_us)
{
    return (uint32_t)(ceil_div((uint64_t)speed * reaction_us, 1000000u) +
           ceil_div((uint64_t)speed * (uint32_t)speed, 2u * (uint32_t)deceleration));
}

static int32_t positive_limit(int32_t measured, int32_t requested)
{
    int32_t result = measured > requested ? measured : requested;
    return result > 0 ? result : 0;
}

static int32_t negative_limit(int32_t measured, int32_t requested)
{
    int32_t result = measured < requested ? measured : requested;
    return result < 0 ? -result : 0;
}

static uint32_t radius_ceil(uint32_t x, uint32_t y)
{
    uint32_t squared = x * x + y * y;
    uint32_t low = 0, high = x + y;
    while (low < high) {
        uint32_t mid = low + (high - low) / 2u;
        if (mid * mid < squared) low = mid + 1u;
        else high = mid;
    }
    return low;
}

static int velocity_valid(const local_velocity_t *v)
{
    return v->vx_mm_s >= -5000 && v->vx_mm_s <= 5000 &&
           v->vy_mm_s >= -5000 && v->vy_mm_s <= 5000 &&
           v->wz_mrad_s >= -10000 && v->wz_mrad_s <= 10000;
}

static enum local_guard_result evaluate(
    local_map_t *map, const local_plan_pose_t *pose, const local_guard_config_t *config,
    const local_velocity_t *measured, const local_velocity_t *requested,
    int front_scan_fresh, int front_guard_clear, uint32_t now_us,
    local_velocity_t *allowed, int allow_unknown)
{
    local_velocity_t command;
    int32_t rotation, peak, wheel[4], i;
    int32_t pos_x, neg_x, pos_y, neg_y, max_x_speed, max_y_speed, max_yaw_speed;
    uint64_t time_x, time_y, time_yaw, horizon, yaw_distance, total_distance, deflection;
    int64_t xmin, xmax, ymin, ymax, middle_x, middle_y, center_x, center_y;
    int64_t half_x, half_y, radius, angle, extent, x, y;
    int32_t start_half_length, start_half_width;
    int confirmed_footprint;
    if (!allowed || !requested) return LOCAL_GUARD_INVALID;
    allowed->vx_mm_s = allowed->vy_mm_s = allowed->wz_mrad_s = 0;
    if (!requested->vx_mm_s && !requested->vy_mm_s && !requested->wz_mrad_s)
        return LOCAL_GUARD_STOP;
    if (!map || !map->initialized || !pose || !config || !measured ||
        !velocity_valid(requested) || !velocity_valid(measured) ||
        !footprint_orientation_valid(pose->cos_q15, pose->sin_q15) ||
        pose->body_length_mm <= 0 || pose->body_length_mm > 2000 ||
        pose->body_width_mm <= 0 || pose->body_width_mm > 2000 ||
        pose->margin_mm < 0 || pose->margin_mm > 1000 ||
        config->geometry_mm <= 0 || config->geometry_mm > 2000 ||
        config->max_wheel_mm_s <= 0 || config->max_wheel_mm_s > 5000 ||
        config->min_lateral_mm_s < 300 || config->min_lateral_mm_s > config->max_wheel_mm_s ||
        config->brake_x_mm_s2 <= 0 || config->brake_y_mm_s2 <= 0 ||
        config->brake_yaw_mrad_s2 <= 0 || config->reaction_us > 5000000u ||
        config->observation_uncertainty_us > 5000000u)
        return LOCAL_GUARD_INVALID;
    if (!front_scan_fresh) return LOCAL_GUARD_NO_FRONT_SCAN;
    if (!front_guard_clear) return LOCAL_GUARD_FRONT_BLOCKED;

    command = *requested;
    rotation = (int32_t)((int64_t)command.wz_mrad_s * config->geometry_mm / 1000);
    wheel[0] = command.vx_mm_s - command.vy_mm_s - rotation;
    wheel[1] = command.vx_mm_s + command.vy_mm_s + rotation;
    wheel[2] = command.vx_mm_s + command.vy_mm_s - rotation;
    wheel[3] = command.vx_mm_s - command.vy_mm_s + rotation;
    peak = 0;
    for (i = 0; i < 4; ++i)
        if (abs32(wheel[i]) > peak) peak = abs32(wheel[i]);
    if (peak > config->max_wheel_mm_s) {
        command.vx_mm_s = (int32_t)((int64_t)command.vx_mm_s * config->max_wheel_mm_s / peak);
        command.vy_mm_s = (int32_t)((int64_t)command.vy_mm_s * config->max_wheel_mm_s / peak);
        command.wz_mrad_s = (int32_t)((int64_t)command.wz_mrad_s * config->max_wheel_mm_s / peak);
    }
    if (requested->vy_mm_s && abs32(command.vy_mm_s) < config->min_lateral_mm_s)
        return LOCAL_GUARD_LATERAL_TOO_SLOW;

    pos_x = positive_limit(measured->vx_mm_s, command.vx_mm_s);
    neg_x = negative_limit(measured->vx_mm_s, command.vx_mm_s);
    pos_y = positive_limit(measured->vy_mm_s, command.vy_mm_s);
    neg_y = negative_limit(measured->vy_mm_s, command.vy_mm_s);
    max_x_speed = pos_x > neg_x ? pos_x : neg_x;
    max_y_speed = pos_y > neg_y ? pos_y : neg_y;
    max_yaw_speed = abs32(measured->wz_mrad_s) > abs32(command.wz_mrad_s) ?
                    abs32(measured->wz_mrad_s) : abs32(command.wz_mrad_s);
    time_x = stop_time_us(max_x_speed, config->brake_x_mm_s2);
    time_y = stop_time_us(max_y_speed, config->brake_y_mm_s2);
    time_yaw = stop_time_us(max_yaw_speed, config->brake_yaw_mrad_s2);
    horizon = time_x > time_y ? time_x : time_y;
    if (time_yaw > horizon) horizon = time_yaw;
    horizon += config->reaction_us;
    horizon += config->observation_uncertainty_us;
    if (!allow_unknown && horizon >= (uint64_t)map->ttl_ticks * LOCAL_MAP_TICK_US)
        return LOCAL_GUARD_EXPIRED;

    yaw_distance = stop_distance(max_yaw_speed, config->brake_yaw_mrad_s2,
                                 config->reaction_us);
    total_distance = ceil_div((uint64_t)(max_x_speed + max_y_speed) * horizon, 1000000u);
    angle = yaw_distance < 2000u ? (int64_t)yaw_distance : 2000;
    deflection = ceil_div(total_distance * (uint64_t)angle, 1000u);
    xmin = -(int64_t)stop_distance(neg_x, config->brake_x_mm_s2, config->reaction_us) -
           (int64_t)deflection;
    xmax = (int64_t)stop_distance(pos_x, config->brake_x_mm_s2, config->reaction_us) +
           (int64_t)deflection;
    ymin = -(int64_t)stop_distance(neg_y, config->brake_y_mm_s2, config->reaction_us) -
           (int64_t)deflection;
    ymax = (int64_t)stop_distance(pos_y, config->brake_y_mm_s2, config->reaction_us) +
           (int64_t)deflection;
    middle_x = (xmin + xmax) / 2;
    middle_y = (ymin + ymax) / 2;
    center_x = pose->x_mm + (middle_x * pose->cos_q15 - middle_y * pose->sin_q15) / Q15_ONE;
    center_y = pose->y_mm + (middle_x * pose->sin_q15 + middle_y * pose->cos_q15) / Q15_ONE;

    start_half_length = pose->body_length_mm / 2;
    start_half_width = pose->body_width_mm / 2;
    radius = radius_ceil((uint32_t)start_half_length, (uint32_t)start_half_width);
    angle = yaw_distance < 1000u ? (int64_t)yaw_distance : 1000;
    half_x = start_half_length + (int64_t)ceil_div((uint64_t)start_half_width * (uint64_t)angle, 1000u);
    half_y = start_half_width + (int64_t)ceil_div((uint64_t)start_half_length * (uint64_t)angle, 1000u);
    if (half_x > radius) half_x = radius;
    if (half_y > radius) half_y = radius;
    half_x += pose->margin_mm + 2 + (xmax - xmin + 1) / 2;
    half_y += pose->margin_mm + 2 + (ymax - ymin + 1) / 2;
    extent = half_x + half_y + HALF_CELL_MM;
    if (center_x < INT32_MIN + extent || center_x > INT32_MAX - extent ||
        center_y < INT32_MIN + extent || center_y > INT32_MAX - extent ||
        extent > LOCAL_MAP_SIDE * LOCAL_MAP_CELL_MM)
        return LOCAL_GUARD_UNOBSERVED;

    /* A grid cell may cross the circle even when the swept footprint does not. */
    confirmed_footprint = local_map_confirmed_footprint(map, (int32_t)center_x,
        (int32_t)center_y, pose->cos_q15, pose->sin_q15,
        (int32_t)half_x, (int32_t)half_y, now_us, (uint32_t)horizon);

    for (y = floor_cell((int32_t)(center_y - extent));
         y <= floor_cell((int32_t)(center_y + extent)); ++y) {
        for (x = floor_cell((int32_t)(center_x - extent));
             x <= floor_cell((int32_t)(center_x + extent)); ++x) {
            uint32_t remaining = 0;
            enum local_map_state state;
            if (!footprint_cell_intersects((int32_t)x, (int32_t)y, (int32_t)center_x,
                (int32_t)center_y, pose->cos_q15, pose->sin_q15, (int32_t)half_x, (int32_t)half_y))
                continue;
            state = local_map_get(map, (int32_t)x, (int32_t)y, now_us, &remaining);
            if (state == LOCAL_MAP_OCCUPIED) return LOCAL_GUARD_OCCUPIED;
            if (confirmed_footprint && x >= map->origin_x_cell && y >= map->origin_y_cell &&
                x < map->origin_x_cell + LOCAL_MAP_SIDE &&
                y < map->origin_y_cell + LOCAL_MAP_SIDE) continue;
            if (!allow_unknown && state == LOCAL_MAP_UNKNOWN &&
                !footprint_cell_inside((int32_t)x, (int32_t)y, pose->x_mm, pose->y_mm,
                    pose->cos_q15, pose->sin_q15,
                    start_half_length - pose->margin_mm - 1,
                    start_half_width - pose->margin_mm - 1) &&
                !local_map_confirmed_free(map, (int32_t)x, (int32_t)y, now_us, (uint32_t)horizon))
                return LOCAL_GUARD_UNOBSERVED;
            if (!allow_unknown && state == LOCAL_MAP_FREE && remaining < horizon &&
                !local_map_confirmed_free(map, (int32_t)x, (int32_t)y, now_us, (uint32_t)horizon))
                return LOCAL_GUARD_EXPIRED;
        }
    }
    *allowed = command;
    return LOCAL_GUARD_ALLOWED;
}

enum local_guard_result local_guard_evaluate(
    local_map_t *map, const local_plan_pose_t *pose, const local_guard_config_t *config,
    const local_velocity_t *measured, const local_velocity_t *requested,
    int front_scan_fresh, int front_guard_clear, uint32_t now_us, local_velocity_t *allowed)
{
    return evaluate(map, pose, config, measured, requested,
                    front_scan_fresh, front_guard_clear, now_us, allowed, 0);
}

enum local_guard_result local_guard_evaluate_exploration(
    local_map_t *map, const local_plan_pose_t *pose, const local_guard_config_t *config,
    const local_velocity_t *measured, const local_velocity_t *requested,
    int front_scan_fresh, int front_guard_clear, uint32_t now_us, local_velocity_t *allowed)
{
    if (!allowed || !requested || requested->vx_mm_s < 0 ||
        requested->vx_mm_s > ROBOT_EXPLORATION_FORWARD_MM_S ||
        requested->vy_mm_s != 0 || requested->wz_mrad_s < -300 || requested->wz_mrad_s > 300) {
        if (allowed) allowed->vx_mm_s = allowed->vy_mm_s = allowed->wz_mrad_s = 0;
        return LOCAL_GUARD_INVALID;
    }
    return evaluate(map, pose, config, measured, requested,
                    front_scan_fresh, front_guard_clear, now_us, allowed,
                    requested->vx_mm_s <= ROBOT_UNOBSERVED_FORWARD_MM_S);
}
